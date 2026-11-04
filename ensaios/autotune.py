#!/usr/bin/env python3
"""Roda o AutoTune do ArduPilot no mesmo SITL dos ensaios e guarda os ganhos.

O SITL dos ensaios sobe sempre com -w (EEPROM apagada), entao o que o AutoTune
salva no veiculo morre com ele. O que fica e a pasta de resultado:

    resultados/autotune/<carimbo>/
        ganhos.yaml    ganhos antes e depois, e os blocos prontos para o campo
                       ganhos: de controladores/nativo.yaml e cc_pid.yaml
        autotune.txt   as mensagens do AutoTune, com o tempo simulado
        params.parm    dump completo, tirado do .BIN
        *.BIN, sitl.log

O voo segue o roteiro do proprio ArduPilot (Tools/autotest/arducopter.py,
AutoTuneSwitch): decola em GUIDED, passa para LOITER com os manches no centro,
entra em AUTOTUNE e espera o "AutoTune: Success". Depois POUSA SEM SAIR DO
AUTOTUNE — so o desarme dentro do modo grava os ganhos
(AC_AutoTune::disarmed); trocar de modo antes descarta tudo.

    ./autotune.py                    # rolagem, arfagem e guinada (AUTOTUNE_AXES 7)
    ./autotune.py --eixos 4          # so guinada
    ./autotune.py --aggr 0.1         # mais agressivo que o padrao 0.075
"""

from __future__ import annotations

import argparse
import datetime as dt
import shutil
import sys
import time
from pathlib import Path

import yaml

from run_ensaio import (PARAMS_BASE, RESULTADOS, LOGS_SITL, log, _rel, subir_sitl,
                        conectar, aplicar_params, reiniciar, esperar_param, set_param,
                        esperar_pronto, armar_e_decolar, aplicar_rc_override,
                        pedir_streams, derrubar_sitl, extrair_params)

PASTA_AUTOTUNE = RESULTADOS / "autotune"

# Manches no centro: rolagem, arfagem, acelerador (subida zero no LOITER e no
# AUTOTUNE) e guinada. Qualquer coisa fora da zona morta conta como piloto
# assumindo, e o AutoTune pausa (AC_AutoTune::run).
MANCHES_CENTRO = {1: 1500, 2: 1500, 3: 1500, 4: 1500}
# acelerador embaixo: desce em AUTOTUNE ate o detector de pouso desarmar
MANCHES_DESCER = {1: 1500, 2: 1500, 3: 1000, 4: 1500}

PARAMS_AUTOTUNE = {
    # O log completo dos ensaios da ~0,4 MB por segundo simulado; o AutoTune voa
    # dezenas de minutos. Aqui fica so o necessario para conferir o voo:
    # atitude media, GPS, PM, CTUN, entrada RC, corrente, saida RC e motores.
    # As mensagens ATUN/ATDE do AutoTune nao dependem do bitmask.
    "LOG_BITMASK": (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 6)
                   | (1 << 9) | (1 << 10) | (1 << 17),
    # o override de RC nunca expira: com speedup, os 3 s padrao de tempo
    # simulado passam em fracao de segundo de parede
    "RC_OVERRIDE_TIME": -1,
}

# O que o AutoTune grava (AC_AutoTune_Multi::save_tuning_gains). FF, FLTT, FLTD e
# SMAX ele restaura aos originais; o FLTE de guinada ele sintoniza.
EIXOS = {"RLL": "rolagem", "PIT": "arfagem", "YAW": "guinada"}
GANHOS_TAXA = ("P", "I", "D")
GANHOS_AUTOTUNE = (
    [f"ATC_RAT_{e}_{g}" for e in EIXOS for g in GANHOS_TAXA]
    + ["ATC_RAT_YAW_FLTE"]
    + [f"ATC_ANG_{e}_P" for e in EIXOS]
    + ["ATC_ACC_R_MAX", "ATC_ACC_P_MAX", "ATC_ACC_Y_MAX"]
)

MODO_AUTOTUNE = 15


def ler_param(m, nome, tentativas=5):
    """Valor atual de um parametro, por PARAM_REQUEST_READ.

    Cada tentativa le PARAM_VALUE ate achar o nome pedido: chegam tambem os de
    outros parametros (respostas atrasadas de escritas anteriores), e aceitar so
    a primeira mensagem esgotava as tentativas com respostas alheias.
    """
    for _ in range(tentativas):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      nome.encode(), -1)
        fim = time.monotonic() + 2.0
        while time.monotonic() < fim:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == nome:
                return round(float(msg.param_value), 6)
    raise SystemExit(f"nao consegui ler o parametro {nome}")


def ler_params(m, nomes):
    return {nome: ler_param(m, nome) for nome in nomes}


def mudar_modo(m, nome, numero, manches, timeout=30):
    """Troca o modo e confere pelo HEARTBEAT, reenviando os manches."""
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        aplicar_rc_override(m, manches)
        m.set_mode(numero)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == numero:
            log(f"modo {nome}")
            return
    raise SystemExit(f"o veiculo nao aceitou o modo {nome}")


class Diario:
    """Mensagens do veiculo com o tempo simulado, na tela e no autotune.txt."""

    def __init__(self, caminho):
        self.f = open(caminho, "w", encoding="utf-8")
        self.t_sim = 0.0

    def registrar(self, msg):
        if msg.get_type() == "ATTITUDE":
            self.t_sim = msg.time_boot_ms / 1000.0
        elif msg.get_type() == "STATUSTEXT":
            linha = f"{self.t_sim:8.1f} s  {msg.text}"
            self.f.write(linha + "\n")
            self.f.flush()
            log(f"  {linha}")

    def fechar(self):
        self.f.close()


def esperar_texto(m, diario, manches, sucesso, falha=(), timeout_sim=5000.0,
                  parar_no_desarme=False):
    """Le mensagens ate um STATUSTEXT com ``sucesso``; reenvia os manches.

    Devolve o texto que encerrou a espera. ``falha`` aborta com SystemExit. O
    limite e de tempo SIMULADO (o mesmo do autotest do ArduPilot, 5000 s), com
    uma rede de seguranca de parede bem folgada.
    """
    from pymavlink import mavutil
    t0 = diario.t_sim
    fim_parede = time.monotonic() + 4 * 3600
    ultimo_rc = ultimo_dado = 0.0
    while time.monotonic() < fim_parede:
        agora = time.monotonic()
        if agora - ultimo_rc > 0.1:
            aplicar_rc_override(m, manches)
            ultimo_rc = agora
        msg = m.recv_match(type=["ATTITUDE", "STATUSTEXT", "HEARTBEAT"],
                           blocking=True, timeout=0.2)
        if msg is None:
            if agora - ultimo_dado > 3:
                pedir_streams(m)         # o pedido de telemetria expira; renova
                ultimo_dado = agora
            continue
        ultimo_dado = agora
        diario.registrar(msg)
        tipo = msg.get_type()
        if tipo == "STATUSTEXT":
            if any(s in msg.text for s in sucesso):
                return msg.text
            if any(s in msg.text for s in falha):
                raise SystemExit(f"AutoTune falhou: {msg.text}")
        elif tipo == "HEARTBEAT" and parar_no_desarme and msg.get_srcSystem() == m.target_system:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return "desarmado"
        if diario.t_sim - t0 > timeout_sim:
            raise SystemExit(f"o AutoTune nao terminou em {timeout_sim:.0f} s simulados")
    raise SystemExit("o AutoTune nao terminou dentro do limite de parede")


def para_cc_pid(ganhos):
    """Os mesmos ganhos com o prefixo do backend PID do AC_CustomControl.

    O ATC_ACC_*_MAX continua ATC_: quem modela o alvo e o AC_AttitudeControl
    nativo, tambem com o AC_CustomControl ligado, e o alvo tem de ser o mesmo
    nos dois controladores.
    """
    return {(n if n.startswith("ATC_ACC_") else n.replace("ATC_", "CC2_", 1)): v
            for n, v in ganhos.items()}


def salvar_ganhos(caminho, originais, sintonizados, meta):
    mudaram = {n: v for n, v in sintonizados.items() if abs(v - originais[n]) > 1e-6}
    doc = {
        "autotune": meta,
        "originais": originais,
        "sintonizados": sintonizados,
        "mudaram": sorted(mudaram),
        # blocos prontos para o campo ganhos: dos controladores (copia: o mesmo
        # dict duas vezes sai como ancora/alias &id001 no YAML)
        "nativo": dict(sintonizados),
        "cc_pid": para_cc_pid(sintonizados),
    }
    with open(caminho, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, allow_unicode=True)
    return mudaram


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eixos", type=int, default=7,
                    help="AUTOTUNE_AXES: 1 rolagem, 2 arfagem, 4 guinada, 8 guinada "
                         "com D (padrao 7)")
    ap.add_argument("--aggr", type=float, default=0.075,
                    help="AUTOTUNE_AGGR (padrao do firmware 0.075)")
    ap.add_argument("--min-d", type=float, default=0.0005,
                    help="AUTOTUNE_MIN_D (padrao do firmware 0.0005)")
    ap.add_argument("--altitude", type=float, default=15.0)
    ap.add_argument("--speedup", type=int, default=10)
    ap.add_argument("--instancia", type=int, default=0)
    ap.add_argument("--param", action="append", default=[], metavar="NOME=VALOR",
                    help="parametro extra do veiculo, aplicado antes do reboot (repete); "
                         "ex.: --param INS_GYRO_FILTER=10, para sintonizar o nativo com o "
                         "giroscopio mais lento")
    args = ap.parse_args()
    extras = {}
    for item in args.param:
        nome, _, valor = item.partition("=")
        extras[nome.strip()] = float(valor)

    carimbo = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    pasta = PASTA_AUTOTUNE / carimbo
    pasta.mkdir(parents=True, exist_ok=True)
    log(f"resultado em {_rel(pasta)}")
    antes = {p.name for p in LOGS_SITL.glob("*.BIN")} if LOGS_SITL.exists() else set()

    params_tune = {"AUTOTUNE_AXES": args.eixos, "AUTOTUNE_AGGR": args.aggr,
                   "AUTOTUNE_MIN_D": args.min_d, **extras}
    if extras:
        log(f"parametros extras: {extras}")
    sitl = None
    diario = Diario(pasta / "autotune.txt")
    t_parede = time.monotonic()
    try:
        sitl = subir_sitl(args.speedup, args.instancia, saida=pasta / "sitl.log")
        m = conectar(args.instancia, sitl=sitl)
        aplicar_params(m, {**PARAMS_BASE, **PARAMS_AUTOTUNE, **params_tune},
                       "de base e do AutoTune")
        m = reiniciar(m, args.instancia, sitl=sitl)

        # o vento.lua registra os WND_* alguns segundos depois do boot; o ar fica
        # parado (WND_MODE 0) durante todo o AutoTune
        if not esperar_param(m, "WND_MODE"):
            raise SystemExit("os parametros WND_* nunca apareceram — o vento.lua nao carregou")
        aplicar_params(m, {"WND_MODE": 0}, "(ar parado)")

        originais = ler_params(m, GANHOS_AUTOTUNE)
        esperar_pronto(m)
        armar_e_decolar(m, args.altitude)

        # AUTOTUNE so entra a partir de STABILIZE, ALT_HOLD, LOITER ou POSHOLD;
        # de LOITER ele segura a posicao entre as torcoes (use_poshold)
        mudar_modo(m, "LOITER", 5, MANCHES_CENTRO)
        time.sleep(1)
        mudar_modo(m, "AUTOTUNE", MODO_AUTOTUNE, MANCHES_CENTRO)
        t_tune = diario.t_sim
        esperar_texto(m, diario, MANCHES_CENTRO, sucesso=("AutoTune: Success",),
                      falha=("AutoTune: Failed", "Failed to level",
                             "determination failed"))
        duracao = diario.t_sim - t_tune
        log(f"AutoTune concluido em {duracao:.0f} s simulados")

        # pouso sem sair do modo: o desarme em AUTOTUNE e o que grava os ganhos
        log("descendo em AUTOTUNE para gravar os ganhos no desarme")
        esperar_texto(m, diario, MANCHES_DESCER, sucesso=("AutoTune: Saved",),
                      timeout_sim=300, parar_no_desarme=True)
        time.sleep(2)
        sintonizados = ler_params(m, GANHOS_AUTOTUNE)
        m.close()
    finally:
        diario.fechar()
        derrubar_sitl(sitl)

    time.sleep(2)
    novos = sorted(p for p in LOGS_SITL.glob("*.BIN") if p.name not in antes)
    if novos:
        origem = max(novos, key=lambda p: p.stat().st_mtime)
        shutil.copy(origem, pasta / origem.name)
        extrair_params(pasta / origem.name, pasta / "params.parm")
        log(f"log copiado: {origem.name} ({origem.stat().st_size / 1e6:.1f} MB)")

    meta = {"carimbo": carimbo, **params_tune,
            "duracao_sim_s": round(duracao, 1),
            "duracao_parede_s": round(time.monotonic() - t_parede, 1)}
    mudaram = salvar_ganhos(pasta / "ganhos.yaml", originais, sintonizados, meta)
    if not mudaram:
        raise SystemExit("nenhum ganho mudou — o AutoTune nao gravou (o desarme foi "
                         "fora do modo AUTOTUNE?)")

    print()
    print(f"{'parametro':20s} {'antes':>10s} {'AutoTune':>10s}")
    for n in GANHOS_AUTOTUNE:
        marca = "  *" if n in mudaram else ""
        print(f"{n:20s} {originais[n]:10.5g} {sintonizados[n]:10.5g}{marca}")
    log(f"ganhos em {_rel(pasta / 'ganhos.yaml')}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
