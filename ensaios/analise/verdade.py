"""Atitude verdadeira do SITL (CCTQ) e as métricas nessa base (revisão de 3/11, item 2.2).

A mensagem CCTQ (firmware/ardupilot.patch, AC_CustomControl::update, só no SITL) grava a
cada laço o quatérnio da atitude do modelo físico. A convenção foi conferida contra o
SIM, que o SITL grava a 10 Hz: nos mesmos instantes, a diferença ficou em 0,01°, a
resolução do SIM, nos três eixos; contra a ANG, sem atraso (5/11/2026).

A base verdadeira troca a atitude medida (roll, pitch, yaw_rel) e os erros de atitude
pelas versões da CCTQ, menos um desvio constante por eixo: a média de (verdade − AHRS)
no trecho de referência, o meio segundo antes da perturbação (de 0,5 a 0,05 s antes da
janela do distúrbio); sem distúrbio, o pairado antes do perfil. O desvio tira o viés
entre as bases no instante em que a perturbação começa: o rumo verdadeiro difere do
rumo do AHRS em que o alvo de guinada foi travado, e rolagem e arfagem têm vieses de
centésimos a décimos de grau. Um trecho curto, colado à perturbação, e não o trecho
calmo inteiro: com ruído de giroscópio, a diferença entre as bases vaga devagar
(desvio padrão de 0,06° em rolagem com SIM_GYR_RND = 20, contra 0,002° sem ruído), e a
média de 4,5 s deixava, no instante do torque, um resto de até 0,2° somado ao pico.
Assim, nas duas bases o erro parte do mesmo nível, e o que difere é a dinâmica.

As métricas saem em ``metricas_verdade.json``, ao lado do ``metricas.json`` (AHRS), e a
série em ``serie_verdade.csv``. Este módulo não muda nada da análise na base do AHRS.

    python -m analise.verdade resultados/rolagem resultados/guinada [--binario SHA]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

from . import frequencia, metricas, tabelas, veiculo
from .dryden_ref import Perfil
from .eixos import COLUNAS, EIXOS
from .leitura import (TIPOS_PADRAO, _embrulhar180, ler_log, ler_params, serie_unificada,
                      torque_segurado)

ARQ_METRICAS = "metricas_verdade.json"
ARQ_SERIE = "serie_verdade.csv"
PAIRADO_ANTES = (-1.8, -0.1)        # s de tempo do perfil: o pairado antes do perfil


def angulos(ct):
    """Rolagem, arfagem e guinada (graus; guinada desdobrada) do quatérnio da CCTQ."""
    w, x, y, z = (np.asarray(ct[k], dtype=float) for k in ("Q1", "Q2", "Q3", "Q4"))
    rol = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    arf = np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    gui = np.degrees(np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))
    return rol, arf, gui


REFERENCIA_ANTES = (0.5, 0.05)        # s antes da janela do distúrbio


def trecho_referencia(grade, col, jan):
    """Máscara do trecho em que se mede o desvio entre as bases, ou None."""
    tp = col.get("tp")
    if tp is None or len(grade) < 2:
        return None
    dt = float(np.median(np.diff(grade)))
    if jan is not None:
        sel = (grade >= jan[0] - REFERENCIA_ANTES[0]) & (grade <= jan[0] - REFERENCIA_ANTES[1])
        if sel.sum() * dt >= 0.3:
            return sel
    sel = np.isfinite(tp) & (tp >= PAIRADO_ANTES[0]) & (tp <= PAIRADO_ANTES[1])
    return sel if sel.sum() * dt >= 0.5 else None


def base_verdadeira(dados, grade, col, jan):
    """(col com a atitude e os erros verdadeiros, {eixo: desvio verdade − AHRS em graus}).

    Devolve (None, {}) se o log não tem CCTQ ou não há trecho de referência.
    """
    ct = dados.get("CCTQ")
    if ct is None or len(ct.get("t", [])) < 10:
        return None, {}
    sel = trecho_referencia(grade, col, jan)
    if sel is None:
        return None, {}
    rol, arf, gui = angulos(ct)

    def na_grade(v):
        return np.interp(grade, ct["t"], v, left=np.nan, right=np.nan)

    novo = dict(col)
    desvios = {}
    for eixo, v in (("roll", rol), ("pitch", arf)):
        c = COLUNAS[eixo]
        if c["med"] not in col:
            continue
        v = na_grade(v)
        off = float(np.nanmean(v[sel] - col[c["med"]][sel]))
        novo[c["med"]] = v - off
        if c["ref"] in col:
            novo[c["erro"]] = col[c["ref"]] - novo[c["med"]]
        desvios[eixo] = off
    if "erro_yaw" in col and "yaw_des" in col:
        # erro verdadeiro − erro do AHRS = rumo do AHRS − rumo verdadeiro (embrulhado)
        erro_v = _embrulhar180(col["yaw_des"] - na_grade(gui))
        dif = _embrulhar180(erro_v - col["erro_yaw"])
        off = float(np.nanmean(dif[sel]))
        novo["erro_yaw"] = col["erro_yaw"] + (dif - off)
        if "yaw_des_rel" in col:
            novo["yaw_rel"] = col["yaw_des_rel"] - novo["erro_yaw"]
        if "yaw" in col:
            novo["yaw"] = np.mod(col["yaw"] - (dif - off), 360.0)
        desvios["yaw"] = -off
    return novo, desvios


def _yaml(caminho):
    return yaml.safe_load(caminho.read_text(encoding="utf-8")) if caminho.exists() else {}


def calcular_pasta(pasta):
    """Métricas na base verdadeira de um voo; grava os arquivos e devolve o dict (ou None)."""
    from run_ensaio import degraus_do_voo   # noqa: E402  (mesma regra do analisar)

    pasta = Path(pasta)
    bins = sorted(pasta.glob("*.BIN"))
    if not bins:
        return None
    cen, ctl = _yaml(pasta / "cenario.yaml"), _yaml(pasta / "controlador.yaml")
    dados = ler_log(bins[0], TIPOS_PADRAO + ("CCTQ",))
    if "CCTQ" not in dados:
        return None
    grade, col = serie_unificada(dados, msg_pid=ctl.get("msg_pid", "PIDR"),
                                 duracao_s=(cen.get("voo") or {}).get("duracao_s"))
    if len(grade) == 0:
        return None
    perfil = Perfil.de_dict(cen.get("vento", {})) if cen.get("vento") else None
    eixos_dist = [e for e in metricas.eixos_disturbio(perfil, col) if COLUNAS[e]["erro"] in col]
    jan = metricas.janela_disturbio(grade, col, perfil) if eixos_dist else None
    novo, desvios = base_verdadeira(dados, grade, col, jan)
    if novo is None:
        return None

    parm = ler_params(pasta / "params.parm") if (pasta / "params.parm").exists() else {}
    desc = veiculo.descrever(parm, frame_type=int(parm.get("FRAME_TYPE", 1))) if parm else None
    degraus = degraus_do_voo(cen["voo"]) if cen.get("voo") else None
    m = metricas.calcular(grade, novo, perfil, degraus=degraus,
                          inercia_kgm2=veiculo.inercias(desc) if desc else None)
    m["base"] = "verdadeira (CCTQ)"
    m["desvio_entre_bases_deg"] = desvios
    if perfil is not None and (perfil.modo & 64):
        segurado = torque_segurado(dados, grade)
        for eixo in metricas.eixos_com_torque(perfil, novo):
            c = COLUNAS[eixo]
            est = frequencia.estimar(grade, novo, perfil, entrada=c["torque"], saida=c["med"],
                                     rotulo=f"{eixo} / {c['torque']} (verdade)",
                                     u=segurado.get(c["torque"]))
            if est is not None:
                m[tabelas.chave_resposta_freq(eixo)] = frequencia.metricas(est)
    tabelas.salvar_metricas(m, pasta / ARQ_METRICAS)

    nomes = ["tp"] + [COLUNAS[e][k] for e in EIXOS for k in ("ref", "med", "erro")]
    serie = {k: novo[k] for k in nomes if k in novo}
    tabelas.salvar_series(grade, serie, pasta / ARQ_SERIE)
    return m


def pastas_de_voo(raizes, binario=None):
    """Pastas de voo (com .BIN) sob as raízes, só as do binário dado, se houver."""
    for raiz in raizes:
        for bin_ in sorted(Path(raiz).rglob("*.BIN")):
            pasta = bin_.parent
            if binario:
                v = pasta / "versao.json"
                if not v.exists() or json.loads(v.read_text()).get("binario_sha256", "") != binario:
                    continue
            yield pasta


def _uma(pasta):
    try:
        return pasta, calcular_pasta(pasta) is not None, None
    except Exception as exc:              # um voo ruim não para a reanálise
        return pasta, False, str(exc)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("raizes", nargs="+", type=Path)
    ap.add_argument("--binario", help="só os voos deste hash do binário do SITL")
    ap.add_argument("--processos", type=int, default=1, help="voos analisados em paralelo")
    ap.add_argument("--pular-feitos", action="store_true",
                    help="pula os voos que já têm metricas_verdade.json")
    a = ap.parse_args(argv)
    pastas = [p for p in pastas_de_voo(a.raizes, a.binario)
              if not (a.pular_feitos and (p / ARQ_METRICAS).exists())]
    n = falhas = 0
    if a.processos > 1:
        from multiprocessing import Pool
        with Pool(a.processos) as pool:
            res = list(pool.imap_unordered(_uma, pastas))
    else:
        res = map(_uma, pastas)
    for pasta, ok, erro in res:
        n += ok
        if erro:
            falhas += 1
            print(f"FALHOU {pasta}: {erro}", file=sys.stderr)
    print(f"{n} voo(s) na base verdadeira; {falhas} falha(s)")


if __name__ == "__main__":
    main()
