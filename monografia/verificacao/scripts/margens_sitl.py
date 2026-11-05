#!/usr/bin/env python3
"""Revisão de 3/11 (Fase 6): margem de fase e cruzamento medidos no SITL, nas varreduras do lote.

Na planta w' = -a w + v, com v = b u + d, a varredura de torque d dá, pela resposta em
frequência do esforço ao torque, H = U/D, a sensibilidade na entrada da planta,
S = V/D = 1 + b H, e a malha aberta nesse ponto, L = 1/S - 1. O cruzamento de ganho
(|L| = 1) e a margem de fase saem de L, só nas frequências de coerência boa. O torque
é o segurado entre as escritas do injetor (validacao.torque_segurado): a coluna do
series.csv é interpolada e atrasaria a fase.

Substitui a conta do relatório de 2/11 (voos antigos). Escreve ../margens_sitl.txt.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[2]
sys.path.insert(0, str(RAIZ / "modelagem"))
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402

RES = RAIZ / "ensaios" / "resultados"
LOTE = (RAIZ / "ensaios" / "lotes" / "binario_lote.sha256").read_text().split()[0]
EIXOS = (("rolagem", "roll", "tqx", "u_roll"), ("arfagem", "pitch", "tqy", "u_pitch"),
         ("guinada", "yaw", "tqz", "u_yaw"))
SLUGS = ("nativo", "cc_pid", "cc_lqi", "cc_eso", "cc_lqi_mesmo_ruido", "cc_lqg", "cc_lqg_mesmo_ruido")
COER_MIN = 0.9


def margem(f, L):
    mod = np.abs(L)
    desce = np.nonzero((mod[:-1] >= 1.0) & (mod[1:] < 1.0))[0]
    if not len(desce):
        return None, None
    i = desce[-1]
    x = (0.0 - math.log(mod[i])) / (math.log(mod[i + 1]) - math.log(mod[i]))
    fc = math.exp(math.log(f[i]) + x * (math.log(f[i + 1]) - math.log(f[i])))
    fase = np.unwrap(np.angle(L))
    ph = fase[i] + x * (fase[i + 1] - fase[i])
    mf = (math.degrees(ph) + 180.0 + 180.0) % 360.0 - 180.0
    return 2 * math.pi * fc, mf


def main():
    mod = P.carregar_modelo()
    linhas = [f"Margem de fase e cruzamento medidos nas varreduras de torque do binário {LOTE[:12]} "
              f"(L = 1/S - 1, S = 1 + b U/D; coerência >= {COER_MIN}); modelo: planta do portão."]
    for pasta_eixo, eixo, col_tq, col_u in EIXOS:
        for slug in SLUGS:
            voos = [p for p in sorted((RES / pasta_eixo / "torque_chirp" / slug).glob("2026-*"))
                    if (p / "versao.json").exists()
                    and json.loads((p / "versao.json").read_text())["binario_sha256"] == LOTE]
            if not voos:
                continue
            pasta = voos[0]
            cen = __import__("yaml").safe_load((pasta / "cenario.yaml").read_text())["vento"]
            f0, f1, t0, dur = (float(cen[k]) for k in ("WND_TQF0", "WND_TQF1", "WND_TQT0", "WND_TQDUR"))
            col = V.ler_series(pasta / "series.csv")
            _, wnd = V.ler_bat_wnd(pasta)
            tp = col["tp"]
            dentro = np.isfinite(tp) & (tp >= t0) & (tp <= t0 + dur)
            t = col["t"][dentro]
            d = V.torque_segurado(t, wnd, col_tq)
            u = col[col_u][dentro]
            f, H, coer = V.resposta_freq(t, d, u, f0, f1)
            ok = coer >= COER_MIN
            S = 1.0 + mod.b[eixo] * H
            L = 1.0 / S - 1.0
            wc, mf = margem(f[ok], L[ok])
            # a mesma conta no modelo: os ganhos do voo e a planta do portao
            p = V.ms.ler_parm(pasta / "params.parm")
            nome, prefixo = V.controlador_do_ensaio(pasta, p)
            ts = 1.0 / float(p.get("SCHED_LOOP_RATE", 400))
            ctl, _ = V.montar_controlador(nome, prefixo, p, eixo, mod.a, mod.b[eixo], ts)
            an = P.analisar(ctl, mod.a, mod.b[eixo], ts, float(p.get("INS_GYRO_FILTER", 20)),
                            planta=P.planta_real(eixo))
            mm = an.margens
            txt = (f"{pasta_eixo:8s} {slug:20s} " +
                   (f"SITL wc {wc:5.1f} rad/s MF {mf:5.1f} graus" if wc else "SITL sem cruzamento na faixa") +
                   f" | modelo wc {mm['wc']:5.1f} MF {mm['mf_graus']:5.1f}")
            linhas.append(txt + f"  ({ok.sum()} de {len(ok)} frequências com coerência boa)")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "margens_sitl.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
