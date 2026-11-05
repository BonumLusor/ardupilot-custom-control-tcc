#!/usr/bin/env python3
"""Revisão de 3/11, Fases 4b e 4c: teste do pulso de duração exata e do d^ congelado.

4b: nos voos do nativo (medicoes/teste_4b4c), o impulso entregue é a integral da
    perturbação verdadeira reconstruída da CCTQ (perturbacao_verdadeira) em volta do
    pulso, contra o pedido (amplitude x duração). Também mostra a conferência do
    perfil contra a referência em Python (metricas.json, "reprodutibilidade").
4c: no pulso de saturação de guinada (12 rad/s2 por 0,5 s), o d^ do ESO e do LQG
    (CCLY.Dh) no voo novo e no último voo antigo, sem o congelamento: o máximo de d^
    contra o torque injetado e o maior salto de d^ de um laço para o outro.
Escreve ../teste_4b4c.txt.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import perturbacao_verdadeira as PV  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

RES = A.RAIZ / "ensaios" / "resultados"
COL_TQ = {"roll": "WND_TQX", "yaw": "WND_TQZ"}


def impulso(pasta, eixo, mod):
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text())["vento"]
    amp, dur, t0 = float(cen[COL_TQ[eixo]]), float(cen["WND_TQDUR"]), float(cen["WND_TQT0"])
    d = leitura.ler_log(next(pasta.glob("*.BIN")), ("CCTQ", "WND", "RATE"))
    origem = leitura.origem_perfil(d)
    t, w = PV.taxas_verdadeiras(d["CCTQ"])
    i = PV.EIXO_IDX[eixo]
    u = np.interp(t, d["RATE"]["t"], d["RATE"][E.SAIDA_RATE[eixo]])
    ts = float(np.median(np.diff(t)))
    a, b = mod.a, mod.b[eixo]
    fi = math.exp(-a * ts)
    beta = (1 - fi) / a
    dv = np.full(len(t), np.nan)
    dv[:-1] = (w[1:, i] - fi * w[:-1, i]) / beta - b * u[:-1]
    tp = t - origem
    jan = (tp >= t0 - 0.1) & (tp <= t0 + dur + 0.1)
    base = np.nanmedian(dv[(tp >= t0 - 1.0) & (tp < t0 - 0.2)])
    imp = float(np.nansum((dv[jan] - base)) * ts)
    rep = json.loads((pasta / "metricas.json").read_text()).get("reprodutibilidade", {})
    return amp * dur, imp, rep


def dhat(pasta):
    d = leitura.ler_log(next(pasta.glob("*.BIN")), ("CCLY",))
    dh = d["CCLY"]["Dh"]
    return float(np.max(dh)), float(np.max(np.abs(np.diff(dh))))


def voo_antigo(slug):
    base = RES / "saturacao" / "guinada" / "torque_pulso" / slug
    pastas = sorted(p for p in base.glob("2026-1*") if any(p.glob("*.BIN")))
    return pastas[-1] if pastas else None


def main():
    mod = P.carregar_modelo()
    linhas = ["4b: impulso pedido x entregue (integral da perturbação verdadeira), voos do nativo"]
    for cen, eixo in (("rolagem/torque_impulso", "roll"), ("guinada/torque_impulso", "yaw"),
                      ("guinada/torque_pulso", "yaw")):
        pasta = A.voo("teste_4b4c", cen, "nativo")
        if pasta is None:
            linhas.append(f"  {cen}: sem voo")
            continue
        ped, ent, rep = impulso(pasta, eixo, mod)
        linhas.append(f"  {cen:24s}: pedido {ped:.3f} rad/s, entregue {ent:.3f} rad/s "
                      f"({100 * (ent / ped - 1):+.1f} %); conferência do perfil: {rep}")
    linhas.append("4c: d^ de guinada no pulso de saturação (12 rad/s² por 0,5 s): máximo e maior salto por laço")
    for slug in ("cc_eso", "cc_lqg"):
        novo = A.voo("teste_4b4c", "saturacao/guinada/torque_pulso", slug)
        velho = voo_antigo(slug)
        for nome, pasta in (("sem congelar", velho), ("congelado", novo)):
            if pasta is None:
                continue
            mx, salto = dhat(pasta)
            linhas.append(f"  {slug:7s} {nome:13s}: d^ máximo {mx:.2f} rad/s²; maior salto {salto:.3f} rad/s² "
                          f"({pasta.relative_to(RES)})")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "teste_4b4c.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
