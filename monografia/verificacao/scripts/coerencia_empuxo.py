#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): o d^ do LQG acompanha o empuxo coletivo?

Hipótese: no SITL, o empuxo coletivo flutua com o controle de altura (que reage às
atualizações da altura estimada), e o torque produzido muda com ele; o LQG, de
estimação de perturbação rápida, rejeita esse torque, e o esforço sobe — sem que
seja ruído do giroscópio. Mede a coerência entre a perturbação estimada (CCLx.Dh)
e a saída de empuxo (RATE.AOut), e a do esforço do nativo (RATE.xOut) com AOut, nos
trechos calmos dos voos sem nada do EKF (CC4_DBGANG = 2) e dos nativos.
Escreve ../coerencia_empuxo.txt.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import signal

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
from analise import leitura  # noqa: E402

FS = 400.0


def serie(t, y, jan, grade_t):
    return [np.interp(g, t, y) for g in grade_t]


def coerencia(x_seg, y_seg):
    cs = []
    for x, y in zip(x_seg, y_seg):
        f, c = signal.coherence(signal.detrend(x), signal.detrend(y), fs=FS, nperseg=min(512, len(x)))
        cs.append((f, c, len(x)))
    f = cs[0][0]
    c = np.average([c for _, c, _ in cs if len(c) == len(f)], axis=0,
                   weights=[n for ff, _, n in cs if len(ff) == len(f)])
    return f, c


def faixa(f, c, a, b):
    sel = (f >= a) & (f <= b)
    return float(np.mean(c[sel])), float(np.max(c[sel]))


def main():
    linhas = ["coerência com a saída de empuxo (RATE.AOut) nos trechos calmos: média / máxima por faixa"]
    for rot, cen, eixo in E.CASOS:
        s = E.SUF[eixo]
        for nome, base, slug in (("LQG sem EKF", "ablacao_ins_taxa", "cc_lqg"), ("nativo", "ablacao_ahrs", "nativo")):
            pasta = A.voo(base, cen, slug)
            if pasta is None:
                continue
            tipos = ("RATE", "WND") + ((f"CCL{s}",) if slug != "nativo" else ())
            d = leitura.ler_log(next(pasta.glob("*.BIN")), tipos)
            jan = E.trechos(d, pasta)
            rate = d["RATE"]
            grades = [rate["t"][(rate["t"] >= a) & (rate["t"] <= b)] for a, b in jan]
            grades = [g for g in grades if len(g) > 512]
            aout = [np.interp(g, rate["t"], rate["AOut"]) for g in grades]
            if slug == "nativo":
                x = [np.interp(g, rate["t"], rate[E.SAIDA_RATE[eixo]]) for g in grades]
                sinal = "u"
            else:
                cl = d[f"CCL{s}"]
                x = [np.interp(g, cl["t"], cl["Dh"]) for g in grades]
                sinal = "d^"
            f, c = coerencia(x, aout)
            partes = [f"{a:g}-{b:g} Hz {faixa(f, c, a, b)[0]:.2f}/{faixa(f, c, a, b)[1]:.2f}"
                      for a, b in ((0.5, 3), (3, 7), (7, 15), (15, 50))]
            linhas.append(f"{rot:16s} {nome:12s} {sinal:3s}: " + "; ".join(partes))
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "coerencia_empuxo.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
