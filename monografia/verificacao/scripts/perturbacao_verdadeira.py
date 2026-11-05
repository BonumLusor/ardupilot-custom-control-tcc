#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): o SITL tem torque de perturbação no pairado?

Com a atitude verdadeira do simulador (CCTQ, a cada laço), reconstrói, sem ruído de
sensor:

  - a velocidade angular verdadeira do corpo, pela diferença central dos quatérnios;
  - a perturbação verdadeira, pelo modelo discreto com segurador do projeto,
        d_k = (w_{k+1} - phi w_k) / beta - b u_k ,
    com o esforço u_k que o controlador mandou nesse laço;
  - o erro de medida do ângulo: o erro de ângulo que o controlador usou (CCLx.Eth,
    ou o do AHRS para o nativo) menos o erro verdadeiro.

O modelo do projeto supõe d = 0 no trecho calmo e um erro de medida igual à integral
do ruído do giroscópio (eta). Compara o valor eficaz de d com o do d^ do LQG
(CCLx.Dh) e a coerência entre os dois. Escreve ../perturbacao_verdadeira.txt e .png.
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy import signal

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

EIXO_IDX = {"roll": 0, "yaw": 2}


def taxas_verdadeiras(ct):
    """w do corpo (rad/s) a cada laço, por diferença central dos quatérnios."""
    q = np.column_stack([ct["Q1"], ct["Q2"], ct["Q3"], ct["Q4"]])
    t = ct["t"]

    def rotvec(qa, qb):
        # rotação relativa qa^-1 * qb, no corpo, como vetor de rotação
        w1, x1, y1, z1 = qa[:, 0], -qa[:, 1], -qa[:, 2], -qa[:, 3]
        w2, x2, y2, z2 = qb[:, 0], qb[:, 1], qb[:, 2], qb[:, 3]
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        s = np.sign(w)
        v = np.column_stack([x, y, z]) * s[:, None]
        ang = 2.0 * np.arctan2(np.linalg.norm(v, axis=1), np.abs(w))
        n = np.linalg.norm(v, axis=1)
        eixo = np.divide(v, n[:, None], out=np.zeros_like(v), where=n[:, None] > 0)
        return eixo * ang[:, None]

    w = np.full((len(t), 3), np.nan)
    rv = rotvec(q[:-2], q[2:])
    w[1:-1] = rv / (t[2:] - t[:-2])[:, None]
    return t, w


def analisar(pasta, eixo, slug, mod):
    s = E.SUF[eixo]
    tipos = ("CCTQ", "WND", "RATE", "ANG") + ((f"CCL{s}",) if slug != "nativo" else ())
    d = leitura.ler_log(next(pasta.glob("*.BIN")), tipos)
    if "CCTQ" not in d:
        return None
    jan = E.trechos(d, pasta)
    t, w = taxas_verdadeiras(d["CCTQ"])
    i = EIXO_IDX[eixo]
    if slug == "nativo":
        u = np.interp(t, d["RATE"]["t"], d["RATE"][E.SAIDA_RATE[eixo]])
        dh = None
    else:
        cl = d[f"CCL{s}"]
        u = np.interp(t, cl["t"], cl["U"])
        dh = np.interp(t, cl["t"], cl["Dh"])
    ts = np.median(np.diff(t))
    a, b = mod.a, mod.b[eixo]
    fi = math.exp(-a * ts)
    beta = (1 - fi) / a
    dv = np.full(len(t), np.nan)
    dv[:-1] = (w[1:, i] - fi * w[:-1, i]) / beta - b * u[:-1]
    segs_d = E.recorta(t, dv, jan)
    segs_d = [x[np.isfinite(x)] for x in segs_d]
    res = {"d_rms": E.rms(segs_d)}
    f, p = E.psd(segs_d)
    res["psd"] = (f, p)
    if dh is not None:
        segs_h = [x[np.isfinite(y)] for x, y in zip(E.recorta(t, dh, jan), E.recorta(t, dv, jan))]
        res["dh_rms"] = E.rms(segs_h)
        cs = [signal.coherence(signal.detrend(x), signal.detrend(y), fs=400.0, nperseg=min(512, len(x)))
              for x, y in zip(segs_d, segs_h) if len(x) == len(y) and len(x) > 512]
        if cs:
            fc = cs[0][0]
            c = np.mean([cc for _, cc in cs if len(cc) == len(fc)], axis=0)
            res["coer"] = {faixa: float(np.mean(c[(fc >= faixa[0]) & (fc <= faixa[1])]))
                           for faixa in ((0.5, 3), (3, 7), (7, 15))}
    return res


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mod = P.carregar_modelo()
    linhas = ["perturbação verdadeira d = (w[k+1] - phi w[k])/beta - b u[k], no trecho calmo (rad/s²)"]
    fig, eixos = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    for k, (rot, cen, eixo) in enumerate(E.CASOS):
        ax = eixos.flat[k]
        for nome, base, slug in (("nativo", "verdade_ahrs", "nativo"), ("LQG (AHRS)", "verdade_ahrs", "cc_lqg"),
                                 ("LQG (sem EKF)", "ablacao_ins_taxa", "cc_lqg")):
            pasta = A.voo(base, cen, slug)
            if pasta is None:
                continue
            r = analisar(pasta, eixo, slug, mod)
            if r is None:
                continue
            txt = f"{rot:16s} {nome:14s}: d verdadeiro eficaz {r['d_rms']:.4f}"
            if "dh_rms" in r:
                txt += f"; d^ eficaz {r['dh_rms']:.4f}"
            if "coer" in r:
                txt += "; coerência d x d^ " + ", ".join(f"{a:g}-{b:g} Hz {c:.2f}" for (a, b), c in r["coer"].items())
            linhas.append(txt)
            f, p = r["psd"]
            ax.loglog(f[1:], p[1:], label=f"d verdadeiro, {nome}")
        ax.set_title(rot, loc="left", fontsize=9)
        ax.grid(True, which="both", lw=0.3)
    eixos[0, 0].legend(fontsize=7)
    for ax in eixos[1]:
        ax.set_xlabel("Frequência (Hz)")
    fig.tight_layout()
    fig.savefig(AQUI.parent / "perturbacao_verdadeira.png", dpi=110)
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "perturbacao_verdadeira.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
