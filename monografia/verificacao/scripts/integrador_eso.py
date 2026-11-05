#!/usr/bin/env python3
"""Revisão de 3/11, item 3.3: o LQR com ESO contém um integrador.

Substituindo a lei do backend (u = k_th e_th + k_w e_w - d^/b, com e_w = -giro e
referências nulas) no observador (w^, d^), o estado do controlador s = [w^ d^]
evolui com

    A_c = [[fi - l1, 0], [-l2, 1]]

triangular, com autovalor em z = 1. O resíduo da função de transferência de e_th
para u nesse polo, dividido por Ts, é o ganho integral equivalente: o LQI integra
com x_I += Ts e_th e u = k_i x_I, ou seja, k_i Ts / (z - 1).

Usa os ganhos que voaram (ensaios/controladores/cc_eso.yaml e cc_lqi.yaml) e o
período do laço do modelo do projeto. Escreve ../integrador_eso.txt.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import signal

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "modelagem"))
import projeto  # noqa: E402


def params(nome):
    return yaml.safe_load((RAIZ / "ensaios" / "controladores" / f"{nome}.yaml").read_text())["ganhos"]


def main():
    mod = projeto.carregar_modelo()
    ts = mod.ts
    eso, lqi = params("cc_eso"), params("cc_lqi")
    a = eso["CC3_A"]
    linhas = [f"Ts = {ts:.6f} s, a = {a}"]
    for eixo, s in (("rolagem", "X"), ("arfagem", "Y"), ("guinada", "Z")):
        b, kth, kw = eso[f"CC3_B_{s}"], eso[f"CC3_KTH_{s}"], eso[f"CC3_KW_{s}"]
        l1, l2 = eso[f"CC3_ESO_L1_{s}"], eso[f"CC3_ESO_L2_{s}"]
        fi = np.exp(-a * ts)
        beta = (1 - fi) / a
        Ac = np.array([[fi - l1, 0.0], [-l2, 1.0]])
        Bc = np.array([[beta * b * kth, l1 - beta * b * kw], [0.0, l2]])   # entradas [e_th, giro]
        Cc = np.array([[0.0, -1.0 / b]])
        Dc = np.array([[kth, -kw]])
        autov = np.sort(np.linalg.eigvals(Ac).real)[::-1]
        num, _ = signal.ss2tf(Ac, Bc, Cc, Dc, input=0)
        ki_eq = np.polyval(num.ravel(), 1.0) / (1.0 - (fi - l1)) / ts
        ki_lqi = lqi[f"CC3_KI_{s}"]
        linhas.append(f"{eixo:8s} autovalores de A_c {autov[0]:.6f} e {autov[1]:.4f}; "
                      f"k_i equivalente do ESO {ki_eq:.3f}; k_i do LQI {ki_lqi:.3f}; "
                      f"diferença {100 * (ki_eq / ki_lqi - 1):+.1f} %")
    texto = "\n".join(linhas)
    print(texto)
    (Path(__file__).resolve().parents[1] / "integrador_eso.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
