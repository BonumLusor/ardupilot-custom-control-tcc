#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): a causa do ruído do LQG no SITL.

No SITL, a derivada do erro do ângulo medido e o erro da taxa medida não são
perfeitamente coerentes (0,93 a 0,98 de 1 a 10 Hz; erro_medidas.py e a coerência
calculada na mesma análise), enquanto o modelo os supõe idênticos: o ângulo integra
exatamente o ruído do giroscópio. O filtro de Kalman do LQG, projetado com ruído de
medida quase nulo, confia nas duas medidas e joga a inconsistência em d^.

Aqui o ângulo medido do modelo integra, além do ruído do giroscópio, um ruído
independente com uma fração da sua potência (malha.simular, angulo_indep), e se
mede o esforço eficaz do LQG (cc_lqg) e do nativo, contra os valores do SITL nos
voos de RND 20. Escreve ../angulo_indep.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ruido_absoluto as R  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402

FRACOES = (0.0, 0.02, 0.05, 0.10)
SITL = {("roll", "LQG"): None, ("yaw", "LQG"): None}


def esforco(mod, eixo, fazer, frac):
    sig = math.sqrt(P.intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, R.SEGUNDOS, 1.0 / mod.fs_laco)
    us = []
    for s in R.SEMENTES:
        r = malha.simular(fazer(), mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica, fs_laco=mod.fs_laco,
                          fc_giro=mod.fc_giro, ruido_giro=sig, semente=s, angulo_indep=frac,
                          **P.opcoes_malha(P.PLANTA_REAL))
        us.append(float(np.std(np.asarray(r["u"])[::3][int(5 * mod.fs_laco):])))
    return float(np.mean(us))


def main():
    mod = P.carregar_modelo()
    linhas = [f"u eficaz no modelo com RND 20, por fração independente {FRACOES}; SITL = medido nos voos de RND 20"]
    for eixo in ("roll", "yaw"):
        for nome, slug, fazer in (("LQG", "cc_lqg", lambda e=eixo: R.lqg_voado(mod, e)),
                                  ("nativo", "nativo", lambda e=eixo: P.novo_controlador(mod, e, "nativo"))):
            vals = [esforco(mod, eixo, fazer, f) for f in FRACOES]
            med = R.medido(eixo, slug)
            linhas.append(f"{P.NOMES[eixo]:8s} {nome:6s}: " + "; ".join(f"{f:.2f} -> {v:.2e}" for f, v in zip(FRACOES, vals))
                          + f" | SITL {med:.2e}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "angulo_indep.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
