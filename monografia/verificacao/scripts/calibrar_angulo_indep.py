#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (reprojeto do LQG): calibra a fração de ruído independente no ângulo.

Critério, fixado antes: por eixo, a fração f (de malha.simular(angulo_indep=f)) com que
o modelo reproduz o esforço eficaz do cc_lqg atual medido no SITL com RND 20 (voos da
ablação, mesmo firmware). Busca por bisseção em log(f), com 3 sementes de 60 s. O teste
fora da amostra é o voo do LQG reprojetado. Escreve ../calibrar_angulo_indep.txt.
"""
import math
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import angulo_indep as AI  # noqa: E402
import ruido_absoluto as R  # noqa: E402
import projeto as P  # noqa: E402


def main():
    mod = P.carregar_modelo()
    linhas = ["fração de ruído independente no ângulo que reproduz o esforço do cc_lqg no SITL (RND 20)"]
    for eixo in ("roll", "yaw"):
        alvo = R.medido(eixo, "cc_lqg")
        fazer = lambda e=eixo: R.lqg_voado(mod, e)
        lo, hi = math.log(1e-3), math.log(1.0)
        for _ in range(9):
            meio = 0.5 * (lo + hi)
            u = AI.esforco(mod, eixo, fazer, math.exp(meio))
            if u < alvo:
                lo = meio
            else:
                hi = meio
        f = math.exp(0.5 * (lo + hi))
        u = AI.esforco(mod, eixo, fazer, f)
        linhas.append(f"{P.NOMES[eixo]:8s}: f = {f:.4f}; u do modelo {u:.3e}, SITL {alvo:.3e}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "calibrar_angulo_indep.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
