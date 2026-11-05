#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): o modelo erra o ruído do nativo ou o do LQG?

Simula, como projeto.ruido_no_tempo, o esforço sem perturbação com o ruído do
giroscópio de RND 20 (e o ângulo do AHRS com o mesmo ruído e um laço de atraso),
para o nativo e para o cc_lqg com os ganhos que voaram, e compara com o esforço
eficaz medido no trecho calmo dos voos de RND 20 da ablação (mesmo firmware).
Escreve ../ruido_absoluto.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import controladores as C  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402
import yaml  # noqa: E402
from analise import leitura  # noqa: E402

SEMENTES = (1, 2, 3)
SEGUNDOS = 60.0


def lqg_voado(mod, eixo):
    g = yaml.safe_load((A.RAIZ / "ensaios" / "controladores" / "cc_lqg.yaml").read_text())["ganhos"]
    s = A.SUFIXO[eixo]
    L = np.array([[g[f"CC4_L{i}T_{s}"], g[f"CC4_L{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    L_r = np.array([[g[f"CC4_LR{i}T_{s}"], g[f"CC4_LR{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    w_ref = math.radians(g[f"CC4_WMAN_{s}"])
    return C.LQG(g[f"CC4_KTH_{s}"], g[f"CC4_KW_{s}"], L, g["CC4_A"], g[f"CC4_B_{s}"], mod.ts,
                 g["CC4_FLTW"], g[f"CC4_TAUC_{s}"], fc_alvo=g["CC4_FLTW"],
                 L_r=L_r if w_ref > 0 else None, tau_r=g[f"CC4_TAUR_{s}"], w_ref=w_ref,
                 tau_alfa=g["CC4_TMAN"])


def simulado(mod, eixo, fazer):
    sig = math.sqrt(P.intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, SEGUNDOS, 1.0 / mod.fs_laco)
    passo = max(int(round(mod.fs_fisica / mod.fs_laco)), 1)
    us = []
    for s in SEMENTES:
        r = malha.simular(fazer(), mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica,
                          fs_laco=mod.fs_laco, fc_giro=mod.fc_giro, ruido_giro=sig, semente=s,
                          **P.opcoes_malha(P.PLANTA_REAL))
        us.append(float(np.std(r["u"][::passo][int(5 * mod.fs_laco):])))
    return float(np.mean(us))


def medido(eixo, slug):
    pasta = A.voo("ablacao_ahrs", "ruido/rnd20/torque_pulso", slug)
    if slug == "nativo":
        d = leitura.ler_log(next(pasta.glob("*.BIN")), ("WND", "RATE"))
        return E.rms(E.recorta(d["RATE"]["t"], d["RATE"][E.SAIDA_RATE[eixo]], E.trechos(d, pasta)))
    s = E.SUF[eixo]
    d = leitura.ler_log(next(pasta.glob("*.BIN")), (f"CCL{s}", "WND"))
    return E.rms(E.recorta(d[f"CCL{s}"]["t"], d[f"CCL{s}"]["U"], E.trechos(d, pasta)))


def main():
    mod = P.carregar_modelo()
    linhas = [f"RND 20, acelerador {P.ACEL_PAIRADO}; simulação de {SEGUNDOS:g} s, sementes {SEMENTES}"]
    for eixo in ("roll", "yaw"):
        for nome, slug, fazer in (("nativo", "nativo", lambda e=eixo: P.novo_controlador(mod, e, "nativo")),
                                  ("LQG", "cc_lqg", lambda e=eixo: lqg_voado(mod, e))):
            s, m = simulado(mod, eixo, fazer), medido(eixo, slug)
            linhas.append(f"{P.NOMES[eixo]:8s} {nome:6s}: u eficaz simulado {s:.5f}, medido {m:.5f} "
                          f"(medido/simulado {m / s:.2f})")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "ruido_absoluto.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
