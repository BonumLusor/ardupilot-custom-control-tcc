#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5: verificação fora da amostra do LQG reprojetado.

Para o nativo, o cc_lqg e o cc_lqg_mesmo_ruido reprojetados (YAMLs atuais), compara o
esforço eficaz no trecho calmo do voo de RND 20 (medicoes/reprojeto) com o previsto
pelo modelo com o ruído independente do ângulo (projeto.planta_real), em rolagem e em
guinada. A calibração usou o LQG antigo; este é o teste com controladores novos.
Escreve ../verificar_reprojeto.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import controladores as C  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

SEMENTES = (1, 2, 3)
SEGUNDOS = 60.0
CENARIO = "ruido/rnd20/torque_pulso"


def lqg_do_yaml(mod, eixo, slug):
    g = yaml.safe_load((A.RAIZ / "ensaios" / "controladores" / f"{slug}.yaml").read_text())["ganhos"]
    s = A.SUFIXO[eixo]
    L = np.array([[g[f"CC4_L{i}T_{s}"], g[f"CC4_L{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    L_r = np.array([[g[f"CC4_LR{i}T_{s}"], g[f"CC4_LR{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    w_ref = math.radians(g[f"CC4_WMAN_{s}"])
    return C.LQG(g[f"CC4_KTH_{s}"], g[f"CC4_KW_{s}"], L, g["CC4_A"], g[f"CC4_B_{s}"], mod.ts,
                 g["CC4_FLTW"], g[f"CC4_TAUC_{s}"], fc_alvo=g["CC4_FLTW"],
                 L_r=L_r if w_ref > 0 else None, tau_r=g[f"CC4_TAUR_{s}"], w_ref=w_ref,
                 tau_alfa=g["CC4_TMAN"])


def previsto(mod, eixo, fazer):
    sig = math.sqrt(P.intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, SEGUNDOS, 1.0 / mod.fs_laco)
    us = []
    for s in SEMENTES:
        r = malha.simular(fazer(), mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica, fs_laco=mod.fs_laco,
                          fc_giro=mod.fc_giro, ruido_giro=sig, semente=s,
                          **P.opcoes_malha(P.planta_real(eixo)))
        us.append(float(np.std(np.asarray(r["u"])[::3][int(5 * mod.fs_laco):])))
    return float(np.mean(us))


def medido(eixo, slug):
    pasta = A.voo("reprojeto", CENARIO, slug)
    if pasta is None:
        return None
    s = E.SUF[eixo]
    if slug == "nativo":
        d = leitura.ler_log(next(pasta.glob("*.BIN")), ("WND", "RATE"))
        return E.rms(E.recorta(d["RATE"]["t"], d["RATE"][E.SAIDA_RATE[eixo]], E.trechos(d, pasta)))
    d = leitura.ler_log(next(pasta.glob("*.BIN")), (f"CCL{s}", "WND"))
    return E.rms(E.recorta(d[f"CCL{s}"]["t"], d[f"CCL{s}"]["U"], E.trechos(d, pasta)))


def main():
    mod = P.carregar_modelo()
    linhas = [f"esforço eficaz no trecho calmo, {CENARIO}: previsto pelo modelo com ANGULO_INDEP "
              f"{P.ANGULO_INDEP} contra medido no SITL"]
    for eixo in ("roll", "yaw"):
        ref = None
        for slug, fazer in (("nativo", lambda e=eixo: P.novo_controlador(mod, e, "nativo")),
                            ("cc_lqg", lambda e=eixo: lqg_do_yaml(mod, e, "cc_lqg")),
                            ("cc_lqg_mesmo_ruido", lambda e=eixo: lqg_do_yaml(mod, e, "cc_lqg_mesmo_ruido"))):
            p, m = previsto(mod, eixo, fazer), medido(eixo, slug)
            if slug == "nativo":
                ref = (p, m)
            txt = f"{P.NOMES[eixo]:8s} {slug:19s}: previsto {p:.3e}"
            if m is not None:
                txt += f", medido {m:.3e} (medido/previsto {m / p:.2f})"
                if slug != "nativo" and ref and ref[1]:
                    txt += f"; razão ao nativo: prevista {p / ref[0]:.2f}, medida {m / ref[1]:.2f}"
            linhas.append(txt)
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "verificar_reprojeto.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
