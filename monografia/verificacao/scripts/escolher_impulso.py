#!/usr/bin/env python3
"""Revisão de 3/11, item 2.6 (Fase 4b): a amplitude do impulso curto de torque.

Pulso retangular de 20 ms pelo SIM_TWIST_TIME. Critérios, fixados antes:
  - rolagem e arfagem: desvio do nativo abaixo de 1 grau (o limite linear do
    controlador de raiz quadrada do nativo), com margem: alvo de 0,8 grau;
  - guinada: o esforço máximo de todos os controladores dentro da folga da
    alocação (~0,5 no pairado), com margem: alvo de |u| <= 0,35;
  - em todos, o pico bem acima da faixa calma com RND 20 (3 sigma).
Simula no modelo (planta_real) o nativo, o LQI, o ESO, os dois LQG (dos YAMLs)
e, em guinada, o LQI de mesmo ruído e o nativo_guinada.
Escreve ../escolher_impulso.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import verificar_reprojeto as VR  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402
import controladores as C  # noqa: E402
import yaml  # noqa: E402

DURACAO = 0.020                    # s
T0, T_FIM = 1.0, 6.0
ALVO_DESVIO = {"roll": 0.8}        # graus, nativo
ALVO_U = {"yaw": 0.35}             # |u| máximo, todos


def nativo_guinada(mod):
    g = yaml.safe_load((VR.A.RAIZ / "ensaios" / "controladores" / "nativo_guinada.yaml").read_text())["ganhos"]
    p = dict(mod.p)
    p.update({k: float(v) for k, v in g.items()})
    return C.CascataPID(V.ganhos(p, "ATC", "yaw"), "yaw", mod.ts, raiz=True)


def controladores(mod, proj, eixo):
    cs = [("nativo", lambda: P.novo_controlador(mod, eixo, "nativo")),
          ("LQI", lambda: P.novo_controlador(mod, eixo, "lqi", proj[eixo]["lqi"]["ganhos"])),
          ("ESO", lambda: P.novo_controlador(mod, eixo, "eso", proj[eixo]["eso"]["ganhos"])),
          ("LQG", lambda: VR.lqg_do_yaml(mod, eixo, "cc_lqg")),
          ("LQG (mesmo ruído)", lambda: VR.lqg_do_yaml(mod, eixo, "cc_lqg_mesmo_ruido"))]
    if eixo == "yaw":
        cs += [("LQI (mesmo ruído)", lambda: P.novo_controlador(mod, eixo, "lqi", proj[eixo]["lqi_ruido"]["ganhos"])),
               ("nativo_guinada", lambda: nativo_guinada(mod))]
    return cs


def resposta(mod, eixo, fazer, impulso):
    amp = impulso / DURACAO
    t = np.arange(0.0, T_FIM, 1.0 / mod.fs_laco)
    escada = (np.array([0.0, T0, T0 + DURACAO]), np.array([0.0, amp, 0.0]))
    r = malha.simular(fazer(), mod.a, mod.b[eixo], t, escada=escada, fs_fisica=mod.fs_fisica,
                      fs_laco=mod.fs_laco, fc_giro=mod.fc_giro, **P.opcoes_malha(P.planta_real(eixo)))
    th = np.degrees(np.asarray(r["theta"]))
    return float(np.max(np.abs(th))), float(np.max(np.abs(np.asarray(r["u"]))))


def main():
    mod = P.carregar_modelo()
    proj = P.projetar(mod)
    linhas = [f"pulso retangular de {DURACAO * 1e3:g} ms; desvio máximo (graus) e |u| máximo no modelo"]
    escolha = {}
    for eixo in ("roll", "yaw"):
        cs = controladores(mod, proj, eixo)
        # o desvio e |u| escalam com o impulso na faixa linear: mede com 1 rad/s e escala
        base = {nome: resposta(mod, eixo, fazer, 1.0) for nome, fazer in cs}
        if eixo in ALVO_DESVIO:
            imp = ALVO_DESVIO[eixo] / base["nativo"][0]
        else:
            imp = ALVO_U[eixo] / max(u for _, u in base.values())
        imp = math.floor(imp * 100.0) / 100.0       # arredonda para baixo, em 0,01 rad/s
        escolha[eixo] = imp
        linhas.append(f"{P.NOMES[eixo]}: impulso {imp:.2f} rad/s (amplitude {imp / DURACAO:.1f} rad/s² "
                      f"por {DURACAO * 1e3:g} ms)")
        for nome, fazer in cs:
            dv, u = resposta(mod, eixo, fazer, imp)
            linhas.append(f"  {nome:18s}: desvio {dv:.3f} graus; |u| máx {u:.3f}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "escolher_impulso.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
