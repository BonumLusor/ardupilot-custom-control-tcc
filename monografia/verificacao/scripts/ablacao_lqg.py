#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3c): ablação do ângulo do AHRS no LQG.

Mesmo firmware e mesmos cenários para três configurações: o nativo, o LQG com o
ângulo do AHRS (CC4_DBGANG = 0) e o LQG com o ângulo integrado dos incrementos do
INS, sem as correções do EKF (CC4_DBGANG = 1). Nos trechos calmos, compara:

  - o esforço eficaz do LQG com o do nativo, nas duas fontes de ângulo;
  - a razão prevista pelo modelo para o ruído branco do giroscópio
    (projeto.analisar, planta completa, ganhos dos YAMLs que voaram);
  - os picos de 5 Hz (fusão do GPS) no esforço e na inovação do ângulo.

Se, com a integral do INS, a razão medida cair para a prevista, o excesso vem das
correções do EKF no ângulo. Escreve ../ablacao_lqg.txt e ../ablacao_lqg.png.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[2]
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(RAIZ / "modelagem"))
import ruido_lqg_espectros as E  # noqa: E402
import controladores as C  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

MED = RAIZ / "ensaios" / "resultados" / "verificacao_2026-11-04" / "medicoes"
SUFIXO = {"roll": "X", "yaw": "Z"}


def voo(base, cenario, slug):
    pastas = sorted(p for p in (MED / base / cenario / slug).glob("2026-*") if any(p.glob("*.BIN")))
    return pastas[-1] if pastas else None


def razao_prevista(mod, eixo):
    """Ruído branco do giroscópio no esforço, LQG (cc_lqg) / nativo, no modelo."""
    g = yaml.safe_load((RAIZ / "ensaios" / "controladores" / "cc_lqg.yaml").read_text())["ganhos"]
    s = SUFIXO[eixo]
    L = np.array([[g[f"CC4_L{i}T_{s}"], g[f"CC4_L{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    L_r = np.array([[g[f"CC4_LR{i}T_{s}"], g[f"CC4_LR{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    w_ref = math.radians(g[f"CC4_WMAN_{s}"])
    ctl = C.LQG(g[f"CC4_KTH_{s}"], g[f"CC4_KW_{s}"], L, g["CC4_A"], g[f"CC4_B_{s}"], mod.ts,
                g["CC4_FLTW"], g[f"CC4_TAUC_{s}"], fc_alvo=g["CC4_FLTW"],
                L_r=L_r if w_ref > 0 else None, tau_r=g[f"CC4_TAUR_{s}"], w_ref=w_ref,
                tau_alfa=g["CC4_TMAN"])
    an_l = P.analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, planta=P.PLANTA_REAL)
    an_n = P.analisar(P.novo_controlador(mod, eixo, "nativo"), mod.a, mod.b[eixo], mod.ts,
                      mod.fc_giro, planta=P.PLANTA_REAL)
    return an_l.ruido_branco / an_n.ruido_branco


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mod = P.carregar_modelo()
    prev = {e: razao_prevista(mod, e) for e in ("roll", "yaw")}
    linhas = [f"razão prevista LQG/nativo para ruído branco do giroscópio: rolagem {prev['roll']:.2f}, "
              f"guinada {prev['yaw']:.2f}"]
    fig, eixos = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    for k, (rot, cen, eixo) in enumerate(E.CASOS):
        ax = eixos.flat[k]
        pn = voo("ablacao_ahrs", cen, "nativo")
        pa = voo("ablacao_ahrs", cen, "cc_lqg")
        pi = voo("ablacao_ins", cen, "cc_lqg")
        pt = voo("ablacao_ins_taxa", cen, "cc_lqg")
        if None in (pn, pa, pi):
            linhas.append(f"{rot}: falta voo")
            continue
        s = E.SUF[eixo]
        dn = leitura.ler_log(next(pn.glob("*.BIN")), ("WND", "ANG", "RATE"))
        un = E.recorta(dn["RATE"]["t"], dn["RATE"][E.SAIDA_RATE[eixo]], E.trechos(dn, pn))
        linhas.append(f"{rot}: u eficaz do nativo {E.rms(un):.5f}")
        f, pun = E.psd(un)
        ax.loglog(f[1:], pun[1:], label="u, nativo")
        for rotulo, pasta in (("AHRS", pa), ("ângulo do INS", pi), ("ângulo e taxa do INS", pt)):
            if pasta is None:
                continue
            d = leitura.ler_log(next(pasta.glob("*.BIN")), (f"CCK{s}", f"CCL{s}", "WND"))
            jan = E.trechos(d, pasta)
            u = E.recorta(d[f"CCL{s}"]["t"], d[f"CCL{s}"]["U"], jan)
            it = E.recorta(d[f"CCK{s}"]["t"], np.degrees(d[f"CCK{s}"]["It"]), jan)
            fu, pu = E.psd(u)
            _, pit = E.psd(it)
            linhas.append(f"  LQG com {rotulo:21s}: u eficaz {E.rms(u):.5f} ({E.rms(u) / E.rms(un):.2f}x o nativo; "
                          f"previsto {prev[eixo]:.2f}x); inovação do ângulo {E.rms(it):.4f} graus; "
                          f"pico de 5 Hz no u {E.pico(fu, pu, 5.0):.1f}, na inovação {E.pico(fu, pit, 5.0):.1f}")
            ax.loglog(fu[1:], pu[1:], label=f"u, LQG ({rotulo})")
        ax.axvline(5.0, color="0.6", lw=0.6, ls=":")
        ax.set_title(rot, loc="left", fontsize=9)
        ax.grid(True, which="both", lw=0.3)
    for ax in eixos[1]:
        ax.set_xlabel("Frequência (Hz)")
    for ax in eixos[:, 0]:
        ax.set_ylabel("Densidade espectral de u")
    eixos[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(AQUI.parent / "ablacao_lqg.png", dpi=110)
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "ablacao_lqg.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
