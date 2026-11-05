#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): o ruído das medidas do LQG no SITL é o do modelo?

Com a atitude verdadeira (CCTQ), no trecho calmo dos voos do cc_lqg com RND 20:

  erro do ângulo: o ângulo que o backend usou (alvo - CCLx.Eth, com o alvo de ANG)
                  menos o verdadeiro do laço anterior (o backend lê o AHRS um laço
                  atrasado);
  erro da taxa:   a taxa que o backend usou (-CCLx.Ew; o alvo de taxa é nulo no
                  pairado) menos a taxa verdadeira passada pelo mesmo passa-baixas
                  de 20 Hz.

Compara com os mesmos erros numa simulação de 60 s do modelo (malha.simular, ruído
de RND 20, ângulo do AHRS da planta completa). Escreve ../erro_medidas.txt e .png.
"""
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import perturbacao_verdadeira as PV  # noqa: E402
import ruido_absoluto as R  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402
from blocos import PassaBaixas2p  # noqa: E402

ALVO = {"roll": "DesRoll", "yaw": "DesYaw"}


def euler(ct):
    w, x, y, z = ct["Q1"], ct["Q2"], ct["Q3"], ct["Q4"]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return {"roll": roll, "yaw": np.unwrap(yaw)}


def lpf(x, fs, fc):
    f = PassaBaixas2p(fs, fc)
    return np.array([f.aplicar(v) for v in x])


def sitl(pasta, eixo, mod):
    s = E.SUF[eixo]
    d = leitura.ler_log(next(pasta.glob("*.BIN")), ("CCTQ", "WND", "ANG", f"CCL{s}"))
    jan = E.trechos(d, pasta)
    ct, cl, ang = d["CCTQ"], d[f"CCL{s}"], d["ANG"]
    t = ct["t"]
    th_v = euler(ct)[eixo]
    _, w = PV.taxas_verdadeiras(ct)
    w_v = np.nan_to_num(w[:, PV.EIXO_IDX[eixo]])
    alvo = np.unwrap(np.radians(np.interp(t, ang["t"], ang[ALVO[eixo]])))
    eth = np.interp(t, cl["t"], cl["Eth"])
    ew = np.interp(t, cl["t"], cl["Ew"])
    th_med = alvo - eth
    e_ang = np.full(len(t), np.nan)
    e_ang[1:] = th_med[1:] - th_v[:-1]
    e_tax = -ew - lpf(w_v, 1.0 / np.median(np.diff(t)), mod.fc_giro)
    return (E.recorta(t, e_ang, jan), E.recorta(t, e_tax, jan))


def modelo(mod, eixo):
    sig = math.sqrt(P.intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, R.SEGUNDOS, 1.0 / mod.fs_laco)
    passo = max(int(round(mod.fs_fisica / mod.fs_laco)), 1)
    r = malha.simular(R.lqg_voado(mod, eixo), mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica,
                      fs_laco=mod.fs_laco, fc_giro=mod.fc_giro, ruido_giro=sig, semente=1,
                      **P.opcoes_malha(P.PLANTA_REAL))
    n = len(r["giro"])
    th = np.asarray(r["theta"])[::passo][:n]
    w = np.asarray(r["w"])[::passo][:n]
    # angulo_med é a medida do próprio laço (o atraso fica na entrada do controlador)
    e_ang = np.asarray(r["angulo_med"]) - th
    e_tax = np.asarray(r["giro"]) - lpf(w, mod.fs_laco, mod.fc_giro)
    n0 = int(5 * mod.fs_laco)
    return [e_ang[n0:]], [e_tax[n0:]]


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mod = P.carregar_modelo()
    linhas = ["erros eficazes das medidas no trecho calmo, RND 20 (ângulo em graus, taxa em graus/s)"]
    fig, eixos = plt.subplots(2, 2, figsize=(9, 6))
    for col, eixo in enumerate(("roll", "yaw")):
        casos = [("modelo", modelo(mod, eixo))]
        for nome, base in (("SITL, AHRS", "verdade_ahrs"), ("SITL, sem EKF", "ablacao_ins_taxa")):
            pasta = A.voo(base, "ruido/rnd20/torque_pulso", "cc_lqg")
            if pasta is not None:
                casos.append((nome, sitl(pasta, eixo, mod)))
        for nome, (ea, et) in casos:
            ea = [x[np.isfinite(x)] for x in ea]
            linhas.append(f"{P.NOMES[eixo]:8s} {nome:14s}: ângulo {math.degrees(E.rms(ea)):.4f}; "
                          f"taxa {math.degrees(E.rms(et)):.4f}")
            for lin, segs in ((0, ea), (1, et)):
                f, p = E.psd(segs)
                eixos[lin, col].loglog(f[1:], p[1:], label=nome)
        eixos[0, col].set_title(f"{P.NOMES[eixo]}: erro do ângulo", loc="left", fontsize=9)
        eixos[1, col].set_title(f"{P.NOMES[eixo]}: erro da taxa", loc="left", fontsize=9)
    for ax in eixos.flat:
        ax.grid(True, which="both", lw=0.3)
    eixos[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(AQUI.parent / "erro_medidas.png", dpi=110)
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "erro_medidas.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
