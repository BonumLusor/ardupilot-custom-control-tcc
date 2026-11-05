#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): em que ponto o ruído do LQG se separa do modelo.

No trecho calmo do voo de RND 20 (cc_lqg, ângulo do AHRS) e numa simulação de 60 s
com o mesmo ruído (projeto.ruido_no_tempo), compara o valor eficaz:

  - das medidas que entram no estimador: e_th (erro de ângulo) e e_w (erro de taxa);
  - dos estados estimados: theta^ (x0), e_w^ (x1), d^ (x4), eta^ (x5), b_g^ (x6);
  - das parcelas do esforço: k_th x0, k_w x1 e -x4/b, e do próprio u.

Se as medidas batem e os estados não, o estimador do firmware difere do espelho;
se as medidas já diferem, a diferença vem da planta ou do ruído.
Escreve ../ruido_parcelas.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_absoluto as R  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import malha  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402


def sitl(eixo):
    pasta = A.voo("ablacao_ahrs", "ruido/rnd20/torque_pulso", "cc_lqg")
    s = E.SUF[eixo]
    d = leitura.ler_log(next(pasta.glob("*.BIN")), (f"CCL{s}", f"CCK{s}", "WND"))
    jan = E.trechos(d, pasta)
    cl, ck = d[f"CCL{s}"], d[f"CCK{s}"]
    ser = {"e_th": cl["Eth"], "e_w": cl["Ew"], "x0": ck["ThH"], "x1": ck["WeH"], "x4": cl["Dh"],
           "x5": ck["EtaH"], "x6": ck["Bg"], "u": cl["U"]}
    return {k: E.rms(E.recorta(cl["t"], v, jan)) for k, v in ser.items()}


def modelo(mod, eixo, ctl):
    sig = math.sqrt(P.intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, R.SEGUNDOS, 1.0 / mod.fs_laco)
    passo = max(int(round(mod.fs_fisica / mod.fs_laco)), 1)
    acc = {}
    for semente in R.SEMENTES:
        c = R.lqg_voado(mod, eixo)
        r = malha.simular(c, mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica, fs_laco=mod.fs_laco,
                          fc_giro=mod.fc_giro, ruido_giro=sig, semente=semente,
                          **P.opcoes_malha(P.PLANTA_REAL))
        n0 = int(5 * mod.fs_laco)
        ser = {"e_th": -np.asarray(r["angulo_med"]), "e_w": -np.asarray(r["giro"]),
               "x0": np.asarray(r["th_hat"]), "x1": -np.asarray(r["w_hat"]),
               "x4": np.asarray(r["d_hat"]), "x5": np.asarray(r["eta_hat"]),
               "x6": np.asarray(r["bg_hat"]), "u": np.asarray(r["u"])[::passo]}
        for k, v in ser.items():
            v = v[n0:]
            acc.setdefault(k, []).append(float(np.std(v)))
    return {k: float(np.mean(v)) for k, v in acc.items()}, c


def main():
    mod = P.carregar_modelo()
    linhas = ["valor eficaz no trecho calmo, RND 20, cc_lqg; SITL contra modelo (razão)"]
    for eixo in ("roll", "yaw"):
        s = sitl(eixo)
        m, c = modelo(mod, eixo, None)
        linhas.append(f"{P.NOMES[eixo]} (k_th {c.k_th:.4g}, k_w {c.k_w:.4g}, b {c.b:.4g}):")
        for k in ("e_th", "e_w", "x0", "x1", "x4", "x5", "x6", "u"):
            linhas.append(f"  {k:5s} SITL {s[k]:.3e}  modelo {m[k]:.3e}  ({s[k] / m[k]:.2f})")
        linhas.append(f"  parcelas no SITL: k_th x0 {c.k_th * s['x0']:.2e}, k_w x1 {c.k_w * s['x1']:.2e}, "
                      f"x4/b {s['x4'] / c.b:.2e}")
        linhas.append(f"  parcelas no modelo: k_th x0 {c.k_th * m['x0']:.2e}, k_w x1 {c.k_w * m['x1']:.2e}, "
                      f"x4/b {m['x4'] / c.b:.2e}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "ruido_parcelas.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
