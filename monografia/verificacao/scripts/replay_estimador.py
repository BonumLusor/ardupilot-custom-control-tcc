#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3d): o estimador do firmware é o do espelho Python?

Em malha aberta: o espelho do LQG (controladores.LQG, ganhos do cc_lqg.yaml) recebe,
laço a laço, as medidas gravadas pelo backend no voo de RND 20 (CCLx.Eth e CCLx.Ew),
partindo dos estados gravados no início de cada trecho calmo. Se ele reproduzir os
estados e o esforço gravados, o firmware faz o que o espelho faz, e a diferença de
ruído entre o SITL e o modelo vem do que entra no controlador (planta, ruído,
medidas). Escreve ../replay_estimador.txt.
"""
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import ablacao_lqg as A  # noqa: E402
import ruido_absoluto as R  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402


def main():
    mod = P.carregar_modelo()
    pasta = A.voo("ablacao_ahrs", "ruido/rnd20/torque_pulso", "cc_lqg")
    linhas = [f"voo {pasta.relative_to(A.MED)}; espelho em malha aberta sobre as medidas gravadas"]
    for eixo in ("roll", "yaw"):
        s = E.SUF[eixo]
        d = leitura.ler_log(next(pasta.glob("*.BIN")), (f"CCL{s}", f"CCK{s}", "WND"))
        cl, ck = d[f"CCL{s}"], d[f"CCK{s}"]
        erros = {k: [] for k in ("x0", "x1", "x4", "u")}
        ref = {k: [] for k in erros}
        for a, b in E.trechos(d, pasta):
            sel = np.nonzero((cl["t"] >= a) & (cl["t"] <= b))[0]
            if len(sel) < 400:
                continue
            ctl = R.lqg_voado(mod, eixo)
            i0 = sel[0]
            # estado inicial: o gravado (os do filtro e o atraso partem das medidas)
            x = np.zeros(8)
            x[0], x[1], x[4], x[5], x[6] = ck["ThH"][i0], ck["WeH"][i0], cl["Dh"][i0], ck["EtaH"][i0], ck["Bg"][i0]
            x[2], x[7] = cl["Ew"][i0], cl["Eth"][i0]
            ctl.x = x.copy()
            if ctl.duplo:
                ctl.x_r = x.copy()
            ctl.u_fb_ant = cl["U"][i0 - 1] - cl["Uff"][i0 - 1]
            for j in sel[1:]:
                u = ctl.passo(cl["Eth"][j], 0.0, 0.0, -cl["Ew"][j])
                xm = ctl.x
                for k, mv, lv in (("x0", xm[0], ck["ThH"][j]), ("x1", xm[1], ck["WeH"][j]),
                                  ("x4", xm[4], cl["Dh"][j]), ("u", u, cl["U"][j] - cl["Uff"][j])):
                    erros[k].append(mv - lv)
                    ref[k].append(lv)
        linhas.append(f"{P.NOMES[eixo]}:")
        for k in erros:
            e, r = np.asarray(erros[k]), np.asarray(ref[k])
            linhas.append(f"  {k}: eficaz gravado {np.std(r):.3e}; erro eficaz do espelho {np.std(e):.3e} "
                          f"({100 * np.std(e) / np.std(r):.1f} % do gravado)")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "replay_estimador.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
