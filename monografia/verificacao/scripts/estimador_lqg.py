#!/usr/bin/env python3
"""Revisão de 3/11 (Fase 6): o estimador do LQG reprojetado, na teoria e no SITL.

1. Estimativa da perturbação num degrau de torque. Teoria: o erro de estimação evolui
   sozinho, e(k+1) = (I - L C) Phi e(k) (forma corrente), e um degrau em d começa com
   o erro todo em d; d^/d = 1 - e_d. SITL: o d^ gravado (CCLx.Dh) nos degraus de torque
   do lote, contra o torque injetado; tempos até 50 % e 90 % e o valor final (média do
   último segundo da janela).
2. Princípio da separação: com a planta igual ao modelo do estimador, os polos da malha
   fechada (o espelho do código, linearizado) contra a união dos polos da realimentação
   de estados (Phi + Gam K) e dos do erro de estimação ((I - L C) Phi).

Ganhos lidos dos params.parm dos voos do lote; em guinada, com alfa = 0 (pairado), o
estimador lento. Substitui a conta do relatório de 2/11 (LQG anterior ao reprojeto).
Escreve ../estimador_lqg.txt.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[2]
sys.path.insert(0, str(RAIZ / "modelagem"))
sys.path.insert(0, str(RAIZ / "ensaios"))
import controladores as C  # noqa: E402
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402
from analise.leitura import ler_log  # noqa: E402

RES = RAIZ / "ensaios" / "resultados"
LOTE = (RAIZ / "ensaios" / "lotes" / "binario_lote.sha256").read_text().split()[0]
EIXOS = (("rolagem", "roll", "CCLR", "Tx"), ("arfagem", "pitch", "CCLP", "Ty"),
         ("guinada", "yaw", "CCLY", "Tz"))
IDX_D = 4


def primeiro(cen, slug):
    for p in sorted((RES / cen / slug).glob("2026-*")):
        v = p / "versao.json"
        if v.exists() and json.loads(v.read_text())["binario_sha256"] == LOTE and not p.name.endswith("_falhou"):
            return p
    return None


def tempos(t, frac):
    """Primeiros instantes em que frac passa de 0,5 e de 0,9."""
    out = []
    for nivel in (0.5, 0.9):
        i = np.flatnonzero(frac >= nivel)
        out.append(float(t[i[0]]) if len(i) else float("nan"))
    return out


def main():
    mod = P.carregar_modelo()
    linhas = [f"Estimador do LQG, ganhos e voos do binário {LOTE[:12]}."]
    for slug in ("cc_lqg", "cc_lqg_mesmo_ruido"):
        for pasta_eixo, eixo, msg, col_t in EIXOS:
            pasta = primeiro(f"{pasta_eixo}/torque_degrau", slug)
            if pasta is None:
                continue
            p = V.ms.ler_parm(pasta / "params.parm")
            ts = 1.0 / float(p.get("SCHED_LOOP_RATE", 400))
            ctl, g = V.montar_controlador("cc_lqg", "CC4", p, eixo, mod.a, mod.b[eixo], ts)
            n = ctl.Phi.shape[0]
            E = (np.eye(n) - ctl.L @ ctl.C) @ ctl.Phi
            e = np.zeros(n)
            e[IDX_D] = 1.0
            hist = []
            for _ in range(int(3.0 / ts)):
                hist.append(1.0 - e[IDX_D])
                e = E @ e
            t50, t90 = tempos(np.arange(len(hist)) * ts, np.array(hist))

            d = ler_log(next(pasta.glob("*.BIN")), (msg, "WND"))
            w = d["WND"]
            on = np.flatnonzero(np.asarray(w[col_t]) != 0)
            t0, tau = float(w["t"][on[0]]), float(w[col_t][on[0]])
            tl, dh = d[msg]["t"], np.asarray(d[msg]["Dh"], dtype=float)
            sel = tl >= t0
            frac = dh[sel] / tau
            s50, s90 = tempos(tl[sel] - t0, frac)
            fim = float(np.mean(frac[(tl[sel] - t0 > 9.0) & (tl[sel] - t0 < 10.0)]))

            # separacao: planta = modelo do estimador; u_fb = K x
            K = np.zeros(n)
            K[0], K[1], K[IDX_D] = g["k_th"], g["k_w"], -1.0 / mod.b[eixo]
            F, G, H, J = C.linearizar(ctl)
            Phi, Gam, Cm = ctl.Phi, ctl.Gam.reshape(n, 1), ctl.C
            # o espelho linearizado le as medidas nas coordenadas da planta (angulo e
            # taxa medidos), e o modelo do estimador esta em coordenadas de erro: y = -C x
            Acl = np.block([[Phi - Gam @ J @ Cm, Gam @ H], [-G @ Cm, F]])
            pc = np.linalg.eigvals(Acl)
            uniao = np.concatenate([np.linalg.eigvals(Phi + Gam @ K.reshape(1, n)), np.linalg.eigvals(E)])
            # cada polo da uniao contra o mais proximo da malha fechada (atribuicao)
            dist = np.abs(uniao[:, None] - pc[None, :])
            r, c = linear_sum_assignment(dist)
            dif = dist[r, c]
            extras = np.delete(pc, c)
            linhas.append(
                f"{slug:20s} {pasta_eixo:8s} teoria 50/90 %: {t50:.3f}/{t90:.3f} s | SITL {s50:.3f}/{s90:.3f} s, "
                f"d^ final/torque {fim:.3f} | separação: maior diferença {dif.max():.1e}; "
                f"{len(extras)} polo(s) a mais no código: {np.round(np.abs(extras), 5)}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "estimador_lqg.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
