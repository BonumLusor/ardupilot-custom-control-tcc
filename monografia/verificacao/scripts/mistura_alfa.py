#!/usr/bin/env python3
"""Revisão de 3/11, item 3.4: a mistura alfa dos dois estimadores do LQG.

O estimador rápido só existe em guinada (TAU_RAPIDO = TAU_ESTIMADOR em rolagem e
arfagem). Para os dois LQG que voaram (cc_lqg e cc_lqg_mesmo_ruido), com os CC4_*
dos YAMLs:

  condição 1: na planta das margens (projeto.PLANTA_REAL) e no modelo do
              estimador, eta não depende de u nem dos demais estados;
  condição 2: os dois estimadores diferem só no decaimento de eta (e no L);
  condição 3: alfa em [0, 1] (no firmware, alfa = MIN(m, 1) com m >= 0; aqui só
              se registra).

Depois trava alfa em 0; 0,25; 0,5; 0,75 e 1, lineariza o espelho
(controladores.linearizar) e fecha a malha com a planta das margens
(projeto.analisar): a maior distância entre os polos de cada alfa e os de alfa = 0
e a margem de fase de cada um. Escreve ../mistura_alfa.txt.
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "modelagem"))
import controladores as C  # noqa: E402
import projeto  # noqa: E402

ALFAS = (0.0, 0.25, 0.5, 0.75, 1.0)


class LQGAlfaFixo(C.LQG):
    """O espelho do LQG com o peso alfa travado (e m sempre zero)."""

    def __init__(self, *args, alfa=0.0, **kw):
        super().__init__(*args, **kw)
        self.alfa_fixo = alfa

    def passo(self, e_th, w_t, a_t, giro, limite=False):
        e_th = C._limitar(e_th, self.e_max)
        e_w = self.filtro_alvo.aplicar(w_t) - giro
        u_ff = (a_t + self.a * w_t) / self.b if self.ff else 0.0
        y = np.array([e_th, e_w])
        x_pred = self.Phi @ self.x + self.Gam * self.u_fb_ant
        self.x = x_pred + self.L @ (y - self.C @ x_pred)
        x_pred = self.Phi_r @ self.x_r + self.Gam_r * self.u_fb_ant
        self.x_r = x_pred + self.L_r @ (y - self.C @ x_pred)
        self.m = 0.0
        x = self.alfa_fixo * self.x_r + (1.0 - self.alfa_fixo) * self.x
        u = u_ff + self.k_th * x[0] + self.k_w * x[1] - x[4] / self.b
        self.u_fb_ant = C._limitar(u, self.u_max) - u_ff
        return u


def ganhos(nome):
    g = yaml.safe_load((RAIZ / "ensaios" / "controladores" / f"{nome}.yaml").read_text())["ganhos"]
    s = "Z"
    L = np.array([[g[f"CC4_L{i}T_{s}"], g[f"CC4_L{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    L_r = np.array([[g[f"CC4_LR{i}T_{s}"], g[f"CC4_LR{i}W_{s}"]] for i in range(1, C.N_LQG + 1)])
    return dict(a=g["CC4_A"], fc=g["CC4_FLTW"], b=g[f"CC4_B_{s}"], k_th=g[f"CC4_KTH_{s}"],
                k_w=g[f"CC4_KW_{s}"], L=L, L_r=L_r, tau_c=g[f"CC4_TAUC_{s}"],
                tau_r=g[f"CC4_TAUR_{s}"], w_ref=math.radians(g[f"CC4_WMAN_{s}"]),
                tau_alfa=g["CC4_TMAN"])


def condicoes(mod, g, linhas):
    pl = projeto.planta_com_filtro(mod.a, mod.b["yaw"], mod.ts, mod.fc_giro, **projeto.PLANTA_REAL)
    i_eta = 4                                        # quinto estado da planta com ruido_angulo
    fora = [j for j in range(pl.A.shape[0]) if j != i_eta and abs(pl.A[i_eta, j]) > 1e-12]
    linhas.append(f"  condição 1, planta (tau_AHRS = {projeto.PLANTA_REAL['tau_c']} s): linha de eta "
                  f"fora da diagonal {'nula' if not fora else fora}; Bu[eta] = {pl.Bu[i_eta, 0]:.1e}")
    Phi, Gam, Cm = C.modelo_lqg(g["a"], g["b"], g["fc"], mod.ts, g["tau_c"])
    Phi_r, Gam_r, Cm_r = C.modelo_lqg(g["a"], g["b"], g["fc"], mod.ts, g["tau_r"])
    i5 = C.ESTADOS_LQG.index("eta") if hasattr(C, "ESTADOS_LQG") and "eta" in C.ESTADOS_LQG else 5
    fora = [j for j in range(C.N_LQG) if j != i5 and abs(Phi[i5, j]) > 1e-12]
    linhas.append(f"  condição 1, estimador: linha de eta fora da diagonal {'nula' if not fora else fora}; "
                  f"Gam[eta] = {Gam[i5]:.1e}")
    dif = np.argwhere(np.abs(Phi_r - Phi) > 1e-12)
    linhas.append(f"  condição 2: Phi difere em {[tuple(int(v) for v in d) for d in dif]} "
                  f"(eta = estado {i5}); |dGam| = {np.max(np.abs(Gam_r - Gam)):.1e}; "
                  f"|dC| = {np.max(np.abs(Cm_r - Cm)):.1e}; tau_c = {g['tau_c']} s, tau_r = {g['tau_r']} s")


def main():
    mod = projeto.carregar_modelo()
    linhas = [f"Ts = {mod.ts} s; planta das margens: {projeto.PLANTA_REAL}",
              "condição 3: alfa = MIN(m, 1), m = MAX(|w_t|/w_ref, decai*m) >= 0 "
              "(firmware/AC_CustomControl_LQG.cpp e controladores.LQG)"]
    for nome in ("cc_lqg", "cc_lqg_mesmo_ruido"):
        g = ganhos(nome)
        linhas.append(f"{nome} (guinada):")
        condicoes(mod, g, linhas)
        ref = None
        for alfa in ALFAS:
            ctl = LQGAlfaFixo(g["k_th"], g["k_w"], g["L"], g["a"], g["b"], mod.ts, g["fc"], g["tau_c"],
                              fc_alvo=g["fc"], L_r=g["L_r"], tau_r=g["tau_r"], w_ref=g["w_ref"],
                              tau_alfa=g["tau_alfa"], alfa=alfa)
            an = projeto.analisar(ctl, mod.a, mod.b["yaw"], mod.ts, mod.fc_giro,
                                  planta=projeto.PLANTA_REAL)
            p = an.polos_z
            if ref is None:
                ref = p
                dist = 0.0
            else:
                custo = np.abs(ref[:, None] - p[None, :])
                i, j = linear_sum_assignment(custo)
                dist = float(np.max(custo[i, j]))
            mg = an.margens
            linhas.append(f"  alfa {alfa:4.2f}: |polo| max {np.max(np.abs(p)):.6f}; "
                          f"maior distância aos polos de alfa = 0: {dist:.1e}; margens {mg}")
    texto = "\n".join(linhas)
    print(texto)
    (Path(__file__).resolve().parents[1] / "mistura_alfa.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
