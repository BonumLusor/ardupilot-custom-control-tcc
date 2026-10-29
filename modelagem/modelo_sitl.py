#!/usr/bin/env python3
"""Modelo de atitude da aeronave do SITL: pairado e linearizacao numerica.

Segue o roteiro da Parte 3 do minicurso "Introducao a Modelagem de Drones"
(Fischer, IFSP, SNCT 2025) — parametros fisicos, ponto de equilibrio do voo
pairado, linearizacao numerica por diferencas centrais —, mas com a cadeia que
o controlador de atitude enfrenta de fato, portada do codigo do ArduPilot
(commit ce9c2d01a4):

    u = [u_x u_y u_z] em [-1, 1]  (saida do controlador de taxa)
      -> mistura do quad X              AP_MotorsMatrix (fatores +-0,5)
      -> ganho de compensacao           Thrust_Linearization::get_compensation_gain
                                        (bateria e densidade do ar)
      -> linearizacao de empuxo         Thrust_Linearization::thrust_to_actuator
      -> PWM -> comando c do motor      SIM_Motor::pwm_to_command
      -> empuxo, torque de reacao       SIM_Motor::calc_thrust / calculate_forces
         e arrasto de momento
      -> aceleracao angular             SIM_Frame::calculate_forces

e escreve o resultado na forma de um modelo de primeira ordem por eixo,

    dw/dt = -a*w + b*u + d,

com d a perturbacao em rad/s2 (SIM_TWIST, WND_TQ). Os numeros saem dos
parametros gravados num voo (params.parm), e nao de valores de documentacao.

    ./modelo_sitl.py                               # params.parm do ensaio mais recente
    ./modelo_sitl.py --parm caminho/params.parm --altitude 15
    ./modelo_sitl.py --tensao-repouso 12.44        # BAT.VoltR do log do voo
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent
RESULTADOS = RAIZ.parent / "ensaios" / "resultados"

GRAVIDADE = 9.80665          # GRAVITY_MSS do ArduPilot
ALTITUDE_CASA = 584.0        # m, local CMAC, o padrao do sim_vehicle.py
EIXOS = ("rolagem", "arfagem", "guinada")

# Quad X: (angulo a partir da frente em graus, fator de guinada), na ordem
# M1..M4. E a mesma tabela no firmware (AP_MotorsMatrix.cpp, MOTOR_FRAME_TYPE_X)
# e na fisica (SIM_Frame.cpp, quad_x_motors). +1 = CCW visto de cima.
MOTORES_X = ((45.0, +1.0), (-135.0, +1.0), (-45.0, -1.0), (135.0, -1.0))


# --------------------------------------------------------------------------
# parametros
# --------------------------------------------------------------------------

def ler_parm(caminho):
    """params.parm de um ensaio (NOME,valor por linha) -> dict."""
    p = {}
    for linha in Path(caminho).read_text(encoding="utf-8").splitlines():
        if "," in linha:
            nome, valor = linha.split(",", 1)
            p[nome.strip()] = float(valor)
    return p


def parm_mais_recente():
    """O params.parm do ensaio mais recente em ensaios/resultados/."""
    candidatos = [c for c in RESULTADOS.rglob("params.parm")
                  if "autotune" not in c.parts and not c.parent.name.endswith("_falhou")]
    if not candidatos:
        raise SystemExit(f"nenhum params.parm em {RESULTADOS}; use --parm")
    return max(candidatos, key=lambda c: c.parent.name)


def densidade_ar(alt_amsl):
    """Atmosfera padrao de 1976 na troposfera (AP_Baro::get_air_density_for_alt_amsl)."""
    r0 = 6356766.0
    h = r0 * alt_amsl / (r0 + alt_amsl)            # altitude geopotencial
    t0, rho0, lapso, r_ar = 288.15, 1.225, -0.0065, 287.05287
    t = t0 + lapso * h
    return rho0 * (t / t0) ** (-GRAVIDADE / (lapso * r_ar) - 1.0)


class Aeronave:
    """O quad X do SITL e o firmware que o comanda, com os parametros de um voo."""

    def __init__(self, p, alt_amsl, v_repouso=None):
        self.p = p
        self.m = p["SIM_FRM_MASS"]
        self.diag = p["SIM_FRM_DIAG_SZ"]
        self.expo_sim = p["SIM_FRM_EXPO"]
        self.rho = densidade_ar(alt_amsl)

        # SIM_Frame::update_parameters
        rho_ref = densidade_ar(p["SIM_FRM_REF_ALT"])
        empuxo_pairado = self.m * GRAVIDADE
        potencia_pairado = p["SIM_FRM_REF_AMP"] * p["SIM_FRM_REF_VOLT"]
        v_saida_pairado = 2.0 * potencia_pairado / empuxo_pairado
        area_efetiva = empuxo_pairado / (0.5 * rho_ref * v_saida_pairado ** 2)
        self.v_max = v_saida_pairado / math.sqrt(max(p["SIM_FRM_HOVR_THR"], 0.01))
        self.area_ef = area_efetiva / 4.0                  # por motor
        self.area_real = p["SIM_FRM_DISCAREA"] / 4.0       # por motor
        self.fator_potencia = potencia_pairado / empuxo_pairado
        self.mdrag = self._coef_arrasto_momento(rho_ref)
        self.rot_ref = p["SIM_FRM_ROT_RATE"]

        # sem modelo JSON: metade da massa num anel de raio DIAG_SZ/2 (SIM_Frame.cpp)
        jxx = self.m * 0.25 * (self.diag * 0.5) ** 2
        self.J = np.array([jxx, jxx, self.m * 0.5 * (self.diag * 0.5) ** 2])

        # motor i a DIAG_SZ do CENTRO (SIM_Motor::setup_params)
        ang = np.radians([a for a, _ in MOTORES_X])
        self.pos = np.stack([self.diag * np.cos(ang), self.diag * np.sin(ang),
                             np.zeros(4)], axis=1)
        self.giro_sim = np.array([y for _, y in MOTORES_X])

        # AP_MotorsMatrix::add_motor + normalise_rpy_factors
        fr = np.cos(ang + math.pi / 2)
        fp = np.cos(ang)
        fy = self.giro_sim.copy()
        self.fatores = np.stack([0.5 * fr / np.abs(fr).max(),
                                 0.5 * fp / np.abs(fp).max(),
                                 0.5 * fy / np.abs(fy).max()], axis=1)

        # bateria: o SITL cede R*I; o firmware usa a tensao de repouso estimada,
        # V + I*R_est (BAT.VoltR), com a resistencia R_est que ele aprende em voo.
        # Com R_est = R, ela e a propria tensao sem carga, o padrao.
        self.v_bat0 = p["SIM_FRM_BAT_VOLT"]
        self.r_bat = p["SIM_FRM_BAT_RES"]
        self.v_repouso = self.v_bat0 if v_repouso is None else v_repouso

    def _coef_arrasto_momento(self, rho_ref):
        """SIM_FRM_MDRAG, reduzido se exceder o arrasto de referencia (SIM_Frame.cpp)."""
        p = self.p
        cos_t = math.cos(math.radians(p["SIM_FRM_REF_ANG"]))
        arrasto = self.m * GRAVIDADE * math.tan(math.radians(p["SIM_FRM_REF_ANG"]))
        empuxo_ref = self.m * GRAVIDADE / cos_t
        md = (cos_t * p["SIM_FRM_MDRAG"] * p["SIM_FRM_REF_SPD"] * cos_t
              * math.sqrt(empuxo_ref * rho_ref * p["SIM_FRM_DISCAREA"]))
        coef = p["SIM_FRM_MDRAG"]
        return coef * arrasto / md if md > arrasto else coef

    # ---------------- firmware ----------------

    def _lift_max(self):
        v = self._v_filt()
        e = self.p["MOT_THST_EXPO"]
        return v * (1 - e) + e * v * v

    def _v_filt(self):
        vmax, vmin = self.p["MOT_BAT_VOLT_MAX"], self.p["MOT_BAT_VOLT_MIN"]
        if vmax <= 0 or vmin >= vmax:
            return 1.0
        vmin = max(vmin, 0.6 * vmax)
        return min(max(self.v_repouso, vmin), vmax) / vmax

    def ganho_compensacao(self):
        """1/lift_max, vezes a correcao de densidade do ar (get_compensation_gain)."""
        razao = self.rho / 1.225
        g = 1.0 / self._lift_max()
        if 0.3 < razao < 1.5:
            g /= min(max(razao, 0.5), 1.25)
        return g

    def empuxo_para_comando(self, t):
        """Fracao de empuxo t -> comando c do motor no SITL.

        thrust_to_actuator e output_to_pwm do firmware, seguidos do
        pwm_to_command do SITL; com MOT_SPIN_* = SIM_FRM_SPIN_* e MOT_PWM_* =
        SIM_FRM_PWM_*, sobra apply_thrust_curve_and_volt_scaling.
        """
        p = self.p
        t = np.clip(t, 0.0, 1.0)
        e = min(max(p["MOT_THST_EXPO"], -1.0), 1.0)
        v = self._v_filt()
        r = ((e - 1.0) + np.sqrt((1.0 - e) ** 2 + 4.0 * e * self._lift_max() * t)) / (2.0 * e)
        act = p["MOT_SPIN_MIN"] + (p["MOT_SPIN_MAX"] - p["MOT_SPIN_MIN"]) * np.clip(r / v, 0, 1)
        pwm = p["MOT_PWM_MIN"] + (p["MOT_PWM_MAX"] - p["MOT_PWM_MIN"]) * act
        pmin = p["SIM_FRM_PWM_MIN"] + p["SIM_FRM_SPIN_MIN"] * (p["SIM_FRM_PWM_MAX"] - p["SIM_FRM_PWM_MIN"])
        pmax = p["SIM_FRM_PWM_MIN"] + p["SIM_FRM_SPIN_MAX"] * (p["SIM_FRM_PWM_MAX"] - p["SIM_FRM_PWM_MIN"])
        return np.clip((pwm - pmin) / (pmax - pmin), 0.0, 1.0)

    def mistura(self, acel, u):
        """Esforcos normalizados -> fracao de empuxo de cada motor (regime linear)."""
        g = self.ganho_compensacao()
        return g * (acel + self.fatores @ np.asarray(u, float))

    # ---------------- SITL ----------------

    def tensao_sitl(self, empuxos):
        """Tensao da bateria simulada, com a queda R*I da corrente dos motores."""
        v = self.v_bat0
        for _ in range(20):                 # ponto fixo: I depende de V
            corrente = np.sum(self.fator_potencia * np.abs(empuxos)) / max(v, 0.1)
            v = self.v_bat0 - corrente * self.r_bat
        return v

    def empuxo(self, c, escala_tensao):
        """SIM_Motor::calc_thrust, sem velocidade de entrada no rotor.

        O SITL subtrai v_entrada**2 de v_saida**2; o termo e quadratico em w e
        nao contribui para a linearizacao em torno de w = 0.
        """
        e = self.expo_sim
        v_out = escala_tensao * self.v_max * np.sqrt((1 - e) * c + e * c * c)
        return 0.5 * self.rho * self.area_ef * v_out ** 2

    def forcas(self, c, w, escala_tensao):
        """Empuxo total e torque no corpo para comandos c e velocidade angular w."""
        T = self.empuxo(c, escala_tensao)
        empuxo_vet = np.stack([np.zeros(4), np.zeros(4), -T], axis=1)
        vel_motor = -np.cross(self.pos, w)                  # -(posicao x gyro)
        k_md = self.mdrag * np.sqrt(self.rho * self.area_real * T)
        forca = empuxo_vet - vel_motor * k_md[:, None]
        torque = np.cross(self.pos, forca).sum(axis=0)
        # torque de reacao: fator_guinada * 0,05*D*T*c em torno de z
        torque[2] += np.sum(self.giro_sim * 0.05 * self.diag * T * c)
        return T.sum(), torque

    def acel_angular(self, w, u, acel):
        """dw/dt do SITL para velocidade angular w e esforcos u."""
        t = self.mistura(acel, u)
        c = self.empuxo_para_comando(t)
        # a tensao depende da corrente de pairado; congelada no equilibrio
        T0 = self.empuxo(c, 1.0)
        esc = self.tensao_sitl(T0) / self.v_bat0
        _, torque = self.forcas(c, np.asarray(w, float), esc)
        dw = torque / self.J
        if self.rot_ref > 0:
            dw = dw - np.asarray(w, float) * math.radians(400.0) / self.rot_ref
        return dw

    def empuxo_total(self, acel):
        t = self.mistura(acel, np.zeros(3))
        c = self.empuxo_para_comando(t)
        T0 = self.empuxo(c, 1.0)
        return self.forcas(c, np.zeros(3), self.tensao_sitl(T0) / self.v_bat0)[0]

    # ---------------- equilibrio e linearizacao ----------------

    def pairado(self):
        """Acelerador em que o empuxo total iguala o peso (bissecao)."""
        lo, hi = 0.05, 0.95
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if self.empuxo_total(mid) < self.m * GRAVIDADE:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def linearizar(self, acel, eps_w=1e-4, eps_u=1e-4):
        """A = d(dw/dt)/dw e B = d(dw/dt)/du por diferencas centrais (minicurso, Parte 3)."""
        A = np.zeros((3, 3))
        B = np.zeros((3, 3))
        z = np.zeros(3)
        for k in range(3):
            d = np.zeros(3)
            d[k] = eps_w
            A[:, k] = (self.acel_angular(d, z, acel) - self.acel_angular(-d, z, acel)) / (2 * eps_w)
            d = np.zeros(3)
            d[k] = eps_u
            B[:, k] = (self.acel_angular(z, d, acel) - self.acel_angular(z, -d, acel)) / (2 * eps_u)
        return A, B


def formulas_fechadas(av, acel):
    """Os mesmos coeficientes pelas expressoes do Capitulo 4, para conferencia."""
    t = av.mistura(acel, np.zeros(3))[0]
    c = float(av.empuxo_para_comando(t))
    T0 = float(av.empuxo(np.array([c]), 1.0)[0])
    esc = av.tensao_sitl(np.full(4, T0)) / av.v_bat0
    T = float(av.empuxo(np.array([c]), esc)[0])
    e = av.expo_sim
    # dT/dc no SITL e kappa incremental dQ/dT
    dTdc = T / ((1 - e) * c + e * c * c) * ((1 - e) + 2 * e * c)
    kappa = 0.05 * av.diag * c
    kappa_inc = 0.05 * av.diag * (c + T / dTdc)
    lxy = av.diag / math.sqrt(2)
    k_t = av.m * GRAVIDADE / (4 * acel)            # N por unidade de fracao de empuxo
    k_md = av.mdrag * math.sqrt(av.rho * av.area_real * T)
    a_md_xy = 4 * k_md * lxy ** 2 / av.J[0]
    a_md_z = 4 * k_md * av.diag ** 2 / av.J[2]
    a_rot = math.radians(400.0) / av.rot_ref
    return {
        "c_pairado": c, "T_pairado": T, "kappa": kappa, "kappa_inc": kappa_inc,
        "k_t": k_t, "k_md": k_md, "a_md": (a_md_xy, a_md_xy, a_md_z), "a_rot": a_rot,
        "b": (av.m * GRAVIDADE * lxy / (2 * acel * av.J[0]),
              av.m * GRAVIDADE * lxy / (2 * acel * av.J[1]),
              2 * kappa_inc * k_t / av.J[2]),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--parm", type=Path, help="params.parm de um ensaio (padrao: o mais recente)")
    ap.add_argument("--altitude", type=float, default=15.0,
                    help="altura do voo sobre o local de decolagem, em m (padrao 15)")
    ap.add_argument("--tensao-repouso", type=float,
                    help="tensao de repouso estimada pelo firmware, em V (BAT.VoltR do "
                         "log); padrao: a tensao sem carga SIM_FRM_BAT_VOLT")
    args = ap.parse_args()

    caminho = args.parm or parm_mais_recente()
    p = ler_parm(caminho)
    av = Aeronave(p, ALTITUDE_CASA + args.altitude, args.tensao_repouso)

    print(f"parametros: {caminho}")
    print(f"densidade do ar: {av.rho:.4f} kg/m3 a {ALTITUDE_CASA + args.altitude:.0f} m")
    print(f"inercia Jxx Jyy Jzz: {av.J[0]:.5f} {av.J[1]:.5f} {av.J[2]:.5f} kg.m2")
    print("fatores de mistura (rolagem, arfagem, guinada):")
    for i, f in enumerate(av.fatores, 1):
        print(f"  M{i}: {f[0]:+.2f} {f[1]:+.2f} {f[2]:+.2f}")

    acel = av.pairado()
    print(f"\nPAIRADO")
    print(f"  acelerador previsto {acel:.4f}   aprendido no voo (MOT_THST_HOVER) "
          f"{p.get('MOT_THST_HOVER', float('nan')):.4f}")
    ff = formulas_fechadas(av, acel)
    print(f"  comando do motor c = {ff['c_pairado']:.4f}, empuxo por motor {ff['T_pairado']:.3f} N")
    print(f"  tensao de repouso usada pelo firmware {av.v_repouso:.3f} V, "
          f"lift_max {av._lift_max():.4f} (MOTB.LiftMax no log)")
    print(f"  ganho de compensacao (bateria e densidade) {av.ganho_compensacao():.4f}")
    print(f"  kappa = Q/T = {ff['kappa']:.5f} m;  dQ/dT = {ff['kappa_inc']:.5f} m")
    print(f"  k_t = mg/(4 t_h) = {ff['k_t']:.3f} N;  arrasto de momento k_md = {ff['k_md']:.4f} N.s/m")

    A, B = av.linearizar(acel)
    np.set_printoptions(precision=4, suppress=True)
    print("\nLINEARIZACAO NUMERICA (diferencas centrais), dw/dt = A w + B u")
    print("A =\n", A)
    print("B =\n", B)

    print(f"\n{'eixo':10s} {'a num.':>8s} {'a fech.':>8s} {'b num.':>9s} {'b fech.':>9s} "
          f"{'1/a (s)':>8s}")
    for k, eixo in enumerate(EIXOS):
        a_f = ff["a_md"][k] + ff["a_rot"]
        print(f"{eixo:10s} {-A[k, k]:8.3f} {a_f:8.3f} {B[k, k]:9.2f} {ff['b'][k]:9.2f} "
              f"{-1 / A[k, k]:8.3f}")
    fora = np.abs(B - np.diag(np.diag(B))).max() / np.abs(np.diag(B)).min()
    print(f"\nacoplamento entre eixos em B: {fora:.1e} do menor termo da diagonal")
    print(f"a = arrasto de momento {ff['a_md'][0]:.3f} + resistencia rotacional {ff['a_rot']:.4f} 1/s")


if __name__ == "__main__":
    main()
