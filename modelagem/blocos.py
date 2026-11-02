"""Blocos do firmware usados na malha simulada: filtros, PID e raiz quadrada.

Cada um e o espelho em Python de uma rotina do ArduPilot (commit ce9c2d01a4),
com o nome dela na docstring, para que a simulacao enfrente os mesmos
filtros e limites que o controlador enfrenta no voo.
"""

from __future__ import annotations

import math

# AC_AttitudeControl.h: limites do controlador de raiz quadrada, em rad/s2
ACC_CTRL = {"roll": (math.radians(40), math.radians(720)),
            "pitch": (math.radians(40), math.radians(720)),
            "yaw": (math.radians(10), math.radians(120))}


def alfa_passa_baixas(dt, fc):
    """calc_lowpass_alpha_dt do AP_Math; fc <= 0 desliga o filtro."""
    if fc <= 0:
        return 1.0
    rc = 1.0 / (2.0 * math.pi * fc)
    return dt / (dt + rc)


class PassaBaixas2p:
    """LowPassFilter2p (biquad Butterworth) do AP_Filter."""

    def __init__(self, fs, fc):
        self.ativo = fc > 0
        if not self.ativo:
            return
        fr = fs / fc
        ohm = math.tan(math.pi / fr)
        c = 1.0 + 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm
        self.b0 = ohm * ohm / c
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c
        self.a2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * ohm + ohm * ohm) / c
        self.d1 = self.d2 = 0.0

    def aplicar(self, x):
        if not self.ativo:
            return x
        d0 = x - self.d1 * self.a1 - self.d2 * self.a2
        y = d0 * self.b0 + self.d1 * self.b1 + self.d2 * self.b2
        self.d2, self.d1 = self.d1, d0
        return y


class PID:
    """AC_PID::update_all sem notch, slew nem limite de PD (todos desligados)."""

    def __init__(self, kp, ki, kd, imax, fltt, flte, fltd, dt):
        self.kp, self.ki, self.kd, self.imax = kp, ki, kd, imax
        self.at = alfa_passa_baixas(dt, fltt)
        self.ae = alfa_passa_baixas(dt, flte)
        self.ad = alfa_passa_baixas(dt, fltd)
        self.dt = dt
        self.alvo = self.erro = self.deriv = self.integ = 0.0

    def atualizar(self, alvo, medida, limite=False):
        self.alvo += self.at * (alvo - self.alvo)
        erro_ant = self.erro
        self.erro += self.ae * ((self.alvo - medida) - self.erro)
        self.deriv += self.ad * ((self.erro - erro_ant) / self.dt - self.deriv)
        # AC_PID::update_i: com o limite dos motores ativo, o integrador so anda
        # no sentido que o descarrega
        if self.ki and (not limite or self.integ * self.erro < 0):
            self.integ = min(max(self.integ + self.erro * self.ki * self.dt,
                                 -self.imax), self.imax)
        return self.erro * self.kp + self.integ + self.deriv * self.kd


def raiz_quadrada(erro, p, lim, dt):
    """sqrt_controller do AP_Math."""
    if lim <= 0:
        taxa = erro * p
    else:
        lin = lim / (p * p)
        if erro > lin:
            taxa = math.sqrt(2.0 * lim * (erro - lin / 2.0))
        elif erro < -lin:
            taxa = -math.sqrt(2.0 * lim * (-erro - lin / 2.0))
        else:
            taxa = erro * p
    return min(max(taxa, -abs(erro) / dt), abs(erro) / dt)
