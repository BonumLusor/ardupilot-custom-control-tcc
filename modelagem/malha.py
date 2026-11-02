"""Malha fechada de um eixo de atitude, com qualquer controlador de controladores.py.

    planta       dw/dt = -a*w + b*u + d,  dtheta/dt = w      (modelo_sitl.py)
                 integrada a SIM_RATE_HZ (1200 Hz), u segurado entre lacos,
                 Euler explicito como SIM_Aircraft::update_dynamics
    giroscopio   ruido branco opcional e passa-baixas de 2a ordem
                 (LowPassFilter2p) em INS_GYRO_FILTER
    controlador  um passo por laco de controle (SCHED_LOOP_RATE, 400 Hz), com
                 atraso opcional de lacos inteiros entre o calculo e a aplicacao

O angulo do AHRS entra sem filtro, e o alvo, quando ha, chega como no
firmware: angulo, velocidade e aceleracao angulares alvo.

Duas opcoes, desligadas por padrao, aproximam a malha do que o SITL entrega
(Etapa 0 do plano do LQG, conferidas nos logs):

    ruido_angulo   o angulo do AHRS integra os delta-angulos SEM o passa-baixas
                   do giroscopio (AP_InertialSensor_Backend, preditor de saida do
                   EKF3), e portanto o MESMO ruido: theta_med = theta + eta, com
                   d(eta)/dt = n - eta/tau_ahrs (tau_ahrs: a correcao lenta do EKF)
    atraso_angulo  run_custom_controller roda antes de read_AHRS (Copter.cpp):
                   o erro de angulo e o alvo chegam com esse numero de lacos de
                   atraso, e o giroscopio chega atual. No nativo tambem: o laco
                   de angulo roda em update_flight_mode, depois do de taxa
"""

from __future__ import annotations

import numpy as np

from blocos import PassaBaixas2p


def escada_em(t_sim, escada):
    """O torque segurado entre as escritas do injetor, nos instantes ``t_sim``."""
    if escada is None:
        return np.zeros_like(t_sim)
    t_esc, val_esc = escada
    idx = np.searchsorted(t_esc, t_sim, side="right") - 1
    return np.where(idx >= 0, val_esc[np.clip(idx, 0, len(val_esc) - 1)], 0.0)


def simular(ctl, a, b, t_grade, escada=None, ref=None, fs_fisica=1200.0, fs_laco=400.0,
            fc_giro=20.0, atraso_lacos=0, ruido_giro=0.0, semente=0, u_max=None,
            ruido_angulo=False, tau_ahrs=None, atraso_angulo=0, vies_giro=0.0):
    """Simula de t_grade[0] a t_grade[-1] e devolve um dict com as series.

    ``escada`` = (t, valor): o torque em rad/s2 como o injetor o escreveu.
    ``ref`` = (t, theta_t, w_t, a_t), em rad, rad/s e rad/s2; sem ela, o alvo e
    a atitude zero. ``ruido_giro`` e o desvio padrao, em rad/s, do ruido somado
    a velocidade angular antes do filtro. Com ``u_max``, o esforco aplicado e
    saturado e o controlador recebe o sinal de limite no laco seguinte.
    ``ruido_angulo``, ``tau_ahrs`` e ``atraso_angulo``: ver a docstring do modulo.
    ``vies_giro`` (rad/s) soma ao giroscopio filtrado o vies residual que a
    correcao do EKF deixa (o angulo do AHRS nao o integra).

    Chaves: t, theta, w, u (na taxa da fisica), d, angulo_deg (theta na grade,
    em graus) e, por laco de controle, t_laco, giro (o giroscopio filtrado),
    angulo_med (o angulo do AHRS) e cada sinal interno do controlador.
    """
    passos_por_laco = max(int(round(fs_fisica / fs_laco)), 1)
    dt = 1.0 / fs_fisica
    giro = PassaBaixas2p(fs_fisica, fc_giro)

    t0, t1 = float(t_grade[0]), float(t_grade[-1])
    n = int((t1 - t0) * fs_fisica) + 1
    t_sim = t0 + np.arange(n) * dt
    # o injetor escreve o torque a 50 Hz e o SITL o segura ate a escrita seguinte
    d = escada_em(t_sim, escada)
    ruido = (np.random.default_rng(semente).standard_normal(n) * ruido_giro
             if ruido_giro > 0 else None)

    i_laco = np.arange(0, n, passos_por_laco)
    if ref is not None:
        t_r, th_r, w_r, a_r = ref
        alvo = (np.interp(t_sim[i_laco], t_r, th_r), np.interp(t_sim[i_laco], t_r, w_r),
                np.interp(t_sim[i_laco], t_r, a_r))
    else:
        alvo = None

    w = th = 0.0
    w_f = 0.0
    eta = 0.0
    vaza = dt / tau_ahrs if tau_ahrs else 0.0
    # o u do laco ja vale ate o laco seguinte (SITL_State::_fdm_input_step roda a
    # fisica com a ultima saida dos motores); atraso_lacos atrasa alem disso
    fila = [0.0] * atraso_lacos
    fila_ang = [None] * atraso_angulo
    u = 0.0
    limite = False
    ang = np.empty(n)
    vel = np.empty(n)
    esf = np.empty(n)
    giro_laco = np.empty(len(i_laco))
    ang_med = np.empty(len(i_laco))
    sinais = {}
    j = 0
    for k in range(n):
        w_f = giro.aplicar(w if ruido is None else w + ruido[k])
        if ruido_angulo and ruido is not None:
            eta += ruido[k] * dt - eta * vaza
        if k % passos_por_laco == 0:
            th_med = th + eta
            if alvo is None:
                entrada = (-th_med, 0.0, 0.0)
            else:
                entrada = (alvo[0][j] - th_med, alvo[1][j], alvo[2][j])
            if atraso_angulo:
                # o backend le a atitude e o alvo do laco anterior
                fila_ang.append(entrada)
                antiga = fila_ang.pop(0)
                entrada = antiga if antiga is not None else (0.0, 0.0, 0.0)
            e_th, w_t, a_t = entrada
            giro_laco[j] = w_f + vies_giro
            ang_med[j] = th_med
            u_novo = ctl.passo(e_th, w_t, a_t, w_f + vies_giro, limite)
            if u_max is not None:
                limite = abs(u_novo) >= u_max
                u_novo = min(max(u_novo, -u_max), u_max)
            fila.append(u_novo)
            u = fila.pop(0)
            for nome, v in ctl.sinais.items():
                sinais.setdefault(nome, np.empty(len(i_laco)))[j] = v
            j += 1
        w += (-a * w + b * u + d[k]) * dt
        th += w * dt
        ang[k] = th
        vel[k] = w
        esf[k] = u
    saida = {"t": t_sim, "theta": ang, "w": vel, "u": esf, "d": d,
             "angulo_deg": np.degrees(np.interp(t_grade, t_sim, ang)),
             "t_laco": t_sim[i_laco], "giro": giro_laco, "angulo_med": ang_med}
    saida.update(sinais)
    return saida
