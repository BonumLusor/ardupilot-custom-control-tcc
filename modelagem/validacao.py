#!/usr/bin/env python3
"""Validacao do modelo linear de atitude contra os ensaios em SITL.

Para cada ensaio de torque gravado, simula a malha fechada com o modelo do
Capitulo 4 e o controlador que voou, alimentada pelo torque que o injetor
registrou no proprio voo (coluna tq* do series.csv), e calcula as metricas com
o mesmo codigo da analise dos ensaios (ensaios/analise/metricas.py), na mesma
janela. Assim, modelo e SITL sao comparados numero a numero.

A malha simulada, por eixo:

    planta       dw/dt = -a*w + b*u + d,  dtheta/dt = w      (modelo_sitl.py)
                 integrada a SIM_RATE_HZ (1200 Hz), u segurado entre laços
    giroscopio   passa-baixas de 2a ordem (LowPassFilter2p) em INS_GYRO_FILTER
    angulo       nativo: controlador de raiz quadrada com ATC_ACC_*_MAX/2
                 (AC_AttitudeControl::update_ang_vel_target_from_att_error);
                 cc_pid: so o P (AC_CustomControl_PID::update)
    taxa         AC_PID::update_all — filtros FLTT, FLTE e FLTD, integrador
                 limitado a IMAX — a SCHED_LOOP_RATE (400 Hz)
    cc_lqi,      o espelho do backend AC_CustomControl_LQR (controladores.py),
    cc_eso       com os ganhos CC3_* gravados no voo

A malha em si esta em malha.py, e os blocos do firmware, em blocos.py.

com a, b da linearizacao numerica, usando a tensao de repouso que o firmware
estimou no mesmo voo (BAT.VoltR do .BIN). Nas varreduras, compara tambem a
resposta em frequencia torque -> angulo, estimada como na analise dos ensaios.

    ./validacao.py                     # ensaios de torque mais recentes
    ./validacao.py --desde 2026-10-27  # so os ensaios a partir desta data
    ./validacao.py --figuras ../monografia/figuras --tabela ../monografia/tabelas/validacao.tex
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import signal

RAIZ = Path(__file__).resolve().parent
PROJETO = RAIZ.parent
RESULTADOS = PROJETO / "ensaios" / "resultados"
sys.path.insert(0, str(PROJETO / "ensaios"))
sys.path.insert(0, str(RAIZ))

from analise import metricas as M          # noqa: E402
import controladores as C                  # noqa: E402
import malha                               # noqa: E402
import modelo_sitl as ms                   # noqa: E402

EIXOS = {
    # eixo: (indice no modelo, sufixo ATC_RAT_*, sufixo ATC_ANG_*, ACC_*_MAX,
    #        coluna de torque, coluna de angulo, coluna de erro)
    "roll": (0, "RLL", "RLL", "R", "tqx", "roll", "erro_roll"),
    "pitch": (1, "PIT", "PIT", "P", "tqy", "pitch", "erro_pitch"),
    "yaw": (2, "YAW", "YAW", "Y", "tqz", "yaw_rel", "erro_yaw"),
}
NOMES = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
# rotulo de cada controlador (pasta de resultados) nas tabelas, e a ordem delas
ROTULOS = {"nativo": "nativo", "cc_pid": "\\codigo{cc\\_pid}",
           "nativo_lqi": "nativo (ganhos do LQI)", "cc_lqi": "LQI", "cc_eso": "LQR+ESO",
           "cc_lqi_mesmo_ruido": "LQI (mesmo ruído)", "cc_pid_ff": "PID com FF",
           "cc_lqi_sem_ff": "LQI sem FF", "cc_lqg": "LQG",
           "cc_lqg_mesmo_ruido": "LQG (mesmo ruído)"}
ENSAIOS_TORQUE = ("torque_pulso", "torque_degrau", "torque_chirp", "torque_estocastico")
# as tabelas modelo x SITL (validacao.tex, validacao_proposto.tex) sao so dos
# pulsos e degraus destes grupos, os de planta nominal e sem ruido
GRUPOS_TABELA = ("rolagem", "arfagem", "guinada", "rolagem_arfagem_guinada")
ENSAIOS_TABELA = ("torque_pulso", "torque_degrau")
COERENCIA_MINIMA = 0.8          # a mesma de ensaios/analise/frequencia.py


# --------------------------------------------------------------------------
# leitura de um ensaio
# --------------------------------------------------------------------------

def ler_series(caminho):
    with open(caminho, newline="", encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    col = {}
    for chave in linhas[0]:
        try:
            col[chave] = np.array([float(r[chave]) if r[chave] not in ("", "nan") else np.nan
                                   for r in linhas])
        except ValueError:
            continue
    return col


def ler_bat_wnd(pasta):
    """Do .BIN: mediana de BAT.VoltR com os motores girando, e o torque injetado.

    O torque volta como o injetor o escreveu --- {"t", "tqx", "tqy", "tqz"}, um
    valor por escrita da mensagem WND, no instante da escrita --- para ser
    segurado ate a escrita seguinte, como o SITL o aplica. Dois atalhos dariam
    errado: o series.csv, interpolado linearmente na grade, adianta essa escada
    em meio passo; e o campo Tp e o tempo NOMINAL da amostra (passo*DT), que o
    laco do Lua, a ~44 Hz e nao 50, alcanca com folga variavel de ate um passo.
    O instante vem de ler_log, a mesma base de tempo do series.csv.
    """
    from pymavlink import DFReader
    from analise.leitura import ler_log
    caminho = sorted(pasta.glob("*.BIN"))[0]
    w = ler_log(caminho)["WND"]
    wnd = {"t": w["t"], "tqx": w["Tx"], "tqy": w["Ty"], "tqz": w["Tz"]}
    log = DFReader.DFReader_binary(str(caminho))
    v = []
    while True:
        msg = log.recv_match(type="BAT")
        if msg is None:
            break
        if msg.Curr > 5.0 and msg.VoltR > 0:
            v.append(msg.VoltR)
    return (float(np.median(v)) if v else None), wnd


def torque_segurado(t, wnd, col_tq):
    """O torque de ``ler_bat_wnd`` nos instantes ``t``, segurado ate a escrita seguinte.

    E a entrada certa das respostas em frequencia theta/d: a coluna do
    series.csv e interpolada e atrasa a fase em pi*f*T (cerca de -35 graus a
    8 Hz), igual no SITL e no modelo.
    """
    i = np.clip(np.searchsorted(wnd["t"], t, side="right") - 1, 0, len(wnd["t"]) - 1)
    return np.asarray(wnd[col_tq], dtype=float)[i]


def controlador_do_ensaio(pasta, p):
    """O controlador que voou ('nativo', 'cc_pid', 'cc_lqi' ou 'cc_eso') e o
    prefixo dos seus ganhos. Os voos do nativo com outros ganhos (nativo_lqi)
    contam como nativo: e o mesmo codigo."""
    tipo = int(p.get("CC_TYPE", 0))
    if tipo == 0 or pasta.parent.name == "nativo":
        return "nativo", "ATC"
    if tipo == 3:
        return ("cc_eso" if int(p.get("CC3_MODO", 0)) == 1 else "cc_lqi"), "CC3"
    if tipo == 4:
        return "cc_lqg", "CC4"
    return "cc_pid", "CC2"


def ganhos(p, prefixo, eixo):
    _, rat, ang, acc, *_ = EIXOS[eixo]
    g = lambda nome, padrao=0.0: float(p.get(f"{prefixo}_RAT_{rat}_{nome}", padrao))
    return {
        "kp": g("P"), "ki": g("I"), "kd": g("D"), "imax": g("IMAX", 0.5),
        "fltt": g("FLTT"), "flte": g("FLTE"), "fltd": g("FLTD"),
        "kff": g("FF"), "kdff": g("D_FF"),
        "ang_p": float(p[f"{prefixo}_ANG_{ang}_P"]),
        "acc_max": math.radians(float(p.get(f"ATC_ACC_{acc}_MAX", 0.0))),
    }


def ganhos_cc3(p, eixo, a, b):
    """Parametros do backend AC_CustomControl_LQR (CC3_*) de um eixo.

    Os vetores do backend (AP_Vector3f) tem os sufixos _X, _Y e _Z, na ordem
    rolagem, arfagem e guinada. Sem CC3_A ou CC3_B no voo, valem os do modelo.
    """
    s = {"roll": "X", "pitch": "Y", "yaw": "Z"}[eixo]
    v = lambda nome: float(p.get(f"CC3_{nome}_{s}", 0.0))
    e_max = math.radians(v("EMAX"))
    return {"modo": int(p.get("CC3_MODO", 0)), "ff": bool(int(p.get("CC3_FF", 1))),
            "k_i": v("KI"), "k_th": v("KTH"), "k_w": v("KW"),
            "l1": v("ESO_L1"), "l2": v("ESO_L2"),
            "a": float(p.get("CC3_A", 0.0)) or a, "b": v("B") or b,
            "e_max": e_max if e_max > 0 else None, "fc_alvo": float(p.get("CC3_FLTW", 0.0))}


def sub_amostras_giro(p):
    """Sub-amostras que o SITL media em cada amostra de 1 kHz do giroscopio do IMU0.

    Com o bit 0 de INS_FAST_SAMPLE (padrao 1), AP_InertialSensor_SITL::generate_gyro
    sorteia 8 sub-amostras, cada uma com o seu ruido, e entrega a media delas
    (gyro_accum /= nsamples): o desvio padrao do ruido cai por raiz(8).
    """
    return 8 if int(p.get("INS_FAST_SAMPLE", 1)) & 1 else 1


def ganhos_cc4(p, eixo, a, b):
    """Parametros do backend AC_CustomControl_LQG (CC4_*) de um eixo: K, L (8 x 2) e o modelo.

    Com CC4_WMAN > 0, o estimador rapido da manobra (LR, TAUR, TMAN); os voos de
    antes dele nao tem esses parametros e saem com um estimador so.
    """
    s = {"roll": "X", "pitch": "Y", "yaw": "Z"}[eixo]
    v = lambda nome: float(p.get(f"CC4_{nome}_{s}", 0.0))
    e_max = math.radians(v("EMAX"))
    L = np.array([[v(f"L{i}T"), v(f"L{i}W")] for i in range(1, C.N_LQG + 1)])
    w_man = v("WMAN")
    L_r = (np.array([[v(f"LR{i}T"), v(f"LR{i}W")] for i in range(1, C.N_LQG + 1)])
           if w_man > 0 else None)
    return {"ff": bool(int(p.get("CC4_FF", 1))), "k_th": v("KTH"), "k_w": v("KW"), "L": L,
            "a": float(p.get("CC4_A", 0.0)) or a, "b": v("B") or b,
            "fc": float(p.get("CC4_FLTW", 20.0)), "tau_c": v("TAUC") or 0.3,
            "e_max": e_max if e_max > 0 else None, "L_r": L_r, "tau_r": v("TAUR") or 0.3,
            "w_ref": math.radians(w_man), "tau_alfa": float(p.get("CC4_TMAN", 1.0)) or 1.0}


def ruido_do_voo(p, col, sel, fs_fisica):
    """Desvio padrao, em rad/s por passo da fisica, do ruido de giroscopio do voo.

    O SITL soma a cada sub-amostra do giroscopio um ruido uniforme de amplitude
    SIM_GYR1_RND (graus/s) vezes o acelerador, a media de (PWM - 1000)/1000 dos
    motores (AP_InertialSensor_SITL), e entrega a 1 kHz a media de
    sub_amostras_giro(p) sub-amostras. O desvio padrao do uniforme e a
    amplitude/raiz(3), e a media divide por raiz(8); o resultado e passado para a
    taxa da fisica com a mesma densidade espectral. Conferido nos voos de
    ruido/ pela derivada do angulo do AHRS (conferir_ruido). O ruido sorteado e
    outro, entao a comparacao e estatistica, nao amostra a amostra.
    """
    rnd = float(p.get("SIM_GYR1_RND", 0.0))
    if rnd <= 0 or "m1" not in col:
        return 0.0
    pwm = np.nanmean([np.nanmean(col[f"m{i}"][sel]) for i in range(1, 5)])
    acelerador = min(max((pwm - 1000.0) / 1000.0, 0.0), 1.0)
    return (math.radians(rnd) * acelerador / math.sqrt(3.0 * sub_amostras_giro(p))
            * math.sqrt(fs_fisica / 1000.0))


def montar_controlador(nome, prefixo, p, eixo, a, b, dt_laco):
    """(espelho do controlador que voou, ganhos dele) para um eixo."""
    if nome in ("nativo", "cc_pid"):
        g = ganhos(p, prefixo, eixo)
        # a alimentacao direta pura do cc_pid_ff (CC2_FF), quando ligada
        s = {"roll": "X", "pitch": "Y", "yaw": "Z"}[eixo]
        b_ff = float(p.get(f"CC2_FF_B_{s}", 0.0))
        ff = ((float(p.get("CC2_FF_A", 0.0)), b_ff)
              if nome == "cc_pid" and int(p.get("CC2_FF", 0)) == 1 and b_ff > 0 else None)
        return C.CascataPID(g, eixo, dt_laco, raiz=(nome == "nativo"), ff=ff), g
    if nome == "cc_lqg":
        g = ganhos_cc4(p, eixo, a, b)
        ctl = C.LQG(g["k_th"], g["k_w"], g["L"], g["a"], g["b"], dt_laco, g["fc"], g["tau_c"],
                    ff=g["ff"], e_max=g["e_max"], fc_alvo=g["fc"], L_r=g["L_r"],
                    tau_r=g["tau_r"], w_ref=g["w_ref"], tau_alfa=g["tau_alfa"])
        return ctl, g
    g = ganhos_cc3(p, eixo, a, b)
    if nome == "cc_lqi":
        ctl = C.LQI(g["k_i"], g["k_th"], g["k_w"], g["a"], g["b"], dt_laco,
                    ff=g["ff"], e_max=g["e_max"], fc_alvo=g["fc_alvo"])
    else:
        ctl = C.LQRESO(g["k_th"], g["k_w"], g["l1"], g["l2"], g["a"], g["b"], dt_laco,
                       ff=g["ff"], e_max=g["e_max"], fc_alvo=g["fc_alvo"])
    return ctl, g


# --------------------------------------------------------------------------
# simulacao
# --------------------------------------------------------------------------

def simular(t_grade, escada, a, b, ctl, fs_fisica, fs_laco, fc_giro, atraso_lacos=0,
            ruido_giro=0.0, **opcoes):
    """Malha fechada linear de um eixo (malha.simular). Devolve o angulo (graus) na grade.

    ``escada`` = (t, valor): o torque como o injetor o escreveu, na base de tempo
    da grade.
    """
    return malha.simular(ctl, a, b, t_grade, escada=escada, fs_fisica=fs_fisica,
                         fs_laco=fs_laco, fc_giro=fc_giro, atraso_lacos=atraso_lacos,
                         ruido_giro=ruido_giro, **opcoes)["angulo_deg"]


def identificar_planta(pasta, eixo, t_ini, t_fim, f0, f1, fc_giro, a_fixo=None):
    """a e b da planta, no dominio da frequencia, a partir de uma varredura de torque.

    O torque injetado d e externo a malha e nao se correlaciona com mais nada no
    SITL (giroscopio sem ruido). Da planta, j*w*Omega = -a*Omega + b*U + D, e,
    dividindo por D,

        j*w*T_dw - 1 = -a*T_dw + b*T_du,

    linear em (a, b), com T_dw e T_du estimadas por Welch contra d. Na faixa em que
    a coerencia e alta, a*Omega e pequeno perto de j*w*Omega, e a e b se
    compensam; por isso sai tambem b com a fixo em ``a_fixo``. A velocidade
    angular registrada (RATE) e a filtrada pelo passa-baixas do giroscopio, que e
    descontado. Le o .BIN direto, a 400 Hz: o torque e reconstruido como o
    injetor o aplica, segurado entre as escritas da mensagem WND.
    """
    from pymavlink import DFReader
    campo_w, campo_u, campo_tq = {"roll": ("R", "ROut", "Tx"), "pitch": ("P", "POut", "Ty"),
                                  "yaw": ("Y", "YOut", "Tz")}[eixo]
    log = DFReader.DFReader_binary(str(sorted(pasta.glob("*.BIN"))[0]))
    rate_t, rate_w, rate_u, wnd_t, wnd_tq, wnd_tp = [], [], [], [], [], []
    while True:
        msg = log.recv_match(type=["RATE", "WND"])
        if msg is None:
            break
        if msg.get_type() == "RATE":
            rate_t.append(msg.TimeUS * 1e-6)
            rate_w.append(math.radians(getattr(msg, campo_w)))
            rate_u.append(getattr(msg, campo_u))
        else:
            wnd_t.append(msg.TimeUS * 1e-6)
            wnd_tq.append(getattr(msg, campo_tq))
            wnd_tp.append(msg.Tp)
    rate_t, rate_w, rate_u = map(np.asarray, (rate_t, rate_w, rate_u))
    wnd_t, wnd_tq, wnd_tp = map(np.asarray, (wnd_t, wnd_tq, wnd_tp))
    # janela da varredura, pelo tempo de perfil do injetor
    dentro_w = (wnd_tp >= t_ini) & (wnd_tp <= t_fim)
    ta, tb = wnd_t[dentro_w][0], wnd_t[dentro_w][-1]
    sel = (rate_t >= ta) & (rate_t <= tb)
    t, w, u = rate_t[sel], rate_w[sel], rate_u[sel]
    i = np.clip(np.searchsorted(wnd_t, t, side="right") - 1, 0, len(wnd_t) - 1)
    d = wnd_tq[i]

    fs = 1.0 / float(np.median(np.diff(t)))
    nperseg = max(int(len(d) / 8), 64)
    f, gdd = signal.welch(d, fs=fs, nperseg=nperseg)
    _, gdw = signal.csd(d, w, fs=fs, nperseg=nperseg)
    _, gdu = signal.csd(d, u, fs=fs, nperseg=nperseg)
    _, cw = signal.coherence(d, w, fs=fs, nperseg=nperseg)
    _, cu = signal.coherence(d, u, fs=fs, nperseg=nperseg)
    ok = (f >= f0) & (f <= f1) & (cw >= COERENCIA_MINIMA) & (cu >= COERENCIA_MINIMA)
    jw = 2j * np.pi * f[ok]
    wc = 2.0 * np.pi * fc_giro
    filtro = wc ** 2 / (jw ** 2 + math.sqrt(2.0) * wc * jw + wc ** 2)
    t_dw = (gdw / gdd)[ok] / filtro
    t_du = (gdu / gdd)[ok]
    lhs = jw * t_dw - 1.0
    A = np.column_stack([-t_dw, t_du])
    A = np.vstack([A.real, A.imag])
    y = np.concatenate([lhs.real, lhs.imag])
    (a, b), *_ = np.linalg.lstsq(A, y, rcond=None)
    res = {"a": float(a), "b": float(b), "faixa_hz": (float(f[ok][0]), float(f[ok][-1])),
           "pontos": int(ok.sum()), "fs": fs}
    if a_fixo is not None:
        y_b = y - A[:, 0] * a_fixo
        res["b_a_fixo"] = float(np.linalg.lstsq(A[:, 1:], y_b, rcond=None)[0][0])
    return res


def resposta_freq(t, u, y, f0, f1):
    """Mesma estimativa de ensaios/analise/frequencia.py (Welch, ~8 segmentos)."""
    fs = 1.0 / float(np.median(np.diff(t)))
    nperseg = max(int(len(u) / 8), 64)
    f, puu = signal.welch(u, fs=fs, nperseg=nperseg)
    _, pyu = signal.csd(u, y, fs=fs, nperseg=nperseg)
    _, coer = signal.coherence(u, y, fs=fs, nperseg=nperseg)
    banda = (f >= f0) & (f <= f1) & (f > 0)
    return f[banda], (pyu / np.maximum(puu, 1e-18))[banda], coer[banda]


# --------------------------------------------------------------------------
# o caminho do ruido do giroscopio (Etapa 0 do plano do LQG)
# --------------------------------------------------------------------------

# Constante de tempo da correcao lenta do angulo do AHRS. O erro do angulo,
# medido contra a atitude verdadeira da mensagem SIM nos voos de ruido/, tem
# autocorrelacao que cai a 1/e em 2 a 3,7 s, e 3 s reproduz o desvio dele em
# rolagem. Nao e o EK3_TAU_OUTPUT (0,25 s), que so alinha o preditor de saida ao
# estado fundido, que integra os mesmos delta-angulos. Acima de ~1 Hz o erro e
# a integral do ruido do giroscopio (a derivada do angulo bate a +-9 %); abaixo
# de ~0,5 Hz o EKF soma uma deriva propria que o modelo nao tem, maior na
# arfagem e muito maior na guinada (desvio por trecho de 76, ~136 e ~300
# mgraus, contra no maximo ~105 de um passeio aleatorio puro). Ela fica abaixo
# da banda das malhas e pesa igual sobre todos os controladores.
TAU_AHRS = 3.0
COL_ANG = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw"}
COL_RATE = {"roll": ("R", "ROut"), "pitch": ("P", "POut"), "yaw": ("Y", "YOut")}


def _passa_altas(x, n=100):
    """x menos a sua media movel de n amostras (0,25 s a 400 Hz): a banda do ruido."""
    return x - np.convolve(x, np.ones(n) / n, mode="same")


def _angulos_verdadeiros(sim):
    """Rolagem, arfagem e guinada (rad) do quaternion verdadeiro da mensagem SIM."""
    q1, q2, q3, q4 = sim["Q1"], sim["Q2"], sim["Q3"], sim["Q4"]
    return {"roll": np.arctan2(2 * (q1 * q2 + q3 * q4), 1 - 2 * (q2 * q2 + q3 * q3)),
            "pitch": np.arcsin(np.clip(2 * (q1 * q3 - q4 * q2), -1.0, 1.0)),
            "yaw": np.arctan2(2 * (q1 * q4 + q2 * q3), 1 - 2 * (q3 * q3 + q4 * q4))}


def conferir_ruido(pasta, tau_ahrs=TAU_AHRS, duracao_sim=60.0):
    """Ruido do giroscopio e do angulo do AHRS: SITL contra o modelo, num voo de ruido/.

    Nos trechos calmos do voo (antes do torque e depois da recuperacao), mede o
    desvio padrao, na banda do ruido (_passa_altas), do giroscopio filtrado
    (RATE), da derivada do angulo do AHRS (ANG), do esforco (RATE.*Out) e do
    erro do angulo contra a atitude verdadeira (SIM, a 25 Hz). O modelo simula
    o mesmo controlador sem perturbacao, com o ruido de ruido_do_voo, em tres
    versoes: a completa (angulo com o mesmo ruido e um laco de atraso, como o
    backend o le), sem o ruido no angulo, e com o ruido de antes da correcao
    de raiz(8). Devolve {eixo: {grandeza: (sitl, modelo, sem_ruido_angulo,
    ruido_antigo)}} e as series para a figura.
    """
    import yaml
    from analise.leitura import ler_log
    p = ms.ler_parm(pasta / "params.parm")
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text(encoding="utf-8"))
    col = ler_series(pasta / "series.csv")
    vr, _ = ler_bat_wnd(pasta)
    av = ms.Aeronave(p, ms.ALTITUDE_CASA + float(cen["voo"].get("altitude_m", 15)), vr)
    A, B = av.linearizar(av.pairado())
    nome_ctl, prefixo = controlador_do_ensaio(pasta, p)
    d = ler_log(sorted(pasta.glob("*.BIN"))[0], ("ANG", "RATE", "SIM", "WND"))

    v = cen["vento"]
    t0, dur = float(v["WND_TQT0"]), float(v["WND_TQDUR"])
    fim = float(cen["voo"]["duracao_s"]) - 1.0
    trechos = [(1.0, t0 - 0.5)]
    if not int(v["WND_MODE"]) & 32:
        trechos.append((t0 + max(dur * 8, 4.0) + 1.0, fim))
    tp0 = d["WND"]["t"][0] - d["WND"]["Tp"][0]

    def no_calmo(tab):
        tp = tab["t"] - tp0
        return [(tp >= a) & (tp <= b) for a, b in trechos]

    fs_fisica = float(p.get("SIM_RATE_HZ", 1200))
    fs_laco = float(p.get("SCHED_LOOP_RATE", 400))
    dt_laco = max(int(round(fs_fisica / fs_laco)), 1) / fs_fisica
    fc = float(p.get("INS_GYRO_FILTER", 20))
    sel = np.isfinite(col["tp"]) & (col["tp"] >= trechos[0][0]) & (col["tp"] <= trechos[0][1])
    sigma = ruido_do_voo(p, col, sel, fs_fisica)
    verdade = _angulos_verdadeiros(d["SIM"])

    def dp(partes):
        return float(np.sqrt(np.mean(np.concatenate([q[100:-100] ** 2 for q in partes]))))

    saida = {}
    for eixo, (i, *_r) in EIXOS.items():
        a, b = -A[i, i], B[i, i]
        ang = np.radians(d["ANG"][COL_ANG[eixo]])
        if eixo == "yaw":
            ang = np.unwrap(ang)
        sitl = {"giro": [], "dtheta": [], "u": [], "eta": []}
        for m in no_calmo(d["RATE"]):
            sitl["giro"].append(_passa_altas(np.radians(d["RATE"][COL_RATE[eixo][0]][m])))
            sitl["u"].append(_passa_altas(d["RATE"][COL_RATE[eixo][1]][m]))
        for m in no_calmo(d["ANG"]):
            sitl["dtheta"].append(_passa_altas(np.diff(ang[m]) / np.diff(d["ANG"]["t"][m])))
        verd = np.unwrap(verdade[eixo]) if eixo == "yaw" else verdade[eixo]
        eta_partes = []
        for m in no_calmo(d["SIM"]):
            e = np.interp(d["SIM"]["t"][m], d["ANG"]["t"], ang) - verd[m]
            eta_partes.append(e - e.mean())
        res = {"giro": [dp(sitl["giro"])], "dtheta": [dp(sitl["dtheta"])], "u": [dp(sitl["u"])],
               "eta": [float(np.std(np.concatenate(eta_partes)))]}
        series = {"sitl": sitl}
        t = np.arange(0.0, duracao_sim, 1.0 / fs_laco)
        for rot, s, com_ang in (("modelo", sigma, True), ("sem_ruido_angulo", sigma, False),
                                ("ruido_antigo", sigma * math.sqrt(sub_amostras_giro(p)), True)):
            ctl, _g = montar_controlador(nome_ctl, prefixo, p, eixo, a, b, dt_laco)
            r = malha.simular(ctl, a, b, t, fs_fisica=fs_fisica, fs_laco=fs_laco, fc_giro=fc,
                              ruido_giro=s, semente=1, ruido_angulo=com_ang, tau_ahrs=tau_ahrs,
                              atraso_angulo=1)
            giro = _passa_altas(r["giro"])
            dth = _passa_altas(np.diff(r["angulo_med"]) * fs_laco)
            u = _passa_altas(r["u"][::max(int(round(fs_fisica / fs_laco)), 1)])
            t_25 = np.arange(2.0, duracao_sim, 0.04)
            eta = np.interp(t_25, r["t_laco"], r["angulo_med"]) - np.interp(t_25, r["t"],
                                                                              r["theta"])
            # os mesmos trechos do voo, cada um sem a sua media, como no SITL
            n_tr = [int(m.sum()) for m in no_calmo(d["SIM"])]
            pedacos, i0 = [], 0
            while i0 + max(n_tr) <= len(eta):
                for n_i in n_tr:
                    pedacos.append(eta[i0:i0 + n_i] - eta[i0:i0 + n_i].mean())
                    i0 += n_i
            res["giro"].append(dp([giro]))
            res["dtheta"].append(dp([dth]))
            res["u"].append(dp([u]))
            res["eta"].append(float(np.std(np.concatenate(pedacos))))
            series[rot] = {"giro": giro, "dtheta": dth}
        saida[eixo] = {"valores": res, "series": series}
    return {"pasta": pasta, "controlador": nome_ctl, "slug": pasta.parent.name,
            "sigma": sigma, "fs_laco": fs_laco, "eixos": saida}


def imprimir_ruido(r):
    print(f"{cenario_de(r['pasta'])} {r['slug']}: sigma do ruido (1 kHz) "
          f"{math.degrees(r['sigma']) / math.sqrt(1.2):.2f} graus/s")
    rotulos = {"giro": "giro filtrado (graus/s)", "dtheta": "derivada do angulo (graus/s)",
               "u": "esforco u", "eta": "erro do angulo (mgraus)"}
    escala = {"giro": math.degrees(1), "dtheta": math.degrees(1), "u": 1.0,
              "eta": math.degrees(1) * 1e3}
    print(f"  {'eixo':8s} {'grandeza':30s} {'SITL':>9s} {'modelo':>9s} {'err%':>6s} "
          f"{'sem eta':>9s} {'ruido x raiz8':>13s}")
    for eixo, e in r["eixos"].items():
        for chave, vals in e["valores"].items():
            s, m, sem, ant = (x * escala[chave] for x in vals)
            print(f"  {NOMES[eixo]:8s} {rotulos[chave]:30s} {s:9.4f} {m:9.4f} "
                  f"{_erro_pct(m, s):+6.1f} {sem:9.4f} {ant:13.4f}")


def desenhar_ruido(r, destino, eixo="roll"):
    """DEP do giroscopio filtrado e da derivada do angulo do AHRS: SITL e modelo."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker

    azul, laranja, cinza = "#2a78d6", "#eb6834", "#8a8984"
    tinta, grade = "#0b0b0b", "#e4e3df"
    fs = r["fs_laco"]
    ser = r["eixos"][eixo]["series"]

    def dep(partes):
        acc, n = None, 0
        for x in partes:
            if len(x) < 512:
                continue
            f, pxx = signal.welch(x, fs=fs, nperseg=512)
            acc = pxx if acc is None else acc + pxx
            n += 1
        return f, acc / n

    plt.rcParams.update({"font.size": 9, "font.family": "serif"})
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8), sharey=True)
    for ax, chave, titulo in ((eixos[0], "giro", "Giroscópio filtrado"),
                              (eixos[1], "dtheta", "Derivada do ângulo do AHRS")):
        f, ps = dep(ser["sitl"][chave])
        ax.loglog(f[1:], np.degrees(np.sqrt(ps[1:])), "o", color=azul, ms=2.5, label="SITL")
        f, pm = dep([ser["modelo"][chave]])
        ax.loglog(f[1:], np.degrees(np.sqrt(pm[1:])), color=laranja, lw=1.5,
                  label="Modelo")
        if chave == "dtheta":
            f, p0 = dep([ser["sem_ruido_angulo"][chave]])
            ax.loglog(f[1:], np.degrees(np.sqrt(p0[1:])), color=cinza, lw=1.2, ls="--",
                      label="Modelo, ângulo sem ruído")
        ax.set_title(titulo, fontsize=9, color=tinta)
        ax.set_xlabel("Frequência (Hz)")
        ax.grid(True, which="both", color=grade, lw=0.5)
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
        ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(
            lambda x, _p: f"{x:g}".replace(".", ",")))
    eixos[0].set_ylabel("DEP$^{1/2}$ (°/s/$\\sqrt{\\mathrm{Hz}}$)")
    eixos[1].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    Path(destino).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destino)
    plt.close(fig)
    return destino


# --------------------------------------------------------------------------
# um ensaio
# --------------------------------------------------------------------------

def validar(pasta, atraso_lacos=0):
    p = ms.ler_parm(pasta / "params.parm")
    met = json.loads((pasta / "metricas.json").read_text(encoding="utf-8"))
    cen = __import__("yaml").safe_load((pasta / "cenario.yaml").read_text(encoding="utf-8"))
    col = ler_series(pasta / "series.csv")
    vento = cen["vento"]
    modo = int(vento["WND_MODE"])

    vr, wnd = ler_bat_wnd(pasta)
    av = ms.Aeronave(p, ms.ALTITUDE_CASA + float(cen["voo"].get("altitude_m", 15)), vr)
    A, B = av.linearizar(av.pairado())
    nome_ctl, prefixo = controlador_do_ensaio(pasta, p)

    t = col["t"]
    tp = col["tp"]
    jan = met.get("janela_disturbio_s")
    t_tq0 = float(vento["WND_TQT0"])
    # comeca 1 s antes do torque, com a aeronave em equilibrio
    sel = np.isfinite(tp) & (tp >= t_tq0 - 1.0)
    t_g = t[sel]

    saida = {"pasta": pasta, "controlador": nome_ctl, "slug": pasta.parent.name,
             "v_repouso": vr, "eixos": {}}
    for eixo, (i, *_resto) in EIXOS.items():
        col_tq, col_ang, col_erro = _resto[3], _resto[4], _resto[5]
        amp = float(vento[{"roll": "WND_TQX", "pitch": "WND_TQY", "yaw": "WND_TQZ"}[eixo]])
        if amp == 0 or col_tq not in col:
            continue
        a, b = -A[i, i], B[i, i]
        fs_fisica = float(p.get("SIM_RATE_HZ", 1200))
        fs_laco = float(p.get("SCHED_LOOP_RATE", 400))
        dt_laco = max(int(round(fs_fisica / fs_laco)), 1) * (1.0 / fs_fisica)
        ctl, g = montar_controlador(nome_ctl, prefixo, p, eixo, a, b, dt_laco)
        # Todos com o angulo como o SITL o entrega (um laco atrasado, com o ruido
        # do giroscopio integrado), a planta do portao (projeto.PLANTA_REAL): o
        # backend le o read_AHRS do laco anterior, e o laco de angulo nativo
        # roda depois do de taxa. Antes so o LQG o tinha, e a fila de atuacao do
        # malha.simular tinha um laco a mais; nos que so usam o giroscopio, um
        # erro compensava o outro (temporizacao conferida nos logs em 01/11).
        opcoes = {"ruido_angulo": True, "tau_ahrs": TAU_AHRS, "atraso_angulo": 1}
        ang_sim = simular(t_g, (wnd["t"], wnd[col_tq]), a, b, ctl,
                          fs_fisica=fs_fisica, fs_laco=fs_laco,
                          fc_giro=float(p.get("INS_GYRO_FILTER", 20)),
                          atraso_lacos=atraso_lacos,
                          ruido_giro=ruido_do_voo(p, col, sel, fs_fisica), **opcoes)
        # erro = alvo - medida; o alvo e o angulo de equilibrio antes do torque
        erro_sitl = col[col_erro][sel]
        base = np.nanmean(erro_sitl[t_g < t_g[0] + 0.9])
        erro_sim = base - ang_sim
        r = {"a": a, "b": b, "ganhos": g, "t": t_g, "erro_sitl": erro_sitl,
             "erro_sim": erro_sim, "torque": col[col_tq][sel]}
        if modo & 64:
            f0, f1 = float(vento["WND_TQF0"]), float(vento["WND_TQF1"])
            dentro = (tp[sel] >= t_tq0) & (tp[sel] <= t_tq0 + float(vento["WND_TQDUR"]))
            # torque segurado entre as escritas do injetor, como a fisica o aplica
            u = torque_segurado(t_g[dentro], wnd, col_tq)
            f, h_sitl, coer = resposta_freq(t_g[dentro], u, col[col_ang][sel][dentro], f0, f1)
            _, h_sim, _ = resposta_freq(t_g[dentro], u, ang_sim[dentro] + 0.0, f0, f1)
            ok = coer >= COERENCIA_MINIMA
            r["frf"] = {"f": f, "h_sitl": h_sitl, "h_sim": h_sim, "coer": coer, "ok": ok}
            if ok.any():
                dm = 20 * np.log10(np.abs(h_sim[ok]) / np.abs(h_sitl[ok]))
                df = np.degrees(np.angle(h_sim[ok] / h_sitl[ok]))
                r["frf_erro"] = {"faixa_hz": (float(f[ok][0]), float(f[ok][-1])),
                                 "mag_db_max": float(np.max(np.abs(dm))),
                                 "mag_db_med": float(np.median(np.abs(dm))),
                                 "fase_deg_max": float(np.max(np.abs(df)))}
            r["identificacao"] = identificar_planta(
                pasta, eixo, t_tq0, t_tq0 + float(vento["WND_TQDUR"]), f0, f1,
                float(p.get("INS_GYRO_FILTER", 20)), a_fixo=a)
        elif jan:
            m_sitl = met.get(f"disturbio_{eixo}", {})
            # o mesmo limiar do voo (piso no ruido calmo do SITL) e a mesma
            # permanencia, para que modelo e SITL sejam medidos pela mesma regra
            m_sim = M.metricas_disturbio(t_g, erro_sim, jan,
                                         faixa_calma_deg=m_sitl.get("faixa_calma_deg"),
                                         permanencia_s=M.PERMANENCIA_RECUPERACAO_S)
            r["metricas"] = {"sim": m_sim, "sitl": m_sitl}
        saida["eixos"][eixo] = r
    return saida


def cenario_de(pasta):
    """resultados/<cenario>/<controlador>/<carimbo> -> o cenario.

    O cenario e o caminho em cenarios/, com quantos niveis tiver ('grupo/ensaio'
    ou 'familia/variante/ensaio', como ruido/rnd20/torque_pulso): a mesma regra
    de run_ensaio.id_cenario_da_pasta.
    """
    rel = pasta.relative_to(RESULTADOS)
    return "/".join(rel.parts[:-2])


def ensaios_mais_recentes(desde=None):
    pastas = {}
    for met in RESULTADOS.rglob("metricas.json"):
        pasta = met.parent
        if pasta.name.endswith("_falhou") or "autotune" in pasta.parts:
            continue
        rel = pasta.relative_to(RESULTADOS).parts
        # copias de seguranca (antes_*) e revoos/repeticoes da verificacao
        # (verificacao_*) nao sao os voos das tabelas
        if rel[0].startswith(("antes_", "verificacao_")):
            continue
        if len(rel) < 4 or rel[-3] not in ENSAIOS_TORQUE:
            continue
        if desde and pasta.name < desde:
            continue
        chave = rel[:-1]
        if chave not in pastas or pasta.name > pastas[chave].name:
            pastas[chave] = pasta
    return [pastas[k] for k in sorted(pastas)]


# --------------------------------------------------------------------------
# saida
# --------------------------------------------------------------------------

def _fmt(v, casas=2):
    return "—" if v is None else f"{v:.{casas}f}".replace(".", ",")


def _erro_pct(sim, sitl):
    if sim is None or sitl in (None, 0):
        return None
    return 100.0 * (sim - sitl) / sitl


def imprimir(resultados):
    print(f"{'cenario':38s} {'ctl':7s} {'eixo':8s} {'desvio SITL/mod':>17s} {'err%':>6s} "
          f"{'recup. SITL/mod':>17s} {'err%':>6s} {'RMSE err%':>9s}")
    for r in resultados:
        for eixo, e in r["eixos"].items():
            cen = cenario_de(r["pasta"])
            if "metricas" in e:
                s, m = e["metricas"]["sitl"], e["metricas"]["sim"]
                dv = (s.get("desvio_max_deg"), m.get("desvio_max_deg"))
                rc = (s.get("tempo_recuperacao_s"), m.get("tempo_recuperacao_s"))
                rm = (s.get("rmse_deg"), m.get("rmse_deg"))
                fe = lambda x: "—" if x is None else f"{x:+.1f}"
                print(f"{cen:38s} {r['controlador']:7s} {eixo:8s} "
                      f"{_fmt(dv[0]):>8s}/{_fmt(dv[1]):<8s} {fe(_erro_pct(dv[1], dv[0])):>6s} "
                      f"{_fmt(rc[0]):>8s}/{_fmt(rc[1]):<8s} {fe(_erro_pct(rc[1], rc[0])):>6s} "
                      f"{fe(_erro_pct(rm[1], rm[0])):>9s}")
            elif "frf_erro" in e:
                fr = e["frf_erro"]
                print(f"{cen:38s} {r['controlador']:7s} {eixo:8s} resposta em frequencia "
                      f"{fr['faixa_hz'][0]:.1f}-{fr['faixa_hz'][1]:.1f} Hz: |dmag| mediana "
                      f"{fr['mag_db_med']:.2f} dB, max {fr['mag_db_max']:.2f} dB; "
                      f"|dfase| max {fr['fase_deg_max']:.1f} graus")
            if "identificacao" in e:
                ide = e["identificacao"]
                print(f"{'':38s} {'':7s} {'':8s} planta identificada "
                      f"({ide['pontos']} pontos, {ide['faixa_hz'][0]:.1f}-{ide['faixa_hz'][1]:.1f} Hz): "
                      f"a = {ide['a']:.2f} (modelo {e['a']:.2f}), b = {ide['b']:.1f} "
                      f"(modelo {e['b']:.1f}, {_erro_pct(ide['b'], e['b']):+.1f}%); com a do "
                      f"modelo, b = {ide['b_a_fixo']:.1f} ({_erro_pct(ide['b_a_fixo'], e['b']):+.1f}%)")


def _ordem_slug(slug):
    ordem = list(ROTULOS)
    return (ordem.index(slug) if slug in ordem else len(ordem), slug)


SLUGS_CAP4 = ("nativo", "cc_pid")
SLUGS_PROPOSTOS = ("cc_lqi", "cc_eso", "cc_lqi_mesmo_ruido", "nativo_lqi", "cc_lqg",
                   "cc_lqg_mesmo_ruido")


def escrever_tabela(resultados, destino, slugs=SLUGS_CAP4, longa=None):
    """Tabela LaTeX (booktabs) dos ensaios de pulso e degrau dos controladores ``slugs``.

    Com ``longa`` = (legenda, rotulo), sai uma longtable, que quebra entre paginas,
    com a legenda e o rotulo dentro dela (a do apendice dos propostos).

    Os ensaios com ruido de giroscopio (ruido/) ficam de fora: o ruido sorteado
    no modelo e outro, e a comparacao ali e estatistica, nao numero a numero.
    """
    linhas = []
    ordem_eixo = {"roll": 0, "pitch": 1, "yaw": 2}
    def chave(item):
        r, eixo = item
        grupo, ensaio = cenario_de(r["pasta"]).rsplit("/", 1)
        return ({"rolagem_arfagem_guinada": 1, "ruido": 2}.get(grupo, 0),
                ensaio != "torque_pulso",
                ordem_eixo[eixo], _ordem_slug(r["slug"]))
    def na_tabela(r):
        grupo, ensaio = cenario_de(r["pasta"]).rsplit("/", 1)
        return grupo in GRUPOS_TABELA and ensaio in ENSAIOS_TABELA

    pares = sorted(((r, eixo) for r in resultados for eixo, e in r["eixos"].items()
                    if "metricas" in e and r["slug"] in slugs and na_tabela(r)), key=chave)
    if not pares:
        return None
    for r, eixo in pares:
        e = r["eixos"][eixo]
        s, m = e["metricas"]["sitl"], e["metricas"]["sim"]
        cen = cenario_de(r["pasta"])
        grupo, ensaio = cen.rsplit("/", 1)
        tipo = {"torque_pulso": "pulso", "torque_degrau": "degrau"}[ensaio]
        ctl = ROTULOS.get(r["slug"], r["slug"])
        simult = {"rolagem_arfagem_guinada": " (3 eixos)",
                  "ruido": " (3 eixos, ruído)"}.get(grupo, "")
        linhas.append(
            f"{NOMES[eixo].capitalize()}{simult} & {tipo} & {ctl} & "
            f"{_fmt(s.get('desvio_max_deg'))} & {_fmt(m.get('desvio_max_deg'))} & "
            f"{_fmt(s.get('tempo_recuperacao_s'))} & {_fmt(m.get('tempo_recuperacao_s'))} & "
            f"{_fmt(s.get('rmse_deg'), 3)} & {_fmt(m.get('rmse_deg'), 3)} \\\\")
    corpo = "\n".join(linhas)
    if longa:
        cab = (" & & & \\multicolumn{2}{c}{Desvio máx. (\\si{\\degree})} & "
               "\\multicolumn{2}{c}{Recuperação (\\si{\\second})} & "
               "\\multicolumn{2}{c}{RMSE (\\si{\\degree})} \\\\\n"
               "\\cmidrule(lr){4-5}\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}\n"
               "Eixo & Torque & Controlador & SITL & Modelo & SITL & Modelo & SITL & Modelo \\\\\n")
        legenda, rotulo = longa
        texto = (
            "% Gerada por modelagem/validacao.py — nao editar a mao.\n"
            "\\begin{longtable}{lllrrrrrr}\n"
            f"\\caption{{{legenda}}}\\label{{{rotulo}}} \\\\\n"
            "\\toprule\n" + cab + "\\midrule\n\\endfirsthead\n"
            "\\toprule\n" + cab + "\\midrule\n\\endhead\n"
            "\\bottomrule\n\\endlastfoot\n" + corpo + "\n\\end{longtable}\n")
        Path(destino).parent.mkdir(parents=True, exist_ok=True)
        Path(destino).write_text(texto, encoding="utf-8")
        return destino
    texto = (
        "% Gerada por modelagem/validacao.py — nao editar a mao.\n"
        "\\begin{tabular}{lllrrrrrr}\n\\toprule\n"
        " & & & \\multicolumn{2}{c}{Desvio máx. (\\si{\\degree})} & "
        "\\multicolumn{2}{c}{Recuperação (\\si{\\second})} & "
        "\\multicolumn{2}{c}{RMSE (\\si{\\degree})} \\\\\n"
        "\\cmidrule(lr){4-5}\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}\n"
        "Eixo & Torque & Controlador & SITL & Modelo & SITL & Modelo & SITL & Modelo \\\\\n"
        "\\midrule\n" + corpo + "\n\\bottomrule\n\\end{tabular}\n")
    Path(destino).parent.mkdir(parents=True, exist_ok=True)
    Path(destino).write_text(texto, encoding="utf-8")
    return destino


def escrever_resumo(resultados, destino, slugs=SLUGS_PROPOSTOS):
    """Resumo da tabela modelo x SITL: por controlador, mediana e maximo de |modelo - SITL|.

    Os mesmos pares de escrever_tabela (pulso e degrau, sem os ensaios de ruido). A
    diferenca e relativa ao SITL, em %, para o desvio maximo e o RMSE; na
    recuperacao, que salta perto do limiar, vai em segundos.
    """
    def na_tabela(r):
        grupo, ensaio = cenario_de(r["pasta"]).rsplit("/", 1)
        return grupo in GRUPOS_TABELA and ensaio in ENSAIOS_TABELA

    por_ctl = {}
    for r in resultados:
        if r["slug"] not in slugs or not na_tabela(r):
            continue
        for eixo, e in r["eixos"].items():
            if "metricas" not in e:
                continue
            s, m = e["metricas"]["sitl"], e["metricas"]["sim"]
            d = por_ctl.setdefault(r["slug"], {"desvio": [], "rmse": [], "rec": []})
            for chave, nome in (("desvio_max_deg", "desvio"), ("rmse_deg", "rmse")):
                if s.get(chave) and m.get(chave) is not None:
                    d[nome].append(abs(100.0 * (m[chave] - s[chave]) / s[chave]))
            if s.get("tempo_recuperacao_s") is not None and m.get("tempo_recuperacao_s") is not None:
                d["rec"].append(abs(m["tempo_recuperacao_s"] - s["tempo_recuperacao_s"]))
    if not por_ctl:
        return None
    linhas = []
    for slug in sorted(por_ctl, key=_ordem_slug):
        d = por_ctl[slug]
        celulas = []
        for nome, casas in (("desvio", 1), ("rmse", 1), ("rec", 2)):
            v = np.asarray(d[nome])
            celulas += [_fmt(float(np.median(v)), casas), _fmt(float(np.max(v)), casas)] if len(v) else ["—", "—"]
        linhas.append(f"{ROTULOS.get(slug, slug)} & {len(d['desvio'])} & " + " & ".join(celulas) + " \\\\")
    texto = (
        "% Gerada por modelagem/validacao.py — nao editar a mao.\n"
        "\\begin{tabular}{lrrrrrrr}\n\\toprule\n"
        " & & \\multicolumn{2}{c}{Desvio máx. (\\%)} & \\multicolumn{2}{c}{RMSE (\\%)} & "
        "\\multicolumn{2}{c}{Recuperação (\\si{\\second})} \\\\\n"
        "\\cmidrule(lr){3-4}\\cmidrule(lr){5-6}\\cmidrule(lr){7-8}\n"
        "Controlador & Ensaios & Mediana & Máx. & Mediana & Máx. & Mediana & Máx. \\\\\n"
        "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")
    Path(destino).parent.mkdir(parents=True, exist_ok=True)
    Path(destino).write_text(texto, encoding="utf-8")
    return destino


def desenhar(resultados, destino, slug="nativo"):
    """Erro de atitude no SITL e no modelo, nos ensaios de pulso e degrau de um controlador."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    azul, laranja = "#2a78d6", "#eb6834"          # paleta validada (dataviz)
    tinta, tinta2, grade = "#0b0b0b", "#52514e", "#e4e3df"
    casos = [("rolagem/torque_pulso", "roll"), ("guinada/torque_pulso", "yaw"),
             ("rolagem/torque_degrau", "roll"), ("guinada/torque_degrau", "yaw")]
    escolhidos = []
    for cen, eixo in casos:
        for r in resultados:
            if (cenario_de(r["pasta"]) == cen and r["slug"] == slug
                    and eixo in r["eixos"]):
                escolhidos.append((cen, eixo, r["eixos"][eixo]))
    if not escolhidos:
        return None

    plt.rcParams.update({"font.size": 9, "axes.edgecolor": tinta2, "axes.labelcolor": tinta,
                         "xtick.color": tinta2, "ytick.color": tinta2,
                         "font.family": "serif"})
    fig, eixos = plt.subplots(2, 2, figsize=(6.3, 4.6))
    for ax, (cen, eixo, e) in zip(eixos.flat, escolhidos):
        t = e["t"] - e["t"][0] - 1.0              # zero no inicio do torque
        ax.plot(t, e["erro_sitl"], color=azul, lw=1.5, label="SITL")
        ax.plot(t, e["erro_sim"], color=laranja, lw=1.5, ls="--", label="Modelo linear")
        ax.axhline(0, color=grade, lw=0.8, zorder=0)
        ax.grid(True, color=grade, lw=0.5)
        ax.set_axisbelow(True)
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
        tipo = "pulso" if "pulso" in cen else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}, torque em {tipo}", color=tinta, fontsize=9)
        ax.set_xlabel("Tempo desde o início do torque (s)")
        ax.set_ylabel("Erro de atitude (°)")
        if tipo == "pulso":
            ax.set_xlim(-0.5, 4.0)
    eixos.flat[0].legend(frameon=False, fontsize=8, labelcolor=tinta)
    fig.tight_layout()
    Path(destino).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destino)
    plt.close(fig)
    return destino


def desenhar_frf(resultados, destino, slug="nativo"):
    """Resposta em frequencia torque -> angulo, SITL e modelo, nas varreduras do nativo."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker

    azul, laranja = "#2a78d6", "#eb6834"
    tinta, tinta2, grade = "#0b0b0b", "#52514e", "#e4e3df"
    ordem = {"rolagem": 0, "guinada": 1}
    casos = sorted(((cenario_de(r["pasta"]), eixo, e) for r in resultados
                    if r["slug"] == slug for eixo, e in r["eixos"].items()
                    if "frf" in e and cenario_de(r["pasta"]).split("/")[0] in ordem),
                   key=lambda c: ordem[c[0].split("/")[0]])
    if not casos:
        return None
    plt.rcParams.update({"font.size": 9, "font.family": "serif"})
    fig, eixos = plt.subplots(2, len(casos), figsize=(6.3, 4.4), sharex="col", squeeze=False)
    for j, (cen, eixo, e) in enumerate(casos):
        fr = e["frf"]
        ok = fr["ok"]
        f = fr["f"]
        # o modelo so na faixa em que a estimativa do SITL e confiavel
        faixa = (f >= f[ok][0]) & (f <= f[ok][-1]) if ok.any() else ok
        transformacoes = ((lambda h: 20 * np.log10(np.abs(h)), "Ganho (dB)"),
                          (lambda h: np.degrees(np.unwrap(np.angle(h))), "Fase (°)"))
        for i, (fun, rot) in enumerate(transformacoes):
            ax = eixos[i, j]
            ax.semilogx(f[ok], fun(fr["h_sitl"])[ok], "o", color=azul, ms=4, label="SITL")
            ax.semilogx(f[faixa], fun(fr["h_sim"])[faixa], color=laranja, lw=1.5,
                        label="Modelo linear")
            ax.grid(True, which="both", color=grade, lw=0.5)
            for lado in ("top", "right"):
                ax.spines[lado].set_visible(False)
            marcas = [x for x in (0.2, 0.5, 1, 2, 5, 10) if f[0] * 0.9 <= x <= f[-1] * 1.1]
            ax.set_xticks(marcas)
            ax.set_xticklabels([f"{x:g}".replace(".", ",") for x in marcas])
            ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
            if j == 0:
                ax.set_ylabel(rot)
            if i == 0:
                ax.set_title(f"{NOMES[eixo].capitalize()}", fontsize=9, color=tinta)
            else:
                ax.set_xlabel("Frequência (Hz)")
    eixos[0, 0].legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(destino)
    plt.close(fig)
    return destino


def polos_guinada(resultados):
    """Polo real lento da guinada, -ki/kp, contra a recuperacao medida."""
    for r in resultados:
        if cenario_de(r["pasta"]) != "guinada/torque_degrau":
            continue
        e = r["eixos"]["yaw"]
        g, a, b = e["ganhos"], e["a"], e["b"]
        if "kp" not in g:           # so as cascatas de PID
            continue
        # malha continua sem filtros: s^3 + (a + b kp) s^2 + b (kp k + ki) s + b ki k
        polos = np.roots([1.0, a + b * g["kp"], b * (g["kp"] * g["ang_p"] + g["ki"]),
                          b * g["ki"] * g["ang_p"]])
        lento = max(p.real for p in polos if abs(p.imag) < 1e-9)
        print(f"guinada/torque_degrau {r['slug']:7s}: polo lento {lento:.4f} 1/s "
              f"(tau {-1 / lento:.1f} s; kp/ki = {g['kp'] / g['ki']:.1f} s); "
              f"polos {np.round(polos, 3)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--desde", help="so ensaios com carimbo a partir deste (AAAA-MM-DD)")
    ap.add_argument("--atraso", type=int, default=0,
                    help="lacos de atraso extra entre o calculo e a aplicacao de u")
    ap.add_argument("--figuras", type=Path, help="pasta onde gravar as figuras (PDF)")
    ap.add_argument("--tabela", type=Path, help="arquivo .tex da tabela de validacao")
    ap.add_argument("--ruido", action="store_true",
                    help="so a conferencia do caminho do ruido (giroscopio e angulo do AHRS) "
                         "nos voos de ruido/torque_pulso")
    args = ap.parse_args()

    if args.ruido:
        for slug in ("nativo", "cc_lqi", "cc_eso"):
            base = RESULTADOS / "ruido" / "torque_pulso" / slug
            voos = sorted(v for v in base.glob("*") if (v / "metricas.json").exists()
                          and not v.name.endswith("_falhou")
                          and (not args.desde or v.name >= args.desde)) if base.exists() else []
            if not voos:
                continue
            r = conferir_ruido(voos[-1])
            imprimir_ruido(r)
            if args.figuras and slug == "nativo":
                print(f"figura: {desenhar_ruido(r, args.figuras / 'validacao_ruido_ahrs.pdf')}")
        return

    pastas = ensaios_mais_recentes(args.desde)
    if not pastas:
        raise SystemExit("nenhum ensaio de torque encontrado")
    resultados = []
    for pasta in pastas:
        r = validar(pasta, args.atraso)
        resultados.append(r)
    vrs = sorted({round(r["v_repouso"], 2) for r in resultados if r["v_repouso"]})
    print(f"{len(resultados)} ensaios; BAT.VoltR: {vrs} V\n")
    imprimir(resultados)
    print()
    polos_guinada(resultados)
    if args.tabela:
        escrever_tabela(resultados, args.tabela)
        print(f"\ntabela: {args.tabela}")
        # o mesmo confronto modelo x SITL para os controladores propostos, que
        # confere o backend em C++ contra o espelho em Python
        d = escrever_tabela(resultados, args.tabela.with_name("validacao_proposto.tex"),
                            slugs=SLUGS_PROPOSTOS,
                            longa=("Métricas de rejeição de torque no SITL e no modelo, para "
                                   "os controladores propostos e para o nativo com os ganhos "
                                   "do LQI.", "tab:validacao-proposto"))
        if d:
            print(f"tabela: {d}")
        d = escrever_resumo(resultados, args.tabela.with_name("validacao_proposto_resumo.tex"))
        if d:
            print(f"tabela: {d}")
    if args.figuras:
        for nome, fn in (("validacao_tempo.pdf", desenhar), ("validacao_freq.pdf", desenhar_frf)):
            d = fn(resultados, args.figuras / nome)
            if d:
                print(f"figura: {d}")
        for slug in ("cc_lqi", "cc_eso", "cc_lqg_mesmo_ruido"):
            d = desenhar(resultados, args.figuras / f"validacao_tempo_{slug}.pdf", slug=slug)
            if d:
                print(f"figura: {d}")


if __name__ == "__main__":
    main()
