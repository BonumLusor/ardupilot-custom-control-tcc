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
import modelo_sitl as ms                   # noqa: E402

EIXOS = {
    # eixo: (indice no modelo, sufixo ATC_RAT_*, sufixo ATC_ANG_*, ACC_*_MAX,
    #        coluna de torque, coluna de angulo, coluna de erro)
    "roll": (0, "RLL", "RLL", "R", "tqx", "roll", "erro_roll"),
    "pitch": (1, "PIT", "PIT", "P", "tqy", "pitch", "erro_pitch"),
    "yaw": (2, "YAW", "YAW", "Y", "tqz", "yaw_rel", "erro_yaw"),
}
NOMES = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
# AC_AttitudeControl.h: limites do controlador de raiz quadrada, em rad/s2
ACC_CTRL = {"roll": (math.radians(40), math.radians(720)),
            "pitch": (math.radians(40), math.radians(720)),
            "yaw": (math.radians(10), math.radians(120))}
ENSAIOS_TORQUE = ("torque_pulso", "torque_degrau", "torque_chirp")
COERENCIA_MINIMA = 0.8          # a mesma de ensaios/analise/frequencia.py


# --------------------------------------------------------------------------
# blocos do firmware
# --------------------------------------------------------------------------

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

    def atualizar(self, alvo, medida):
        self.alvo += self.at * (alvo - self.alvo)
        erro_ant = self.erro
        self.erro += self.ae * ((self.alvo - medida) - self.erro)
        self.deriv += self.ad * ((self.erro - erro_ant) / self.dt - self.deriv)
        if self.ki:
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


def controlador_do_ensaio(pasta, p):
    """'nativo' ou 'cc_pid', e o prefixo dos ganhos que voaram."""
    nativo = int(p.get("CC_TYPE", 0)) == 0 or pasta.parent.name == "nativo"
    return ("nativo", "ATC") if nativo else ("cc_pid", "CC2")


def ganhos(p, prefixo, eixo):
    _, rat, ang, acc, *_ = EIXOS[eixo]
    g = lambda nome, padrao=0.0: float(p.get(f"{prefixo}_RAT_{rat}_{nome}", padrao))
    return {
        "kp": g("P"), "ki": g("I"), "kd": g("D"), "imax": g("IMAX", 0.5),
        "fltt": g("FLTT"), "flte": g("FLTE"), "fltd": g("FLTD"),
        "ang_p": float(p[f"{prefixo}_ANG_{ang}_P"]),
        "acc_max": math.radians(float(p.get(f"ATC_ACC_{acc}_MAX", 0.0))),
    }


# --------------------------------------------------------------------------
# simulacao
# --------------------------------------------------------------------------

def simular(t_grade, escada, a, b, g, eixo, raiz, fs_fisica, fs_laco, fc_giro,
            atraso_lacos=0):
    """Malha fechada linear de um eixo. Devolve o angulo (graus) na grade.

    ``escada`` = (t, valor): o torque como o injetor o escreveu, na base de tempo
    da grade.
    """
    passos_por_laco = max(int(round(fs_fisica / fs_laco)), 1)
    dt = 1.0 / fs_fisica
    dt_laco = passos_por_laco * dt
    pid = PID(g["kp"], g["ki"], g["kd"], g["imax"], g["fltt"], g["flte"], g["fltd"], dt_laco)
    giro = PassaBaixas2p(fs_fisica, fc_giro)
    lo, hi = ACC_CTRL[eixo]
    lim = min(max(g["acc_max"] / 2.0, lo), hi) if (raiz and g["acc_max"] > 0) else 0.0

    t0, t1 = float(t_grade[0]), float(t_grade[-1])
    n = int((t1 - t0) * fs_fisica) + 1
    t_sim = t0 + np.arange(n) * dt
    # o injetor escreve o torque a 50 Hz e o SITL o segura ate a escrita seguinte
    t_esc, val_esc = escada
    idx = np.searchsorted(t_esc, t_sim, side="right") - 1
    d = np.where(idx >= 0, val_esc[np.clip(idx, 0, len(val_esc) - 1)], 0.0)

    w = th = 0.0
    w_f = 0.0
    fila = [0.0] * (atraso_lacos + 1)
    u = 0.0
    ang = np.empty(n)
    for k in range(n):
        w_f = giro.aplicar(w)
        if k % passos_por_laco == 0:
            alvo_taxa = (raiz_quadrada(-th, g["ang_p"], lim, dt_laco) if raiz
                         else g["ang_p"] * (-th))
            fila.append(pid.atualizar(alvo_taxa, w_f))
            u = fila.pop(0)
        # Euler explicito, como SIM_Aircraft::update_dynamics
        w += (-a * w + b * u + d[k]) * dt
        th += w * dt
        ang[k] = th
    return np.degrees(np.interp(t_grade, t_sim, ang))


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

    saida = {"pasta": pasta, "controlador": nome_ctl, "v_repouso": vr, "eixos": {}}
    for eixo, (i, *_resto) in EIXOS.items():
        col_tq, col_ang, col_erro = _resto[3], _resto[4], _resto[5]
        amp = float(vento[{"roll": "WND_TQX", "pitch": "WND_TQY", "yaw": "WND_TQZ"}[eixo]])
        if amp == 0 or col_tq not in col:
            continue
        a, b = -A[i, i], B[i, i]
        g = ganhos(p, prefixo, eixo)
        ang_sim = simular(t_g, (wnd["t"], wnd[col_tq]), a, b, g, eixo,
                          raiz=(nome_ctl == "nativo"),
                          fs_fisica=float(p.get("SIM_RATE_HZ", 1200)),
                          fs_laco=float(p.get("SCHED_LOOP_RATE", 400)),
                          fc_giro=float(p.get("INS_GYRO_FILTER", 20)),
                          atraso_lacos=atraso_lacos)
        # erro = alvo - medida; o alvo e o angulo de equilibrio antes do torque
        erro_sitl = col[col_erro][sel]
        base = np.nanmean(erro_sitl[t_g < t_g[0] + 0.9])
        erro_sim = base - ang_sim
        r = {"a": a, "b": b, "ganhos": g, "t": t_g, "erro_sitl": erro_sitl,
             "erro_sim": erro_sim, "torque": col[col_tq][sel]}
        if modo & 64:
            f0, f1 = float(vento["WND_TQF0"]), float(vento["WND_TQF1"])
            dentro = (tp[sel] >= t_tq0) & (tp[sel] <= t_tq0 + float(vento["WND_TQDUR"]))
            u = col[col_tq][sel][dentro]
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
            m_sim = M.metricas_disturbio(t_g, erro_sim, jan)
            m_sitl = met.get(f"disturbio_{eixo}", {})
            r["metricas"] = {"sim": m_sim, "sitl": m_sitl}
        saida["eixos"][eixo] = r
    return saida


def cenario_de(pasta):
    """resultados/<grupo>/<ensaio>/<controlador>/<carimbo> -> 'grupo/ensaio'."""
    rel = pasta.relative_to(RESULTADOS)
    return f"{rel.parts[0]}/{rel.parts[1]}"


def ensaios_mais_recentes(desde=None):
    pastas = {}
    for met in RESULTADOS.rglob("metricas.json"):
        pasta = met.parent
        if pasta.name.endswith("_falhou") or "autotune" in pasta.parts:
            continue
        rel = pasta.relative_to(RESULTADOS).parts
        if len(rel) != 4 or rel[1] not in ENSAIOS_TORQUE:
            continue
        if desde and pasta.name < desde:
            continue
        chave = (rel[0], rel[1], rel[2])
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


def escrever_tabela(resultados, destino):
    """Tabela LaTeX (booktabs) dos ensaios de pulso e degrau."""
    linhas = []
    ordem_eixo = {"roll": 0, "pitch": 1, "yaw": 2}
    def chave(item):
        r, eixo = item
        grupo, ensaio = cenario_de(r["pasta"]).split("/")
        return (grupo == "rolagem_arfagem_guinada", ensaio != "torque_pulso",
                ordem_eixo[eixo], r["controlador"] != "nativo")
    pares = sorted(((r, eixo) for r in resultados for eixo, e in r["eixos"].items()
                    if "metricas" in e), key=chave)
    for r, eixo in pares:
        e = r["eixos"][eixo]
        s, m = e["metricas"]["sitl"], e["metricas"]["sim"]
        cen = cenario_de(r["pasta"])
        grupo, ensaio = cen.split("/")
        tipo = {"torque_pulso": "pulso", "torque_degrau": "degrau"}[ensaio]
        ctl = "nativo" if r["controlador"] == "nativo" else "\\codigo{cc\\_pid}"
        simult = " (3 eixos)" if grupo == "rolagem_arfagem_guinada" else ""
        linhas.append(
            f"{NOMES[eixo].capitalize()}{simult} & {tipo} & {ctl} & "
            f"{_fmt(s.get('desvio_max_deg'))} & {_fmt(m.get('desvio_max_deg'))} & "
            f"{_fmt(s.get('tempo_recuperacao_s'))} & {_fmt(m.get('tempo_recuperacao_s'))} & "
            f"{_fmt(s.get('rmse_deg'), 3)} & {_fmt(m.get('rmse_deg'), 3)} \\\\")
    corpo = "\n".join(linhas)
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


def desenhar(resultados, destino):
    """Erro de atitude no SITL e no modelo, nos ensaios de pulso e degrau do nativo."""
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
            if (cenario_de(r["pasta"]) == cen and r["controlador"] == "nativo"
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


def desenhar_frf(resultados, destino):
    """Resposta em frequencia torque -> angulo, SITL e modelo, nas varreduras do nativo."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker

    azul, laranja = "#2a78d6", "#eb6834"
    tinta, tinta2, grade = "#0b0b0b", "#52514e", "#e4e3df"
    ordem = {"rolagem": 0, "guinada": 1}
    casos = sorted(((cenario_de(r["pasta"]), eixo, e) for r in resultados
                    if r["controlador"] == "nativo" for eixo, e in r["eixos"].items()
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
        # malha continua sem filtros: s^3 + (a + b kp) s^2 + b (kp k + ki) s + b ki k
        polos = np.roots([1.0, a + b * g["kp"], b * (g["kp"] * g["ang_p"] + g["ki"]),
                          b * g["ki"] * g["ang_p"]])
        lento = max(p.real for p in polos if abs(p.imag) < 1e-9)
        print(f"guinada/torque_degrau {r['controlador']:7s}: polo lento {lento:.4f} 1/s "
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
    args = ap.parse_args()

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
    if args.figuras:
        for nome, fn in (("validacao_tempo.pdf", desenhar), ("validacao_freq.pdf", desenhar_frf)):
            d = fn(resultados, args.figuras / nome)
            if d:
                print(f"figura: {d}")


if __name__ == "__main__":
    main()
