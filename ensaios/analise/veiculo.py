"""Parametros do veiculo e do controlador de um ensaio, lidos do params.parm.

E o que a monografia precisa declarar sobre a planta e sobre o controlador —
frame, massa, geometria, inercia, curva dos motores, ganhos em uso — e sai do
PARM gravado no proprio .BIN: o que estava valendo NAQUELE voo, e nao o que o
YAML ou a documentacao dizem que deveria estar.

DUAS PECULIARIDADES DO MODELO DO SITL, conferidas no codigo e nos logs:

1. ``SIM_FRM_DIAG_SZ`` e documentado como "distancia diagonal motor a motor",
   mas o ``SIM_Motor.cpp:206`` poe CADA MOTOR a essa distancia do CENTRO. O
   braco efetivo e o proprio DIAG_SZ, e a diagonal efetiva e o dobro. Conferido
   no log do rolagem/torque_degrau: sob 10 rad/s2 sustentados, o diferencial
   medido dos motores devolve ~10,5 rad/s2 de contra-torque com braco = DIAG_SZ,
   e so 5,2 com braco = DIAG_SZ/2.
2. Sem modelo JSON, a inercia e ESTIMADA pelo SITL (``SIM_Frame.cpp:823``) como
   metade da massa num anel de raio DIAG_SZ/2: Ixx = Iyy = m*(DIAG_SZ/2)^2/4 e
   Izz = m*(DIAG_SZ/2)^2/2. O raio da inercia e metade do braco do torque —
   incoerente como geometria, mas e a planta que o SITL integra, e e com ela
   que o modelo do TCC tem de bater.

Os ensaios rodam ``-f X`` sem modelo JSON; com um JSON, posicao dos motores e
inercia viriam do arquivo e as contas abaixo deixariam de valer.
"""

from __future__ import annotations

import math

from .eixos import COLUNAS, EIXOS, nome
from . import estilo

G = 9.80665

CLASSES_FRAME = {1: "quad", 2: "hexa", 3: "octa", 4: "octaquad", 5: "Y6",
                 7: "tri", 12: "dodecahexa", 14: "deca"}
TIPOS_FRAME = {0: "+", 1: "X", 2: "V", 3: "H", 4: "V-tail", 5: "A-tail",
               12: "BetaFlight X", 13: "DJI X", 14: "X horario", 18: "BetaFlight X rev."}

# ganhos de cada eixo, na ordem do bloco do resumo: (chave, rotulo, parametro).
# O parametro leva {s} no lugar de RLL/PIT/YAW e vem depois do prefixo da fonte
# (ATC_ no nativo, CC2_ no backend PID do AC_CustomControl).
GANHOS = (
    ("ang_p",    "angulo P",            "ANG_{s}_P"),
    ("rat_p",    "taxa P",              "RAT_{s}_P"),
    ("rat_i",    "taxa I",              "RAT_{s}_I"),
    ("rat_d",    "taxa D",              "RAT_{s}_D"),
    ("rat_ff",   "taxa FF",             "RAT_{s}_FF"),
    ("rat_imax", "taxa IMAX",           "RAT_{s}_IMAX"),
    ("rat_fltt", "filtro do alvo (Hz)", "RAT_{s}_FLTT"),
    ("rat_flte", "filtro do erro (Hz)", "RAT_{s}_FLTE"),
    ("rat_fltd", "filtro do D (Hz)",    "RAT_{s}_FLTD"),
)

# o que vai para o metricas.json e, por ele, para o consolidado.csv
GANHOS_NO_CONSOLIDADO = ("fonte", "ang_p", "rat_p", "rat_i", "rat_d", "rat_ff")
VEICULO_NO_CONSOLIDADO = ("frame", "massa_kg", "braco_m", "diagonal_m",
                          "ixx_kgm2", "iyy_kgm2", "izz_kgm2")

LARGURA_ROTULO = 28


def _n(v, casas=4):
    """Numero com ``casas`` algarismos significativos, ou travessao."""
    if v is None:
        return "—"
    if isinstance(v, float) and v.is_integer() and abs(v) < 1e6:
        return str(int(v))
    return f"{v:.{casas}g}"


def _linha(rotulo, texto):
    return f"  {rotulo:<{LARGURA_ROTULO}}{texto}".rstrip()


def _comando_pairado(hovr, expo):
    """Comando de motor c em [0, 1] em que o empuxo iguala o peso.

    O SITL faz o empuxo proporcional a (1-e)*c + e*c^2, normalizado para que
    esse polinomio valha SIM_FRM_HOVR_THR no pairado (SIM_Motor.cpp,
    calc_thrust). Resolve-se a quadratica.
    """
    if hovr is None or expo is None:
        return None
    if abs(expo) < 1e-9:
        return hovr
    a, b, c = expo, 1.0 - expo, -hovr
    return (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)


def descrever(params, frame_type=None):
    """Grandezas fisicas do veiculo simulado. Ausentes viram None."""
    p = params.get
    classe = p("FRAME_CLASS")
    tipo = p("FRAME_TYPE") if frame_type is None else frame_type
    massa = p("SIM_FRM_MASS")
    diag = p("SIM_FRM_DIAG_SZ")
    hovr = p("SIM_FRM_HOVR_THR")
    expo = p("SIM_FRM_EXPO")

    d = {
        "frame_class": classe, "frame_type": tipo,
        "frame": (f"{CLASSES_FRAME.get(int(classe), f'classe {int(classe)}')} "
                  f"{TIPOS_FRAME.get(int(tipo), f'tipo {int(tipo)}')}")
        if classe is not None and tipo is not None else None,
        "massa_kg": massa,
        "peso_n": massa * G if massa is not None else None,
        "diag_sz_m": diag,
        # braco efetivo = DIAG_SZ (SIM_Motor.cpp:206); ver o cabecalho do modulo
        "braco_m": diag,
        "diagonal_m": 2.0 * diag if diag is not None else None,
        "disco_m2": p("SIM_FRM_DISCAREA"),
        "hovr_thr": hovr, "expo": expo,
        "pwm_min": p("SIM_FRM_PWM_MIN"), "pwm_max": p("SIM_FRM_PWM_MAX"),
        "spin_min": p("SIM_FRM_SPIN_MIN"), "spin_max": p("SIM_FRM_SPIN_MAX"),
        "slew_max": p("SIM_FRM_SLEW_MAX"),
        "bat_volt": p("SIM_FRM_BAT_VOLT"), "bat_cap_ah": p("SIM_FRM_BAT_CAP"),
        "thst_hover": p("MOT_THST_HOVER"),
        "loop_hz": p("SCHED_LOOP_RATE"), "fisica_hz": p("SIM_RATE_HZ"),
        "gyro_filt_hz": p("INS_GYRO_FILTER"), "accel_filt_hz": p("INS_ACCEL_FILTER"),
        "notch": p("INS_HNTCH_ENABLE"),
    }

    n_mot = 4
    angulos = giro = None
    if tipo is not None and classe is not None and int(classe) == 1:
        try:
            _, angulos = estilo.geometria_quad(tipo)
            giro = estilo.giro_quad(tipo)
        except ValueError:
            pass
    d["angulos"], d["giro"] = angulos, giro

    # braco de cada motor em torno dos eixos x e y: e ele que gera momento
    if diag is not None and angulos:
        d["braco_rolagem_m"] = diag * sum(abs(math.sin(math.radians(a))) for a in angulos) / len(angulos)
        d["braco_arfagem_m"] = diag * sum(abs(math.cos(math.radians(a))) for a in angulos) / len(angulos)
    else:
        d["braco_rolagem_m"] = d["braco_arfagem_m"] = None

    # inercia estimada pelo SITL (SIM_Frame.cpp:823)
    if massa is not None and diag is not None:
        r = diag * 0.5
        d["ixx_kgm2"] = d["iyy_kgm2"] = massa * 0.25 * r * r
        d["izz_kgm2"] = massa * 0.5 * r * r
    else:
        d["ixx_kgm2"] = d["iyy_kgm2"] = d["izz_kgm2"] = None

    # empuxo por motor no pairado e no maximo, e o torque de reacao no pairado
    if massa is not None and hovr:
        d["empuxo_pairado_n"] = massa * G / n_mot
        d["empuxo_max_n"] = massa * G / n_mot / hovr
    else:
        d["empuxo_pairado_n"] = d["empuxo_max_n"] = None
    d["comando_pairado"] = _comando_pairado(hovr, expo)
    if diag is not None and d["comando_pairado"] is not None:
        # SIM_Motor.cpp:91 — torque de reacao = 0.05 * DIAG_SZ * empuxo * comando
        d["reacao_nm_por_n"] = 0.05 * diag * d["comando_pairado"]
    else:
        d["reacao_nm_por_n"] = None
    return d


def inercias(desc):
    """{eixo: momento de inercia} para as metricas, ou None se faltar dado."""
    if desc.get("ixx_kgm2") is None:
        return None
    return {"roll": desc["ixx_kgm2"], "pitch": desc["iyy_kgm2"], "yaw": desc["izz_kgm2"]}


def fonte_ganhos(params, eixo):
    """Prefixo dos parametros do controlador que comanda o ``eixo``.

    O AC_CustomControl so substitui o nativo nos eixos de CC_AXIS_MASK; nos
    demais, quem voa e o ATC_ — e um bloco que mostrasse CC2_ nos tres eixos
    mentiria sobre esses.
    """
    tipo = int(params.get("CC_TYPE") or 0)
    mascara = int(params.get("CC_AXIS_MASK") or 0)
    if tipo > 0 and mascara & COLUNAS[eixo]["bit_cc"]:
        return f"CC{tipo}_"
    return "ATC_"


def ganhos_em_uso(params):
    """{eixo: {fonte, ang_p, rat_p, ...}} com os ganhos que voaram em cada eixo."""
    tab = {}
    for eixo in EIXOS:
        fonte = fonte_ganhos(params, eixo)
        g = {"fonte": fonte}
        for chave, _, padrao in GANHOS:
            g[chave] = params.get(fonte + padrao.format(s=COLUNAS[eixo]["sufixo"]))
        tab[eixo] = g
    return tab


def para_metricas(desc, ganhos):
    """Recorte de veiculo e ganhos que vai para o metricas.json.

    Entra no consolidado.csv junto com as metricas: e o que permite ler, numa
    varredura de ganho, qual linha voou com qual ganho — e, nos ensaios de
    robustez da proposta, com qual massa e inercia.
    """
    veic = {k: desc.get(k) for k in VEICULO_NO_CONSOLIDADO if desc.get(k) is not None}
    g = {eixo: {k: v for k, v in d.items() if k in GANHOS_NO_CONSOLIDADO and v is not None}
         for eixo, d in ganhos.items()}
    return veic, g


def _posicoes(angulos, giro):
    """'M1 diant.dir CCW | M2 tras.esq CCW | ...' a partir da geometria."""
    partes = []
    for k, a in enumerate(angulos):
        frente = "diant" if abs(a) < 90 else "tras"
        lado = "dir" if a > 0 else "esq"
        sentido = "" if giro is None else (" CCW" if giro[k] > 0 else " CW")
        partes.append(f"M{k + 1} {frente}.{lado}{sentido}")
    return " | ".join(partes)


def linhas_veiculo(desc):
    """Bloco VEICULO do resumo.txt."""
    d = desc
    L = ["VEICULO  (modelo fisico do SITL, lido do params.parm deste voo)"]
    if d.get("massa_kg") is None and d.get("diag_sz_m") is None:
        L.append("  sem SIM_FRM_* no params.parm: log de outra fonte que nao o SITL puro")

    ft = ("" if d.get("frame_class") is None
          else f"  (FRAME_CLASS={_n(d['frame_class'])}, FRAME_TYPE={_n(d['frame_type'])})")
    L.append(_linha("frame", f"{d.get('frame') or '—'}{ft}"))
    if d.get("angulos"):
        L.append(_linha("motores, vista de cima", _posicoes(d["angulos"], d.get("giro"))))
    if d.get("massa_kg") is not None:
        L.append(_linha("massa", f"{d['massa_kg']:.3f} kg  ->  peso {d['peso_n']:.2f} N"))
    if d.get("diag_sz_m") is not None:
        L.append(_linha("distancia entre eixos",
                        f"{d['diagonal_m']:.3f} m   diagonal motor a motor, como a fisica usa (nota 1)"))
        L.append(_linha("braco (centro -> motor)",
                        f"{d['braco_m']:.3f} m   = SIM_FRM_DIAG_SZ"))
    if d.get("braco_rolagem_m") is not None:
        L.append(_linha("braco de rolagem/arfagem",
                        f"{d['braco_rolagem_m']:.3f} / {d['braco_arfagem_m']:.3f} m   "
                        "o que cada motor tem de alavanca em x / y"))
    if d.get("ixx_kgm2") is not None:
        L.append(_linha("inercia Ixx | Iyy | Izz",
                        f"{d['ixx_kgm2']:.5f} | {d['iyy_kgm2']:.5f} | {d['izz_kgm2']:.5f} kg.m2"
                        "   estimada pelo SITL (nota 2)"))
    if d.get("empuxo_pairado_n") is not None:
        L.append(_linha("empuxo por motor",
                        f"{d['empuxo_pairado_n']:.2f} N no pairado | {d['empuxo_max_n']:.2f} N maximo"
                        f"   (empuxo/peso {1.0 / d['hovr_thr']:.2f})"))
    if d.get("expo") is not None and d.get("comando_pairado") is not None:
        L.append(_linha("curva de empuxo",
                        f"T = Tmax*((1-e)*c + e*c^2), e = {_n(d['expo'], 3)} (SIM_FRM_EXPO); "
                        f"pairado em c = {d['comando_pairado']:.3f}"))
    if d.get("reacao_nm_por_n") is not None:
        L.append(_linha("torque de reacao (guinada)",
                        f"0.05*DIAG_SZ*T*c = {d['reacao_nm_por_n']:.4f} N.m por N de empuxo "
                        "no pairado (SIM_Motor.cpp:91)"))
    if d.get("disco_m2") is not None:
        L.append(_linha("area de disco (total)", f"{_n(d['disco_m2'], 3)} m2"))
    if d.get("pwm_min") is not None:
        L.append(_linha("motores",
                        f"PWM {_n(d['pwm_min'])}..{_n(d['pwm_max'])}, "
                        f"spin {_n(d['spin_min'], 3)}..{_n(d['spin_max'], 3)}, "
                        f"slew {_n(d['slew_max'])} /s"))
    if d.get("thst_hover") is not None:
        modelo = "" if d.get("hovr_thr") is None else f"   (modelo: SIM_FRM_HOVR_THR {_n(d['hovr_thr'], 3)})"
        L.append(_linha("pairado aprendido", f"MOT_THST_HOVER {d['thst_hover']:.3f}{modelo}"))
    if d.get("bat_volt") is not None:
        cap = ("capacidade ilimitada" if not d.get("bat_cap_ah")
               else f"{_n(d['bat_cap_ah'])} Ah")
        L.append(_linha("bateria", f"{_n(d['bat_volt'], 3)} V (SIM_FRM_BAT_VOLT), {cap}"))
    if d.get("loop_hz") is not None:
        L.append(_linha("laco de controle / fisica",
                        f"{_n(d['loop_hz'])} Hz (SCHED_LOOP_RATE) / {_n(d.get('fisica_hz'))} Hz (SIM_RATE_HZ)"))
    if d.get("gyro_filt_hz") is not None:
        notch = "desligado" if not d.get("notch") else "ligado"
        L.append(_linha("filtros",
                        f"giroscopio {_n(d['gyro_filt_hz'])} Hz, acelerometro "
                        f"{_n(d.get('accel_filt_hz'))} Hz, notch harmonico {notch}"))
    if d.get("diag_sz_m") is not None:
        L += ["  nota 1: SIM_FRM_DIAG_SZ se diz \"diagonal motor a motor\", mas o SITL poe cada",
              "          motor a essa distancia do CENTRO (SIM_Motor.cpp:206).",
              "  nota 2: sem modelo JSON, Ixx = Iyy = m*(DIAG_SZ/2)^2/4 e Izz = m*(DIAG_SZ/2)^2/2",
              "          (SIM_Frame.cpp:823): o raio da inercia e metade do braco."]
    return L


def linhas_controlador(params, ctl=None, controlador_no_log=None):
    """Bloco CONTROLADOR EM USO do resumo.txt: os ganhos que voaram, por eixo."""
    p = params.get
    ctl = ctl or {}
    L = [f"CONTROLADOR EM USO  —  {ctl.get('nome', '—')}"]

    tipo = int(p("CC_TYPE") or 0)
    mascara = int(p("CC_AXIS_MASK") or 0)
    if tipo == 0:
        L.append(_linha("AC_CustomControl", "desligado (CC_TYPE=0): os tres eixos com o PID nativo, ATC_*"))
    else:
        eixos_cc = [nome(e) for e in EIXOS if mascara & COLUNAS[e]["bit_cc"]]
        txt = (f"CC_TYPE={tipo}, CC_AXIS_MASK={mascara} "
               f"({', '.join(eixos_cc) if eixos_cc else 'nenhum eixo'})")
        frac = (controlador_no_log or {}).get("fracao_ativa")
        if frac is not None:
            txt += f"; ativo em {frac * 100:.0f}% da janela"
        L.append(_linha("AC_CustomControl", txt))
        if frac is not None and frac < 0.5:
            L.append("  ATENCAO: o backend quase nao assumiu; na maior parte da janela voou o PID nativo (ATC_*)")

    ganhos = ganhos_em_uso(params)
    cab = "".join(f"{nome(e):<13}" for e in EIXOS)
    L.append(_linha("", cab))
    L.append(_linha("ganhos de", "".join(f"{ganhos[e]['fonte']:<13}" for e in EIXOS)))
    for chave, rotulo, _ in GANHOS:
        L.append(_linha(rotulo, "".join(f"{_n(ganhos[e][chave]):<13}" for e in EIXOS)))

    # Eixos com o backend: quanto o P de taxa dele vale em relacao ao nativo. O
    # backend PID de exemplo nasce com 90% dos ganhos COMPILADOS do ArduPilot,
    # mas o SITL carrega copter.parm, que muda ATC_RAT_YAW_P de 0.18 para 0.3 —
    # em guinada a razao cai para ~0.54, e a comparacao deixa de ser "mesmos
    # ganhos" sem que nada no YAML diga isso.
    if any(ganhos[e]["fonte"] != "ATC_" for e in EIXOS):
        razoes = []
        for e in EIXOS:
            nativo = p(f"ATC_RAT_{COLUNAS[e]['sufixo']}_P")
            usado = ganhos[e]["rat_p"]
            if ganhos[e]["fonte"] == "ATC_" or not nativo or usado is None:
                razoes.append(f"{'—':<13}")
            else:
                razoes.append(f"{usado / nativo:<13.2f}")
        L.append(_linha("taxa P / nativo (ATC_)", "".join(razoes)))

    # modelador de entrada e limites: comuns a todos os controladores, porque o
    # AC_CustomControl recebe o alvo ja modelado pela cadeia nativa
    def lim(v):
        return "sem limite" if not v else _n(v)
    L.append(_linha("acel. maxima (deg/s2)",
                    "".join(f"{_n(p(f'ATC_ACC_{s}_MAX')):<13}" for s in ("R", "P", "Y"))
                    + "ATC_ACC_*_MAX"))
    L.append(_linha("taxa maxima (deg/s)",
                    "".join(f"{lim(p(f'ATC_RATE_{s}_MAX')):<13}" for s in ("R", "P", "Y"))
                    + "ATC_RATE_*_MAX"))
    L.append(_linha("modelador de entrada",
                    f"ATC_INPUT_TC {_n(p('ATC_INPUT_TC'))} s (rolagem/arfagem) | "
                    f"PILOT_Y_RATE_TC {_n(p('PILOT_Y_RATE_TC'))} s (guinada)"))
    L.append(_linha("outros",
                    f"ATC_RATE_FF_ENAB {_n(p('ATC_RATE_FF_ENAB'))} | "
                    f"ATC_ANGLE_MAX {_n(p('ATC_ANGLE_MAX'))} deg | "
                    f"MOT_YAW_HEADROOM {_n(p('MOT_YAW_HEADROOM'))}"))

    # Ganhos listados no YAML que a tabela nao cobre (o backend LQR, por
    # exemplo, nao tem RAT_RLL_P), e os parametros de um backend sem estrutura
    # PID: sem eles, o resumo de um controlador novo nao diria com que ganhos voou.
    mostrados = {g["fonte"] + padrao.format(s=COLUNAS[e]["sufixo"])
                 for e, g in ganhos.items() for _, _, padrao in GANHOS}
    extras = [n for n in (ctl.get("ganhos_de_interesse") or [])
              if n not in mostrados and n in params]
    eixos_cc = [e for e in EIXOS if ganhos[e]["fonte"] != "ATC_"]
    if eixos_cc and all(ganhos[e]["rat_p"] is None for e in eixos_cc):
        extras += sorted(n for n in params
                         if n.startswith(f"CC{tipo}_") and n not in extras)
    for n_par in extras:
        L.append(_linha(n_par, _n(params[n_par])))
    return L


def linhas_resumo(params, ctl=None, controlador_no_log=None, frame_type=None):
    """Os dois blocos — veiculo e controlador — prontos para o resumo.txt."""
    desc = descrever(params, frame_type=frame_type)
    return (linhas_veiculo(desc) + [""]
            + linhas_controlador(params, ctl, controlador_no_log) + [""])
