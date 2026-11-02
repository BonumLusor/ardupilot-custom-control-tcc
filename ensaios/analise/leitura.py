"""Leitura de logs .BIN do ArduPilot para arrays numpy.

Converte o dataflash bruto em tabelas de series temporais alinhadas numa unica
grade de tempo, que e a entrada de todo o resto da analise (metricas, painel,
GIF e tabelas).

Mensagens de interesse, com os nomes de campo conferidos contra logs reais deste
projeto:

    ANG    DesRoll, Roll, DesPitch, Pitch, DesYaw, Yaw     atitude na taxa do laco
    ATT    os mesmos campos, mas sempre a 10 Hz (reserva, quando nao ha ANG)
    RATE   RDes, R, ROut, PDes, P, POut, YDes, Y, YOut
    PIDR   Tar, Act, Err, P, I, D, FF          (idem PIDP, PIDY)
    CCLR   Eth, Ew, XI, Uff, U, Wh, Dh         backend LQR (idem CCLP, CCLY)
    XKF1   PN, PE, PD, VN, VE, VD              posicao/velocidade em NED
    POS    Lat, Lng, Alt, RelHomeAlt           reserva, quando nao ha XKF1
    RCOU   C1..C4                              saida dos motores
    CC     Type, Act                           estado do AC_CustomControl
    WND    Wn, We, Wd, Spd, Dir            vento injetado (ver lua/vento.lua)
           Tx, Ty, Tz                     torque injetado, rad/s^2 no corpo
           Tp, Mode                       tempo do perfil e bitmask do modo
"""

from __future__ import annotations

import numpy as np
from pymavlink import DFReader

from .eixos import CAMPOS_CCL, CAMPOS_PID, COLUNAS, EIXOS

# Mensagens lidas por padrao. WND so existe em ensaios com o injetor ativo.
TIPOS_PADRAO = ("ANG", "ATT", "RATE", "PIDR", "PIDP", "PIDY", "XKF1", "POS",
                "RCOU", "CC", "WND", "MODE", "CCLR", "CCLP", "CCLY")


def ler_log(caminho, tipos=TIPOS_PADRAO):
    """Le um .BIN e devolve {tipo: {campo: array}}.

    Cada tabela ganha um campo ``t`` em segundos, com a origem no primeiro
    registro do log, para que todas as mensagens compartilhem o mesmo zero.
    """
    log = DFReader.DFReader_binary(str(caminho))
    alvo = set(tipos)
    bruto = {}

    while True:
        msg = log.recv_msg()
        if msg is None:
            break
        tipo = msg.get_type()
        if tipo not in alvo:
            continue
        destino = bruto.setdefault(tipo, {})
        for campo in msg.get_fieldnames():
            destino.setdefault(campo, []).append(getattr(msg, campo))

    dados = {}
    t0 = None
    for tipo, campos in bruto.items():
        tabela = {}
        for campo, valores in campos.items():
            try:
                tabela[campo] = np.asarray(valores, dtype=float)
            except (ValueError, TypeError):
                # campos textuais (MODE.Mode, por exemplo) seguem como objeto
                tabela[campo] = np.asarray(valores, dtype=object)
        if "TimeUS" in tabela:
            inicio = tabela["TimeUS"][0]
            t0 = inicio if t0 is None else min(t0, inicio)
        dados[tipo] = tabela

    if t0 is not None:
        for tabela in dados.values():
            if "TimeUS" in tabela:
                tabela["t"] = (tabela["TimeUS"] - t0) * 1e-6

    return dados


def _interp(grade, t, y):
    """Interpola y(t) na grade, sem extrapolar alem dos extremos medidos."""
    if t is None or len(t) < 2:
        return np.full_like(grade, np.nan)
    fora = (grade < t[0]) | (grade > t[-1])
    out = np.interp(grade, t, y)
    out[fora] = np.nan
    return out


def torque_segurado(dados, grade):
    """Torque do injetor na grade como a fisica o aplica: segurado ate a escrita seguinte.

    Devolve {"tqx", "tqy", "tqz"}. As colunas de mesmo nome da serie_unificada
    sao interpoladas entre as escritas do WND, a cada ~22,5 ms (o laco do Lua a
    ~44 Hz), e isso adianta a escada em meio passo: numa resposta theta/d, a fase
    sai atrasada em pi*f*T (cerca de -16 graus a 4 Hz e -35 graus a 8 Hz) e o
    modulo sobe ate 5 a 9 % a 8 Hz. Para o tempo das metricas a diferenca nao
    importa; para a resposta em frequencia, sim. Fora do trecho do WND, NaN.
    """
    wnd = dados.get("WND")
    if wnd is None or "t" not in wnd or len(wnd["t"]) < 2:
        return {}
    t = np.asarray(wnd["t"], dtype=float)
    i = np.searchsorted(t, grade, side="right") - 1
    fora = (i < 0) | (grade > t[-1])
    i = np.clip(i, 0, len(t) - 1)
    out = {}
    for campo, nome in (("Tx", "tqx"), ("Ty", "tqy"), ("Tz", "tqz")):
        if campo in wnd:
            y = np.asarray(wnd[campo], dtype=float)[i]
            y[fora] = np.nan
            out[nome] = y
    return out


def _embrulhar180(graus):
    """Angulo em [-180, 180)."""
    return (np.asarray(graus, dtype=float) + 180.0) % 360.0 - 180.0


def _rumo_de_referencia(rumo_continuo, tp):
    """Rumo travado no inicio do perfil: o zero da guinada relativa.

    E a mediana do rumo COMANDADO no primeiro meio segundo do perfil, quando o
    orquestrador ja trava o rumo e nenhum degrau foi pedido. Sem a coluna ``tp``
    (log sem WND), vale a primeira amostra.
    """
    ok = np.isfinite(rumo_continuo)
    if not ok.any():
        return None
    if tp is not None:
        sel = ok & (tp >= 0.0) & (tp <= 0.5)
        if sel.any():
            return float(np.median(rumo_continuo[sel]))
    return float(rumo_continuo[ok][0])


def origem_perfil(dados):
    """Instante, em tempo de log, em que o perfil de vento zerou (Tp = 0).

    O injetor grava o tempo do PERFIL no campo ``Tp``, entao a origem sai por
    subtracao direta. Ter essa origem e o que permite localizar a rajada no log
    sem depender de quanto durou a decolagem — que varia de corrida para corrida.
    """
    wnd = dados.get("WND")
    if wnd is None or "Tp" not in wnd or len(wnd.get("t", [])) == 0:
        return None
    return float(wnd["t"][0] - wnd["Tp"][0])


def janela_perfil(dados, folga=2.0, duracao_s=None):
    """Intervalo (t_ini, t_fim) do ensaio, com uma folga de cada lado.

    Quando o log traz ``Tp`` e o cenario informa a duracao, a janela e a do PERFIL
    COMANDADO — nao a da mensagem WND. A diferenca importa: o injetor continua
    escrevendo durante o pouso, e integrar o esforco de controle por cima do pouso
    nao mede o controlador de atitude, mede o pouso.
    """
    wnd = dados.get("WND")
    origem = origem_perfil(dados)
    if origem is not None and duracao_s:
        fim_wnd = float(wnd["t"][-1])
        #  A folga existe so ANTES do perfil, para mostrar o trecho calmo que o
        #  precede. Depois dele nao ha folga: o instante em que o perfil termina e
        #  o instante em que o pouso comeca, e em LAND o controlador de posicao
        #  assume e comanda atitude para frear a aeronave. Medido aqui, deixar
        #  2 s de pouso na janela inflava a integral de esforco em 5,8%.
        return origem - folga, min(origem + float(duracao_s), fim_wnd)
    if wnd is not None and len(wnd.get("t", [])) > 1:
        return float(wnd["t"][0] - folga), float(wnd["t"][-1] + folga)
    att = dados.get("ATT")
    if att is not None and len(att.get("t", [])) > 1:
        return float(att["t"][0]), float(att["t"][-1])
    return 0.0, 0.0


def serie_unificada(dados, taxa_hz=400.0, msg_pid="PIDR", janela=None,
                    duracao_s=None):
    """Reamostra tudo numa grade unica e devolve (grade, {coluna: array}).

    E esta tabela que vira o ``series.csv`` do ensaio e alimenta o painel, o GIF
    e as metricas — todas as colunas compartilham o mesmo eixo de tempo, entao
    qualquer comparacao entre elas e direta.

    Os nomes das colunas de cada eixo estao em ``eixos.COLUNAS``. Na guinada,
    ``yaw``/``yaw_des`` sao o rumo absoluto em [0, 360) e ``yaw_rel``/
    ``yaw_des_rel`` sao os mesmos angulos relativos ao rumo do inicio do perfil.

    A grade padrao e de 400 Hz, a taxa do laco de controle. Com os ganhos do
    AutoTune, um degrau de rolagem sobe em 60 ms; numa grade de 50 Hz, o pico do
    sobressinal cabe entre duas amostras.
    """
    if janela is None:
        janela = janela_perfil(dados, duracao_s=duracao_s)
    t_ini, t_fim = janela
    if t_fim <= t_ini:
        return np.zeros(0), {}

    grade = np.arange(t_ini, t_fim, 1.0 / taxa_hz)
    col = {}

    def pegar(tipo, campo, nome, escala=1.0, angular=False):
        tab = dados.get(tipo)
        if tab is None or campo not in tab or "t" not in tab:
            return
        y = tab[campo].astype(float)
        if angular:
            # Rumo desdobrado ANTES de interpolar. Interpolando o [0, 360) cru, a
            # passagem 359 -> 1 vira uma rampa de 358 graus entre duas amostras,
            # e um degrau de guinada que cruza o norte poe um pico falso de erro
            # bem no meio das metricas.
            y = np.degrees(np.unwrap(np.radians(y)))
        col[nome] = _interp(grade, tab["t"], y) * escala

    # atitude (graus). No ArduCopter 4.8 a ATT e escrita SEMPRE a 10 Hz
    # (Copter::ten_hz_logging_loop); com o bit ATTITUDE_FAST do LOG_BITMASK, a
    # atitude alvo e a medida na taxa do laco vao na ANG (Write_ANG), com os
    # mesmos campos. Interpolar a ATT de 10 Hz reduzia um transitorio de 0,1 s a
    # uma ou duas amostras.
    fonte_att = "ANG" if "Roll" in dados.get("ANG", {}) else "ATT"
    pegar(fonte_att, "DesRoll", "roll_des")
    pegar(fonte_att, "Roll", "roll")
    pegar(fonte_att, "DesPitch", "pitch_des")
    pegar(fonte_att, "Pitch", "pitch")
    pegar(fonte_att, "DesYaw", "yaw_des", angular=True)
    pegar(fonte_att, "Yaw", "yaw", angular=True)

    # Por eixo: taxas no corpo (graus/s), saida do controlador de taxa
    # (normalizada) e os termos do PID de taxa. Os de rolagem mantem os nomes de
    # sempre (pid_p, pid_i, ...) e vem da mensagem que o YAML do controlador
    # indicar; arfagem e guinada usam PIDP e PIDY.
    for eixo in EIXOS:
        c = COLUNAS[eixo]
        for campo, nome in zip(c["rate"], (c["taxa_ref"], c["taxa"], c["saida"])):
            pegar("RATE", campo, nome)
        msg = msg_pid if eixo == "roll" else c["msg_pid"]
        for campo, nome in CAMPOS_PID:
            pegar(msg, campo, c["pid"] + nome)
        # sinais internos do backend LQR (so nos voos com CC_TYPE 3)
        for campo, nome in CAMPOS_CCL:
            pegar(c["msg_ccl"], campo, f"{nome}_{eixo}")

    # posicao e velocidade em NED; XKF1 e a fonte preferida
    if "XKF1" in dados:
        for campo, nome in (("PN", "pn"), ("PE", "pe"), ("PD", "pd"),
                            ("VN", "vn"), ("VE", "ve"), ("VD", "vd")):
            pegar("XKF1", campo, nome)
    elif "POS" in dados:
        pegar("POS", "RelHomeAlt", "alt")

    # saida dos motores
    for i in range(1, 5):
        pegar("RCOU", f"C{i}", f"m{i}")

    # vento injetado
    for campo, nome in (("Wn", "wn"), ("We", "we"), ("Wd", "wd"),
                        ("Spd", "w_spd"), ("Dir", "w_dir"),
                        ("Tx", "tqx"), ("Ty", "tqy"), ("Tz", "tqz")):
        pegar("WND", campo, nome)
    if "wn" in col and "we" in col:
        col["w_mod"] = np.hypot(col["wn"], col["we"])
    if "tqx" in col:
        col["tq_mod"] = np.sqrt(col["tqx"] ** 2 + col["tqy"] ** 2 + col["tqz"] ** 2)

    # tempo do PERFIL na grade: e por ele que a rajada e localizada, e nao por um
    # deslocamento suposto a partir do armamento
    origem = origem_perfil(dados)
    if origem is not None:
        col["tp"] = grade - origem

    # erros de atitude, as colunas que as metricas mais usam
    for eixo in ("roll", "pitch"):
        c = COLUNAS[eixo]
        if c["ref"] in col and c["med"] in col:
            col[c["erro"]] = col[c["ref"]] - col[c["med"]]

    #  Guinada RELATIVA ao rumo travado no inicio do perfil. E ela que entra nas
    #  metricas: o rumo absoluto comeca em qualquer valor (352 graus, nos ensaios
    #  deste projeto), e o detector de degrau precisa de um patamar em torno de
    #  zero, como na rolagem. O erro e embrulhado em +-180 a partir do rumo cru,
    #  e a medida relativa sai de "comandada - erro": assim as duas curvas ficam
    #  no mesmo ramo de 360 graus mesmo que os desdobramentos partam de lados
    #  opostos do norte (359,9 e 0,1, por exemplo).
    if "yaw_des" in col and "yaw" in col:
        erro = _embrulhar180(col["yaw_des"] - col["yaw"])
        ref = _rumo_de_referencia(col["yaw_des"], col.get("tp"))
        if ref is not None:
            col["yaw_des_rel"] = col["yaw_des"] - ref
            col["yaw_rel"] = col["yaw_des_rel"] - erro
        col["erro_yaw"] = erro
    # o rumo absoluto volta para [0, 360), como vem no log
    for nome in ("yaw_des", "yaw"):
        if nome in col:
            col[nome] = np.mod(col[nome], 360.0)

    return grade, col


def controlador_ativo(dados, janela=None):
    """(tipo, fracao_ativa_na_janela) do AC_CustomControl, ou (0, 0.0) se ausente.

    Serve para confirmar, a partir do proprio log, que o ensaio rodou com o
    controlador que o YAML pediu — e nao com o nativo porque a chave auxiliar de
    RC nunca subiu.

    A mensagem CC so e gravada nas COMUTACOES (``log_switch``), nao periodicamente.
    Um voo tipico tem tres registros: OFF na inicializacao, ON quando a chave sobe,
    e OFF de novo quando o override de RC expira no pouso. Por isso a fracao e
    calculada com RETENCAO DE ORDEM ZERO sobre a JANELA DO ENSAIO — contar
    registros diria "50% ativo" para um voo inteiramente com o controlador ligado,
    e olhar so o ultimo registro diria "inativo" por causa do pouso.
    """
    cc = dados.get("CC")
    if cc is None or "Type" not in cc or len(cc["t"]) == 0:
        return 0, 0.0
    tipo = int(np.round(np.median(cc["Type"])))

    t_cc = np.asarray(cc["t"], dtype=float)
    act = np.asarray(cc["Act"], dtype=float) > 0.5
    if janela is None:
        return tipo, float(act[-1])

    t_ini, t_fim = janela
    if t_fim <= t_ini:
        return tipo, float(act[-1])
    grade = np.linspace(t_ini, t_fim, 500)
    # retencao de ordem zero: o estado vale ate a proxima comutacao
    idx = np.searchsorted(t_cc, grade, side="right") - 1
    dentro = idx >= 0
    if not dentro.any():
        return tipo, 0.0
    return tipo, float(np.mean(act[idx[dentro]]))


def ler_params(caminho):
    """Le o params.parm de um ensaio e devolve {nome: valor}.

    Serve para a analise saber o que estava valendo NAQUELE voo — o tipo de frame,
    por exemplo, que define onde cada motor fica e sem o qual a vista de cima
    desenharia os rotores no lugar errado.
    """
    vals = {}
    try:
        with open(caminho, encoding="utf-8") as f:
            for linha in f:
                partes = linha.strip().split(",")
                if len(partes) == 2:
                    try:
                        vals[partes[0]] = float(partes[1])
                    except ValueError:
                        pass
    except OSError:
        pass
    return vals
