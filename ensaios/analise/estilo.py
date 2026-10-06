"""Paleta e estilo comuns ao painel e ao GIF.

As figuras deste projeto vao para a monografia impressa, entao o alvo e um unico
tema claro, sem modo escuro. A paleta categorica foi validada para daltonismo:
nas quatro fatias usadas aqui, o pior par adjacente fica em Delta E 9.1 (protan) e
22.9 (visao normal), acima dos pisos de 8 e 15.

Tres das cores ficam abaixo de 3:1 de contraste contra o fundo claro, o que exige
alivio: por isso toda faixa com mais de uma serie leva legenda, e a figura carrega
a tabela de metricas ao lado — a identidade nunca depende so da cor.
"""

from __future__ import annotations

import matplotlib as mpl
import numpy as np

# categoricas, em ordem fixa — nunca cicladas
SERIE = ["#2a78d6",   # 1 azul
         "#eb6834",   # 2 laranja
         "#1baf7a",   # 3 agua
         "#eda100"]   # 4 amarelo

# tinta e superficie
SUPERFICIE = "#fcfcfb"
TINTA = "#0b0b0b"
TINTA_2 = "#52514e"
MUDO = "#898781"
GRADE = "#e6e5e1"

# a referencia (o que foi comandado) e desenhada como linha neutra tracejada:
# nao e uma serie par das medidas, e o alvo contra o qual elas sao lidas
REFERENCIA = MUDO

# realce da janela de disturbio
FAIXA_RAJADA = "#eb6834"
FAIXA_ALPHA = 0.07

LARGURA_DADO = 1.4
LARGURA_REF = 1.2


def aplicar():
    """Aplica o estilo global do projeto ao matplotlib."""
    mpl.rcParams.update({
        "figure.facecolor": SUPERFICIE,
        "axes.facecolor": SUPERFICIE,
        "savefig.facecolor": SUPERFICIE,
        "axes.edgecolor": MUDO,
        "axes.labelcolor": TINTA_2,
        "axes.titlecolor": TINTA,
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "axes.axisbelow": True,
        "grid.color": GRADE,
        "grid.linewidth": 0.7,
        "xtick.color": MUDO,
        "ytick.color": MUDO,
        "xtick.labelcolor": TINTA_2,
        "ytick.labelcolor": TINTA_2,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "text.color": TINTA,
        "font.size": 9,
        "axes.titlesize": 9.5,
        "axes.labelsize": 8.5,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "lines.linewidth": LARGURA_DADO,
        "lines.solid_capstyle": "round",
        "figure.dpi": 110,
    })


def limpar_eixo(ax, ultimo=False):
    """Deixa grade e eixos recessivos: sem molduras superiores e laterais."""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(MUDO)
    ax.spines["bottom"].set_color(MUDO)
    ax.grid(True, axis="y")
    ax.grid(False, axis="x")
    if not ultimo:
        ax.tick_params(labelbottom=False)


def titulo_faixa(ax, texto, unidade=None):
    """Titulo curto a esquerda, no lugar de um ylabel girado (mais legivel)."""
    rotulo = f"{texto}  [{unidade}]" if unidade else texto
    ax.set_title(rotulo, loc="left", pad=6)


def legenda(ax, ncols=3, handles=None):
    """Legenda na linha do titulo, alinhada a direita.

    Dentro da area de plotagem ela acabava por cima do sinal — e justamente nos
    ensaios mais agitados, que sao os que mais precisam ser lidos. ``handles``
    permite juntar na mesma legenda as linhas de um eixo gemeo (twinx).
    """
    extra = {} if handles is None else {"handles": handles}
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0),
              ncols=ncols, borderaxespad=0.0, handlelength=1.6,
              columnspacing=1.4, handletextpad=0.5, **extra)


# ---------------------------------------------------------------------------
# escala divergente para a saida dos motores
# ---------------------------------------------------------------------------
# A saida de motor e lida como DESVIO em torno da media instantanea: dois
# motores acima e dois abaixo e o que produz torque. Isso e polaridade, nao
# magnitude, entao a escala e divergente — dois polos e um cinza neutro no meio.
# Uma rampa sequencial diria "muito/pouco" quando a pergunta e "para que lado".
DIVERGENTE_FRIO = "#2a78d6"     # abaixo da media
DIVERGENTE_NEUTRO = "#f0efec"   # na media
DIVERGENTE_QUENTE = "#d03b3b"   # acima da media


def cor_motor(desvio, escala):
    """Cor de um motor a partir do desvio em relacao a media, em PWM.

    ``escala`` e o desvio que satura a cor. Devolve um RGB interpolado entre o
    polo frio, o neutro e o polo quente.
    """
    import matplotlib.colors as mcolors
    if escala <= 1e-6:
        return DIVERGENTE_NEUTRO
    f = max(-1.0, min(1.0, desvio / escala))
    a = mcolors.to_rgb(DIVERGENTE_NEUTRO)
    b = mcolors.to_rgb(DIVERGENTE_QUENTE if f >= 0 else DIVERGENTE_FRIO)
    k = abs(f)
    return tuple(a[i] + (b[i] - a[i]) * k for i in range(3))


# Geometria do frame de quadrotor usado no projeto: angulo de cada motor a partir
# do nariz, sentido horario (AP_MotorsMatrix.cpp:592). A ordem da lista e a ordem
# das saidas RCOU C1..C4.
FRAMES_QUAD = {
    1: ("X", [45.0, -135.0, -45.0, 135.0]),   # 1=diant.dir 2=tras.esq 3=diant.esq 4=tras.dir
}

# Sentido de giro de cada motor, na mesma ordem: +1 = anti-horario (CCW), -1 =
# horario (CW), os AP_MOTORS_MATRIX_YAW_FACTOR_* do AP_MotorsMatrix.cpp:592. E
# daqui que sai o momento de GUINADA: o torque de reacao de um rotor CCW gira a
# aeronave no sentido horario, que e guinada positiva. Acelerar o par CCW e
# desacelerar o par CW, portanto, guina para a direita.
GIRO_QUAD = {
    1: [+1, +1, -1, -1],                      # 1 e 2 CCW, 3 e 4 CW
}


def geometria_quad(frame_type):
    """(nome, angulos) do frame do ensaio.

    Levanta erro em vez de adivinhar: a geometria define a formula do torque de
    rolagem e onde cada rotor e desenhado. Um palpite errado nao quebra nada —
    apenas produz um numero plausivel e falso, que e pior.
    """
    ft = int(frame_type)
    if ft not in FRAMES_QUAD:
        raise ValueError(
            f"FRAME_TYPE={ft} sem geometria conhecida. O projeto voa em X "
            f"(FRAME_TYPE=1); para analisar outro frame, acrescente os angulos "
            f"dos motores em estilo.FRAMES_QUAD, conforme AP_MotorsMatrix.cpp.")
    return FRAMES_QUAD[ft]


def giro_quad(frame_type):
    """Sentido de giro de cada motor (+1 CCW, -1 CW), na ordem de RCOU C1..C4.

    Mesma politica de ``geometria_quad``: frame desconhecido e erro, nao palpite.
    """
    ft = int(frame_type)
    if ft not in GIRO_QUAD:
        raise ValueError(
            f"FRAME_TYPE={ft} sem sentido de giro conhecido; acrescente-o em "
            f"estilo.GIRO_QUAD, conforme AP_MotorsMatrix.cpp.")
    return GIRO_QUAD[ft]


def diferencial_motores(col, frame_type, eixo):
    """Diferencial de PWM que produz o momento do ``eixo``, ou None.

    Rolagem e arfagem saem da geometria (seno e cosseno do angulo de cada
    motor); guinada, do sentido de giro. A normalizacao e a mesma do painel
    original: metade da soma dos modulos dos pesos, de modo que o numero e o
    quanto um lado esta acima do outro, em PWM.
    """
    if not all(f"m{k}" in col for k in (1, 2, 3, 4)):
        return None
    _, angulos = geometria_quad(frame_type)
    if eixo == "roll":
        pesos = np.sin(np.radians(angulos))
    elif eixo == "pitch":
        # arfagem positiva levanta o nariz: os motores dianteiros sobem
        pesos = np.cos(np.radians(angulos))
    elif eixo == "yaw":
        pesos = np.asarray(giro_quad(frame_type), dtype=float)
    else:
        return None
    norma = float(np.sum(np.abs(pesos))) or 1.0
    return sum(col[f"m{k + 1}"] * pesos[k] for k in range(4)) / norma * 2.0
