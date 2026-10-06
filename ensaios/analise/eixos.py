"""Eixos de atitude: nomes e colunas da serie unificada de cada um.

Toda a analise (metricas, painel, GIF, tabelas) percorre os eixos por esta
tabela, em vez de repetir o codigo de rolagem para cada eixo novo. A ordem de
EIXOS e a ordem de exibicao em todo lugar.

A guinada tem uma diferenca de fundo em relacao aos outros dois: o ATT grava o
RUMO absoluto, em [0, 360). As colunas que entram nas metricas sao, por isso,
``yaw_des_rel`` e ``yaw_rel``, relativas ao rumo travado no inicio do perfil e
continuas na passagem por 360 -> 0 (ver ``leitura.serie_unificada``). As colunas
``yaw`` e ``yaw_des`` seguem com o rumo absoluto, que e o que o GIF precisa para
orientar a aeronave em relacao ao norte.
"""

from __future__ import annotations

EIXOS = ("roll", "pitch", "yaw")

NOMES = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}

#  ref       atitude comandada (alvo ja modelado pelo firmware), graus
#  med       atitude medida, graus
#  erro      ref - med, graus (na guinada, ja embrulhado em +-180)
#  taxa_ref  taxa comandada no corpo, graus/s (RATE.*Des)
#  taxa      taxa medida no corpo, graus/s
#  saida     saida do controlador de taxa para a mistura, normalizada (RATE.*Out)
#  pid       prefixo das colunas com os termos do PID de taxa (Tar, P, I, D, FF...)
#  msg_pid   mensagem do log com esses termos
#  torque    coluna do torque injetado no eixo, rad/s2
#  sufixo    trecho do nome dos parametros do ArduPilot (ATC_RAT_RLL_P, ...)
#  bit_cc    bit do eixo em CC_AXIS_MASK
#  rate      campos (comandada, medida, saida) da mensagem RATE
COLUNAS = {
    "roll": dict(ref="roll_des", med="roll", erro="erro_roll",
                 taxa_ref="p_des", taxa="p", saida="u_roll",
                 pid="pid_", msg_pid="PIDR", torque="tqx",
                 sufixo="RLL", bit_cc=1, rate=("RDes", "R", "ROut")),
    "pitch": dict(ref="pitch_des", med="pitch", erro="erro_pitch",
                  taxa_ref="q_des", taxa="q", saida="u_pitch",
                  pid="pidp_", msg_pid="PIDP", torque="tqy",
                  sufixo="PIT", bit_cc=2, rate=("PDes", "P", "POut")),
    "yaw": dict(ref="yaw_des_rel", med="yaw_rel", erro="erro_yaw",
                taxa_ref="r_des", taxa="r", saida="u_yaw",
                pid="pidy_", msg_pid="PIDY", torque="tqz",
                sufixo="YAW", bit_cc=4, rate=("YDes", "Y", "YOut")),
}

# termos do PID de taxa gravados nas mensagens PIDR/PIDP/PIDY, e o nome que cada
# um ganha na serie (depois do prefixo do eixo)
CAMPOS_PID = (("Tar", "tar"), ("Act", "act"), ("Err", "err"),
              ("P", "p"), ("I", "i"), ("D", "d"), ("FF", "ff"))


def nome(eixo):
    """Nome do eixo em portugues, para rotulos e textos."""
    return NOMES.get(eixo, eixo)


def ordenar(eixos):
    """Os eixos dados, na ordem canonica e sem repeticao."""
    conjunto = set(eixos)
    return [e for e in EIXOS if e in conjunto]


def ativos(metricas, padrao=("roll",)):
    """Eixos exercitados no ensaio, como registrados pelas metricas.

    Ensaios analisados antes de existir a chave ``eixos_ativos`` eram todos de
    rolagem, e e isso que o padrao devolve para eles.
    """
    lista = (metricas or {}).get("eixos_ativos") or list(padrao)
    return ordenar(lista)
