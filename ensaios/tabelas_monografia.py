#!/usr/bin/env python3
"""Gera as tabelas de resultados da monografia a partir do consolidado.csv.

Diferente do resultados/tabela.tex, que tem uma linha por ensaio ja feito, aqui
entra so o ensaio mais recente de cada cenario x controlador — opcionalmente so
os de um lote, a partir de uma data — e cada tabela sai no formato da
monografia (booktabs, virgula decimal), so com o tabular: legenda, rotulo e
fonte ficam no capitulo.

    ./tabelas_monografia.py --desde 2026-10-27
    ./tabelas_monografia.py --desde 2026-10-27 --destino ../monografia/tabelas
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

RAIZ = Path(__file__).resolve().parent
CONSOLIDADO = RAIZ / "resultados" / "consolidado.csv"
DESTINO = RAIZ.parent / "monografia" / "tabelas"

NOMES = {"roll": "Rolagem", "pitch": "Arfagem", "yaw": "Guinada"}
GRUPO_EIXO = {"rolagem": "roll", "arfagem": "pitch", "guinada": "yaw"}
CONTROLADORES = (("nativo", "nativo"), ("cc_pid", r"\codigo{cc\_pid}"), ("cc_lqi", "LQI"),
                 ("cc_eso", "LQR+ESO"), ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)"),
                 ("cc_lqg", "LQG"), ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
# os que voam todas as familias de ensaio novas (a chave cenarios: dos YAMLs)
NUCLEO = (("nativo", "nativo"), ("cc_lqi", "LQI"), ("cc_eso", "LQR+ESO"),
          ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)"), ("cc_lqg", "LQG"),
          ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
# o LQI de mesmo ruido so difere do cc_lqi na guinada; nos outros eixos, repetiria a linha
SO_GUINADA = {"cc_lqi_mesmo_ruido"}


def controladores(eixo):
    return [(slug, nome) for slug, nome in CONTROLADORES
            if eixo == "yaw" or slug not in SO_GUINADA]
CABECALHO = "% Gerada por ensaios/tabelas_monografia.py — nao editar a mao.\n"


def ler(desde=None):
    """{(cenario, controlador): linha} com o ensaio mais recente de cada par."""
    with open(CONSOLIDADO, newline="", encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    ultimos = {}
    for r in linhas:
        if desde and r["ensaio"] < desde:
            continue
        chave = (r["cenario"], r["controlador"])
        if chave not in ultimos or r["ensaio"] > ultimos[chave]["ensaio"]:
            ultimos[chave] = r
    return ultimos


def num(r, chave, casas=2, escala=1.0):
    v = (r or {}).get(chave, "")
    try:
        x = float(v) * escala
    except (TypeError, ValueError):
        return "---"
    return f"{x:.{casas}f}".replace(".", ",")


def tabular(colunas, linhas, cabecalho, nota=None):
    """O tabular da monografia; ``nota`` e uma nota de rodape ou uma lista delas."""
    rodape = "\\bottomrule\n"
    for n in ([nota] if isinstance(nota, str) else (nota or [])):
        rodape += f"\\multicolumn{{{len(colunas)}}}{{l}}{{\\footnotesize {n}}} \\\\\n"
    return (CABECALHO + f"\\begin{{tabular}}{{{colunas}}}\n\\toprule\n{cabecalho}\n\\midrule\n"
            + "\n".join(linhas) + "\n" + rodape + "\\end{tabular}\n")


# Pulso de guinada normalizado pelo impulso registrado (metricas.py, chave
# desvio_max_norm_deg): o laco do injetor a ~44 Hz faz o impulso aplicado variar
# de -6 % a +8 % entre voos, e em guinada o pico vem depois do fim do pulso e
# acompanha o impulso. Em rolagem e arfagem o pico vem durante o pulso e nao
# depende da duracao: la fica o desvio medido.
NOTA_NORM = (r"$^\dagger$ Pulso de guinada: desvio máximo normalizado ao impulso nominal "
             r"do cenário, $\delta_{\max}\,I_{\mathrm{nom}}/I_{\mathrm{reg}}$.")


def pico_pulso(r, eixo, casas=2):
    """Desvio maximo de um pulso: normalizado (com a marca) em guinada, medido nos demais."""
    if eixo != "yaw":
        return num(r, f"disturbio_{eixo}.desvio_max_deg", casas)
    v = num(r, "disturbio_yaw.desvio_max_norm_deg", casas)
    if v == "---":
        # pulso de duracao exata (WND_MODE 512): sem normalizacao, nem marca
        return num(r, "disturbio_yaw.desvio_max_deg", casas)
    return v + r"$^\dagger$"


def tabela_degrau(dados):
    linhas = []
    for grupo, eixo in GRUPO_EIXO.items():
        for slug, nome in controladores(eixo):
            r = dados.get((f"{grupo}/nominal", slug))
            p = f"degrau_{eixo}."
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, p + 'tempo_subida_s', 3)} & "
                          f"{num(r, p + 'sobressinal_pct', 1)} & "
                          f"{num(r, p + 'tempo_acomodacao_s', 2)} & "
                          f"{num(r, p + 'erro_rastreio_max_deg', 2)} & "
                          f"{num(r, p + 'erro_rastreio_rms_deg', 2)} \\\\")
    cab = (r"Eixo & Controlador & Subida & Sobressinal & Acomodação & "
           r"\multicolumn{2}{c}{Erro de rastreamento} \\" "\n" r"\cmidrule(lr){6-7}" "\n"
           r" & & (\si{\second}) & (\%) & (\si{\second}) & máx. (\si{\degree}) & "
           r"eficaz (\si{\degree}) \\")
    return tabular("llrrrrr", linhas, cab)


def tabela_torque(dados):
    linhas = []
    for ensaio, rotulo in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for grupo, eixo in GRUPO_EIXO.items():
            for slug, nome in controladores(eixo):
                r = dados.get((f"{grupo}/{ensaio}", slug))
                d, e = f"disturbio_{eixo}.", f"esforco_{eixo}."
                pico = (pico_pulso(r, eixo) if ensaio == "torque_pulso"
                        else num(r, d + 'desvio_max_deg', 2))
                linhas.append(f"{NOMES[eixo]} & {rotulo} & {nome} & {pico} & "
                              f"{num(r, d + 'tempo_recuperacao_s', 2)} & "
                              f"{num(r, d + 'rmse_deg', 3)} & "
                              f"{num(r, e + 'u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Torque & Controlador & Desvio máx. (\si{\degree}) & "
           r"Recuperação (\si{\second}) & RMSE (\si{\degree}) & $u_{\max}$ \\")
    return tabular("lllrrrr", linhas, cab, nota=NOTA_NORM)


def tabela_simultaneo(dados):
    """Pulso e degrau nos tres eixos, ao lado do mesmo eixo no ensaio isolado."""
    linhas = []
    for ensaio, rotulo in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for grupo, eixo in GRUPO_EIXO.items():
            so = dados.get((f"{grupo}/{ensaio}", "nativo"))
            tres = dados.get((f"rolagem_arfagem_guinada/{ensaio}", "nativo"))
            d = f"disturbio_{eixo}."
            if ensaio == "torque_pulso":
                p_so, p_tres = pico_pulso(so, eixo), pico_pulso(tres, eixo)
            else:
                p_so, p_tres = num(so, d + 'desvio_max_deg'), num(tres, d + 'desvio_max_deg')
            linhas.append(f"{NOMES[eixo]} & {rotulo} & {p_so} & {p_tres} & "
                          f"{num(so, d + 'tempo_recuperacao_s')} & "
                          f"{num(tres, d + 'tempo_recuperacao_s')} \\\\")
    cab = (r" & & \multicolumn{2}{c}{Desvio máx. (\si{\degree})} & "
           r"\multicolumn{2}{c}{Recuperação (\si{\second})} \\" "\n"
           r"\cmidrule(lr){3-4}\cmidrule(lr){5-6}" "\n"
           r"Eixo & Torque & isolado & 3 eixos & isolado & 3 eixos \\")
    return tabular("llrrrr", linhas, cab, nota=NOTA_NORM)


def tabela_frequencia(dados):
    linhas = []
    sufixo = {"roll": "", "pitch": "_pitch", "yaw": "_yaw"}
    for grupo, eixo in GRUPO_EIXO.items():
        for slug, nome in controladores(eixo):
            r = dados.get((f"{grupo}/torque_chirp", slug))
            p = f"resposta_freq{sufixo[eixo]}."
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, p + 'ganho_max', 3)} & "
                          f"{num(r, p + 'f_ganho_max_hz', 1)} & "
                          f"{num(r, p + 'f_queda_3db_hz', 1)} & "
                          f"{num(r, p + 'coerencia_mediana', 2)} \\\\")
    cab = (r"Eixo & Controlador & Ganho máx. & $f$ do máx. (\si{\hertz}) & "
           r"Queda de \SI{3}{\decibel} (\si{\hertz}) & Coerência \\")
    return tabular("llrrrr", linhas, cab)


def tabela_ruido(dados):
    """Ruido de giroscopio: desvio, recuperacao e esforco com e sem ruido, por controlador.

    O par sem ruido e o mesmo ensaio de tres eixos (rolagem_arfagem_guinada/), e
    a razao entre os esforcos quadraticos mede o quanto cada controlador repassa
    o ruido do giroscopio aos motores.
    """
    linhas = []
    for ensaio, rotulo in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for eixo in ("roll", "pitch", "yaw"):
            for slug, nome in controladores(eixo):
                if slug == "cc_pid":
                    continue
                # os voos de ruido 20 foram para ruido/rnd20/; os de antes ficam
                # como reserva
                com = dados.get((f"ruido/rnd20/{ensaio}", slug)) or dados.get((f"ruido/{ensaio}", slug))
                sem = dados.get((f"rolagem_arfagem_guinada/{ensaio}", slug))
                d, e = f"disturbio_{eixo}.", f"esforco_{eixo}.esforco_quad"
                try:
                    razao = f"{float(com[e]) / float(sem[e]):.1f}".replace(".", ",")
                except (TypeError, KeyError, ValueError, ZeroDivisionError):
                    razao = "---"
                if ensaio == "torque_pulso":
                    p_sem, p_com = pico_pulso(sem, eixo), pico_pulso(com, eixo)
                else:
                    p_sem, p_com = num(sem, d + 'desvio_max_deg'), num(com, d + 'desvio_max_deg')
                linhas.append(f"{NOMES[eixo]} & {rotulo} & {nome} & {p_sem} & {p_com} & "
                              f"{num(com, d + 'tempo_recuperacao_s')} & {razao} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & & \multicolumn{2}{c}{Desvio máx. (\si{\degree})} & Recuperação & "
           r"Esforço quadrático \\" "\n" r"\cmidrule(lr){4-5}" "\n"
           r"Eixo & Torque & Controlador & sem ruído & com ruído & com ruído (\si{\second}) & "
           r"com/sem ruído \\")
    return tabular("lllrrrr", linhas, cab, nota=NOTA_NORM)


def tabela_alimentacao_direta(dados):
    """Degrau de atitude: realimentacao {PID, LQI, LQG} x alimentacao direta {sem, com}.

    Separa o que vem da realimentacao do que vem da alimentacao direta pura
    u_ff = (a_t + a*w_t)/b, dada igual aos dois (cc_pid_ff e cc_lqi). O nativo
    entra como referencia: o cc_pid e a mesma cascata sem a raiz quadrada.
    """
    linhas = []
    # o LQG so voa com a alimentacao direta; a comparacao justa dele e com o
    # cc_pid_ff e o cc_lqi, na mesma coluna
    quadro = (("nativo", "PID nativo", "sem"), ("cc_pid", "PID", "sem"),
              ("cc_pid_ff", "PID", "com"), ("cc_lqi_sem_ff", "LQI", "sem"),
              ("cc_lqi", "LQI", "com"), ("cc_lqg_mesmo_ruido", "LQG", "com"))
    for grupo, eixo in GRUPO_EIXO.items():
        for slug, realim, ff in quadro:
            r = dados.get((f"{grupo}/nominal", slug))
            p = f"degrau_{eixo}."
            linhas.append(f"{NOMES[eixo]} & {realim} & {ff} & "
                          f"{num(r, p + 'sobressinal_pct', 1)} & "
                          f"{num(r, p + 'erro_rastreio_max_deg', 2)} & "
                          f"{num(r, p + 'erro_rastreio_rms_deg', 2)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Realimentação & Alim. direta & Sobressinal (\%) & "
           r"\multicolumn{2}{c}{Erro de rastreamento (\si{\degree})} \\" "\n"
           r"\cmidrule(lr){5-6}" "\n"
           r" & & & & máx. & eficaz \\")
    return tabular("lllrrr", linhas, cab)


def tabela_equivalencia(dados):
    """Prova de equivalencia: nativo com os ganhos do LQI contra o proprio LQI."""
    linhas = []
    trio = (("nativo", "nativo (AutoTune)"), ("nativo_lqi", "nativo (ganhos do LQI)"),
            ("cc_lqi", "LQI"))
    for ensaio, rotulo in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for grupo, eixo in (("rolagem", "roll"), ("arfagem", "pitch")):
            for slug, nome in trio:
                r = dados.get((f"{grupo}/{ensaio}", slug))
                d = f"disturbio_{eixo}."
                linhas.append(f"{NOMES[eixo]} & {rotulo} & {nome} & "
                              f"{num(r, d + 'desvio_max_deg')} & "
                              f"{num(r, d + 'tempo_recuperacao_s')} & "
                              f"{num(r, d + 'rmse_deg', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Torque & Controlador & Desvio máx. (\si{\degree}) & "
           r"Recuperação (\si{\second}) & RMSE (\si{\degree}) \\")
    return tabular("lllrrr", linhas, cab)


def _rel(a, b):
    """a/b em %, com sinal, ou '---'."""
    try:
        return f"{100.0 * (float(a) / float(b) - 1.0):+.0f}".replace("-", "$-$")
    except (TypeError, ValueError, ZeroDivisionError):
        return "---"


def _v(r, chave):
    return (r or {}).get(chave)


def tabela_estocastico(dados):
    """Torque estocastico de um eixo: erro eficaz (o que o LQG minimiza) e esforco maximo."""
    linhas = []
    for grupo, eixo in GRUPO_EIXO.items():
        nat = dados.get((f"{grupo}/torque_estocastico", "nativo"))
        d = f"disturbio_{eixo}."
        for slug, nome in NUCLEO:
            if slug in SO_GUINADA and eixo != "yaw":
                continue
            r = dados.get((f"{grupo}/torque_estocastico", slug))
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, d + 'rmse_deg', 3)} & "
                          f"{_rel(_v(r, d + 'rmse_deg'), _v(nat, d + 'rmse_deg'))} & "
                          f"{num(r, d + 'desvio_max_deg', 2)} & "
                          f"{num(r, f'esforco_{eixo}.u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Controlador & RMSE (\si{\degree}) & $\Delta$ nativo (\%) & "
           r"Desvio máx. (\si{\degree}) & $u_{\max}$ \\")
    return tabular("llrrrr", linhas, cab)


NIVEIS = (("0", "rolagem_arfagem_guinada"), ("10", "ruido/rnd10"), ("20", "ruido/rnd20"),
          ("40", "ruido/rnd40"), ("80", "ruido/rnd80"))
# Com SIM_GYR_RND de 40 e 80 a recuperacao nao entra (pergunta 18 da verificacao,
# opcao c). O limiar dela e max(10 % do pico, |media| + 3 sigma do erro calmo)
# (metricas.faixa_calma), e nesses niveis o piso de ruido decide em 26 e 44 das
# 54 medicoes. Cada controlador passa a ser medido contra a propria faixa de
# ruido (em guinada com RND 80, 3,7 graus no LQG e 0,4 no LQI), e o tempo deixa
# de comparar controladores. Com RND 20 o piso decide em 4 de 90 medicoes, todas
# do LQG no degrau, e por pouco; com RND 0 e 10, em nenhuma.
SEM_RECUPERACAO = {"40", "80"}
NOTA_RECUPERACAO = (r"$^\ddagger$ Recuperação não citada: com \codigo{SIM\_GYR\_RND} de 40 e 80, "
                    r"o limiar vem da faixa de ruído de cada controlador.")


def tabela_niveis_ruido(dados, eixo):
    """A fronteira medida: ruido no esforco (trecho calmo) contra a rejeicao, por nivel."""
    linhas = []
    for nivel, grupo in NIVEIS:
        for slug, nome in NUCLEO:
            if slug in SO_GUINADA and eixo != "yaw":
                continue
            pul = dados.get((f"{grupo}/torque_pulso", slug))
            deg = dados.get((f"{grupo}/torque_degrau", slug))
            est = dados.get((f"{grupo}/torque_estocastico", slug))
            d = f"disturbio_{eixo}."
            rec = (r"---$^\ddagger$" if nivel in SEM_RECUPERACAO
                   else num(deg, d + 'tempo_recuperacao_s'))
            linhas.append(f"{nivel} & {nome} & {num(pul, f'calmo_{eixo}.u_rms', 4)} & "
                          f"{pico_pulso(pul, eixo)} & {num(deg, d + 'desvio_max_deg')} & "
                          f"{rec} & {num(est, d + 'rmse_deg', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & Ruído & Pulso & \multicolumn{2}{c}{Degrau} & Estocástico \\" "\n"
           r"\cmidrule(lr){5-6}" "\n"
           r"\codigo{SIM\_GYR\_RND} & Controlador & $u_\mathrm{rms}$ calmo & desvio (\si{\degree}) & "
           r"desvio (\si{\degree}) & recup. (\si{\second}) & RMSE (\si{\degree}) \\")
    notas = ([NOTA_NORM] if eixo == "yaw" else []) + [NOTA_RECUPERACAO]
    return tabular("llrrrrr", linhas, cab, nota=notas)


VARIANTES_MODELO = (("nominal", "rolagem_arfagem_guinada"),
                    (r"empuxo $-22\,\%$", "erro_modelo/empuxo_fraco"),
                    (r"braço $+20\,\%$", "erro_modelo/braco_longo"),
                    (r"braço $-17\,\%$", "erro_modelo/braco_curto"))


def tabela_erro_modelo(dados):
    """Os ganhos do nominal num veiculo diferente: pulso e degrau de torque, rolagem e guinada."""
    linhas = []
    for rot, grupo in VARIANTES_MODELO:
        for slug, nome in NUCLEO:
            pul = dados.get((f"{grupo}/torque_pulso", slug))
            deg = dados.get((f"{grupo}/torque_degrau", slug))
            linhas.append(f"{rot} & {nome} & "
                          f"{num(pul, 'disturbio_roll.desvio_max_deg')} & "
                          f"{num(deg, 'disturbio_roll.desvio_max_deg')} & "
                          f"{num(deg, 'disturbio_roll.tempo_recuperacao_s')} & "
                          f"{pico_pulso(pul, 'yaw')} & "
                          f"{num(deg, 'disturbio_yaw.desvio_max_deg')} & "
                          f"{num(deg, 'disturbio_yaw.tempo_recuperacao_s')} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & \multicolumn{3}{c}{Rolagem} & \multicolumn{3}{c}{Guinada} \\" "\n"
           r"\cmidrule(lr){3-5}\cmidrule(lr){6-8}" "\n"
           r"Veículo & Controlador & pulso (\si{\degree}) & degrau (\si{\degree}) & recup. (\si{\second}) & "
           r"pulso (\si{\degree}) & degrau (\si{\degree}) & recup. (\si{\second}) \\")
    return tabular("llrrrrrr", linhas, cab, nota=NOTA_NORM)


def tabela_falha_motor(dados):
    """Falha parcial do motor 1: desvio e recuperacao em cada eixo."""
    linhas = []
    for nome_cen, rot in (("leve", "leve (0,95)"), ("severa", "severa (0,85)")):
        for slug, nome in NUCLEO:
            r = dados.get((f"falha_motor/{nome_cen}", slug))
            cel = []
            for eixo in ("roll", "pitch", "yaw"):
                cel += [num(r, f"disturbio_{eixo}.desvio_max_deg"),
                        num(r, f"disturbio_{eixo}.tempo_recuperacao_s")]
            linhas.append(f"{rot} & {nome} & " + " & ".join(cel) + " \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & \multicolumn{2}{c}{Rolagem} & \multicolumn{2}{c}{Arfagem} & "
           r"\multicolumn{2}{c}{Guinada} \\" "\n"
           r"\cmidrule(lr){3-4}\cmidrule(lr){5-6}\cmidrule(lr){7-8}" "\n"
           r"Falha & Controlador & desvio (\si{\degree}) & recup. (\si{\second}) & "
           r"desvio (\si{\degree}) & recup. (\si{\second}) & desvio (\si{\degree}) & recup. (\si{\second}) \\")
    return tabular("llrrrrrr", linhas, cab)


def tabela_grande(dados):
    """Angulo grande: degrau de rolagem e manobra combinada a 30 e 45 graus."""
    linhas = []
    for graus in ("30", "45"):
        for ensaio, rot in (("nominal", "rolagem"), ("combinado", "combinada")):
            for slug, nome in NUCLEO:
                r = dados.get((f"grande/{graus}graus/{ensaio}", slug))
                linhas.append(f"{graus} & {rot} & {nome} & "
                              f"{num(r, 'degrau_roll.sobressinal_pct', 1)} & "
                              f"{num(r, 'degrau_roll.erro_rastreio_max_deg')} & "
                              f"{num(r, 'degrau_roll.erro_rastreio_rms_deg')} & "
                              f"{num(r, 'degrau_yaw.erro_rastreio_max_deg')} \\\\")
            linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & & \multicolumn{3}{c}{Rolagem} & Guinada \\" "\n"
           r"\cmidrule(lr){4-6}" "\n"
           r"Graus & Manobra & Controlador & sobressinal (\%) & erro máx. (\si{\degree}) & "
           r"erro eficaz (\si{\degree}) & erro máx. (\si{\degree}) \\")
    return tabular("lllrrrr", linhas, cab)


def tabela_saturacao(dados):
    """Guinada alem da autoridade do mixer: windup depois do pulso e degrau perto do limite."""
    linhas = []
    for ensaio, rot in (("torque_pulso", "pulso 12 rad/s² × 0,5 s"),
                        ("torque_degrau", "degrau 6 rad/s²")):
        for slug, nome in NUCLEO:
            r = dados.get((f"saturacao/guinada/{ensaio}", slug))
            linhas.append(f"{rot} & {nome} & {num(r, 'disturbio_yaw.desvio_max_deg')} & "
                          f"{num(r, 'disturbio_yaw.tempo_recuperacao_s')} & "
                          f"{num(r, 'disturbio_yaw.rmse_deg', 3)} & "
                          f"{num(r, 'esforco_yaw.u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Torque & Controlador & Desvio máx. (\si{\degree}) & Recuperação (\si{\second}) & "
           r"RMSE (\si{\degree}) & $u_{\max}$ \\")
    return tabular("llrrrr", linhas, cab)


def tabela_filtro(dados):
    """Giroscopio filtrado a 20 Hz (o lote principal) e a 10 Hz (variante filtro10).

    Cada controlador foi ressintonizado para o filtro de 10 Hz: o nativo pelo
    AutoTune; o LQG, o LQI e o ESO pelo mesmo criterio (portao: ruido, MF e MF
    robusta nao piores que as do nativo, o menor pico), porque na mesma banda do
    nativo o LQI e o ESO nao alcancam a MF dele em rolagem e arfagem (projeto.py
    --sufixo filtro10). Os tres usam o angulo do AHRS, que nao passa pelo filtro;
    so o LQG reconstroi a taxa com o filtro no modelo: e o teste que isola isso.
    """
    pares = (("nativo", "nativo_filtro10", "nativo"), ("cc_lqi", "cc_lqi_filtro10", "LQI"),
             ("cc_eso", "cc_eso_filtro10", "LQR+ESO"),
             ("cc_lqg_mesmo_ruido", "cc_lqg_filtro10", "LQG"))
    linhas = []
    for grupo, eixo in (("rolagem", "roll"), ("guinada", "yaw")):
        d = f"disturbio_{eixo}."
        for slug20, slug10, nome in pares:
            cel = []
            for slug in (slug20, slug10):
                pul = dados.get((f"{grupo}/torque_pulso", slug))
                deg = dados.get((f"{grupo}/torque_degrau", slug))
                nom = dados.get((f"{grupo}/nominal", slug))
                cel += [pico_pulso(pul, eixo), num(deg, d + "desvio_max_deg"),
                        num(nom, f"degrau_{eixo}.erro_rastreio_max_deg")]
            linhas.append(f"{NOMES[eixo]} & {nome} & " + " & ".join(cel) + " \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & \multicolumn{3}{c}{Filtro de 20 Hz} & \multicolumn{3}{c}{Filtro de 10 Hz} \\" "\n"
           r"\cmidrule(lr){3-5}\cmidrule(lr){6-8}" "\n"
           r"Eixo & Controlador & pulso (\si{\degree}) & degrau (\si{\degree}) & rastreio (\si{\degree}) & "
           r"pulso (\si{\degree}) & degrau (\si{\degree}) & rastreio (\si{\degree}) \\")
    return tabular("llrrrrrr", linhas, cab, nota=NOTA_NORM)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--desde", help="so ensaios com carimbo a partir deste (AAAA-MM-DD)")
    ap.add_argument("--destino", type=Path, default=DESTINO)
    args = ap.parse_args()

    dados = ler(args.desde)
    args.destino.mkdir(parents=True, exist_ok=True)
    for nome, fn in (("resultados_degrau", tabela_degrau), ("resultados_torque", tabela_torque),
                     ("resultados_simultaneo", tabela_simultaneo),
                     ("resultados_frequencia", tabela_frequencia),
                     ("resultados_ruido", tabela_ruido),
                     ("resultados_equivalencia", tabela_equivalencia),
                     ("resultados_alimentacao_direta", tabela_alimentacao_direta),
                     ("resultados_estocastico", tabela_estocastico),
                     ("resultados_niveis_rolagem", lambda d: tabela_niveis_ruido(d, "roll")),
                     ("resultados_niveis_guinada", lambda d: tabela_niveis_ruido(d, "yaw")),
                     ("resultados_erro_modelo", tabela_erro_modelo),
                     ("resultados_falha_motor", tabela_falha_motor),
                     ("resultados_grande", tabela_grande),
                     ("resultados_saturacao", tabela_saturacao),
                     ("resultados_filtro", tabela_filtro)):
        destino = args.destino / f"{nome}.tex"
        destino.write_text(fn(dados), encoding="utf-8")
        print(f"{destino}")


if __name__ == "__main__":
    main()
