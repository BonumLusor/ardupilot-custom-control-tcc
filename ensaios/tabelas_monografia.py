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
CONTROLADORES = (("nativo", "nativo"), ("cc_pid", r"\codigo{cc\_pid}"))
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


def tabular(colunas, linhas, cabecalho):
    return (CABECALHO + f"\\begin{{tabular}}{{{colunas}}}\n\\toprule\n{cabecalho}\n\\midrule\n"
            + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


def tabela_degrau(dados):
    linhas = []
    for grupo, eixo in GRUPO_EIXO.items():
        for slug, nome in CONTROLADORES:
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
            for slug, nome in CONTROLADORES:
                r = dados.get((f"{grupo}/{ensaio}", slug))
                d, e = f"disturbio_{eixo}.", f"esforco_{eixo}."
                linhas.append(f"{NOMES[eixo]} & {rotulo} & {nome} & "
                              f"{num(r, d + 'desvio_max_deg', 2)} & "
                              f"{num(r, d + 'tempo_recuperacao_s', 2)} & "
                              f"{num(r, d + 'rmse_deg', 3)} & "
                              f"{num(r, e + 'u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Torque & Controlador & Desvio máx. (\si{\degree}) & "
           r"Recuperação (\si{\second}) & RMSE (\si{\degree}) & $u_{\max}$ \\")
    return tabular("lllrrrr", linhas, cab)


def tabela_simultaneo(dados):
    """Pulso e degrau nos tres eixos, ao lado do mesmo eixo no ensaio isolado."""
    linhas = []
    for ensaio, rotulo in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for grupo, eixo in GRUPO_EIXO.items():
            so = dados.get((f"{grupo}/{ensaio}", "nativo"))
            tres = dados.get((f"rolagem_arfagem_guinada/{ensaio}", "nativo"))
            d = f"disturbio_{eixo}."
            linhas.append(f"{NOMES[eixo]} & {rotulo} & {num(so, d + 'desvio_max_deg')} & "
                          f"{num(tres, d + 'desvio_max_deg')} & "
                          f"{num(so, d + 'tempo_recuperacao_s')} & "
                          f"{num(tres, d + 'tempo_recuperacao_s')} \\\\")
    cab = (r" & & \multicolumn{2}{c}{Desvio máx. (\si{\degree})} & "
           r"\multicolumn{2}{c}{Recuperação (\si{\second})} \\" "\n"
           r"\cmidrule(lr){3-4}\cmidrule(lr){5-6}" "\n"
           r"Eixo & Torque & isolado & 3 eixos & isolado & 3 eixos \\")
    return tabular("llrrrr", linhas, cab)


def tabela_frequencia(dados):
    linhas = []
    sufixo = {"roll": "", "pitch": "_pitch", "yaw": "_yaw"}
    for grupo, eixo in GRUPO_EIXO.items():
        for slug, nome in CONTROLADORES:
            r = dados.get((f"{grupo}/torque_chirp", slug))
            p = f"resposta_freq{sufixo[eixo]}."
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, p + 'ganho_max', 3)} & "
                          f"{num(r, p + 'f_ganho_max_hz', 1)} & "
                          f"{num(r, p + 'f_queda_3db_hz', 1)} & "
                          f"{num(r, p + 'coerencia_mediana', 2)} \\\\")
    cab = (r"Eixo & Controlador & Ganho máx. & $f$ do máx. (\si{\hertz}) & "
           r"Queda de \SI{3}{\decibel} (\si{\hertz}) & Coerência \\")
    return tabular("llrrrr", linhas, cab)


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
                     ("resultados_frequencia", tabela_frequencia)):
        destino = args.destino / f"{nome}.tex"
        destino.write_text(fn(dados), encoding="utf-8")
        print(f"{destino}")


if __name__ == "__main__":
    main()
