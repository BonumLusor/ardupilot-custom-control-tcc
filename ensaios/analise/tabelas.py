"""Saidas numericas do ensaio.

Os graficos sao a leitura rapida; estes arquivos sao o que sustenta a escolha de
ganho e o que vai para a tabela da monografia.

Por ensaio, ao lado do .BIN:
    metricas.json   todas as metricas, com as janelas em que foram calculadas
    series.csv      as series temporais na grade unica
    resumo.txt      veiculo, ganhos em uso e as mesmas metricas, em texto

Acumulando na raiz de resultados/:
    consolidado.csv uma linha por ensaio — e a tabela que se le para parametrizar
    tabela.tex      a mesma coisa em LaTeX, pronta para \\input{} na monografia
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from . import metricas as _metricas
from .eixos import EIXOS, nome as nome_eixo

# Ordem preferida das colunas do consolidado. O que nao estiver aqui vai depois,
# em ordem alfabetica — assim uma metrica nova nunca some da tabela por esquecimento.
ORDEM_COLUNAS = (
    ["ensaio", "cenario", "controlador", "semente",
     "vento.modulo_max_ms", "vento.modulo_medio_ms", "vento.modulo_std_ms"]
    + [f"degrau_{e}.{k}" for e in EIXOS for k in (
        "amplitude_deg", "tempo_subida_s", "sobressinal_pct", "tempo_acomodacao_s",
        "erro_regime_deg", "erro_rastreio_max_deg", "erro_rastreio_rms_deg")]
    + [f"disturbio_{e}.{k}" for e in EIXOS for k in (
        "desvio_max_deg", "tempo_recuperacao_s", "rmse_deg", "erro_medio_abs_deg")]
    + ["torque.modulo_max_rads2", "torque.impulso_ang_degs"]
    + [f"torque.{e}_{k}" for e in EIXOS for k in ("pico_rads2", "pico_nm", "impulso_degs")]
    + ["deriva.deriva_max_m", "deriva.deriva_final_m",
       "deriva.vel_horiz_max_ms", "deriva.vel_horiz_media_ms"]
    + [f"esforco_{e}.{k}" for e in EIXOS for k in ("esforco_abs", "esforco_quad", "u_max_abs")]
    # o que voou: frame, massa, inercia e os ganhos em uso por eixo
    + [f"veiculo.{k}" for k in ("frame", "massa_kg", "braco_m", "diagonal_m",
                                "ixx_kgm2", "iyy_kgm2", "izz_kgm2")]
    + [f"ganhos.{e}.{k}" for e in EIXOS for k in ("fonte", "ang_p", "rat_p", "rat_i",
                                                    "rat_d", "rat_ff")]
)

# Rotulos e unidades para a tabela LaTeX. So estas colunas entram: a tabela da
# monografia precisa caber na pagina, o consolidado.csv e que guarda tudo.
#
# A tabela tem UMA LINHA POR EIXO de cada ensaio: as metricas de rolagem e de
# guinada tem os mesmos nomes e unidades, e um ensaio combinado vira duas linhas
# em vez de dobrar a largura da tabela. ``{e}`` e trocado pelo eixo da linha.
COLUNAS_LATEX = [
    # O cenario abre a tabela: sem ele, duas linhas com o mesmo controlador e
    # severidades diferentes ficam indistinguiveis, e a tabela mente por omissao.
    ("cenario", "Cenario", ""),
    ("controlador", "Controlador", ""),
    ("torque.{e}_pico_rads2", "Torque", r"rad/s$^2$"),
    ("disturbio_{e}.desvio_max_deg", "Desvio max.", r"$^\circ$"),
    ("disturbio_{e}.tempo_recuperacao_s", "T. recup.", "s"),
    ("disturbio_{e}.rmse_deg", "RMSE", r"$^\circ$"),
    ("deriva.deriva_max_m", "Deriva max.", "m"),
    ("degrau_{e}.erro_rastreio_max_deg", "Erro rastr.", r"$^\circ$"),
    ("degrau_{e}.sobressinal_pct", "Sobressinal", r"\%"),
    ("degrau_{e}.tempo_acomodacao_s", "T. acom.", "s"),
    ("esforco_{e}.esforco_abs", r"$\int|u|\,dt$", ""),
]

# Colunas por eixo de ensaios gravados antes de o torque ganhar valor por eixo.
# Esses ensaios eram todos de rolagem, e o modulo era o proprio torque em x.
RESERVA_LATEX = {"torque.roll_pico_rads2": "torque.modulo_max_rads2"}


def salvar_metricas(m, destino):
    """Grava metricas.json."""
    Path(destino).write_text(
        json.dumps(m, indent=2, ensure_ascii=False, default=float),
        encoding="utf-8")


def salvar_series(grade, col, destino):
    """Grava series.csv — todas as colunas na mesma grade de tempo."""
    nomes = sorted(col)
    with open(destino, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t"] + nomes)
        for i, t in enumerate(grade):
            linha = [f"{t:.4f}"]
            for n in nomes:
                v = col[n][i]
                linha.append("" if not np.isfinite(v) else f"{v:.6g}")
            w.writerow(linha)
    return len(grade), len(nomes) + 1


def _fmt_txt(v):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.4g}"
    return str(v)


def chave_resposta_freq(eixo):
    """Chave das metricas de resposta em frequencia do eixo.

    A de rolagem guarda o nome de sempre, para que os ensaios antigos continuem
    caindo na mesma coluna do consolidado.
    """
    return "resposta_freq" if eixo == "roll" else f"resposta_freq_{eixo}"


def _secoes():
    """(chave, titulo) de cada secao de metricas do resumo, na ordem de leitura."""
    s = [("vento", "VENTO APLICADO"), ("torque", "TORQUE APLICADO")]
    s += [(f"degrau_{e}", f"DEGRAU DE {nome_eixo(e).upper()}") for e in EIXOS]
    s += [(f"disturbio_{e}", f"REJEICAO DE DISTURBIO ({nome_eixo(e)})") for e in EIXOS]
    s += [("deriva", "DERIVA SOB DISTURBIO")]
    s += [(f"esforco_{e}", f"ESFORCO DE CONTROLE ({nome_eixo(e)})") for e in EIXOS]
    s += [(chave_resposta_freq(e), f"RESPOSTA EM FREQUENCIA ({nome_eixo(e)})") for e in EIXOS]
    return s


def salvar_resumo(m, destino, cabecalho=None, preambulo=None):
    """Grava resumo.txt e devolve o texto, para tambem sair no terminal.

    ``preambulo`` sao as linhas que abrem o arquivo logo depois do cabecalho —
    veiculo e ganhos em uso, de ``analise/veiculo.py`` —, para que cada resumo
    diga sozinho com que planta e com que controlador os numeros foram obtidos.
    """
    linhas = []
    if cabecalho:
        linhas += [cabecalho, "=" * len(cabecalho), ""]
    if preambulo:
        linhas += list(preambulo)
        if linhas[-1] != "":
            linhas.append("")

    for secao, titulo in _secoes():
        if secao not in m:
            continue
        d = m[secao]
        if not d:
            # degrau pedido pelo cenario e nao encontrado no log: calar aqui
            # deixaria a ausencia passar por "nao havia degrau"
            if secao.startswith("degrau_"):
                linhas += [titulo, "  comandado pelo cenario, mas nao encontrado no log", ""]
            continue
        linhas.append(titulo)
        for k, v in d.items():
            if k == "janela_s":
                linhas.append(f"  {'janela':<24} {v[0]:.2f} .. {v[1]:.2f} s")
            elif isinstance(v, (list, tuple)) and len(v) == 2:
                linhas.append(f"  {k:<24} {_fmt_txt(v[0])} .. {_fmt_txt(v[1])}")
            else:
                linhas.append(f"  {k:<24} {_fmt_txt(v)}")
        linhas.append("")

    texto = "\n".join(linhas)
    Path(destino).write_text(texto, encoding="utf-8")
    return texto


def _ordenar_colunas(chaves):
    conhecidas = [c for c in ORDEM_COLUNAS if c in chaves]
    resto = sorted(k for k in chaves if k not in set(ORDEM_COLUNAS))
    return conhecidas + resto


def anexar_consolidado(destino, linha):
    """Acrescenta uma linha ao consolidado.csv, reescrevendo o cabecalho se preciso.

    Uma metrica nova em um ensaio posterior amplia o cabecalho e as linhas antigas
    ganham a coluna vazia — ninguem perde dado por ter rodado antes.
    """
    destino = Path(destino)
    antigas = []
    if destino.exists():
        with open(destino, newline="", encoding="utf-8") as f:
            antigas = list(csv.DictReader(f))

    # Reanalisar um ensaio SUBSTITUI a linha dele, nao acrescenta outra. O
    # identificador e (ensaio, cenario, controlador) — reanalise e coisa
    # corriqueira ao ajustar a analise, e duplicatas silenciosas viram medias
    # erradas na tabela da monografia.
    chave = (linha.get("ensaio"), linha.get("cenario"), linha.get("controlador"))
    antigas = [r for r in antigas
               if (r.get("ensaio"), r.get("cenario"), r.get("controlador")) != chave]

    todas = antigas + [{k: linha[k] for k in linha}]
    chaves = set()
    for r in todas:
        chaves.update(r)
    colunas = _ordenar_colunas(chaves)

    with open(destino, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=colunas, restval="")
        w.writeheader()
        for r in todas:
            w.writerow({k: r.get(k, "") for k in colunas})
    return len(todas)


def _tex_escapar(s):
    for a, b in (("\\", r"\textbackslash{}"), ("_", r"\_"), ("&", r"\&"),
                 ("%", r"\%"), ("#", r"\#")):
        s = s.replace(a, b)
    return s


def _vazio(v):
    return v in ("", None)


def _eixos_da_linha(r):
    """Eixos com alguma metrica preenchida numa linha do consolidado."""
    eixos = []
    for e in EIXOS:
        prefixos = (f"degrau_{e}.", f"disturbio_{e}.", f"esforco_{e}.")
        if any(k.startswith(prefixos) and not _vazio(v) for k, v in r.items()):
            eixos.append(e)
    return eixos or ["roll"]


def _valor_latex(r, eixo, chave):
    k = chave.format(e=eixo)
    v = r.get(k, "")
    if _vazio(v) and k in RESERVA_LATEX:
        v = r.get(RESERVA_LATEX[k], "")
    return v


def escrever_tabela_tex(consolidado_csv, destino, legenda=None, rotulo=None):
    """Gera a tabela LaTeX a partir do consolidado.csv, uma linha por eixo."""
    consolidado_csv = Path(consolidado_csv)
    if not consolidado_csv.exists():
        return 0
    with open(consolidado_csv, newline="", encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    if not linhas:
        return 0

    # (linha do consolidado, eixo): um ensaio de rolagem + guinada vira duas
    pares = [(r, e) for r in linhas for e in _eixos_da_linha(r)]

    cols = [c for c in COLUNAS_LATEX
            if any(not _vazio(_valor_latex(r, e, c[0])) for r, e in pares)]
    if not cols:
        return 0
    # A coluna do eixo so entra quando a tabela tem mais de um: uma tabela so de
    # rolagem continua exatamente como era.
    if len({e for _, e in pares}) > 1:
        pos = next((i + 1 for i, c in enumerate(cols) if c[0] == "controlador"), 0)
        cols.insert(pos, ("{eixo}", "Eixo", ""))

    alinhamento = "l" + "r" * (len(cols) - 1)
    saida = [r"\begin{table}[htbp]", r"  \centering",
             f"  \\caption{{{legenda or 'Metricas dos ensaios sob vento.'}}}"]
    if rotulo:
        saida.append(f"  \\label{{{rotulo}}}")
    saida += [f"  \\begin{{tabular}}{{{alinhamento}}}", r"    \hline"]

    cab = " & ".join(
        f"{rot}" + (f" [{un}]" if un else "") for _, rot, un in cols)
    saida += [f"    {cab} \\\\", r"    \hline"]

    for r, e in pares:
        celulas = []
        for chave, _, _ in cols:
            if chave == "{eixo}":
                celulas.append(nome_eixo(e))
                continue
            v = _valor_latex(r, e, chave)
            if _vazio(v):
                celulas.append("—")
                continue
            try:
                celulas.append(f"{float(v):.3g}")
            except (TypeError, ValueError):
                celulas.append(_tex_escapar(str(v)))
        saida.append("    " + " & ".join(celulas) + r" \\")

    saida += [r"    \hline", r"  \end{tabular}", r"\end{table}"]
    Path(destino).write_text("\n".join(saida) + "\n", encoding="utf-8")
    return len(linhas)


def linha_consolidado(m, ensaio, cenario, controlador, semente=None, extra=None):
    """Monta a linha do consolidado a partir das metricas achatadas."""
    linha = {"ensaio": ensaio, "cenario": cenario, "controlador": controlador}
    if semente is not None:
        linha["semente"] = semente
    linha.update({k: f"{v:.6g}" if isinstance(v, float) else v
                  for k, v in _metricas.achatar(m).items()})
    if extra:
        linha.update({k: str(v) for k, v in extra.items()})
    return linha
