#!/usr/bin/env python3
"""Gera as tabelas de resultados da monografia a partir das pastas de voo.

Entram so os voos do binario do SITL congelado (versao.json; por padrao, o binario
compilado agora), na base de atitude pedida: a verdadeira do SITL
(metricas_verdade.json, de analise/verdade.py) ou a do AHRS (metricas.json). Uma
base por tabela, nunca as duas misturadas. Quando um par cenario x controlador tem
mais de um voo (repeticoes ou sementes), a celula e a media, e a nota da tabela diz
quais. Cada tabela sai no formato da monografia (booktabs, virgula decimal), so com
o tabular: legenda, rotulo e fonte ficam no capitulo.

    ./tabelas_monografia.py                       # base verdadeira, binario atual
    ./tabelas_monografia.py --atitude ahrs --destino /tmp/tabelas_ahrs
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import yaml
from scipy import stats

RAIZ = Path(__file__).resolve().parent
RESULTADOS = RAIZ / "resultados"
DESTINO = RAIZ.parent / "monografia" / "tabelas"
# pastas de resultados que nao sao do lote (voos de teste e de medicao)
FORA_DO_LOTE = {"medicoes", "lote_revisao", "verificacao_2026-11-04"}
ARQUIVO_BASE = {"verdadeira": "metricas_verdade.json", "ahrs": "metricas.json"}

NOMES = {"roll": "Rolagem", "pitch": "Arfagem", "yaw": "Guinada"}
GRUPO_EIXO = {"rolagem": "roll", "arfagem": "pitch", "guinada": "yaw"}
CONTROLADORES = (("nativo", "nativo"), ("cc_pid", r"\codigo{cc\_pid}"), ("cc_lqi", "LQI"),
                 ("cc_eso", "LQR+ESO"), ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)"),
                 ("cc_lqg", "LQG"), ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
# os que voam todas as familias de ensaio novas (a chave cenarios: dos YAMLs)
NUCLEO = (("nativo", "nativo"), ("cc_lqi", "LQI"), ("cc_eso", "LQR+ESO"),
          ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)"), ("cc_lqg", "LQG"),
          ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
NOME_NATIVO_GUINADA = "nativo (guinada ressintonizada)"
# o LQI de mesmo ruido so difere do cc_lqi na guinada; nos outros eixos, repetiria a linha
SO_GUINADA = {"cc_lqi_mesmo_ruido", "nativo_guinada"}
NOME_CENARIO = {"torque_impulso": "impulso", "torque_degrau": "degrau", "torque_pulso": "degrau curto",
                "torque_estocastico": "estocástico", "nominal": "degrau de atitude"}


def controladores(eixo):
    return [(slug, nome) for slug, nome in CONTROLADORES
            if eixo == "yaw" or slug not in SO_GUINADA]
CABECALHO = "% Gerada por ensaios/tabelas_monografia.py — nao editar a mao.\n"


BINARIO_LOTE = RAIZ / "lotes" / "binario_lote.sha256"


def binario_atual():
    """Hash do binario do lote congelado (lotes/binario_lote.sha256) ou, sem ele, o compilado agora."""
    if BINARIO_LOTE.exists():
        return BINARIO_LOTE.read_text(encoding="utf-8").split()[0]
    sys.path.insert(0, str(RAIZ))
    import run_ensaio  # noqa: E402
    return run_ensaio.versao_firmware()["binario_sha256"]


def _achatar(m, prefixo=""):
    plano = {}
    for k, v in m.items():
        if isinstance(v, dict):
            plano.update(_achatar(v, f"{prefixo}{k}."))
        else:
            plano[f"{prefixo}{k}"] = v
    return plano


def ler_voos(binario, atitude="verdadeira"):
    """{(cenario, controlador): [linha, ...]}: todos os voos do binario, na base pedida.

    Cada linha e o metricas*.json achatado, com ``ensaio`` (o carimbo) e ``semente``.
    A lista sai em ordem de semente e de carimbo; as sementes pareiam os voos do
    torque estocastico.
    """
    arq = ARQUIVO_BASE[atitude]
    voos, sem_base = defaultdict(list), []
    for v in RESULTADOS.rglob("versao.json"):
        pasta = v.parent
        partes = pasta.relative_to(RESULTADOS).parts
        if len(partes) < 3 or partes[0] in FORA_DO_LOTE:
            continue
        try:
            if json.loads(v.read_text(encoding="utf-8")).get("binario_sha256") != binario:
                continue
        except (OSError, ValueError):
            continue
        if not (pasta / arq).exists():
            sem_base.append(pasta)
            continue
        linha = _achatar(json.loads((pasta / arq).read_text(encoding="utf-8")))
        cen = yaml.safe_load((pasta / "cenario.yaml").read_text(encoding="utf-8")) or {}
        linha["ensaio"] = pasta.name
        linha["pasta"] = str(pasta)
        linha["semente"] = (cen.get("vento") or {}).get("WND_SEED")
        voos[("/".join(partes[:-2]), partes[-2])].append(linha)
    for lista in voos.values():
        lista.sort(key=lambda r: (r["semente"] if r["semente"] is not None else 0, r["ensaio"]))
    if sem_base:
        dica = " (rode python -m analise.verdade)" if atitude == "verdadeira" else " (voo em andamento?)"
        print(f"AVISO: {len(sem_base)} voo(s) do binario sem {arq}{dica}; ex.: {sem_base[0]}",
              file=sys.stderr)
    return voos


def _numero(v):
    if isinstance(v, bool) or v is None:
        return False
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def media(linhas):
    """A linha media de varios voos (so os campos numericos), com ``n`` voos."""
    if len(linhas) == 1:
        return dict(linhas[0], n=1)
    r = {"n": len(linhas)}
    for k in set().union(*linhas):
        vals = [float(l[k]) for l in linhas if _numero(l.get(k))]
        if vals:
            r[k] = sum(vals) / len(vals)
    return r


class Dados(dict):
    """{(cenario, controlador): linha media} que anota as celulas lidas com mais de um voo."""

    def __init__(self, voos):
        super().__init__({k: media(v) for k, v in voos.items()})
        self.voos = voos
        self.repetidos = set()

    def get(self, chave, padrao=None):
        r = super().get(chave, padrao)
        if r is not None and r.get("n", 1) > 1:
            self.repetidos.add(chave)
        return r

    def serie(self, chave, campo):
        """Os valores de ``campo`` em cada voo do par, na ordem da semente."""
        return [float(l[campo]) for l in self.voos.get(chave, []) if _numero(l.get(campo))]

    def por_semente(self, chave, campo):
        return {l["semente"]: float(l[campo]) for l in self.voos.get(chave, [])
                if _numero(l.get(campo))}


def _virgula(x, casas):
    return f"{x:.{casas}f}".replace(".", ",").replace("-", "$-$")


def ic_pareado(dados, chave_c, chave_n, campo):
    """Diferenca relativa (%) ao nativo, media por semente, com IC de 95 % pareado.

    (media, meia largura, n) ou None com menos de 2 pares.
    """
    c, n = dados.por_semente(chave_c, campo), dados.por_semente(chave_n, campo)
    d = [100.0 * (c[s] / n[s] - 1.0) for s in sorted(set(c) & set(n), key=str) if n[s]]
    if len(d) < 2:
        return None
    m = sum(d) / len(d)
    sd = math.sqrt(sum((x - m) ** 2 for x in d) / (len(d) - 1))
    return m, stats.t.ppf(0.975, len(d) - 1) * sd / math.sqrt(len(d)), len(d)


def ic_welch(dados, chave_c, chave_n, campo):
    """(media_c - media_n) / media_n em %, com IC de 95 % de Welch; (media, meia largura, nu)."""
    xc, xn = dados.serie(chave_c, campo), dados.serie(chave_n, campo)
    if len(xc) < 2 or len(xn) < 2:
        return None
    mc, mn = sum(xc) / len(xc), sum(xn) / len(xn)
    vc = sum((x - mc) ** 2 for x in xc) / (len(xc) - 1) / len(xc)
    vn = sum((x - mn) ** 2 for x in xn) / (len(xn) - 1) / len(xn)
    if vc + vn <= 0 or not mn:
        return None
    nu = (vc + vn) ** 2 / (vc ** 2 / (len(xc) - 1) + vn ** 2 / (len(xn) - 1))
    hw = stats.t.ppf(0.975, nu) * math.sqrt(vc + vn)
    return 100.0 * (mc - mn) / mn, 100.0 * hw / mn, nu


def fmt_ic(ic, casas=0):
    if ic is None:
        return "---"
    return f"{_virgula(ic[0], casas)} $\\pm$ {_virgula(ic[1], casas).replace('$-$', '')}"


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
        # nota de uma marca ($^\\dagger$ ...) que nenhuma celula usa fica de fora
        marca = n.split(" ", 1)[0] if n.startswith("$^") else None
        if marca and not any(marca in linha for linha in linhas):
            continue
        rodape += f"\\multicolumn{{{len(colunas)}}}{{l}}{{\\footnotesize {n}}} \\\\\n"
    return (CABECALHO + f"\\begin{{tabular}}{{{colunas}}}\n\\toprule\n{cabecalho}\n\\midrule\n"
            + "\n".join(linhas) + "\n" + rodape + "\\end{tabular}\n")


def _rotulo_cenario(cen):
    partes = cen.split("/")
    ensaio = NOME_CENARIO.get(partes[-1], partes[-1].replace("_", "\\_"))
    grupo = "/".join(partes[:-1]).replace("_", " ")
    return f"{ensaio} de {grupo}" if grupo in GRUPO_EIXO else f"{ensaio} ({grupo})"


def nota_voos(dados, atitude):
    """A nota que toda tabela leva: a base de atitude e quantos voos ha em cada celula."""
    base = "Atitude verdadeira do SITL" if atitude == "verdadeira" else "Atitude estimada pelo AHRS"
    if not dados.repetidos:
        return f"{base}; um voo por célula."
    nomes = dict(CONTROLADORES + NUCLEO + (("nativo_guinada", NOME_NATIVO_GUINADA),))
    por_n = defaultdict(lambda: (set(), set()))
    for cen, slug in dados.repetidos:
        ctl, cens = por_n[dict.get(dados, (cen, slug))["n"]]
        nomes.setdefault(slug, slug.replace("_", "\\_"))
        ctl.add(nomes[slug])
        cens.add(_rotulo_cenario(cen))
    ordem = list(nomes.values())
    partes = [f"média de {n} voos para {', '.join(sorted(c, key=ordem.index))} em {', '.join(sorted(e))}"
              for n, (c, e) in sorted(por_n.items())]
    return f"{base}; um voo por célula, exceto: " + "; ".join(partes) + "."


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
    """Torque estocastico de um eixo: erro eficaz (o que o LQG minimiza) e esforco maximo.

    Media das sementes; a diferenca ao nativo e a media das diferencas por semente
    (os voos de mesma semente veem o mesmo torque), com IC de 95 % pareado.
    """
    linhas = []
    for grupo, eixo in GRUPO_EIXO.items():
        cen = f"{grupo}/torque_estocastico"
        d = f"disturbio_{eixo}."
        lista = list(NUCLEO) + ([("nativo_guinada", NOME_NATIVO_GUINADA)] if eixo == "yaw" else [])
        for slug, nome in lista:
            if slug in SO_GUINADA and eixo != "yaw":
                continue
            r = dados.get((cen, slug))
            ic = None if slug == "nativo" else ic_pareado(dados, (cen, slug), (cen, "nativo"), d + "rmse_deg")
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, d + 'rmse_deg', 3)} & "
                          f"{'---' if slug == 'nativo' else fmt_ic(ic)} & "
                          f"{num(r, d + 'desvio_max_deg', 2)} & "
                          f"{num(r, f'esforco_{eixo}.u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Controlador & RMSE (\si{\degree}) & $\Delta$ nativo (\%) & "
           r"Desvio máx. (\si{\degree}) & $u_{\max}$ \\")
    return (tabular("llrrrr", linhas, cab),
            r"Média das sementes. $\Delta$: média das diferenças ao nativo semente a semente, "
            r"com intervalo de confiança de 95\,\% pareado ($t$ de Student).")


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


def _com_nativo_guinada(eixo):
    lista = list(NUCLEO)
    if eixo == "yaw":
        lista.append(("nativo_guinada", NOME_NATIVO_GUINADA))
    return [(s, n) for s, n in lista if eixo == "yaw" or s not in SO_GUINADA]


def tabela_impulso(dados):
    """Impulso curto de torque (20 ms, duracao exata pelo SIM_TWIST_TIME), nos tres eixos."""
    linhas = []
    for grupo, eixo in GRUPO_EIXO.items():
        d, e = f"disturbio_{eixo}.", f"esforco_{eixo}."
        for slug, nome in _com_nativo_guinada(eixo):
            r = dados.get((f"{grupo}/torque_impulso", slug))
            linhas.append(f"{NOMES[eixo]} & {nome} & {num(r, d + 'desvio_max_deg', 2)} & "
                          f"{num(r, d + 'tempo_recuperacao_s', 2)} & "
                          f"{num(r, d + 'rmse_deg', 3)} & {num(r, e + 'u_max_abs', 3)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Controlador & Desvio máx. (\si{\degree}) & Recuperação (\si{\second}) & "
           r"RMSE (\si{\degree}) & $u_{\max}$ \\")
    return tabular("llrrrr", linhas, cab)


def tabela_ruido_impulso(dados):
    """Impulso nos tres eixos com SIM_GYR_RND = 20: desvio e ruido no esforco, com IC de Welch.

    O nativo e os dois LQG voam 5 vezes; o ruido do giroscopio nao tem semente, e os
    voos nao se pareiam: a diferenca ao nativo e a das medias, com IC de Welch.
    """
    cen = "ruido/rnd20/torque_impulso"
    linhas = []
    for eixo in ("roll", "pitch", "yaw"):
        d = f"disturbio_{eixo}."
        for slug, nome in _com_nativo_guinada(eixo):
            r = dados.get((cen, slug))
            ic = None if slug == "nativo" else ic_welch(dados, (cen, slug), (cen, "nativo"), d + "desvio_max_deg")
            n = (r or {}).get("n", 1 if r else 0)
            linhas.append(f"{NOMES[eixo]} & {nome} & {n} & {num(r, d + 'desvio_max_deg', 2)} & "
                          f"{'---' if slug == 'nativo' or ic is None else fmt_ic(ic)} & "
                          f"{num(r, d + 'tempo_recuperacao_s', 2)} & "
                          f"{num(r, f'calmo_{eixo}.u_rms', 4)} \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r"Eixo & Controlador & Voos & Desvio máx. (\si{\degree}) & $\Delta$ nativo (\%) & "
           r"Recuperação (\si{\second}) & $u_\mathrm{rms}$ calmo \\")
    return (tabular("llrrrrr", linhas, cab),
            r"Médias dos voos. $\Delta$: diferença entre as médias do desvio máximo, relativa à do "
            r"nativo, com intervalo de confiança de 95\,\% de Welch; só onde os dois têm mais de um voo.")


def tabela_nativo_guinada(dados):
    """Guinada: o nativo do AutoTune contra o nativo de guinada ressintonizada, em todos os ensaios."""
    linhas = []

    def linha(rotulo, cen, campo, casas=2, ic=None):
        a, b = dados.get((cen, "nativo")), dados.get((cen, "nativo_guinada"))
        if ic is None:
            rel = _rel(_v(b, campo), _v(a, campo))
        else:
            rel = fmt_ic(ic(dados, (cen, "nativo_guinada"), (cen, "nativo"), campo))
        linhas.append(f"{rotulo} & {num(a, campo, casas)} & {num(b, campo, casas)} & {rel} \\\\")

    d = "disturbio_yaw."
    linha(r"Degrau de atitude: sobressinal (\%)", "guinada/nominal", "degrau_yaw.sobressinal_pct", 1)
    linha(r"Degrau de atitude: erro máx. (\si{\degree})", "guinada/nominal", "degrau_yaw.erro_rastreio_max_deg")
    for ensaio in ("torque_impulso", "torque_pulso", "torque_degrau"):
        rot = NOME_CENARIO[ensaio].capitalize()
        linha(rf"{rot}: desvio máx. (\si{{\degree}})", f"guinada/{ensaio}", d + "desvio_max_deg")
        linha(rf"{rot}: recuperação (\si{{\second}})", f"guinada/{ensaio}", d + "tempo_recuperacao_s")
    linha(r"Estocástico: RMSE (\si{\degree})$^{a}$", "guinada/torque_estocastico", d + "rmse_deg", 3,
          ic=ic_pareado)
    linha(r"Impulso com ruído: desvio máx. (\si{\degree})$^{b}$", "ruido/rnd20/torque_impulso",
          d + "desvio_max_deg", ic=ic_welch)
    linha(r"Impulso com ruído: $u_\mathrm{rms}$ calmo", "ruido/rnd20/torque_impulso", "calmo_yaw.u_rms", 4)
    for ensaio, rot in (("torque_pulso", "Saturação, degrau curto"), ("torque_degrau", "Saturação, degrau")):
        linha(rf"{rot}: desvio máx. (\si{{\degree}})", f"saturacao/guinada/{ensaio}", d + "desvio_max_deg")
        linha(rf"{rot}: recuperação (\si{{\second}})", f"saturacao/guinada/{ensaio}", d + "tempo_recuperacao_s")
    for nome_cen, rot in (("leve", "Falha leve"), ("severa", "Falha severa")):
        linha(rf"{rot}: desvio máx. (\si{{\degree}})", f"falha_motor/{nome_cen}", d + "desvio_max_deg")
        linha(rf"{rot}: recuperação (\si{{\second}})", f"falha_motor/{nome_cen}", d + "tempo_recuperacao_s")
    for rot, grupo in VARIANTES_MODELO[1:]:
        linha(rf"{rot}, degrau: recuperação (\si{{\second}})", f"{grupo}/torque_degrau",
              d + "tempo_recuperacao_s")
    cab = r"Ensaio e métrica & nativo & ressintonizado & $\Delta$ (\%) \\"
    return (tabular("lrrr", linhas, cab,
                    nota=[r"$^{a}$ Média de 5 sementes; $\Delta$ com IC de 95\,\% pareado.",
                          r"$^{b}$ Média de 5 voos; $\Delta$ com IC de 95\,\% de Welch."]),
            "")


def tabela_ablacao(dados):
    """Ablacao do LQG de mesmo ruido em rolagem: sem o estado do atraso do angulo (z) e sem o vies (b_g).

    As variantes foram reprojetadas pela regra do portao (projeto.py --ablacao). Sem o
    modelo do filtro do giroscopio ou sem o erro do AHRS (eta), nenhum ponto da varredura
    cumpre o ruido do nativo: elas nao voam, e o texto da o resultado do modelo.
    """
    linhas = []
    for slug, nome in (("nativo", "nativo"), ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído), completo"),
                       ("cc_lqg_sem_z", r"sem o atraso do ângulo ($z$)"),
                       ("cc_lqg_sem_bg", r"sem o viés do giroscópio ($b_g$)")):
        cel = []
        for ensaio in ("torque_impulso", "torque_degrau"):
            r = dados.get((f"rolagem/{ensaio}", slug))
            cel += [num(r, "disturbio_roll.desvio_max_deg", 3), num(r, "disturbio_roll.tempo_recuperacao_s", 2)]
        linhas.append(f"{nome} & " + " & ".join(cel) + " \\\\")
    cab = (r" & \multicolumn{2}{c}{Impulso} & \multicolumn{2}{c}{Degrau} \\" "\n"
           r"\cmidrule(lr){2-3}\cmidrule(lr){4-5}" "\n"
           r"Controlador & desvio (\si{\degree}) & recup. (\si{\second}) & desvio (\si{\degree}) & "
           r"recup. (\si{\second}) \\")
    return tabular("lrrrr", linhas, cab)


def _transitorio_volta(pasta, atitude, antes=1.0, depois=3.0):
    """Maior |erro de rolagem| depois da volta ao nativo, menos a media do segundo anterior."""
    import csv
    pasta = Path(pasta)
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text(encoding="utf-8")) or {}
    t_volta = (cen.get("voo") or {}).get("volta_nativo_s")
    arq = pasta / ("serie_verdade.csv" if atitude == "verdadeira" else "series.csv")
    if t_volta is None or not arq.exists():
        return None
    with open(arq, newline="", encoding="utf-8") as f:
        tp, e = [], []
        for r in csv.DictReader(f):
            try:
                tp.append(float(r["tp"]))
                e.append(float(r["erro_roll"]))
            except (KeyError, ValueError):
                continue
    pre = [x for t, x in zip(tp, e) if t_volta - antes <= t < t_volta - 0.05]
    pos = [x for t, x in zip(tp, e) if t_volta <= t <= t_volta + depois]
    if not pre or not pos:
        return None
    m = sum(pre) / len(pre)
    return max(abs(x - m) for x in pos)


def tabela_comutacao(dados, atitude):
    """Volta ao nativo sob torque sustentado: o transitorio sem e com a transferencia do integrador."""
    linhas = []
    for slug, nome in (("cc_lqi", "LQI"), ("cc_eso", "LQR+ESO"), ("cc_lqg", "LQG")):
        cel = []
        for cen in ("comutacao/sem_transferencia", "comutacao/com_transferencia"):
            vals = [_transitorio_volta(l["pasta"], atitude) for l in dados.voos.get((cen, slug), [])]
            vals = [v for v in vals if v is not None]
            dados.get((cen, slug))
            cel.append(_virgula(sum(vals) / len(vals), 2) if vals else "---")
        linhas.append(f"{nome} & " + " & ".join(cel) + " \\\\")
    cab = (r" & \multicolumn{2}{c}{Desvio depois da volta (\si{\degree})} \\" "\n"
           r"\cmidrule(lr){2-3}" "\n"
           r"Controlador & sem transferência & com transferência \\")
    return (tabular("lrr", linhas, cab),
            r"Degrau de torque de \SI{10}{\radian\per\second\squared} em rolagem em $t = \SI{6}{\second}$ "
            r"e volta ao nativo em $t = \SI{12}{\second}$; desvio: maior erro de rolagem nos "
            r"\SI{3}{\second} depois da volta, menos a média do segundo anterior.")


def tabela_bases(dados_v, dados_a):
    """Apendice: as duas bases de atitude nos mesmos voos (pico e RMSE)."""
    linhas = []
    casos = (("rolagem/torque_impulso", "roll"), ("rolagem/torque_degrau", "roll"),
             ("rolagem/torque_estocastico", "roll"), ("guinada/torque_impulso", "yaw"),
             ("guinada/torque_degrau", "yaw"), ("guinada/torque_estocastico", "yaw"),
             ("ruido/rnd20/torque_impulso", "roll"), ("ruido/rnd20/torque_impulso", "yaw"))
    for cen, eixo in casos:
        d = f"disturbio_{eixo}."
        for slug, nome in (("nativo", "nativo"), ("cc_lqg", "LQG"), ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)")):
            v, a = dados_v.get((cen, slug)), dados_a.get((cen, slug))
            cel = []
            for campo, casas in (("desvio_max_deg", 3), ("rmse_deg", 3)):
                cel += [num(a, d + campo, casas), num(v, d + campo, casas), _rel(_v(v, d + campo), _v(a, d + campo))]
            linhas.append(f"{_rotulo_cenario(cen)} & {NOMES[eixo]} & {nome} & " + " & ".join(cel) + " \\\\")
        linhas.append(r"\addlinespace")
    linhas.pop()
    cab = (r" & & & \multicolumn{3}{c}{Desvio máx. (\si{\degree})} & \multicolumn{3}{c}{RMSE (\si{\degree})} \\" "\n"
           r"\cmidrule(lr){4-6}\cmidrule(lr){7-9}" "\n"
           r"Ensaio & Eixo & Controlador & AHRS & verdade & $\Delta$ (\%) & AHRS & verdade & $\Delta$ (\%) \\")
    return tabular("lllrrrrrr", linhas, cab)


TABELAS = (("resultados_degrau", tabela_degrau), ("resultados_torque", tabela_torque),
           ("resultados_impulso", tabela_impulso),
           ("resultados_simultaneo", tabela_simultaneo),
           ("resultados_frequencia", tabela_frequencia),
           ("resultados_ruido", tabela_ruido),
           ("resultados_ruido_impulso", tabela_ruido_impulso),
           ("resultados_equivalencia", tabela_equivalencia),
           ("resultados_alimentacao_direta", tabela_alimentacao_direta),
           ("resultados_estocastico", tabela_estocastico),
           ("resultados_niveis_rolagem", lambda d: tabela_niveis_ruido(d, "roll")),
           ("resultados_niveis_guinada", lambda d: tabela_niveis_ruido(d, "yaw")),
           ("resultados_erro_modelo", tabela_erro_modelo),
           ("resultados_falha_motor", tabela_falha_motor),
           ("resultados_grande", tabela_grande),
           ("resultados_saturacao", tabela_saturacao),
           ("resultados_filtro", tabela_filtro),
           ("resultados_nativo_guinada", tabela_nativo_guinada),
           ("resultados_ablacao", tabela_ablacao))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--atitude", choices=("verdadeira", "ahrs"), default="verdadeira",
                    help="base de atitude das metricas (padrao: verdadeira, P3c da revisao)")
    ap.add_argument("--binario", help="hash do binario do SITL (padrao: o de lotes/binario_lote.sha256)")
    ap.add_argument("--destino", type=Path, default=DESTINO)
    args = ap.parse_args()

    binario = args.binario or binario_atual()
    voos = ler_voos(binario, args.atitude)
    print(f"binario {binario[:12]}: {sum(len(v) for v in voos.values())} voos em {len(voos)} pares "
          f"(base {args.atitude})")
    args.destino.mkdir(parents=True, exist_ok=True)

    def gravar(nome, saida, dados):
        tabela, extra = saida if isinstance(saida, tuple) else (saida, "")
        (args.destino / f"{nome}.tex").write_text(tabela, encoding="utf-8")
        nota = nota_voos(dados, args.atitude) + (f" {extra}" if extra else "")
        (args.destino / f"{nome}_nota.tex").write_text(CABECALHO + nota + "\n", encoding="utf-8")
        print(args.destino / f"{nome}.tex")

    for nome, fn in TABELAS + (("resultados_comutacao", lambda d: tabela_comutacao(d, args.atitude)),):
        dados = Dados(voos)
        gravar(nome, fn(dados), dados)
    if args.atitude == "verdadeira":
        dados = Dados(voos)
        gravar("apendice_bases", tabela_bases(dados, Dados(ler_voos(binario, "ahrs"))), dados)


if __name__ == "__main__":
    main()
