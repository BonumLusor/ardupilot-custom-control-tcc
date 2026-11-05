#!/usr/bin/env python3
"""Gera as figuras comparativas do Capitulo 6 a partir dos ensaios no SITL.

Como o tabelas_monografia.py, usa so os voos do binario congelado do SITL, na base
de atitude pedida (a verdadeira, por padrao: o serie_verdade.csv por cima do
series.csv). Cada curva e o primeiro voo do par (semente 1, repeticao 1); a
Figura 26 (comparacao_ruido) e a media dos voos e sai nas duas bases. As curvas
do modelo, onde aparecem, vem do modelagem/projeto.py.

    comparacao_torque.pdf   erro de atitude no degrau curto e no degrau de torque
    comparacao_impulso.pdf  erro de atitude no impulso curto (20 ms)
    comparacao_degrau.pdf   degraus de atitude dos ensaios nominais
    comparacao_frf.pdf      |Theta/D| das varreduras, SITL e modelo
    eso_dhat_sitl.pdf       perturbacao injetada e a estimada pelo ESO no voo
    comparacao_ruido.pdf    esforco e erro eficazes com ruido de giroscopio
    comparacao_ff.pdf       erro de rastreamento: {PID, LQI} x {sem, com} alimentacao direta

    ./figuras_monografia.py                      # base verdadeira, binario atual
    ./figuras_monografia.py --atitude ahrs --destino /tmp/figuras_ahrs
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

RAIZ = Path(__file__).resolve().parent
RESULTADOS = RAIZ / "resultados"
DESTINO = RAIZ.parent / "monografia" / "figuras"
sys.path.insert(0, str(RAIZ))
sys.path.insert(0, str(RAIZ.parent / "modelagem"))

from analise import estilo as E            # noqa: E402

# O LQG das comparacoes e o do portao (cc_lqg_mesmo_ruido): o candidato a ganho
# real sobre o nativo, com o mesmo ruido no esforco; o cc_lqg entra nas figuras da
# fronteira. Os rotulos sao os das tabelas (tabelas_monografia.NUCLEO): "LQG" e o
# cc_lqg, "LQG (mesmo ruído)" o cc_lqg_mesmo_ruido.
CONTROLADORES = (("nativo", "Nativo"), ("cc_lqi", "LQI"), ("cc_eso", "LQR+ESO"),
                 ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
MESMO_RUIDO = ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)")
# o LQI de mesmo ruido so difere do cc_lqi em guinada: como nas tabelas, so aparece la
SO_GUINADA = {"cc_lqi_mesmo_ruido"}
NUCLEO = (("nativo", "Nativo"), ("cc_lqi", "LQI"), ("cc_eso", "LQR+ESO"),
          ("cc_lqi_mesmo_ruido", "LQI (mesmo ruído)"), ("cc_lqg", "LQG"),
          ("cc_lqg_mesmo_ruido", "LQG (mesmo ruído)"))
COR = {"nativo": E.SERIE[0], "cc_lqi": E.SERIE[1], "cc_eso": E.SERIE[2],
       "cc_lqi_mesmo_ruido": E.SERIE[1], "cc_lqg": E.SERIE[3], "cc_lqg_mesmo_ruido": E.SERIE[3]}
# as variantes "(mesmo ruido)" saem tracejadas (ou hachuradas), nas duas familias
TRACO = {"cc_lqi_mesmo_ruido": "--", "cc_lqg_mesmo_ruido": "--"}
NOMES = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
GRUPO = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
COL_ERRO = {"roll": "erro_roll", "pitch": "erro_pitch", "yaw": "erro_yaw"}
COL_ANG = {"roll": ("roll_des", "roll"), "pitch": ("pitch_des", "pitch"),
           "yaw": ("yaw_des_rel", "yaw_rel")}
COL_TQ = {"roll": "tqx", "pitch": "tqy", "yaw": "tqz"}
COL_U = {"roll": "u_roll", "pitch": "u_pitch", "yaw": "u_yaw"}


# --------------------------------------------------------------------------
# leitura
# --------------------------------------------------------------------------

# Os voos e a base de atitude das figuras (main): so os do binario congelado do SITL;
# na base verdadeira, as colunas de atitude e de erro vem do serie_verdade.csv
# (analise/verdade.py), por cima do series.csv.
BASE = {"binario": None, "atitude": "verdadeira"}


def voos_do_par(cenario, slug):
    """As pastas de voo do binario de um cenario x controlador, em ordem de semente e de carimbo."""
    import json
    base = RESULTADOS / cenario / slug
    if not base.exists():
        return []
    pastas = []
    for p in base.iterdir():
        v = p / "versao.json"
        if not (p / "series.csv").exists() or not v.exists():
            continue
        if json.loads(v.read_text(encoding="utf-8")).get("binario_sha256") != BASE["binario"]:
            continue
        if BASE["atitude"] == "verdadeira" and not (p / "serie_verdade.csv").exists():
            continue
        cen = yaml.safe_load((p / "cenario.yaml").read_text(encoding="utf-8")) or {}
        pastas.append(((cen.get("vento") or {}).get("WND_SEED") or 0, p.name, p))
    return [p for _, _, p in sorted(pastas)]


def ensaio_recente(cenario, slug, desde=None):
    """O voo que a figura mostra: o primeiro do binario (semente 1, repeticao 1), ou None."""
    voos = voos_do_par(cenario, slug)
    return voos[0] if voos else None


def ler(pasta):
    import validacao as V
    col = V.ler_series(pasta / "series.csv")
    if BASE["atitude"] == "verdadeira":
        verd = V.ler_series(pasta / "serie_verdade.csv")
        for k, v in verd.items():
            if k in ("t", "tp"):
                continue
            col[k] = v if len(v) == len(col["t"]) else np.interp(col["t"], verd["t"], v)
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text(encoding="utf-8"))
    return col, cen


def trecho_degrau(col, cen, eixo, antes=1.0, depois=4.0):
    """(t desde o inicio da subida do alvo, alvo - y0, medida - y0) no degrau de ida.

    O degrau e localizado pelo instante comandado (voo.t_degrau_s) no tempo de
    perfil gravado no log, e nao por um detector: a ida e a volta tem o mesmo
    modulo, e um detector pode pegar a volta. A origem e o instante em que o
    alvo passa de 2 % da amplitude.
    """
    voo = cen.get("voo") or {}
    amp = float(voo.get(f"degrau_{eixo}_deg") or 0.0)
    if amp == 0.0:
        return None
    c_ref, c_med = COL_ANG[eixo]
    t, tp, ref, med = col["t"], col["tp"], col[c_ref], col[c_med]
    ok = np.isfinite(tp)
    t_cmd = float(voo.get("t_degrau_s", 0.0))
    i0 = np.nonzero(ok & (tp >= t_cmd))[0]
    if not len(i0):
        return None
    t0 = t[i0[0]]
    y0 = float(np.nanmean(ref[(t >= t0 - 0.5) & (t < t0)]))
    sel = (t >= t0 - antes) & (t <= t0 + depois)
    subiu = np.nonzero(sel & (t >= t0) & (np.abs(ref - y0) > 0.02 * abs(amp)))[0]
    if not len(subiu):
        return None
    t_ini = t[subiu[0]]
    return t[sel] - t_ini, ref[sel] - y0, med[sel] - y0


def t_do_torque(col, cen):
    """Instante (base de tempo da serie) em que o torque comeca, pelo tempo de perfil."""
    tp = col["tp"]
    t0 = float(cen["vento"]["WND_TQT0"])
    i = np.nonzero(np.isfinite(tp) & (tp >= t0))[0]
    return float(col["t"][i[0]]) if len(i) else None


def _eixo_limpo(ax):
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)


def _estilo():
    import matplotlib
    matplotlib.use("Agg")
    E.aplicar()
    matplotlib.rcParams.update({"font.family": "serif", "axes.grid": True})
    import matplotlib.pyplot as plt
    return plt


def _salvar(fig, destino):
    import matplotlib.pyplot as plt
    fig.tight_layout()
    destino.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destino)
    plt.close(fig)
    return destino


def _log_legivel(ax):
    """Marcas dos eixos log em decimal com virgula, sem notacao 10^x."""
    import matplotlib.ticker as tk
    fmt = tk.FuncFormatter(lambda v, _: f"{v:g}".replace(".", ","))
    ax.xaxis.set_major_locator(tk.FixedLocator([0.1, 0.2, 0.5, 1, 2, 5, 10]))
    ax.xaxis.set_major_formatter(fmt)
    ax.xaxis.set_minor_formatter(tk.NullFormatter())
    ax.yaxis.set_major_locator(tk.LogLocator(subs=(1.0, 2.0, 5.0)))
    ax.yaxis.set_major_formatter(fmt)
    ax.yaxis.set_minor_formatter(tk.NullFormatter())


def _controladores(eixo):
    return CONTROLADORES + ((MESMO_RUIDO,) if eixo == "yaw" else ())


# --------------------------------------------------------------------------
# figuras
# --------------------------------------------------------------------------

TIPO_TORQUE = {"torque_pulso": "degrau curto", "torque_degrau": "degrau", "torque_impulso": "impulso"}


def fig_torque(desde, destino):
    casos = (("torque_pulso", "roll", (-0.3, 3.0)), ("torque_pulso", "yaw", (-0.3, 4.0)),
             ("torque_degrau", "roll", (-0.3, 4.0)), ("torque_degrau", "yaw", (-1.0, 30.0)))
    return _fig_casos_torque(desde, destino, casos, (2, 2), (6.3, 4.6))


def fig_impulso(desde, destino):
    """Impulso curto (20 ms) em rolagem e em guinada."""
    casos = (("torque_impulso", "roll", (-0.1, 1.5)), ("torque_impulso", "yaw", (-0.1, 4.0)))
    return _fig_casos_torque(desde, destino, casos, (1, 2), (6.3, 2.5))


def _fig_casos_torque(desde, destino, casos, grade, tamanho):
    plt = _estilo()
    fig, eixos = plt.subplots(*grade, figsize=tamanho, squeeze=False)
    algum = False
    for ax, (ensaio, eixo, xlim) in zip(eixos.flat, casos):
        amp = None
        for slug, rotulo in _controladores(eixo):
            pasta = ensaio_recente(f"{GRUPO[eixo]}/{ensaio}", slug, desde)
            if pasta is None:
                continue
            col, cen = ler(pasta)
            t0 = t_do_torque(col, cen)
            if t0 is None:
                continue
            t = col["t"] - t0
            erro = col[COL_ERRO[eixo]]
            base = np.nanmean(erro[(t > -1.0) & (t < -0.1)])
            ax.plot(t, erro - base, color=COR[slug], ls=TRACO.get(slug, "-"), label=rotulo)
            amp = float(cen["vento"][{"roll": "WND_TQX", "yaw": "WND_TQZ"}[eixo]])
            algum = True
        ax.axhline(0, color=E.MUDO, lw=0.8, zorder=0)
        ax.set_xlim(*xlim)
        titulo = f"{NOMES[eixo].capitalize()}, {TIPO_TORQUE[ensaio]}"
        if amp is not None:
            dur = float(cen["vento"].get("WND_TQDUR", 0) or 0)
            titulo += f" de {amp:g} rad/s²".replace(".", ",")
            if ensaio == "torque_impulso":
                titulo += f" por {dur * 1e3:g} ms"
        ax.set_title(titulo, loc="left")
        ax.set_xlabel("Tempo desde o início do torque (s)")
        ax.set_ylabel("Erro de atitude (°)")
        _eixo_limpo(ax)
    eixos[0, 0].legend(loc="lower right")
    # a guinada tem uma linha a mais, o LQI de mesmo ruido: so ela vai na legenda de la
    linhas_g = [ln for ln in eixos[0, 1].get_lines() if ln.get_label() == MESMO_RUIDO[1]]
    if linhas_g:
        eixos[0, 1].legend(handles=linhas_g, loc="lower right")
    return _salvar(fig, destino) if algum else None


def fig_degrau(desde, destino):
    plt = _estilo()
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    algum = False
    for ax, eixo, xlim in zip(eixos, ("roll", "yaw"), ((-0.05, 0.4), (-0.2, 2.5))):
        ref_desenhado = False
        for slug, rotulo in _controladores(eixo):
            pasta = ensaio_recente(f"{GRUPO[eixo]}/nominal", slug, desde)
            if pasta is None:
                continue
            col, cen = ler(pasta)
            trecho = trecho_degrau(col, cen, eixo)
            if trecho is None:
                continue
            tt, r, y = trecho
            ax.plot(tt, y, color=COR[slug], ls=TRACO.get(slug, "-"), label=rotulo)
            if not ref_desenhado:
                ax.plot(tt, r, color=E.REFERENCIA, lw=E.LARGURA_REF, ls="--",
                        label="Alvo (nativo)", zorder=1)
                ref_desenhado = True
            algum = True
        ax.set_xlim(*xlim)
        ax.set_xlabel("Tempo desde o degrau (s)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        _eixo_limpo(ax)
    eixos[0].set_ylabel("Ângulo (°)")
    eixos[1].legend(loc="lower right")
    return _salvar(fig, destino) if algum else None


def fig_frf(desde, destino):
    """|Theta/D| das varreduras de torque no SITL (pontos) e do modelo (linhas).

    O LQG usa o canal do angulo, entao a linha dele e a da planta real
    (projeto.PLANTA_REAL), com os ganhos do proprio voo; as outras ficam na planta
    do projeto, que difere dela menos de 2 % para quem so usa o giroscopio.
    """
    import validacao as V
    import projeto as P
    from analise.leitura import ler_params
    plt = _estilo()
    mod = P.carregar_modelo()
    proj = P.projetar(mod, eixos=("roll", "yaw"))
    variante = {"nativo": "nativo", "cc_lqi": "lqi", "cc_eso": "eso"}
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.9))
    algum = False
    for ax, eixo in zip(eixos, ("roll", "yaw")):
        for slug, rotulo in CONTROLADORES:
            pasta = ensaio_recente(f"{GRUPO[eixo]}/torque_chirp", slug, desde)
            if slug in variante:
                an = proj[eixo][variante[slug]]["analise"]
            elif pasta is not None:
                ctl, _ = V.montar_controlador("cc_lqg", "CC4", ler_params(pasta / "params.parm"),
                                              eixo, mod.a, mod.b[eixo], mod.ts)
                an = P.analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro,
                                planta=P.PLANTA_REAL)
            else:
                continue
            f_m = an.w / (2 * np.pi)
            faixa = (0.1, 10.0)
            if pasta is not None:
                col, cen = ler(pasta)
                v = cen["vento"]
                f0, f1 = float(v["WND_TQF0"]), float(v["WND_TQF1"])
                faixa = (f0, f1)
                tp = col["tp"]
                t0 = float(v["WND_TQT0"])
                dentro = np.isfinite(tp) & (tp >= t0) & (tp <= t0 + float(v["WND_TQDUR"]))
                # torque segurado entre as escritas do injetor, como a fisica o aplica
                _vr, wnd = V.ler_bat_wnd(pasta)
                u = V.torque_segurado(col["t"][dentro], wnd, COL_TQ[eixo])
                f, h, coer = V.resposta_freq(col["t"][dentro], u,
                                             col[COL_ANG[eixo][1]][dentro], f0, f1)
                ok = coer >= V.COERENCIA_MINIMA
                ax.loglog(f[ok], np.abs(h[ok]), "o", ms=3, color=COR[slug], alpha=0.75)
                algum = True
            sel = (f_m >= faixa[0]) & (f_m <= faixa[1])
            ax.loglog(f_m[sel], np.degrees(np.abs(an.S_d[sel])), color=COR[slug], label=rotulo)
        ax.set_xlabel("Frequência (Hz)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        _log_legivel(ax)
        _eixo_limpo(ax)
    eixos[0].set_ylabel("$|\\Theta/D|$ (° por rad/s²)")
    eixos[0].legend(loc="lower right", title="linha: modelo; ponto: SITL", title_fontsize=7)
    return _salvar(fig, destino) if algum else None


def fig_eso(desde, destino):
    plt = _estilo()
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    algum = False
    for ax, (ensaio, eixo, xlim) in zip(eixos, (("torque_pulso", "roll", (-0.2, 1.2)),
                                                ("torque_degrau", "yaw", (-0.5, 3.0)))):
        pasta = ensaio_recente(f"{GRUPO[eixo]}/{ensaio}", "cc_eso", desde)
        if pasta is None:
            continue
        col, cen = ler(pasta)
        if f"d_hat_{eixo}" not in col:
            continue
        t = col["t"] - t_do_torque(col, cen)
        ax.plot(t, col[COL_TQ[eixo]], color=E.REFERENCIA, lw=E.LARGURA_REF, ls="--",
                label="Injetada $d$")
        ax.plot(t, col[f"d_hat_{eixo}"], color=COR["cc_eso"], label="Estimada $\\hat d$")
        ax.set_xlim(*xlim)
        tipo = "pulso" if "pulso" in ensaio else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}, {tipo} (SITL)", loc="left")
        ax.set_xlabel("Tempo desde o início do torque (s)")
        _eixo_limpo(ax)
        algum = True
    eixos[0].set_ylabel("Perturbação (rad/s²)")
    eixos[0].legend(loc="upper right")
    return _salvar(fig, destino) if algum else None


CEN_RUIDO = "ruido/rnd20/torque_impulso"     # o L2 do lote: 5 voos do nativo e dos LQG


def fig_ruido(desde, destino):
    """Esforco e erro eficazes no trecho calmo do impulso com RND 20, por controlador.

    Barras: media dos voos do par (5 no nativo e nos LQG, 1 nos demais); traco: o
    desvio padrao entre eles. Sai na base de atitude do BASE; o main gera as duas.
    """
    plt = _estilo()
    ctl = CONTROLADORES + (MESMO_RUIDO, ("cc_lqg", "LQG"))
    med = {k: {s: [] for s, _ in ctl} for k in ("u", "e")}
    dp = {k: {s: [] for s, _ in ctl} for k in ("u", "e")}
    algum = False
    for slug, _ in ctl:
        voos = [ler(p) for p in voos_do_par(CEN_RUIDO, slug)]
        for eixo in ("roll", "pitch", "yaw"):
            vals = {"u": [], "e": []}
            if not (slug in SO_GUINADA and eixo != "yaw"):
                for col, cen in voos:
                    tp = col["tp"]
                    # do primeiro segundo do perfil ate meio segundo antes do torque
                    calmo = np.isfinite(tp) & (tp >= 1.0) & (tp <= float(cen["vento"]["WND_TQT0"]) - 0.5)
                    for k, c in (("u", COL_U[eixo]), ("e", COL_ERRO[eixo])):
                        x = col[c][calmo]
                        vals[k].append(float(np.sqrt(np.nanmean((x - np.nanmean(x)) ** 2))))
            for k in ("u", "e"):
                med[k][slug].append(float(np.mean(vals[k])) if vals[k] else np.nan)
                dp[k][slug].append(float(np.std(vals[k], ddof=1)) if len(vals[k]) > 1 else np.nan)
                algum |= bool(vals[k])
    if not algum:
        return None
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    x = np.arange(3)
    larg = 0.8 / len(ctl)
    for ax, k, rot in ((eixos[0], "u", "Esforço eficaz $u_{\\mathrm{rms}}$"),
                       (eixos[1], "e", "Erro de atitude eficaz (°)")):
        for j, (slug, rotulo) in enumerate(ctl):
            ax.bar(x + (j - (len(ctl) - 1) / 2) * larg, med[k][slug], larg * 0.92,
                   yerr=dp[k][slug], error_kw={"elinewidth": 0.6, "capsize": 1.5},
                   color=COR[slug], hatch="//" if slug in TRACO else None,
                   edgecolor="white", label=rotulo)
        ax.set_xticks(x)
        ax.set_xticklabels([NOMES[e].capitalize() for e in ("roll", "pitch", "yaw")])
        ax.set_title(rot, loc="left")
        ax.grid(False, axis="x")
        _eixo_limpo(ax)
    eixos[0].legend(loc="upper left", fontsize=7)
    return _salvar(fig, destino)


def fig_alimentacao_direta(desde, destino):
    """Erro de rastreamento no degrau: a cor e a realimentacao, o traco e a alimentacao direta."""
    plt = _estilo()
    quadro = (("cc_pid", "PID, sem alim. direta", E.SERIE[0], "--"),
              ("cc_pid_ff", "PID, com alim. direta", E.SERIE[0], "-"),
              ("cc_lqi_sem_ff", "LQI, sem alim. direta", E.SERIE[1], "--"),
              ("cc_lqi", "LQI, com alim. direta", E.SERIE[1], "-"))
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.9))
    algum = False
    for ax, eixo, xlim in zip(eixos, ("roll", "yaw"), ((-0.05, 0.5), (-0.2, 3.0))):
        for slug, rotulo, cor, traco in quadro:
            pasta = ensaio_recente(f"{GRUPO[eixo]}/nominal", slug, desde)
            if pasta is None:
                continue
            col, cen = ler(pasta)
            trecho = trecho_degrau(col, cen, eixo)
            if trecho is None:
                continue
            tt, r, y = trecho
            ax.plot(tt, r - y, color=cor, ls=traco, label=rotulo)
            algum = True
        ax.axhline(0, color=E.MUDO, lw=0.8, zorder=0)
        ax.set_xlim(*xlim)
        ax.set_xlabel("Tempo desde o degrau (s)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        _eixo_limpo(ax)
    eixos[0].set_ylabel("Erro de rastreamento (°)")
    eixos[1].legend(loc="upper right", fontsize=7)
    return _salvar(fig, destino) if algum else None


def _metricas(cenario, slug, desde):
    pasta = ensaio_recente(cenario, slug, desde)
    arq = "metricas_verdade.json" if BASE["atitude"] == "verdadeira" else "metricas.json"
    if pasta is None or not (pasta / arq).exists():
        return None
    import json
    return json.loads((pasta / arq).read_text(encoding="utf-8"))


NIVEIS = ((0, "rolagem_arfagem_guinada"), (10, "ruido/rnd10"), (20, "ruido/rnd20"),
          (40, "ruido/rnd40"), (80, "ruido/rnd80"))


def fig_fronteira_sitl(desde, destino):
    """A fronteira rejeicao x ruido medida: u eficaz no trecho calmo x desvio, por nivel.

    Cada controlador e uma curva: um ponto por nivel de SIM_GYRn_RND (0, 10, 20, 40,
    80), com o ruido no esforco medido antes do pulso e o desvio no ensaio que
    define a metrica primaria do eixo (pulso em rolagem, degrau em guinada).
    """
    import matplotlib.ticker as tk
    plt = _estilo()
    casos = (("roll", "torque_pulso", "Rolagem: desvio no pulso de torque"),
             ("yaw", "torque_degrau", "Guinada: desvio no degrau de torque"))
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 3.0))
    algum = False
    for ax, (eixo, ensaio, titulo) in zip(eixos, casos):
        for slug, rotulo in NUCLEO:
            if slug in SO_GUINADA and eixo != "yaw":
                continue
            x, y = [], []
            for nivel, grupo in NIVEIS:
                mp = _metricas(f"{grupo}/torque_pulso", slug, desde)
                me = _metricas(f"{grupo}/{ensaio}", slug, desde)
                u = ((mp or {}).get(f"calmo_{eixo}") or {}).get("u_rms")
                dv = ((me or {}).get(f"disturbio_{eixo}") or {}).get("desvio_max_deg")
                if u is None or dv is None:
                    continue
                x.append(u)
                y.append(dv)
            if not x:
                continue
            algum = True
            ax.plot(x, y, color=COR[slug], ls=TRACO.get(slug, "-"), marker="o", ms=3.5,
                    label=rotulo)
        ax.set_xscale("log")
        # sem rotulos nas marcas menores: em rolagem eles se sobrepunham
        ax.xaxis.set_minor_formatter(tk.NullFormatter())
        ax.set_title(titulo, loc="left")
        ax.set_xlabel("Ruído no esforço, $u_{\\mathrm{rms}}$ calmo")
        _eixo_limpo(ax)
    if not algum:
        plt.close(fig)
        return None
    eixos[0].set_ylabel("Desvio máximo (°)")
    # legenda abaixo dos paineis: dentro da guinada ela cobria uma curva
    alcas, rotulos = eixos[1].get_legend_handles_labels()
    fig.legend(alcas, rotulos, loc="lower center", ncol=3, fontsize=7, frameon=False)
    fig.tight_layout(rect=(0, 0.13, 1, 1))
    destino.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destino)
    plt.close(fig)
    return destino


def fig_estocastico(desde, destino):
    """Torque estocastico: erro eficaz de cada controlador, relativo ao nativo, por eixo."""
    plt = _estilo()
    fig, ax = plt.subplots(figsize=(6.3, 2.8))
    x = np.arange(3)
    larg = 0.8 / len(NUCLEO)
    algum = False
    for j, (slug, rotulo) in enumerate(NUCLEO):
        rel = []
        for eixo in ("roll", "pitch", "yaw"):
            if slug in SO_GUINADA and eixo != "yaw":
                rel.append(np.nan)
                continue
            cen = f"{GRUPO[eixo]}/torque_estocastico"
            m, n = _metricas(cen, slug, desde), _metricas(cen, "nativo", desde)
            a = ((m or {}).get(f"disturbio_{eixo}") or {}).get("rmse_deg")
            b = ((n or {}).get(f"disturbio_{eixo}") or {}).get("rmse_deg")
            rel.append(a / b if a and b else np.nan)
        algum = algum or np.isfinite(rel).any()
        ax.bar(x + (j - (len(NUCLEO) - 1) / 2) * larg, rel, larg * 0.92, color=COR[slug],
               hatch="//" if slug in TRACO else None, edgecolor="white", label=rotulo)
    if not algum:
        plt.close(fig)
        return None
    ax.axhline(1.0, color=E.MUDO, lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels([NOMES[e].capitalize() for e in ("roll", "pitch", "yaw")])
    ax.set_ylabel("RMSE (× nativo)")
    ax.grid(False, axis="x")
    _eixo_limpo(ax)
    ax.legend(loc="upper left", fontsize=7, ncol=3)
    return _salvar(fig, destino)


def main():
    import warnings
    warnings.filterwarnings("ignore")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--atitude", choices=("verdadeira", "ahrs"), default="verdadeira",
                    help="base de atitude (padrao: verdadeira, P3c da revisao)")
    ap.add_argument("--binario", help="hash do binario do SITL (padrao: o de lotes/binario_lote.sha256)")
    ap.add_argument("--destino", type=Path, default=DESTINO)
    args = ap.parse_args()
    args.desde = None
    import tabelas_monografia
    BASE["binario"] = args.binario or tabelas_monografia.binario_atual()
    BASE["atitude"] = args.atitude
    if args.atitude == "verdadeira":
        # a Figura 26 sai tambem no AHRS: a diferenca entre as bases e resultado (rev. 2.2)
        BASE["atitude"] = "ahrs"
        d = fig_ruido(None, args.destino / "comparacao_ruido_ahrs.pdf")
        print(f"{'figura: ' + str(d) if d else 'sem ensaios para comparacao_ruido_ahrs.pdf'}")
        BASE["atitude"] = "verdadeira"
    for nome, fn in (("comparacao_torque.pdf", fig_torque), ("comparacao_degrau.pdf", fig_degrau),
                     ("comparacao_frf.pdf", fig_frf), ("eso_dhat_sitl.pdf", fig_eso),
                     ("comparacao_impulso.pdf", fig_impulso),
                     ("comparacao_ruido.pdf", fig_ruido),
                     ("comparacao_ff.pdf", fig_alimentacao_direta),
                     ("comparacao_fronteira.pdf", fig_fronteira_sitl),
                     ("comparacao_estocastico.pdf", fig_estocastico)):
        d = fn(args.desde, args.destino / nome)
        print(f"{'figura: ' + str(d) if d else 'sem ensaios para ' + nome}")


if __name__ == "__main__":
    main()
