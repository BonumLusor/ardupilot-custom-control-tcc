"""Painel estatico de um ensaio: seis faixas no mesmo eixo de tempo.

A leitura pretendida e de cima para baixo, seguindo a cadeia causal do ensaio:
o vento chega, a atitude se desvia, o erro cresce, a malha de taxa reage e o
controlador gasta esforco. Como todas as faixas compartilham o eixo de tempo e a
janela do disturbio aparece sombreada em todas, da para acompanhar um evento
atravessando o sistema inteiro sem trocar de figura.

Cada eixo exercitado pelo ensaio (rolagem, guinada) ganha a sua coluna com a
cadeia inteira, do disturbio daquele eixo aos motores com o diferencial que
produz o momento dele. As colunas ficam lado a lado e no mesmo tempo; um eixo
so da a figura de sempre. Faixas de largura total sobre colunas estreitas
poriam o mesmo instante em abscissas diferentes — e ler um evento na vertical
e o motivo de o painel existir.

A tabela de metricas fica ao lado, na propria figura: ela e o alivio exigido pelas
cores de baixo contraste e evita que a figura circule sem os numeros que a sustentam.
"""

from __future__ import annotations

import textwrap

import numpy as np
import matplotlib

matplotlib.use("Agg")          # sem display: os ensaios rodam headless
import matplotlib.pyplot as plt   # noqa: E402

from . import estilo             # noqa: E402
from .eixos import COLUNAS, EIXOS, ativos, nome   # noqa: E402


def _fmt(v, casas=2, sufixo=""):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "—"
    return f"{v:.{casas}f}{sufixo}"


def _linhas_metricas(m):
    """Monta o bloco lateral como uma lista de (rotulo, valor)."""
    linhas = []

    v = m.get("vento") or {}
    if v:
        linhas += [("VENTO", ""),
                   ("  modulo max", _fmt(v.get("modulo_max_ms"), 2, " m/s")),
                   ("  modulo medio", _fmt(v.get("modulo_medio_ms"), 2, " m/s")),
                   ("  desvio padrao", _fmt(v.get("modulo_std_ms"), 2, " m/s")),
                   ("", "")]

    tq = m.get("torque") or {}
    if tq:
        linhas += [("TORQUE APLICADO", "")]
        por_eixo = [e for e in EIXOS if f"{e}_pico_rads2" in tq]
        if not por_eixo:
            # metricas gravadas antes do torque por eixo
            linhas += [("  pico", _fmt(tq.get("modulo_max_rads2"), 3, " rad/s2")),
                       ("  impulso angular", _fmt(tq.get("impulso_ang_degs"), 1, " deg/s"))]
        for e in por_eixo:
            linhas.append((f"  pico {nome(e)}", _fmt(tq.get(f"{e}_pico_rads2"), 3, " rad/s2")))
            if f"{e}_pico_nm" in tq:
                linhas.append(("    = I x alfa", _fmt(tq.get(f"{e}_pico_nm"), 4, " N.m")))
            if f"{e}_impulso_degs" in tq:
                linhas.append(("    impulso", _fmt(tq.get(f"{e}_impulso_degs"), 1, " deg/s")))
        linhas.append(("", ""))

    for e in EIXOS:
        if f"degrau_{e}" not in m:
            continue
        d = m.get(f"degrau_{e}") or {}
        linhas.append((f"DEGRAU ({nome(e)})", ""))
        if not d:
            linhas += [("  nao encontrado no log", ""), ("", "")]
            continue
        linhas += [("  amplitude", _fmt(d.get("amplitude_deg"), 1, " deg")),
                   ("  t. subida", _fmt(d.get("tempo_subida_s"), 3, " s")),
                   ("  sobressinal", _fmt(d.get("sobressinal_pct"), 1, " %")),
                   ("  t. acomodacao", _fmt(d.get("tempo_acomodacao_s"), 2, " s")),
                   ("  erro regime", _fmt(d.get("erro_regime_deg"), 2, " deg")),
                   ("  erro rastr. max", _fmt(d.get("erro_rastreio_max_deg"), 3, " deg")),
                   ("", "")]

    for e in EIXOS:
        p = m.get(f"disturbio_{e}") or {}
        if p:
            linhas += [(f"DISTURBIO ({nome(e)})", ""),
                       ("  desvio max", _fmt(p.get("desvio_max_deg"), 2, " deg")),
                       ("  t. recuperacao", _fmt(p.get("tempo_recuperacao_s"), 2, " s")),
                       ("  RMSE", _fmt(p.get("rmse_deg"), 3, " deg")),
                       ("  erro medio abs", _fmt(p.get("erro_medio_abs_deg"), 3, " deg")),
                       ("", "")]

    # A deriva NAO aparece aqui de proposito: o TCC e sobre controle de atitude, e
    # posicao nao esta entre as metricas da proposta. Ela continua no
    # metricas.json, no series.csv, no consolidado.csv e na tabela.tex, onde serve
    # de evidencia de que o vento foi de fato aplicado.

    for e in EIXOS:
        es = m.get(f"esforco_{e}") or {}
        if es:
            linhas += [(f"ESFORCO ({nome(e)})", ""),
                       ("  integral |u|", _fmt(es.get("esforco_abs"), 3)),
                       ("  integral u2", _fmt(es.get("esforco_quad"), 4)),
                       ("  |u| max", _fmt(es.get("u_max_abs"), 4)),
                       ("", "")]

    while linhas and linhas[-1] == ("", ""):
        linhas.pop()
    return linhas


def _faixa_disturbio(ax, t, col, eixo=None):
    """Faixa 1: o disturbio aplicado — so o do ``eixo``, quando ha mais de um.

    A faixa mostra o TORQUE quando ele existe, porque e ele que perturba a malha
    de atitude — as faixas de baixo. O vento so vem para o topo quando e o
    unico disturbio do ensaio; senao, seria o grafico mais visivel da figura
    explicando o efeito menos relevante dela.
    """
    tem_tq = "tq_mod" in col and np.nanmax(np.abs(col["tq_mod"])) > 1e-4

    if tem_tq:
        estilo.titulo_faixa(ax, "Torque aplicado (aceleracao angular)", "rad/s²")
        tem = False
        for e_tq, chave, rotulo, slot in (("roll", "tqx", "rolagem", 0),
                                          ("pitch", "tqy", "arfagem", 1),
                                          ("yaw", "tqz", "guinada", 2)):
            if eixo is not None and e_tq != eixo:
                continue
            if chave in col and np.nanmax(np.abs(col[chave])) > 1e-4:
                ax.plot(t, col[chave], color=estilo.SERIE[slot], label=rotulo)
                tem = True
                # a estimativa do ESO (backend LQR, CC3_MODO 1), na mesma escala;
                # no LQI o d_hat fica parado no valor do reset e nao entra
                d_hat = col.get(f"d_hat_{e_tq}")
                if d_hat is not None and np.nanstd(d_hat) > 1e-4:
                    ax.plot(t, d_hat, color=estilo.SERIE[slot], ls="--",
                            label=f"{rotulo}, estimado (ESO)")
        if tem:
            estilo.legenda(ax, ncols=3)
        else:
            ax.axhline(0.0, color=estilo.MUDO, lw=0.8)
            ax.text(0.5, 0.5, f"sem torque em {nome(eixo)} neste ensaio",
                    transform=ax.transAxes, ha="center", va="center",
                    color=estilo.MUDO, fontsize=9)
        if "w_mod" in col and np.nanmax(col["w_mod"]) > 0.05:
            ax.text(0.995, 0.06,
                    f"vento simultaneo: pico {np.nanmax(col['w_mod']):.1f} m/s",
                    transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=7.5, color=estilo.MUDO)
    elif "wn" in col:
        estilo.titulo_faixa(ax, "Vento aplicado", "m/s")
        ax.plot(t, col["wn"], color=estilo.SERIE[0], label="componente norte")
        ax.plot(t, col["we"], color=estilo.SERIE[1], label="componente leste")
        ax.plot(t, col["w_mod"], color=estilo.SERIE[2], label="modulo")
        estilo.legenda(ax, ncols=3)
    else:
        estilo.titulo_faixa(ax, "Disturbio aplicado")
        ax.text(0.5, 0.5, "sem mensagem WND neste log",
                transform=ax.transAxes, ha="center", va="center",
                color=estilo.MUDO, fontsize=9)


def _faixa_motores(ax, t, col, frame_type, eixo):
    """Faixa 6: os quatro motores e o diferencial que produz o momento do eixo.

    E aqui que um disturbio lento aparece. Sob torque em degrau a atitude quase
    nao se move — o integrador a segura —, mas os motores ficam permanentemente
    desbalanceados para produzir o contra-torque. Ler so a atitude daria a
    impressao de que nada aconteceu; a diferenca entre os pares mostra o preco.
    """
    estilo.titulo_faixa(ax, "Saida dos motores", "PWM")
    motores = [ch for ch in ("m1", "m2", "m3", "m4") if ch in col]
    for k, chave in enumerate(motores):
        ax.plot(t, col[chave], color=estilo.SERIE[k], label=f"motor {chave[1]}")
    alcas = list(ax.get_lines())
    if len(motores) == 4:
        #  O diferencial sai da geometria e do sentido de giro do frame do ensaio
        #  (estilo.diferencial_motores): rolagem e arfagem pela posicao de cada
        #  motor, guinada pelo par CCW contra o par CW. Chumbar a formula combina
        #  eixos diferentes assim que o frame muda — e o grafico passa a mostrar
        #  ruido com cara de sinal.
        nome_frame, _ = estilo.geometria_quad(frame_type)
        rotulos_dif = {"roll": "dif. rolagem (dir - esq)",
                       "pitch": "dif. arfagem (diant - tras)",
                       "yaw": "dif. guinada (CCW - CW)"}
        dif = estilo.diferencial_motores(col, frame_type, eixo)
        ax2 = ax.twinx()
        if dif is not None:
            alcas += ax2.plot(t, dif, color=estilo.MUDO if eixo == "roll" else estilo.TINTA_2,
                              lw=1.0, ls=":" if eixo == "roll" else "--",
                              label=rotulos_dif[eixo])
        ax2.set_ylabel(f"diferencial, frame {nome_frame} (PWM)",
                       fontsize=7, color=estilo.MUDO)
        ax2.tick_params(labelsize=7, colors=estilo.MUDO)
        for lado in ("top", "left", "bottom"):
            ax2.spines[lado].set_visible(False)
        ax2.spines["right"].set_color(estilo.GRADE)
    if motores:
        estilo.legenda(ax, ncols=len(alcas), handles=alcas)


def desenhar(grade, col, metricas, controlador="—", cenario="—",
             janela_disturbio=None, titulo=None, frame_type=1, eixos_cc=()):
    """Monta a figura do ensaio e devolve o objeto Figure.

    Cada eixo exercitado ganha uma COLUNA com a cadeia inteira — disturbio,
    atitude, erro, taxa, controlador e motores —, para que um evento continue
    sendo lido na vertical. As colunas compartilham o eixo de tempo, entao a
    rolagem e a guinada de um ensaio combinado se comparam lado a lado.

    ``eixos_cc`` sao os eixos em que o AC_CustomControl esteve no comando. Neles
    a faixa do controlador mostra so a saida: o backend nao grava os proprios
    termos, e o PIDR/PIDY desses eixos registra o PID nativo rodando em sombra,
    com o integrador zerado a cada ciclo pelo motor_set do AC_CustomControl.
    """
    estilo.aplicar()

    eixos_plot = [e for e in ativos(metricas) if COLUNAS[e]["med"] in col] or ["roll"]
    n = len(eixos_plot)

    # com um so eixo, a figura e a de sempre
    fig = plt.figure(figsize=(13.0 + 9.5 * (n - 1), 12.0))
    gs = fig.add_gridspec(6, n + 1, width_ratios=[3.5] * n + [1.0],
                          hspace=0.32, wspace=0.04 if n == 1 else 0.16,
                          left=0.055 if n == 1 else 0.035, right=0.985,
                          top=0.945, bottom=0.05)

    colunas = []                       # colunas[j][i]: faixa i da coluna do eixo j
    for j in range(n):
        faixas = []
        for i in range(6):
            primeiro = colunas[0][0] if colunas else (faixas[0] if faixas else None)
            faixas.append(fig.add_subplot(gs[i, j], sharex=primeiro))
        colunas.append(faixas)
    ax_tab = fig.add_subplot(gs[:, n])
    ax_tab.axis("off")
    todos = [ax for c in colunas for ax in c]

    # Eixo em TEMPO DE PERFIL, quando o log traz a coluna tp. E o que torna dois
    # ensaios comparaveis lado a lado: a decolagem dura diferente a cada corrida,
    # entao o tempo de log poe o mesmo evento em abscissas diferentes.
    if "tp" in col:
        t = col["tp"]
        desloc = float(grade[0] - t[0])
        rotulo_x = "tempo do perfil (s)"
    else:
        t = grade
        desloc = 0.0
        rotulo_x = "tempo de log (s)"

    for (ax_dist, ax_ang, ax_err, ax_taxa, ax_ctl, ax_mot), e in zip(colunas, eixos_plot):
        c = COLUNAS[e]
        nm = nome(e)

        # ---- 1. disturbio aplicado -----------------------------------------
        _faixa_disturbio(ax_dist, t, col, eixo=e if n > 1 else None)

        # ---- 2. atitude: comandada x medida -------------------------------
        if e == "yaw":
            #  a guinada vai relativa ao rumo do inicio do perfil; o rumo
            #  absoluto, que nao diz nada sobre o controlador, fica no titulo
            ok = np.isfinite(col.get("yaw", np.nan)) & np.isfinite(col[c["med"]])
            rumo0 = (float(np.mod(col["yaw"][ok][0] - col[c["med"]][ok][0], 360.0))
                     if np.any(ok) else None)
            sub = f", relativa ao rumo inicial ({rumo0:.1f}°)" if rumo0 is not None else ""
            estilo.titulo_faixa(ax_ang, f"Guinada{sub}", "deg")
        else:
            estilo.titulo_faixa(ax_ang, nm.capitalize(), "deg")
        if c["ref"] in col:
            ax_ang.plot(t, col[c["ref"]], color=estilo.REFERENCIA, ls="--",
                        lw=estilo.LARGURA_REF, label="comandada")
        ax_ang.plot(t, col[c["med"]], color=estilo.SERIE[0], label="medida")
        estilo.legenda(ax_ang, ncols=2)

        # ---- 3. erro de atitude ---------------------------------------------
        estilo.titulo_faixa(ax_err, f"Erro de {nm} (comandada - medida)", "deg")
        if c["erro"] in col:
            ax_err.axhline(0.0, color=estilo.MUDO, lw=0.8)
            ax_err.plot(t, col[c["erro"]], color=estilo.SERIE[0])

        # ---- 4. taxa ----------------------------------------------------------
        estilo.titulo_faixa(ax_taxa, f"Taxa de {nm}", "deg/s")
        if c["taxa_ref"] in col:
            ax_taxa.plot(t, col[c["taxa_ref"]], color=estilo.REFERENCIA, ls="--",
                         lw=estilo.LARGURA_REF, label="comandada")
        if c["taxa"] in col:
            ax_taxa.plot(t, col[c["taxa"]], color=estilo.SERIE[0], label="medida")
        estilo.legenda(ax_taxa, ncols=2)

        # ---- 5. termos do controlador e esforco --------------------------------
        if e in eixos_cc:
            estilo.titulo_faixa(ax_ctl, f"AC_CustomControl, {nm}: saida u", "normalizado")
            if c["saida"] in col:
                ax_ctl.plot(t, col[c["saida"]], color=estilo.SERIE[3])
            # no lugar da legenda, que com uma serie so repetiria o titulo
            ax_ctl.text(1.0, 1.0, "o backend nao grava os proprios termos P, I, D",
                        transform=ax_ctl.transAxes, ha="right", va="bottom",
                        fontsize=7.5, color=estilo.MUDO)
        else:
            estilo.titulo_faixa(ax_ctl, f"Controlador de taxa de {nm}: termos e saida",
                                "normalizado")
            rotulos = [(c["pid"] + "p", "termo P", 0), (c["pid"] + "i", "termo I", 1),
                       (c["pid"] + "d", "termo D", 2), (c["saida"], "saida u", 3)]
            tem = False
            for chave, rotulo, slot in rotulos:
                if chave in col:
                    ax_ctl.plot(t, col[chave], color=estilo.SERIE[slot], label=rotulo)
                    tem = True
            if tem:
                estilo.legenda(ax_ctl, ncols=4)

        # ---- 6. saida dos motores ----------------------------------------------
        _faixa_motores(ax_mot, t, col, frame_type, e)
        ax_mot.set_xlabel(rotulo_x)

    # ---- janela do disturbio, sombreada em todas as faixas -----------------
    if janela_disturbio and janela_disturbio[1] > janela_disturbio[0]:
        jd = (janela_disturbio[0] - desloc, janela_disturbio[1] - desloc)
        for ax in todos:
            ax.axvspan(jd[0], jd[1],
                       color=estilo.FAIXA_RAJADA, alpha=estilo.FAIXA_ALPHA, lw=0)
        for faixas in colunas:
            faixas[0].text(jd[0], 1.0, " janela do disturbio",
                           transform=faixas[0].get_xaxis_transform(),
                           va="bottom", ha="left", fontsize=7.5, color=estilo.SERIE[1])

    for faixas in colunas:
        for i, ax in enumerate(faixas):
            estilo.limpar_eixo(ax, ultimo=(i == len(faixas) - 1))
    if len(t):
        colunas[0][0].set_xlim(t[0], t[-1])

    # ---- bloco de metricas --------------------------------------------------
    # Cabecalho primeiro: controlador e cenario sao texto livre e podem ser
    # longos, entao o valor vai ABAIXO do rotulo, com quebra, em vez de disputar
    # a mesma linha com ele.
    y = 0.985
    for rotulo, valor in (("CONTROLADOR", controlador), ("CENARIO", cenario)):
        ax_tab.text(0.0, y, rotulo, transform=ax_tab.transAxes, fontsize=7.5,
                    fontweight="bold", color=estilo.MUDO, va="top", ha="left")
        y -= 0.020
        ax_tab.text(0.0, y, textwrap.fill(str(valor), 30),
                    transform=ax_tab.transAxes, fontsize=8.6,
                    color=estilo.TINTA, va="top", ha="left")
        y -= 0.020 * (1 + str(valor).count("\n") + len(str(valor)) // 30) + 0.014

    # o passo encolhe se a lista nao couber — ensaio combinado tem o dobro dela
    linhas = _linhas_metricas(metricas)
    altura = sum(0.014 if (r, v) == ("", "") else 0.0235 for r, v in linhas)
    escala = min(1.0, (y - 0.01) / altura) if altura > 0 else 1.0
    for rotulo, valor in linhas:
        if rotulo == "" and valor == "":
            y -= 0.014 * escala
            continue
        cabecalho = rotulo and not rotulo.startswith(" ")
        ax_tab.text(0.0, y, rotulo, transform=ax_tab.transAxes,
                    fontsize=8.2 if cabecalho else 8.0,
                    fontweight="bold" if cabecalho else "normal",
                    color=estilo.TINTA if cabecalho else estilo.TINTA_2,
                    va="top", ha="left")
        if valor:
            ax_tab.text(1.0, y, str(valor), transform=ax_tab.transAxes,
                        fontsize=8.0, color=estilo.TINTA,
                        va="top", ha="right", family="monospace")
        y -= 0.0235 * escala

    if titulo:
        fig.suptitle(titulo, x=0.055 if n == 1 else 0.035, ha="left", fontsize=12.5,
                     fontweight="bold", color=estilo.TINTA)

    return fig


def salvar(fig, destino_png, destino_pdf=None, dpi=300):
    """Grava PNG para leitura rapida e PDF vetorial para a monografia."""
    fig.savefig(destino_png, dpi=dpi)
    if destino_pdf:
        fig.savefig(destino_pdf)      # vetorial, sem rasterizar
    plt.close(fig)
