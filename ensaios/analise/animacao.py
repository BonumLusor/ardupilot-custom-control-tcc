"""GIF do ensaio, reconstruido do proprio .BIN.

Tres vistas sincronizadas por um unico cursor de tempo:

  esquerda, acima   atitude em 3D — a aeronave inclinando, com a seta do vento
                    aplicado. A camera acompanha o veiculo, entao a vista mostra
                    ATITUDE, nao deriva.
  esquerda, abaixo  vista de cima com a forca de cada motor: e aqui que um
                    disturbio lento aparece.
  direita           as curvas do ensaio com o cursor correndo — uma por eixo
                    exercitado (rolagem, guinada) e o disturbio —, ligando o que
                    se ve a esquerda ao numero que sai nas metricas.

Separar atitude de deriva nao e capricho: num quadrotor sob vento, o corpo tem
~0,5 m e a deriva chega a dezenas de metros. Numa vista so, ou a aeronave vira um
ponto ou a deriva sai do quadro.

Usa o writer ``pillow`` do matplotlib — o PIL ja acompanha o venv, entao nao ha
dependencia nova nem ffmpeg envolvido.
"""

from __future__ import annotations

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt              # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter   # noqa: E402

from . import estilo                          # noqa: E402
from .eixos import COLUNAS, EIXOS, ativos, nome   # noqa: E402

# geometria do modelo desenhado (metros)
BRACO = 0.25
RAIO_ROTOR = 0.10
# fracao do lado da caixa 3D ocupada pela aeronave — mantem o corpo legivel
ESCALA_VISTA = 0.9


def _rot_ned(roll, pitch, yaw):
    """Matriz de rotacao corpo(FRD) -> NED, convencao aeronautica Z-Y-X."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _ned_para_vista(v):
    """NED -> eixos de exibicao (x=leste, y=norte, z=cima)."""
    return np.array([v[1], v[0], -v[2]])


def _modelo_quad(angulos_deg):
    """Pontos do quadrotor em FRD: (bracos, discos de rotor, nariz).

    ``angulos_deg`` vem da geometria do frame do ensaio, lida do params.parm.
    Chumbar a geometria poria os rotores fora de lugar assim que o frame mudasse,
    e a vista de cima atribuiria o esforco ao motor errado.
    """
    angs = np.radians(angulos_deg)
    rotores = [np.array([BRACO * np.cos(a), BRACO * np.sin(a), 0.0]) for a in angs]
    bracos = [(np.zeros(3), r) for r in rotores]

    theta = np.linspace(0, 2 * np.pi, 24)
    discos = []
    for r in rotores:
        circ = np.stack([r[0] + RAIO_ROTOR * np.cos(theta),
                         r[1] + RAIO_ROTOR * np.sin(theta),
                         np.zeros_like(theta)])
        discos.append(circ)

    nariz = (np.zeros(3), np.array([BRACO * 1.5, 0.0, 0.0]))
    return bracos, discos, nariz


def janela_eventos(grade, col, metricas, folga_antes=2.0, folga_depois=3.0):
    """Recorta o GIF ao trecho com eventos: do degrau ate o fim da recuperacao.

    Sem esse recorte, o GIF cobre o perfil inteiro e gasta a quase totalidade dos
    quadros com a aeronave parada. Medido no primeiro GIF deste projeto: dos 400
    quadros, SEIS tinham movimento. O resultado passava a impressao de um
    controlador lento quando o que ha e uma animacao mal enquadrada — o degrau
    leva 0,22 s, e um pulso de torque, 0,3 s.
    """
    if metricas is None:
        return float(grade[0]), float(grade[-1])

    marcos = []
    for eixo in EIXOS:
        deg = metricas.get(f"degrau_{eixo}") or {}
        if deg.get("t_degrau_s") is not None:
            marcos.append(float(deg["t_degrau_s"]))
    jan = metricas.get("janela_disturbio_s")
    if jan:
        marcos += [float(jan[0]), float(jan[1])]

    if not marcos:
        return float(grade[0]), float(grade[-1])
    return (max(float(grade[0]), min(marcos) - folga_antes),
            min(float(grade[-1]), max(marcos) + folga_depois))


def _amostrar(grade, fps, max_frames, janela, velocidade):
    """Indices dos quadros e a velocidade REAL de reproducao.

    ``velocidade`` e a razao desejada entre tempo de voo e tempo de tela (1.0 =
    tempo real, 0.5 = camera lenta). Se o teto de quadros nao permitir, a funcao
    devolve a velocidade que efetivamente saiu — que e a que vai rotulada na
    figura. Um GIF que mente sobre a propria velocidade e pior que um GIF lento.
    """
    t_ini, t_fim = janela
    sel = np.nonzero((grade >= t_ini) & (grade <= t_fim))[0]
    if len(sel) == 0:
        sel = np.arange(len(grade))
    dt_grade = float(np.median(np.diff(grade))) if len(grade) > 1 else 0.02

    passo = max(int(round((velocidade / fps) / dt_grade)), 1)
    idx = sel[::passo]
    if len(idx) > max_frames:
        idx = idx[np.linspace(0, len(idx) - 1, max_frames).astype(int)]

    if len(idx) > 1:
        dt_quadro = float(np.median(np.diff(grade[idx])))
        real = dt_quadro * fps
    else:
        real = velocidade
    return idx, real


def gerar(grade, col, destino, janela=None, fps=20, max_frames=400,
          controlador="—", cenario="—", metricas=None, velocidade=1.0,
          frame_type=1):
    """Renderiza o GIF do ensaio em ``destino``. Devolve o numero de quadros."""
    if len(grade) == 0:
        return 0
    if janela is None:
        janela = janela_eventos(grade, col, metricas)

    idx, vel_real = _amostrar(grade, fps, max_frames, janela, velocidade)
    if len(idx) < 2:
        return 0

    estilo.aplicar()

    def coluna(nome, padrao=0.0):
        if nome not in col:
            return np.full(len(grade), padrao)
        v = np.asarray(col[nome], dtype=float)
        return np.nan_to_num(v, nan=padrao)

    roll = np.radians(coluna("roll"))
    pitch = np.radians(coluna("pitch"))
    # rumo ABSOLUTO: a vista 3D desenha a aeronave e o vento no referencial NED
    yaw = np.radians(coluna("yaw"))
    pn, pe, pd = coluna("pn"), coluna("pe"), coluna("pd")
    wn, we = coluna("wn"), coluna("we")
    w_mod = np.hypot(wn, we)

    # eixos de atitude exercitados pelo ensaio: cada um ganha uma curva a direita
    eixos_plot = [e for e in ativos(metricas) if COLUNAS[e]["med"] in col] or ["roll"]
    n = len(eixos_plot)

    # eixo em tempo de perfil, igual ao do painel — e o mesmo relogio que o
    # rodape mostra, entao a leitura nao precisa converter nada de cabeca
    eixo = np.asarray(col["tp"], dtype=float) if "tp" in col else np.asarray(grade)
    rotulo_x = "tempo do perfil (s)" if "tp" in col else "tempo de log (s)"

    # Esquerda: atitude 3D sobre a vista dos motores. Direita: uma faixa por eixo
    # exercitado e, embaixo, o disturbio. As duas colunas tem grades proprias,
    # para que a direita ganhe faixas sem espremer a vista 3D.
    fig = plt.figure(figsize=(12.5, 6.4 + 1.6 * (n - 1)))
    # com mais de um eixo o rodape desce para a segunda linha do cabecalho, e a
    # grade desce junto para o titulo da primeira faixa nao ficar embaixo dele
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.15],
                          left=0.02, right=0.965, top=0.905 if n == 1 else 0.865,
                          bottom=0.10 if n == 1 else 0.08, wspace=0.16)
    gs_esq = gs[0, 0].subgridspec(2, 1, height_ratios=[1.35, 1.0], hspace=0.45)
    gs_dir = gs[0, 1].subgridspec(n + 1, 1, hspace=0.45,
                                  height_ratios=[1.35] * n + [1.0])

    ax3d = fig.add_subplot(gs_esq[0, 0], projection="3d")
    axtop = fig.add_subplot(gs_esq[1, 0])
    ax_ang = [fig.add_subplot(gs_dir[0, 0])]
    for j in range(1, n):
        ax_ang.append(fig.add_subplot(gs_dir[j, 0], sharex=ax_ang[0]))
    axw = fig.add_subplot(gs_dir[n, 0], sharex=ax_ang[0])

    # ---- vista 3D de atitude ------------------------------------------------
    nome_frame, angulos = estilo.geometria_quad(frame_type)
    bracos, discos, nariz = _modelo_quad(angulos)
    lado = BRACO * 1.6 / ESCALA_VISTA

    linhas_bracos = [ax3d.plot([], [], [], color=estilo.TINTA_2, lw=2.0)[0]
                     for _ in bracos]
    linhas_discos = [ax3d.plot([], [], [], color=estilo.SERIE[0], lw=1.5)[0]
                     for _ in discos]
    linha_nariz, = ax3d.plot([], [], [], color=estilo.SERIE[1], lw=2.2)
    seta_vento, = ax3d.plot([], [], [], color=estilo.SERIE[2], lw=2.6)
    ax3d.set_title("Atitude (camera acompanha o veiculo)", loc="left",
                   fontsize=9.5, color=estilo.TINTA)
    ax3d.set_xlabel("leste", fontsize=7.5, labelpad=-8)
    ax3d.set_ylabel("norte", fontsize=7.5, labelpad=-8)
    ax3d.set_zlabel("cima", fontsize=7.5, labelpad=-8)
    # A vista 3D existe para mostrar INCLINACAO, nao para medir distancia: os
    # numeros dos eixos so competiriam com o corpo da aeronave por espaco.
    for eixo_3d in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        eixo_3d.set_ticklabels([])
        eixo_3d.set_ticks([])
        eixo_3d.pane.set_alpha(0.0)
        eixo_3d.line.set_color(estilo.GRADE)   # moldura recessiva, nao um esqueleto preto
    ax3d.view_init(elev=22, azim=-58)
    try:
        ax3d.set_box_aspect((1, 1, 0.75), zoom=1.45)
    except TypeError:                       # matplotlib sem o parametro zoom
        ax3d.set_box_aspect((1, 1, 0.75))
    ax3d.grid(True, color=estilo.GRADE, lw=0.5)

    # ---- vista de cima: forca de cada motor ---------------------------------
    #  E aqui que um disturbio lento se enxerga. Sob torque em degrau a atitude
    #  quase nao se move — o integrador a segura —, mas os motores ficam
    #  permanentemente desbalanceados produzindo o contra-torque. A cor mostra o
    #  DESVIO em torno da media instantanea (escala divergente: quente acima,
    #  frio abaixo), e o numero mostra o PWM absoluto.
    mot = [coluna(f"m{k}") for k in (1, 2, 3, 4)]
    tem_motores = any(float(np.max(np.abs(v))) > 1 for v in mot)

    axtop.set_title(f"Forca dos motores — frame {nome_frame}", loc="left",
                    fontsize=9.5, color=estilo.TINTA)
    axtop.set_aspect("equal")
    axtop.set_xlim(-2.1, 2.1)
    axtop.set_ylim(-2.0, 2.0)
    axtop.axis("off")

    discos_topo, textos_topo, bracos_topo = [], [], []
    if tem_motores:
        # x = direita, y = frente. O angulo do ArduPilot conta do nariz no sentido
        # horario, entao x = sin(a) e y = cos(a).
        pos = [(np.sin(np.radians(a)), np.cos(np.radians(a))) for a in angulos]
        for (px, py) in pos:
            bracos_topo.append(axtop.plot([0, px], [0, py], color=estilo.MUDO,
                                          lw=2.0, zorder=1)[0])
        for k, (px, py) in enumerate(pos):
            c = plt.Circle((px, py), 0.36, facecolor=estilo.DIVERGENTE_NEUTRO,
                           edgecolor=estilo.TINTA_2, lw=1.2, zorder=2)
            axtop.add_patch(c)
            discos_topo.append(c)
            textos_topo.append(axtop.text(px, py, "", ha="center", va="center",
                                          fontsize=8.5, zorder=3,
                                          color=estilo.TINTA, family="monospace"))
            axtop.text(px * 1.78, py * 1.78, f"M{k + 1}", ha="center", va="center",
                       fontsize=7, color=estilo.MUDO, zorder=3)
        #  O indicador de nariz vai para o canto, e nao sobre o eixo central:
        #  dependendo do frame ha um rotor sobre esse eixo, e a seta cairia em
        #  cima dele.
        axtop.annotate("", xy=(-1.75, 1.85), xytext=(-1.75, 1.35),
                       arrowprops=dict(arrowstyle="-|>", color=estilo.SERIE[1], lw=1.6))
        axtop.text(-1.62, 1.58, "nariz", fontsize=7, color=estilo.SERIE[1],
                   va="center")
    else:
        axtop.text(0.5, 0.5, "sem RCOU neste log", transform=axtop.transAxes,
                   ha="center", va="center", color=estilo.MUDO, fontsize=9)

    #  escala de cor fixa no voo inteiro: se ela reescalasse a cada quadro, um
    #  desbalanceamento minusculo no pairado pareceria tao severo quanto o do
    #  disturbio
    #  PISO na escala. Sem ele, um desbalanceamento de meio PWM — que e ruido —
    #  pinta o rotor de vermelho pleno e sugere um esforco que nao existe. Com o
    #  piso, so um diferencial de fato relevante satura a cor. O valor de
    #  saturacao vai escrito na figura, porque cor sem escala nao e medida.
    PISO_COR_PWM = 5.0
    if tem_motores:
        pilha = np.vstack([v[idx] for v in mot])
        desvios = pilha - pilha.mean(axis=0, keepdims=True)
        escala_cor = max(float(np.percentile(np.abs(desvios), 99)), PISO_COR_PWM)
        axtop.text(0.5, -0.06,
                   f"cor: desvio da media, satura em ±{escala_cor:.0f} PWM",
                   transform=axtop.transAxes, ha="center", va="top",
                   fontsize=7, color=estilo.MUDO)
    else:
        escala_cor = 1.0

    # ---- curvas com cursor --------------------------------------------------
    # uma faixa por eixo exercitado; a guinada vai relativa ao rumo inicial,
    # como no painel
    cursores = []
    medidas = {}
    for ax, e in zip(ax_ang, eixos_plot):
        c = COLUNAS[e]
        titulo = "Guinada, relativa ao rumo inicial" if e == "yaw" else nome(e).capitalize()
        estilo.titulo_faixa(ax, titulo, "deg")
        ref = coluna(c["ref"], np.nan)
        if np.isfinite(ref).any():
            ax.plot(eixo, ref, color=estilo.REFERENCIA, ls="--",
                    lw=estilo.LARGURA_REF, label="comandada")
        medidas[e] = coluna(c["med"])
        ax.plot(eixo, medidas[e], color=estilo.SERIE[0], label="medida")
        estilo.legenda(ax, ncols=2)
        cursores.append(ax.axvline(eixo[idx[0]], color=estilo.SERIE[1], lw=1.4))
        estilo.limpar_eixo(ax)

    # A faixa de baixo mostra o disturbio que de fato perturba a atitude: torque
    # quando ha, vento quando e o unico. Num ensaio de torque, um grafico de vento
    # zerado ocuparia um quarto da figura para nao dizer nada.
    tqx = coluna("tqx")
    tqy = coluna("tqy")
    tqz = coluna("tqz")
    tem_tq = float(np.max(np.abs(tqx) + np.abs(tqy) + np.abs(tqz))) > 1e-4

    if tem_tq:
        estilo.titulo_faixa(axw, "Torque aplicado", "rad/s²")
        for v, rotulo, slot in ((tqx, "rolagem", 0), (tqy, "arfagem", 1),
                                (tqz, "guinada", 2)):
            if float(np.max(np.abs(v))) > 1e-4:
                axw.plot(eixo, v, color=estilo.SERIE[slot], label=rotulo)
        estilo.legenda(axw, ncols=3)
    else:
        estilo.titulo_faixa(axw, "Vento aplicado", "m/s")
        # O modulo vai como faixa preenchida ATRAS das componentes: como ele
        # coincide com a componente dominante sempre que a outra e pequena,
        # desenhado como linha ele simplesmente cobriria essa componente.
        axw.fill_between(eixo, 0.0, w_mod, color=estilo.SERIE[2], alpha=0.16,
                         lw=0, label="modulo")
        axw.plot(eixo, wn, color=estilo.SERIE[0], label="norte")
        axw.plot(eixo, we, color=estilo.SERIE[1], label="leste")
        estilo.legenda(axw, ncols=3)
    cursores.append(axw.axvline(eixo[idx[0]], color=estilo.SERIE[1], lw=1.4))
    axw.set_xlabel(rotulo_x)
    estilo.limpar_eixo(axw, ultimo=True)
    ax_ang[0].set_xlim(eixo[idx[0]], eixo[idx[-1]])

    # componentes de torque que entram no rodape: as que o ensaio aplica
    comp_tq = [(sufixo, v) for sufixo, v in (("x", tqx), ("y", tqy), ("z", tqz))
               if float(np.max(np.abs(v))) > 1e-4]

    # O rodape vive na faixa do titulo, a direita: embaixo ele disputaria espaco
    # com o rotulo do eixo da vista de topo. Com mais de um eixo ele fica longo
    # demais para dividir a linha com o titulo e desce para a linha de baixo.
    rodape = fig.text(0.965, 0.962 if n == 1 else 0.915, "", fontsize=8.5,
                      color=estilo.TINTA_2, family="monospace", va="center", ha="right")
    fig.suptitle(f"{controlador}  ·  {cenario}", x=0.02, y=0.962, ha="left",
                 va="center", fontsize=11.5, fontweight="bold", color=estilo.TINTA)
    # a velocidade vai NA figura: sem ela, quem assiste nao tem como separar a
    # dinamica da aeronave da cadencia da animacao
    rotulo_vel = ("tempo real" if abs(vel_real - 1.0) < 0.05
                  else f"{vel_real:.2f}x o tempo real")
    fig.text(0.02, 0.915, rotulo_vel, fontsize=8, color=estilo.MUDO,
             ha="left", va="center")

    # escala da seta de vento: o maior vento do ensaio ocupa 80% do semi-lado
    w_max = max(float(np.max(w_mod[idx])), 1e-6)
    esc_seta_3d = (lado * 0.8) / w_max

    def desenhar(k):
        i = idx[k]
        R = _rot_ned(roll[i], pitch[i], yaw[i])
        centro = np.array([pn[i], pe[i], pd[i]])

        # a camera segue o veiculo: tudo e desenhado relativo ao centro
        for linha, (a, b) in zip(linhas_bracos, bracos):
            pa = _ned_para_vista(R @ a)
            pb = _ned_para_vista(R @ b)
            linha.set_data_3d([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]])

        for linha, disco in zip(linhas_discos, discos):
            p = np.apply_along_axis(lambda c: _ned_para_vista(R @ c), 0, disco)
            linha.set_data_3d(p[0], p[1], p[2])

        pa = _ned_para_vista(R @ nariz[0])
        pb = _ned_para_vista(R @ nariz[1])
        linha_nariz.set_data_3d([pa[0], pb[0]], [pa[1], pb[1]], [pa[2], pb[2]])

        # seta do vento: parte de tras do veiculo e aponta para onde o ar vai
        v = np.array([wn[i], we[i], 0.0]) * esc_seta_3d
        vv = _ned_para_vista(v)
        seta_vento.set_data_3d([-vv[0], 0.0], [-vv[1], 0.0], [0.0, 0.0])

        ax3d.set_xlim(-lado, lado)
        ax3d.set_ylim(-lado, lado)
        ax3d.set_zlim(-lado * 0.75, lado * 0.75)

        # vista de cima dos motores
        if tem_motores:
            vals = [float(v[i]) for v in mot]
            media = sum(vals) / len(vals)
            for c, txt, val in zip(discos_topo, textos_topo, vals):
                c.set_facecolor(estilo.cor_motor(val - media, escala_cor))
                txt.set_text(f"{val:.0f}")

        # cursores e rodape
        for cursor in cursores:
            cursor.set_xdata([eixo[i], eixo[i]])
        if tem_tq and [s for s, _ in comp_tq] == ["x"]:
            disturbio = f"torque = {tqx[i]:+6.2f} rad/s2   "
        elif tem_tq:
            disturbio = (f"torque {'/'.join(s for s, _ in comp_tq)} = "
                         + "/".join(f"{v[i]:+.2f}" for _, v in comp_tq) + " rad/s2   ")
        else:
            disturbio = f"vento = {w_mod[i]:5.2f} m/s   "
        atitude = "".join(f"{nome(e)} = {medidas[e][i]:+7.2f} deg   " for e in eixos_plot)
        rodape.set_text(
            f"t = {eixo[i]:6.2f} s   "
            + atitude
            + disturbio
            + f"alt = {-pd[i]:6.2f} m")

        return (*linhas_bracos, *linhas_discos, linha_nariz, seta_vento,
                *discos_topo, *textos_topo, *cursores, rodape)

    anim = FuncAnimation(fig, desenhar, frames=len(idx), interval=1000 / fps,
                         blit=False)
    anim.save(str(destino), writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)
    return len(idx)
