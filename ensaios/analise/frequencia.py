"""Resposta em frequencia a partir de uma varredura (chirp).

E o que transforma o ensaio de chirp num resultado: sozinha, a serie temporal de
uma varredura nao responde nada. Aqui ela vira

    H(f) = Pyu(f) / Puu(f)      resposta de disturbio -> atitude
    gamma^2(f)                  coerencia: onde a estimativa merece credito

A **coerencia e obrigatoria**, nao enfeite. Numa varredura, cada frequencia e
excitada por poucos ciclos, e as pontas da faixa recebem pouquissima energia —
dividir um espectro pelo outro ali produz um numero grande e sem sentido. Foi o
que apareceu no primeiro ensaio deste projeto: um pico de ganho em 8 Hz que era
so ruido dividido por quase nada. Com a coerencia ao lado, esse ponto se denuncia.

Usa o metodo de Welch (``scipy.signal``), com segmentos sobrepostos: troca
resolucao em frequencia por variancia menor, que e o compromisso certo quando se
quer ler a forma da curva e nao um pico estreito.
"""

from __future__ import annotations

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
from scipy import signal                 # noqa: E402

from . import estilo                     # noqa: E402

# coerencia abaixo disto: a estimativa nao e confiavel e nao entra nas metricas
COERENCIA_MINIMA = 0.8


def estimar(grade, col, perfil, entrada="tqx", saida="roll", rotulo=None):
    """Estima H(f) e a coerencia dentro da janela da varredura.

    ``entrada`` e ``saida`` sao colunas da serie: torque do eixo e atitude do
    mesmo eixo (``tqz`` e ``yaw_rel`` na guinada). ``rotulo`` e o nome que vai no
    grafico. Devolve None quando o cenario nao tem chirp ou nao ha dados
    suficientes.
    """
    if perfil is None or not (perfil.modo & 64):
        return None
    if entrada not in col or saida not in col or "tp" not in col:
        return None

    tp = col["tp"]
    dentro = (tp >= perfil.tqt0) & (tp <= perfil.tqt0 + perfil.tqdur)
    u = np.asarray(col[entrada], dtype=float)[dentro]
    y = np.asarray(col[saida], dtype=float)[dentro]
    t = np.asarray(grade, dtype=float)[dentro]
    ok = np.isfinite(u) & np.isfinite(y)
    u, y, t = u[ok], y[ok], t[ok]
    if len(t) < 64:
        return None

    fs = 1.0 / float(np.median(np.diff(t)))
    # ~8 segmentos: resolucao suficiente para a forma da curva, variancia baixa
    nperseg = max(int(len(u) / 8), 64)

    f, Puu = signal.welch(u, fs=fs, nperseg=nperseg)
    _, Pyu = signal.csd(u, y, fs=fs, nperseg=nperseg)
    _, coer = signal.coherence(u, y, fs=fs, nperseg=nperseg)

    H = Pyu / np.maximum(Puu, 1e-18)
    banda = (f >= perfil.tqf0) & (f <= perfil.tqf1) & (f > 0)

    return {
        "f": f[banda],
        "mag": np.abs(H[banda]),
        "fase_deg": np.degrees(np.angle(H[banda])),
        "coerencia": coer[banda],
        "fs": fs,
        "nperseg": nperseg,
        "entrada": entrada,
        "saida": saida,
        "rotulo": rotulo or f"{saida} / {entrada}",
    }


def metricas(est):
    """Numeros da resposta em frequencia, só onde a coerencia sustenta."""
    if est is None:
        return {}
    bom = est["coerencia"] >= COERENCIA_MINIMA
    if not bom.any():
        return {"coerencia_max": float(np.max(est["coerencia"])),
                "aviso": "coerencia sempre abaixo do minimo — varredura sem "
                         "energia suficiente ou resposta dominada por ruido"}

    f, mag = est["f"][bom], est["mag"][bom]
    i = int(np.argmax(mag))
    # -3 dB a partir do pico, andando para a frente: banda util da rejeicao
    lim = mag[i] / np.sqrt(2.0)
    depois = np.nonzero((np.arange(len(mag)) > i) & (mag < lim))[0]
    return {
        "faixa_confiavel_hz": [float(f[0]), float(f[-1])],
        "ganho_max": float(mag[i]),
        "f_ganho_max_hz": float(f[i]),
        "f_queda_3db_hz": float(f[depois[0]]) if len(depois) else None,
        "coerencia_mediana": float(np.median(est["coerencia"][bom])),
        "pontos_confiaveis": int(bom.sum()),
    }


def desenhar(est, destino, controlador="—", cenario="—", unidade="deg / (rad/s²)"):
    """Tres faixas: magnitude, fase e coerencia, no mesmo eixo de frequencia."""
    if est is None:
        return None
    estilo.aplicar()

    fig, eixos = plt.subplots(3, 1, figsize=(9.0, 8.4), sharex=True,
                              gridspec_kw=dict(hspace=0.28, height_ratios=[1.3, 1, 0.8]))

    bom = est["coerencia"] >= COERENCIA_MINIMA

    ax = eixos[0]
    estilo.titulo_faixa(ax, f"Ganho  {est.get('rotulo') or est['saida'] + ' / ' + est['entrada']}",
                        unidade)
    ax.loglog(est["f"], est["mag"], color=estilo.SERIE[0])
    # o trecho sem coerencia sai apagado, para nao ser lido como resultado
    ax.loglog(est["f"][~bom], est["mag"][~bom], ".", color=estilo.MUDO, ms=3)
    estilo.limpar_eixo(ax)
    ax.grid(True, which="both", axis="both")

    ax = eixos[1]
    estilo.titulo_faixa(ax, "Fase", "graus")
    ax.semilogx(est["f"], est["fase_deg"], color=estilo.SERIE[0])
    ax.semilogx(est["f"][~bom], est["fase_deg"][~bom], ".", color=estilo.MUDO, ms=3)
    ax.axhline(-180.0, color=estilo.MUDO, lw=0.8, ls=":")
    estilo.limpar_eixo(ax)
    ax.grid(True, which="both", axis="both")

    ax = eixos[2]
    estilo.titulo_faixa(ax, "Coerencia", "0 a 1")
    ax.semilogx(est["f"], est["coerencia"], color=estilo.SERIE[0])
    ax.axhline(COERENCIA_MINIMA, color=estilo.SERIE[1], lw=1.0, ls="--")
    ax.text(est["f"][0], COERENCIA_MINIMA, f" limiar {COERENCIA_MINIMA}",
            fontsize=7.5, color=estilo.SERIE[1], va="bottom")
    ax.set_ylim(0, 1.05)
    ax.set_xlabel("frequencia (Hz)")
    estilo.limpar_eixo(ax, ultimo=True)
    ax.grid(True, which="both", axis="both")

    fig.suptitle(f"{controlador}  ·  {cenario}", x=0.02, ha="left",
                 fontsize=12, fontweight="bold", color=estilo.TINTA)
    fig.text(0.02, 0.945,
             f"Welch, fs={est['fs']:.0f} Hz, {est['nperseg']} amostras por segmento",
             fontsize=7.5, color=estilo.MUDO, ha="left")

    fig.savefig(destino, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return destino
