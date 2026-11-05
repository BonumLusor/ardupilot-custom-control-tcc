#!/usr/bin/env python3
"""Revisão de 3/11, item 2.5 (Fase 3a e 3b): de onde vem o ruído no esforço do LQG.

Nos voos de pulso de torque (voos antigos: é diagnóstico, não resultado), nos
trechos calmos — Tp de 1 s até 0,5 s antes do torque e de 4 s depois dele até o fim
do perfil —, para o LQG e para o nativo, em rolagem e em guinada, com RND 0 e 20:

  3a: valor eficaz e densidade espectral (Welch) do esforço u e das inovações do
      LQG, a do ângulo (It) e a da taxa (Iw);
  3b: o resíduo r = ângulo do AHRS - integral do giroscópio do controlador
      (ANG e RATE, a 400 Hz), sem a tendência linear: é o que o EKF soma ao
      ângulo além do giroscópio. O LQG modela esse resíduo como eta, excitado só
      pelo ruído do giroscópio; com RND 0, o modelo espera r ~ 0.

Com RND 0, o giroscópio só tem o ruído de base do SITL. Se u e It tiverem energia
nas taxas de fusão do EKF3 (GPS a 5 Hz, bússola a 30 Hz), a causa está no ângulo.
Escreve ../ruido_lqg_espectros.txt e ../ruido_lqg_espectros.png.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import signal

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "ensaios"))
from analise import leitura  # noqa: E402

RES = RAIZ / "ensaios" / "resultados"
SAIDA = Path(__file__).resolve().parents[1]
FS = 400.0
CASOS = (  # (rótulo, cenário, eixo)
    ("rolagem, RND 0", "rolagem/torque_pulso", "roll"),
    ("rolagem, RND 20", "ruido/rnd20/torque_pulso", "roll"),
    ("guinada, RND 0", "guinada/torque_pulso", "yaw"),
    ("guinada, RND 20", "ruido/rnd20/torque_pulso", "yaw"),
)
SUF = {"roll": "R", "yaw": "Y"}
ANGULO = {"roll": "Roll", "yaw": "Yaw"}
TAXA = {"roll": "R", "yaw": "Y"}
SAIDA_RATE = {"roll": "ROut", "yaw": "YOut"}
FUSAO = {"GPS": 5.0, "bússola": 30.0}


def voo(cenario, slug):
    pastas = sorted(p for p in (RES / cenario / slug).glob("2026-1*") if any(p.glob("*.BIN")))
    pastas = [p for p in pastas if p.name >= "2026-10-27"]
    return pastas[-1] if pastas else None


def trechos(d, pasta):
    """Máscaras de tempo de log dos trechos calmos, pelo tempo do perfil."""
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text())
    vento = cen.get("vento", {})
    t0 = float(vento.get("WND_TQT0", 6.0))
    dur = float(cen.get("voo", {}).get("duracao_s", 30.0))
    origem = leitura.origem_perfil(d)
    return [(origem + 1.0, origem + t0 - 0.5), (origem + t0 + 4.0, origem + dur - 0.5)]


def recorta(t, y, janelas):
    return [y[(t >= a) & (t <= b)] for a, b in janelas if np.sum((t >= a) & (t <= b)) > 256]


def psd(segmentos):
    """Welch sobre cada trecho, média ponderada pelo número de janelas."""
    fs, ps, pesos = None, [], []
    for s in segmentos:
        s = signal.detrend(np.asarray(s, dtype=float))
        f, p = signal.welch(s, fs=FS, nperseg=min(512, len(s)))
        if fs is None or len(f) == len(fs):
            fs = f
            ps.append(p)
            pesos.append(len(s))
    return fs, np.average(ps, axis=0, weights=pesos)


def rms(segmentos):
    x = np.concatenate([s - np.mean(s) for s in segmentos])
    return float(np.sqrt(np.mean(x ** 2)))


def residuo(d, eixo, janelas):
    """AHRS - integral do giroscópio, por trecho, sem a tendência linear (graus)."""
    ang, rate = d["ANG"], d["RATE"]
    segs = []
    for a, b in janelas:
        m = (ang["t"] >= a) & (ang["t"] <= b)
        if m.sum() < 256:
            continue
        t = ang["t"][m]
        th = np.unwrap(np.radians(ang[ANGULO[eixo]][m]))
        w = np.interp(t, rate["t"], np.radians(rate[TAXA[eixo]]))
        integ = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
        segs.append(np.degrees(signal.detrend(th - th[0] - integ)))
    return segs


def pico(f, p, perto, larg=1.0):
    """Razão entre a densidade na faixa perto +- larg e a mediana de 2 a 50 Hz."""
    base = np.median(p[(f >= 2) & (f <= 50)])
    sel = (f >= perto - larg) & (f <= perto + larg)
    return float(np.max(p[sel]) / base) if sel.any() and base > 0 else float("nan")


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    linhas = []
    fig, eixos = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    for k, (rot, cen, eixo) in enumerate(CASOS):
        ax = eixos.flat[k]
        p_lqg, p_nat = voo(cen, "cc_lqg"), voo(cen, "nativo")
        if p_lqg is None or p_nat is None:
            linhas.append(f"{rot}: sem voo")
            continue
        s = SUF[eixo]
        d = leitura.ler_log(next(p_lqg.glob("*.BIN")), (f"CCK{s}", f"CCL{s}", "WND", "ANG", "RATE"))
        dn = leitura.ler_log(next(p_nat.glob("*.BIN")), ("WND", "ANG", "RATE"))
        jan, jan_n = trechos(d, p_lqg), trechos(dn, p_nat)
        ck, cl = d[f"CCK{s}"], d[f"CCL{s}"]
        u = recorta(cl["t"], cl["U"], jan)
        it = recorta(ck["t"], np.degrees(ck["It"]), jan)
        iw = recorta(ck["t"], np.degrees(ck["Iw"]), jan)
        un = recorta(dn["RATE"]["t"], dn["RATE"][SAIDA_RATE[eixo]], jan_n)
        r_l, r_n = residuo(d, eixo, jan), residuo(dn, eixo, jan_n)
        linhas.append(f"{rot} ({p_lqg.relative_to(RES)}; nativo {p_nat.relative_to(RES)}):")
        linhas.append(f"  u eficaz: LQG {rms(u):.5f}, nativo {rms(un):.5f} ({rms(u) / rms(un):.1f}x)")
        linhas.append(f"  inovações eficazes do LQG: ângulo {rms(it):.4f} graus, taxa {rms(iw):.4f} graus/s")
        linhas.append(f"  resíduo AHRS - integral do giroscópio, eficaz: voo do LQG {rms(r_l):.4f} graus, "
                      f"voo do nativo {rms(r_n):.4f} graus")
        f, pu = psd(u)
        _, pun = psd(un)
        _, pit = psd(it)
        _, pr = psd(r_l)
        for nome, perto in FUSAO.items():
            linhas.append(f"  pico perto de {perto:g} Hz ({nome}), sobre a mediana de 2-50 Hz: "
                          f"u do LQG {pico(f, pu, perto):.1f}; inovação do ângulo {pico(f, pit, perto):.1f}; "
                          f"resíduo {pico(f, pr, perto):.1f}")
        ax.loglog(f[1:], pu[1:], label="u, LQG")
        ax.loglog(f[1:], pun[1:], label="u, nativo")
        ax.loglog(f[1:], pit[1:] * np.max(pu) / np.max(pit), label="inovação do ângulo (escala livre)", alpha=0.7)
        for perto in FUSAO.values():
            ax.axvline(perto, color="0.6", lw=0.6, ls=":")
        ax.set_title(rot, loc="left", fontsize=9)
        ax.grid(True, which="both", lw=0.3)
    for ax in eixos[1]:
        ax.set_xlabel("Frequência (Hz)")
    for ax in eixos[:, 0]:
        ax.set_ylabel("Densidade espectral")
    eixos[0, 0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(SAIDA / "ruido_lqg_espectros.png", dpi=110)
    texto = "\n".join(linhas)
    print(texto)
    (SAIDA / "ruido_lqg_espectros.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
