#!/usr/bin/env python3
"""Revisão de 3/11, bloco L4 do lote: o determinismo do SITL em voos repetidos.

Para rolagem/torque_impulso e rolagem/torque_degrau, com o nativo e o cc_lqg, em cada
binário do SITL que tem voos do L4:
  - o impulso entregue (integral da perturbação verdadeira, teste_4b4c.impulso) e o
    número de passos da física de 1200 Hz que ele equivale;
  - o pico e o EQM do desvio de rolagem na atitude verdadeira (CCTQ, menos a média do
    trecho calmo de 1 s antes do torque; EQM de t0 a t0 + 3 s) e no AHRS (metricas.json);
  - a amplitude (máximo - mínimo) / média nos 3 voos, o número do critério do L4.

O primeiro binário (c289ac6d...) contava a duração do SIM_TWIST_TIME em milissegundos
inteiros: o impulso de 20 ms durava 23 ou 24 passos. O segundo (4f669ec0...) conta em
microssegundos (firmware/ardupilot.patch, SIM_Aircraft.cpp). Escreve ../l4_determinismo.txt.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import teste_4b4c as T  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

RES = T.RES / "rolagem"
CENARIOS = ("torque_impulso", "torque_degrau")
SLUGS = ("nativo", "cc_lqg")
JANELA = 3.0          # s depois de t0, para o EQM verdadeiro


def desvio_verdadeiro(pasta):
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text())["vento"]
    t0 = float(cen["WND_TQT0"])
    d = leitura.ler_log(next(pasta.glob("*.BIN")), ("CCTQ", "WND"))
    ct = d["CCTQ"]
    tp = ct["t"] - leitura.origem_perfil(d)
    w, x, y, z = ct["Q1"], ct["Q2"], ct["Q3"], ct["Q4"]
    rol = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    calmo = (tp >= t0 - 1.0) & (tp < t0 - 0.05)
    dv = rol - rol[calmo].mean()
    jan = (tp >= t0) & (tp <= t0 + JANELA)
    return float(np.max(np.abs(dv[jan]))), float(np.sqrt(np.mean(dv[jan] ** 2))), float(rol[calmo].std())


def amplitude(xs):
    return 100.0 * (max(xs) - min(xs)) / np.mean(xs)


def main():
    mod = P.carregar_modelo()
    passo = 1.0 / mod.fs_fisica
    voos = defaultdict(list)          # (sha, cenario, slug) -> pastas
    for cen in CENARIOS:
        for slug in SLUGS:
            for pasta in sorted((RES / cen / slug).glob("2026-*")):
                v = pasta / "versao.json"
                if v.exists() and (pasta / "metricas.json").exists():
                    voos[(json.loads(v.read_text())["binario_sha256"], cen, slug)].append(pasta)
    shas = sorted({k[0] for k, ps in voos.items() if len(ps) >= 3},
                  key=lambda s: min(p.name for k, ps in voos.items() if k[0] == s for p in ps))
    linhas = ["L4: voos repetidos de rolagem, por binário do SITL. Verdade = CCTQ menos a média do "
              f"trecho calmo; EQM verdadeiro de t0 a t0 + {JANELA:g} s; AHRS = metricas.json."]
    for sha in shas:
        linhas.append(f"binário {sha[:12]}")
        for cen in CENARIOS:
            for slug in SLUGS:
                ps = voos.get((sha, cen, slug), [])
                if len(ps) < 3:
                    continue
                vp, ve, ap, ae = [], [], [], []
                for pasta in ps:
                    pv, ev, sd = desvio_verdadeiro(pasta)
                    m = json.loads((pasta / "metricas.json").read_text())["disturbio_roll"]
                    vp.append(pv); ve.append(ev); ap.append(m["desvio_max_deg"]); ae.append(m["rmse_deg"])
                    txt = (f"    {pasta.name}: verdade pico {pv:.4f} EQM {ev:.4f} (calmo dp {sd:.4f}); "
                           f"AHRS pico {ap[-1]:.4f} EQM {ae[-1]:.4f}")
                    if cen == "torque_impulso":
                        amp = yaml.safe_load((pasta / "cenario.yaml").read_text())["vento"]["WND_TQX"]
                        _, ent, _ = T.impulso(pasta, "roll", mod)
                        txt += f"; impulso entregue {ent:.4f} rad/s = {ent / float(amp) / passo:.1f} passos"
                    linhas.append(txt)
                linhas.append(f"  {cen} {slug}: amplitude verdade pico {amplitude(vp):.2f} % EQM {amplitude(ve):.2f} %; "
                              f"AHRS pico {amplitude(ap):.2f} % EQM {amplitude(ae):.2f} %")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "l4_determinismo.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
