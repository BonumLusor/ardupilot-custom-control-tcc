#!/usr/bin/env python3
"""Revisão de 3/11, bloco L4 do lote: o determinismo do SITL em voos repetidos.

Para o impulso e o degrau de torque de rolagem e de arfagem, com o nativo e os dois LQG,
em cada binário do SITL que tem voos repetidos (3 ou mais do mesmo par):
  - o pico e o EQM do desvio na atitude verdadeira (metricas_verdade.json, a mesma base
    das tabelas, analise.verdade) e no AHRS (metricas.json);
  - a amplitude (máximo - mínimo) / média, o número do critério do L4 (1 %);
  - no impulso, o impulso entregue (integral da perturbação reconstruída da atitude
    verdadeira com o modelo, teste_4b4c.impulso) em passos da física de 1200 Hz.

E, no binário do lote, o impulso entregue em todos os voos de impulso sem ruído
(rolagem, arfagem e guinada), contra o pedido.

O primeiro binário (c289ac6d...) contava a duração do SIM_TWIST_TIME em milissegundos
inteiros: o impulso de 20 ms durava 23 ou 24 passos. O segundo (4f669ec0..., o do lote)
conta em microssegundos (firmware/ardupilot.patch, SIM_Aircraft.cpp). Nos voos do
primeiro binário, a verdade sai sempre daqui, da CCTQ menos a média do segundo antes do
torque (alguns têm metricas_verdade.json da prévia, outros não).
Escreve ../l4_determinismo.txt.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
import perturbacao_verdadeira as PV  # noqa: E402
import ruido_lqg_espectros as E  # noqa: E402
import teste_4b4c as T  # noqa: E402
import projeto as P  # noqa: E402
from analise import leitura  # noqa: E402

# o auxiliar do impulso conhece rolagem e guinada; a arfagem segue o mesmo molde
PV.EIXO_IDX.setdefault("pitch", 1)
E.SAIDA_RATE.setdefault("pitch", "POut")
T.COL_TQ.setdefault("pitch", "WND_TQY")

EIXOS = (("rolagem", "roll"), ("arfagem", "pitch"))
CENARIOS = ("torque_impulso", "torque_degrau")
SLUGS = ("nativo", "cc_lqg", "cc_lqg_mesmo_ruido")
LOTE = (T.A.RAIZ / "ensaios" / "lotes" / "binario_lote.sha256").read_text().split()[0]
JANELA = 3.0          # s depois de t0, para o EQM verdadeiro local (binário antigo)


def verdade_local(pasta, eixo):
    """Pico e EQM verdadeiros sem metricas_verdade.json (binário antigo, só rolagem)."""
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text())["vento"]
    t0 = float(cen["WND_TQT0"])
    d = leitura.ler_log(next(pasta.glob("*.BIN")), ("CCTQ", "WND"))
    ct = d["CCTQ"]
    tp = ct["t"] - leitura.origem_perfil(d)
    w, x, y, z = ct["Q1"], ct["Q2"], ct["Q3"], ct["Q4"]
    if eixo == "roll":
        ang = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    else:
        ang = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    calmo = (tp >= t0 - 1.0) & (tp < t0 - 0.05)
    dv = ang - ang[calmo].mean()
    jan = (tp >= t0) & (tp <= t0 + JANELA)
    return float(np.max(np.abs(dv[jan]))), float(np.sqrt(np.mean(dv[jan] ** 2)))


def amplitude(xs):
    return 100.0 * (max(xs) - min(xs)) / np.mean(xs)


def versao(pasta):
    v = pasta / "versao.json"
    return json.loads(v.read_text()).get("binario_sha256", "") if v.exists() else ""


def passos_entregues(pasta, eixo, mod):
    cen = yaml.safe_load((pasta / "cenario.yaml").read_text())["vento"]
    amp = float(cen[T.COL_TQ[eixo]])
    pedido, ent, _ = T.impulso(pasta, eixo, mod)
    return ent, ent / pedido, ent / amp * mod.fs_fisica


def main():
    mod = P.carregar_modelo()
    voos = defaultdict(list)          # (sha, pasta_eixo, cenario, slug) -> pastas
    for pasta_eixo, _ in EIXOS:
        for cen in CENARIOS:
            for slug in SLUGS:
                for pasta in sorted((T.RES / pasta_eixo / cen / slug).glob("2026-*")):
                    if pasta.name.endswith("_falhou") or not (pasta / "metricas.json").exists():
                        continue
                    voos[(versao(pasta), pasta_eixo, cen, slug)].append(pasta)
    shas = sorted({k[0] for k, ps in voos.items() if len(ps) >= 3 and k[0]},
                  key=lambda s: min(p.name for k, ps in voos.items() if k[0] == s for p in ps))
    linhas = ["L4: voos repetidos (3 ou mais do mesmo par), por binário do SITL. Verdade = "
              "metricas_verdade.json (analise.verdade, a base das tabelas); AHRS = metricas.json. "
              "Amplitude = (máx - mín) / média."]
    pior = defaultdict(float)           # (sha, slug, base) -> maior amplitude
    for sha in shas:
        linhas.append(f"binário {sha[:12]}{' (o do lote)' if sha == LOTE else ''}")
        for pasta_eixo, eixo in EIXOS:
            for cen in CENARIOS:
                for slug in SLUGS:
                    ps = voos.get((sha, pasta_eixo, cen, slug), [])
                    if len(ps) < 3:
                        continue
                    vp, ve, ap, ae = [], [], [], []
                    for pasta in ps:
                        chave = f"disturbio_{eixo}"
                        mv = pasta / "metricas_verdade.json"
                        if mv.exists() and sha == LOTE:   # o binário antigo: um só método nos 3 voos
                            d = json.loads(mv.read_text())[chave]
                            pv, ev, orig = d["desvio_max_deg"], d["rmse_deg"], ""
                        else:
                            pv, ev = verdade_local(pasta, eixo)
                            orig = " (verdade local)"
                        m = json.loads((pasta / "metricas.json").read_text())[chave]
                        vp.append(pv); ve.append(ev); ap.append(m["desvio_max_deg"]); ae.append(m["rmse_deg"])
                        txt = (f"    {pasta.name}: verdade pico {pv:.4f} EQM {ev:.4f}{orig}; "
                               f"AHRS pico {ap[-1]:.4f} EQM {ae[-1]:.4f}")
                        if cen == "torque_impulso":
                            ent, frac, passos = passos_entregues(pasta, eixo, mod)
                            txt += f"; impulso entregue {ent:.4f} rad/s ({100 * (frac - 1):+.1f} %, {passos:.1f} passos)"
                        linhas.append(txt)
                    av = max(amplitude(vp), amplitude(ve))
                    aa = max(amplitude(ap), amplitude(ae))
                    pior[(sha, slug, "verdade")] = max(pior[(sha, slug, "verdade")], av)
                    pior[(sha, slug, "AHRS")] = max(pior[(sha, slug, "AHRS")], aa)
                    linhas.append(f"  {pasta_eixo}/{cen} {slug} ({len(ps)} voos): amplitude verdade pico "
                                  f"{amplitude(vp):.2f} % EQM {amplitude(ve):.2f} %; AHRS pico "
                                  f"{amplitude(ap):.2f} % EQM {amplitude(ae):.2f} %")
        linhas.append("  maior amplitude (pico ou EQM) por controlador: " + "; ".join(
            f"{slug} verdade {pior[(sha, slug, 'verdade')]:.2f} % / AHRS {pior[(sha, slug, 'AHRS')]:.2f} %"
            for slug in SLUGS if (sha, slug, "verdade") in pior))

    # impulso entregue em todos os voos de impulso sem ruído do lote
    linhas.append(f"impulso entregue, voos de impulso sem ruído do binário {LOTE[:12]} (reconstrução com o modelo):")
    todos = []
    for pasta_eixo, eixo in EIXOS + (("guinada", "yaw"),):
        fr = []
        for pasta in sorted((T.RES / pasta_eixo / "torque_impulso").glob("*/2026-*")):
            if pasta.name.endswith("_falhou") or versao(pasta) != LOTE:
                continue
            _, frac, _ = passos_entregues(pasta, eixo, mod)
            fr.append(100 * (frac - 1))
        todos += fr
        if fr:
            linhas.append(f"  {pasta_eixo}: {len(fr)} voos, desvio do pedido médio {np.mean(fr):+.2f} %, "
                          f"de {min(fr):+.2f} a {max(fr):+.2f} %")
    if todos:
        linhas.append(f"  todos: {len(todos)} voos, |desvio| máximo {max(abs(x) for x in todos):.2f} %")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "l4_determinismo.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
