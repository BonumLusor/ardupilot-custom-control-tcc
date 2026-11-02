#!/usr/bin/env python3
"""Gera o ensaio.gif das pastas de resultado que ainda nao o tem, em paralelo.

O GIF e a parte cara da analise (~1 min por voo), e os lotes rodam com
--sem-gif para nao dobrar o tempo de voo. Este script o renderiza depois, so a
partir do que a analise ja gravou em cada pasta (series.csv, metricas.json,
cenario.yaml, controlador.yaml e params.parm) — sem refazer as metricas e sem
tocar no consolidado.csv, entao pode usar varios processos.

    python ensaios/gerar_gifs.py                       # tudo em resultados/ sem GIF
    python ensaios/gerar_gifs.py ensaios/resultados/guinada --processos 8
    python ensaios/gerar_gifs.py --todos               # refaz inclusive os que ja tem
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np
import yaml

RAIZ = Path(__file__).resolve().parent
RESULTADOS = RAIZ / "resultados"
sys.path.insert(0, str(RAIZ))


def pastas_de_ensaio(caminhos, todos=False):
    """Pastas com series.csv e metricas.json (e sem ensaio.gif, salvo com ``todos``)."""
    achadas = []
    for c in caminhos:
        for met in sorted(Path(c).rglob("metricas.json")):
            pasta = met.parent
            if pasta.name.endswith("_falhou") or "autotune" in pasta.parts:
                continue
            if not (pasta / "series.csv").exists():
                continue
            if todos or not (pasta / "ensaio.gif").exists():
                achadas.append(pasta)
    return achadas


def _ler_series(caminho):
    import csv
    with open(caminho, newline="", encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    col = {}
    for chave in linhas[0]:
        try:
            col[chave] = np.array([float(r[chave]) if r[chave] not in ("", "nan") else np.nan
                                   for r in linhas])
        except ValueError:
            continue
    return col


def renderizar(pasta):
    """Renderiza o GIF de uma pasta; devolve (pasta, quadros, segundos, erro)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from analise import animacao
    from analise.leitura import ler_params
    t0 = time.monotonic()
    try:
        ler = lambda nome: (yaml.safe_load((pasta / nome).read_text(encoding="utf-8"))
                            if (pasta / nome).exists() else {}) or {}
        cen, ctl = ler("cenario.yaml"), ler("controlador.yaml")
        col = _ler_series(pasta / "series.csv")
        m = json.loads((pasta / "metricas.json").read_text(encoding="utf-8"))
        parm = ler_params(pasta / "params.parm") if (pasta / "params.parm").exists() else {}
        # o nome do cenario e o caminho em resultados/ (run_ensaio.id_cenario_da_pasta)
        rel = pasta.resolve().relative_to(RESULTADOS.resolve()).parts
        id_cen = "/".join(rel[:-2])
        # grava num temporario e renomeia: um GIF interrompido no meio nao fica com
        # o nome final (pastas_de_ensaio o daria como pronto)
        parcial = pasta / "ensaio.parcial.gif"
        n = animacao.gerar(col["t"], col, parcial,
                           controlador=ctl.get("nome", pasta.parent.name),
                           cenario=cen.get("nome", id_cen), metricas=m,
                           frame_type=int(parm.get("FRAME_TYPE", 1)))
        parcial.replace(pasta / "ensaio.gif")
        plt.close("all")
        return str(pasta), n, time.monotonic() - t0, None
    except Exception as exc:          # um GIF ruim nao para os outros
        return str(pasta), 0, time.monotonic() - t0, f"{type(exc).__name__}: {exc}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("caminhos", nargs="*", type=Path, default=[RESULTADOS])
    ap.add_argument("--todos", action="store_true", help="refaz tambem os que ja tem GIF")
    ap.add_argument("--processos", type=int, default=max(mp.cpu_count() // 2, 1))
    args = ap.parse_args()

    pastas = pastas_de_ensaio(args.caminhos, args.todos)
    print(f"[gifs] {len(pastas)} pasta(s) para renderizar, {args.processos} processo(s)",
          flush=True)
    t0 = time.monotonic()
    falhas = []
    with mp.get_context("fork").Pool(args.processos) as pool:
        for i, (pasta, n, s, erro) in enumerate(pool.imap_unordered(renderizar, pastas), 1):
            rel = Path(pasta).relative_to(RESULTADOS) if Path(pasta).is_relative_to(RESULTADOS) else pasta
            if erro:
                falhas.append((pasta, erro))
                print(f"[gifs] {i}/{len(pastas)} FALHOU {rel}: {erro}", flush=True)
            else:
                print(f"[gifs] {i}/{len(pastas)} {rel}: {n} quadros em {s:.0f} s", flush=True)
    print(f"[gifs] fim: {len(pastas) - len(falhas)} de {len(pastas)} em "
          f"{(time.monotonic() - t0) / 60:.0f} min", flush=True)
    return 1 if falhas else 0


if __name__ == "__main__":
    sys.exit(main())
