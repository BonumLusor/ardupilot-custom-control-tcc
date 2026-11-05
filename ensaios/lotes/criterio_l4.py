#!/usr/bin/env python3
"""Critério de determinismo do bloco L4 (plano da revisão de 3/11, seção 5), fixado antes do voo.

Para rolagem/torque_impulso e rolagem/torque_degrau, com o nativo e o cc_lqg, nos voos
com o binário atual do SITL: se a amplitude (máximo - mínimo) do pico e do EQM de
rolagem nos 3 voos ficar <= 1 % da média, sai 0 (segue o lote). Senão, imprime o par e
sai 1: o lote para, para se ver se a variação tem conserto antes de voar o resto.
A recuperação fica fora do critério (salta perto do limiar).
"""
import json
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / "ensaios"))
import run_ensaio as R  # noqa: E402

PARES = [(c, k) for c in ("rolagem/torque_impulso", "rolagem/torque_degrau") for k in ("nativo", "cc_lqg")]
LIMITE = 0.01


def main():
    sha = R.versao_firmware()["binario_sha256"]
    falhou = False
    for cen, slug in PARES:
        vals = {"desvio_max_deg": [], "rmse_deg": []}
        for pasta in sorted((R.RESULTADOS / cen / slug).glob("2026-*")):
            v, m = pasta / "versao.json", pasta / "metricas.json"
            if not v.exists() or not m.exists() or json.loads(v.read_text()).get("binario_sha256") != sha:
                continue
            d = json.loads(m.read_text()).get("disturbio_roll") or {}
            for k in vals:
                if d.get(k) is not None:
                    vals[k].append(float(d[k]))
        for k, xs in vals.items():
            if len(xs) < 3:
                print(f"{cen} {slug} {k}: só {len(xs)} voos")
                falhou = True
                continue
            amp = (max(xs) - min(xs)) / (sum(xs) / len(xs))
            ok = amp <= LIMITE
            print(f"{cen:24s} {slug:7s} {k:15s} n={len(xs)} amplitude {100 * amp:.2f} % {'ok' if ok else 'FALHOU'}")
            falhou |= not ok
    sys.exit(1 if falhou else 0)


if __name__ == "__main__":
    main()
