#!/usr/bin/env python3
"""Revisão de 3/11 (Fase 6): a integral de Bode discreta com os controladores do lote.

Para cada controlador, com os ganhos do params.parm de um voo do lote e a planta do
portão (projeto.planta_real: filtro do giroscópio, segurador, ruído do AHRS no ângulo e
um laço de atraso), calcula a sensibilidade na entrada da planta, S = 1/(1 + L), em
theta = w*T de 0 a pi, e
  - a integral (1/pi) * int_0^pi ln|S| dtheta, que a teoria dá nula sem polos instáveis
    de malha aberta (sung1988, mohtadi1990);
  - as áreas negativa e positiva de ln|S| (mesma normalização), que a integral nula
    obriga a serem iguais;
  - o pico de |S| (a menor distância de L a -1).
Os números substituem os do relatório de 2/11, feitos com o LQG anterior ao
reprojeto do ruído (ANGULO_INDEP). Escreve ../bode_areas.txt.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[2]
sys.path.insert(0, str(RAIZ / "modelagem"))
import modelo_sitl as ms  # noqa: E402
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402

RES = RAIZ / "ensaios" / "resultados"
LOTE = (RAIZ / "ensaios" / "lotes" / "binario_lote.sha256").read_text().split()[0]
SLUGS = ("nativo", "cc_lqi", "cc_eso", "cc_lqi_mesmo_ruido", "cc_lqg", "cc_lqg_mesmo_ruido")
EIXOS = (("roll", "rolagem"), ("pitch", "arfagem"), ("yaw", "guinada"))


def params_do_lote(slug):
    for pasta in sorted((RES / "rolagem" / "nominal" / slug).glob("2026-*")) + \
            sorted((RES / "guinada" / "nominal" / slug).glob("2026-*")):
        v = pasta / "versao.json"
        if v.exists() and json.loads(v.read_text())["binario_sha256"] == LOTE:
            return pasta, ms.ler_parm(pasta / "params.parm")
    return None, None


def areas(an_ctl, mod, eixo, ts, fc):
    """(integral, área negativa, área positiva, pico de |S|) de ln|S| em theta de 0 a pi."""
    th = np.concatenate([np.logspace(-7, -2, 4000, endpoint=False), np.linspace(1e-2, math.pi, 200000)])
    an = P.analisar(an_ctl, mod.a, mod.b[eixo], ts, fc, w=th / ts, planta=P.planta_real(eixo))
    lns = np.log(np.abs(1.0 / (1.0 + an.L)))
    tot = np.trapezoid(lns, th) / math.pi
    neg = np.trapezoid(np.minimum(lns, 0.0), th) / math.pi
    pos = np.trapezoid(np.maximum(lns, 0.0), th) / math.pi
    return tot, -neg, pos, float(np.exp(lns.max())), an.estavel()


def main():
    mod = P.carregar_modelo()
    ts = 1.0 / 400.0
    linhas = [f"Integral de Bode discreta, (1/pi) int_0^pi ln|S(e^jtheta)| dtheta, planta do portão, "
              f"ganhos dos voos do binário {LOTE[:12]}.",
              "controlador         eixo      integral    área neg.  área pos.  pico |S|"]
    for slug in SLUGS:
        pasta, p = params_do_lote(slug)
        if p is None:
            linhas.append(f"{slug}: sem voo do lote")
            continue
        nome, prefixo = V.controlador_do_ensaio(pasta, p)
        fc = float(p.get("INS_GYRO_FILTER", 20.0))
        for eixo, rot in EIXOS:
            ctl, _ = V.montar_controlador(nome, prefixo, p, eixo, mod.a, mod.b[eixo], ts)
            tot, neg, pos, pico, est = areas(ctl, mod, eixo, ts, fc)
            linhas.append(f"{slug:19s} {rot:9s} {tot:+.2e}   {neg:.4f}     {pos:.4f}     {pico:.3f}"
                          f"{'' if est else '  (instável?)'}")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "bode_areas.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
