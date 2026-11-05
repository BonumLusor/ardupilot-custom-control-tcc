#!/usr/bin/env python3
"""Revisão de 3/11, P3 revisto em 5/11: o erro lento do AHRS na base verdadeira.

A base verdadeira (ensaios/analise/verdade.py) é a atitude da CCTQ menos um desvio
constante, medido no meio segundo antes da perturbação. Nos voos do lote, a diferença
verdade − AHRS não é constante: o rumo do AHRS se afasta do verdadeiro em até ~1°
(mais com ruído de giroscópio) ao longo de dezenas de segundos, e a rolagem, em ~0,1°.
O erro é do estimador do firmware, igual com qualquer controlador, e nenhum controlador
o vê.

Este script mede:
  1. a diferença verdade − AHRS ao longo do voo, de 5 em 5 s, em voos de controladores
     diferentes (o padrão se repete);
  2. as métricas da perturbação com a parte lenta da diferença tirada (passa-baixas
     Butterworth de 2.ª ordem, fase zero, em 0,05 e 0,1 Hz), contra as do AHRS e as da
     base verdadeira das tabelas (desvio constante).
Sem ruído, tirada a parte lenta, a verdade fica igual ao AHRS a menos de ~2 %: a base
verdadeira só acrescentava o erro lento do AHRS. Por isso as tabelas saem no AHRS, e a
verdadeira vai para o Apêndice D. Escreve ../deriva_ahrs.txt.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt

AQUI = Path(__file__).resolve().parent
RAIZ = AQUI.parents[2]
sys.path.insert(0, str(RAIZ / "ensaios"))
from analise import tabelas, verdade as V  # noqa: E402
from analise.eixos import COLUNAS  # noqa: E402
from analise.leitura import _embrulhar180, ler_log, origem_perfil  # noqa: E402

RES = RAIZ / "ensaios" / "resultados"
LOTE = (RAIZ / "ensaios" / "lotes" / "binario_lote.sha256").read_text().split()[0]
CORTE = {"fc": None}


def primeiro_voo(cen, slug):
    for p in sorted((RES / cen / slug).glob("2026-*")):
        v = p / "versao.json"
        if v.exists() and json.loads(v.read_text())["binario_sha256"] == LOTE and not p.name.endswith("_falhou"):
            return p
    return None


def diferenca_no_voo(pasta):
    """(tempos do perfil, guinada verdade − AHRS, rolagem verdade − AHRS), médias de 1 s a cada 5 s."""
    d = ler_log(next(pasta.glob("*.BIN")), ("CCTQ", "ATT", "WND"))
    o = origem_perfil(d)
    rol, _, gui = V.angulos(d["CCTQ"])
    tc, ta = d["CCTQ"]["t"] - o, d["ATT"]["t"] - o
    dy = _embrulhar180(np.interp(ta, tc, gui) - d["ATT"]["Yaw"])
    dr = np.interp(ta, tc, rol) - d["ATT"]["Roll"]
    ts = np.arange(-20.0, min(float(ta[-1]), 60.0), 5.0)
    med = lambda x: [float(np.nanmean(x[(ta >= t - 0.5) & (ta <= t + 0.5)])) for t in ts]
    return ts, med(dy), med(dr)


def _lento(x, dt):
    ok = np.isfinite(x)
    y = np.interp(np.arange(len(x)), np.flatnonzero(ok), x[ok])
    b, a = butter(2, CORTE["fc"] * 2 * dt)
    return filtfilt(b, a, y)


def base_sem_deriva(dados, grade, col, jan):
    """Como verdade.base_verdadeira, com a parte lenta da diferença no lugar do desvio constante."""
    ct = dados["CCTQ"]
    rol, arf, gui = V.angulos(ct)
    dt = float(np.median(np.diff(grade)))
    na = lambda v: np.interp(grade, ct["t"], v, left=np.nan, right=np.nan)
    novo = dict(col)
    for eixo, v in (("roll", rol), ("pitch", arf)):
        c = COLUNAS[eixo]
        if c["med"] not in col:
            continue
        v = na(v)
        novo[c["med"]] = v - _lento(v - col[c["med"]], dt)
        if c["ref"] in col:
            novo[c["erro"]] = col[c["ref"]] - novo[c["med"]]
    if "erro_yaw" in col and "yaw_des" in col:
        dif = _embrulhar180(_embrulhar180(col["yaw_des"] - na(gui)) - col["erro_yaw"])
        novo["erro_yaw"] = col["erro_yaw"] + (dif - _lento(dif, dt))
    return novo, {}


def metricas_sem_deriva(pasta, eixo, fc):
    """As métricas da perturbação com a parte lenta tirada, sem gravar nada."""
    CORTE["fc"] = fc
    orig = (V.base_verdadeira, tabelas.salvar_metricas, tabelas.salvar_series)
    V.base_verdadeira = base_sem_deriva
    tabelas.salvar_metricas = tabelas.salvar_series = lambda *a, **k: None
    try:
        return V.calcular_pasta(pasta)[f"disturbio_{eixo}"]
    finally:
        V.base_verdadeira, tabelas.salvar_metricas, tabelas.salvar_series = orig


def main():
    linhas = [f"Erro lento do AHRS nos voos do binário {LOTE[:12]}."]
    linhas.append("1. Diferença verdade − AHRS (graus), média de 1 s a cada 5 s do tempo do perfil:")
    for cen, slug in (("guinada/torque_impulso", "nativo"), ("guinada/torque_impulso", "cc_lqg"),
                      ("rolagem/torque_impulso", "nativo"), ("guinada/torque_degrau", "nativo"),
                      ("ruido/rnd20/torque_impulso", "nativo")):
        ts, dy, dr = diferenca_no_voo(primeiro_voo(cen, slug))
        linhas.append(f"  {cen} × {slug}, t = " + " ".join(f"{t:+.0f}" for t in ts))
        linhas.append("    guinada: " + " ".join(f"{x:+.2f}" for x in dy))
        linhas.append("    rolagem: " + " ".join(f"{x:+.3f}" for x in dr))
    linhas.append("2. Métricas da perturbação: AHRS | verdade das tabelas (desvio constante) | "
                  "verdade sem a parte lenta, 0,05 Hz | 0,1 Hz")
    casos = [("guinada/torque_degrau", s, "yaw") for s in ("nativo", "cc_lqi", "cc_lqg", "cc_lqg_mesmo_ruido")]
    casos += [("guinada/torque_impulso", s, "yaw") for s in ("nativo", "cc_lqg_mesmo_ruido")]
    casos += [("rolagem/torque_degrau", s, "roll") for s in ("nativo", "cc_lqg_mesmo_ruido")]
    casos += [("rolagem/torque_impulso", s, "roll") for s in ("nativo", "cc_lqg_mesmo_ruido")]
    casos += [("ruido/rnd20/torque_impulso", s, e) for e in ("roll", "yaw") for s in ("nativo", "cc_lqg_mesmo_ruido")]
    maior = 0.0
    for cen, slug, eixo in casos:
        p = primeiro_voo(cen, slug)
        a = json.loads((p / "metricas.json").read_text())[f"disturbio_{eixo}"]
        v = json.loads((p / "metricas_verdade.json").read_text())[f"disturbio_{eixo}"]
        f5, f10 = metricas_sem_deriva(p, eixo, 0.05), metricas_sem_deriva(p, eixo, 0.1)
        linhas.append(f"  {cen} × {slug} ({eixo}):")
        for chave, rot in (("desvio_max_deg", "pico"), ("rmse_deg", "EQM"), ("tempo_recuperacao_s", "recuperação")):
            vals = [m.get(chave) for m in (a, v, f5, f10)]
            linhas.append(f"    {rot:12s}" + " | ".join("---" if x is None else f"{x:.3f}" for x in vals))
            if "ruido" not in cen and chave != "tempo_recuperacao_s" and vals[0]:
                maior = max(maior, abs(vals[2] / vals[0] - 1), abs(vals[3] / vals[0] - 1))
        linhas.append(f"    {'faixa calma':12s}" + " | ".join(f"{m.get('faixa_calma_deg', 0):.3f}" for m in (a, v, f5, f10)))
    linhas.append(f"Sem ruído, pico e EQM sem a parte lenta contra o AHRS: diferença máxima {100 * maior:.1f} %.")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI.parent / "deriva_ahrs.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
