#!/usr/bin/env python3
"""Fase 2 do plano da revisao de 3/11: mede, nos voos de medicao, os numeros do Cap. 3.

  1. Erro de rastreamento do alvo modelado no degrau de rolagem (ANG: DesRoll - Roll),
     com ATC_INPUT_TC = 0,02 e 0,10.
  2. Atraso entre o instante programado do degrau (Tp = 6 s) e do retorno (Tp = 11 s)
     e a saida do alvo ANG.DesRoll do patamar (2 % da amplitude), com a leitura
     atual da fila e com a antiga (--fila-antiga). Mesmo metodo da verificacao de 2/11
     (inventario, C03-122).
  3. Tamanho do .BIN com e sem os bits 18, 19 e 21 do LOG_BITMASK, e a duracao do log.

Uso: python medir_fase2.py  (le medicoes/, escreve fase2_medicoes.txt ao lado)
"""
import sys
from pathlib import Path

import numpy as np

# os voos e a saida ficam em ensaios/resultados/verificacao_2026-11-04/
AQUI = Path(__file__).resolve().parents[3] / "ensaios" / "resultados" / "verificacao_2026-11-04"
sys.path.insert(0, str(AQUI.parents[1]))           # ensaios/
from analise import leitura                          # noqa: E402

T_DEGRAU, T_RETORNO = 6.0, 11.0                      # perfil de rolagem/nominal
AMPLITUDE = 10.0                                     # graus


def voo(nome):
    pastas = sorted((AQUI / "medicoes" / nome).glob("rolagem/nominal/nativo/*/"))
    if not pastas:
        raise SystemExit(f"sem voo em medicoes/{nome}")
    pasta = pastas[-1]
    return pasta, next(pasta.glob("*.BIN"))


def saida_do_patamar(t, alvo, t_prog, antes, depois):
    """Primeiro instante depois de t_prog - 0,5 s em que o alvo sai 2 % do patamar."""
    lim = 0.02 * abs(depois - antes)
    i = np.searchsorted(t, t_prog - 0.5)
    fora = np.nonzero(np.abs(alvo[i:] - antes) > lim)[0]
    return float(t[i + fora[0]]) if len(fora) else float("nan")


def medir(nome):
    pasta, binario = voo(nome)
    d = leitura.ler_log(binario, ("ANG", "WND"))
    ang, origem = d["ANG"], leitura.origem_perfil(d)
    tp = ang["t"] - origem                          # tempo do perfil
    des, med = ang["DesRoll"], ang["Roll"]
    # 1. erro de rastreamento no degrau: do degrau ate o retorno
    j = (tp >= T_DEGRAU) & (tp < T_RETORNO)
    erro_max = float(np.max(np.abs(des[j] - med[j])))
    # 2. atrasos
    a_deg = saida_do_patamar(tp, des, T_DEGRAU, 0.0, AMPLITUDE) - T_DEGRAU
    a_ret = saida_do_patamar(tp, des, T_RETORNO, AMPLITUDE, 0.0) - T_RETORNO
    # 3. tamanho e duracao do log
    mb = binario.stat().st_size / 1e6
    dur = float(max(tab["t"][-1] for tab in d.values() if "t" in tab))
    return dict(pasta=pasta.relative_to(AQUI), erro_max=erro_max, atraso_degrau=a_deg,
                atraso_retorno=a_ret, mb=mb, dur=dur)


def main():
    linhas = []
    res = {n: medir(n) for n in ("input_tc_002", "input_tc_010", "fila_antiga", "log_bits_18_19_21")}
    for n, r in res.items():
        linhas.append(f"{n:20s} {r['pasta']}")
        linhas.append(f"    erro max. DesRoll-Roll no degrau: {r['erro_max']:.3f} graus")
        linhas.append(f"    atraso do degrau {r['atraso_degrau']:.3f} s, do retorno {r['atraso_retorno']:.3f} s")
        linhas.append(f"    .BIN {r['mb']:.1f} MB em {r['dur']:.0f} s de log ({r['mb'] / r['dur'] * 60:.1f} MB/min)")
    base, bits = res["input_tc_002"], res["log_bits_18_19_21"]
    linhas.append("")
    linhas.append(f"bits 18/19/21: {bits['mb']:.1f} MB contra {base['mb']:.1f} MB "
                  f"({bits['mb'] / base['mb']:.2f}x; taxa {bits['mb'] / bits['dur'] * 60:.1f} "
                  f"contra {base['mb'] / base['dur'] * 60:.1f} MB/min)")
    texto = "\n".join(linhas)
    print(texto)
    (AQUI / "fase2_medicoes.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
