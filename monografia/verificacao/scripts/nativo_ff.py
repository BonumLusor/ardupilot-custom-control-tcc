#!/usr/bin/env python3
"""Revisão de 3/11, item 2.4 (opção A): alimentação direta no PID nativo só por parâmetros.

O nativo com ATC_RAT_*_FF = a/b e ATC_RAT_*_D_FF = 1/b, contra o nativo, no modelo
(planta_real). No AC_PID, FF e D_FF multiplicam o alvo de taxa inteiro (a saída do
controlador de ângulo mais w_ff, filtrada pelo FLTT) e a sua derivada; por isso D_FF
age também como realimentação de velocidade. Compara margem de fase, ganho de ruído,
rejeição de torque (pulso, impulso, degrau) e rastreamento do degrau de atitude.
Decide-se voar o nativo_ff só se margem e ruído forem aceitáveis (decisão do autor).
Escreve ../nativo_ff.txt.
"""
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "modelagem"))
import controladores as C  # noqa: E402
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402

FAIXA_D_FF = (0.0, 0.02)          # @Range de ATC_RAT_*_D_FF no commit fixado
FAIXA_FF = (0.0, 0.5)


def cascata(mod, eixo, ff):
    g = dict(V.ganhos(mod.p, "ATC", eixo))
    if ff:
        g["kff"] = mod.a / mod.b[eixo]
        g["kdff"] = 1.0 / mod.b[eixo]
    return C.CascataPID(g, eixo, mod.ts, raiz=True), g


def main():
    mod = P.carregar_modelo()
    linhas = ["nativo contra nativo com FF = a/b e D_FF = 1/b, no modelo (planta real)"]
    for eixo in ("roll", "yaw"):
        pasta = P.GRUPO[eixo]
        alvo = P.alvo_nominal(eixo)
        res = {}
        for nome, ff in (("nativo", False), ("nativo + FF/D_FF", True)):
            ctl, g = cascata(mod, eixo, ff)
            an = P.analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, planta=P.planta_real(eixo))
            r = {"mf": an.margens["mf_graus"], "wc": an.margens["wc"], "ruido": an.ruido_branco}
            for ensaio in ("torque_pulso", "torque_impulso", "torque_degrau"):
                s = P.simular_torque(mod, eixo, cascata(mod, eixo, ff)[0], f"{pasta}/{ensaio}")
                m = s["metricas"]
                r[ensaio] = (m.get("desvio_max_deg"), m.get("tempo_recuperacao_s"))
            if alvo is not None:
                m = P.simular_degrau(mod, eixo, cascata(mod, eixo, ff)[0], alvo,
                                     planta=P.planta_real(eixo))["metricas"]
                r["degrau_atitude"] = (m.get("sobressinal_pct"), m.get("erro_rastreio_max_deg"))
            res[nome] = (r, g)
        linhas.append(f"{P.NOMES[eixo]}: FF = {mod.a / mod.b[eixo]:.4g} (faixa {FAIXA_FF}), "
                      f"D_FF = {1.0 / mod.b[eixo]:.4g} (faixa {FAIXA_D_FF})"
                      + ("  [D_FF fora da faixa documentada]" if 1.0 / mod.b[eixo] > FAIXA_D_FF[1] else ""))
        base = res["nativo"][0]
        for nome, (r, _) in res.items():
            txt = (f"  {nome:17s}: wc {r['wc']:.1f} rad/s, MF {r['mf']:.1f} graus, ruído {r['ruido']:.4f} "
                   f"({r['ruido'] / base['ruido']:.2f}x)")
            for ensaio in ("torque_pulso", "torque_impulso", "torque_degrau"):
                dv, rec = r[ensaio]
                f2 = lambda v, c: "---" if v is None else f"{v:.{c}f}"
                txt += f"; {ensaio.replace('torque_', '')} {f2(dv, 3)} graus / {f2(rec, 2)} s"
            if "degrau_atitude" in r:
                so, er = r["degrau_atitude"]
                txt += f"; degrau de atitude: sobressinal {so:.1f} %, erro máx. {er:.3f} graus"
            linhas.append(txt)
    texto = "\n".join(linhas)
    print(texto)
    (RAIZ / "monografia" / "verificacao" / "nativo_ff.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
