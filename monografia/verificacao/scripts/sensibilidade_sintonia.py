#!/usr/bin/env python3
"""Revisão de 3/11, item 2.10: sensibilidade da regra de sintonia ao alvo.

A regra (projeto.sintonizar) fixa a frequência de cruzamento do nativo, exige margem
de fase não menor que a dele e amortecimento dominante de ao menos ZETA_MIN, e
escolhe o ponto de modo mais lento mais rápido. Rolagem e arfagem são fisicamente
iguais, mas os alvos do nativo diferem ~1 a 2,5 % (ganhos do AutoTune), e a regra
escolheu T_I = 0,171 e 0,263 s no LQI e omega_o = 9,09 e 7,86 rad/s no ESO.

Aqui o alvo de cruzamento e o piso de margem de fase variam de -5 % a +5 % em torno
dos do nativo de cada eixo, e se registra o ponto escolhido. Também se projeta
cada eixo com o alvo do outro e com a média dos dois. Os ganhos voados não mudam.
Escreve ../sensibilidade_sintonia.txt.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "modelagem"))
import projeto as P  # noqa: E402

ESCALAS = (0.95, 0.975, 1.0, 1.025, 1.05)


def alvo_do_nativo(mod, eixo):
    nat = P.novo_controlador(mod, eixo, "nativo")
    return P.analisar(nat, mod.a, mod.b[eixo], mod.ts, mod.fc_giro)


def escolher(mod, eixo, variante, wc, mf, ruido):
    alvo = SimpleNamespace(margens={"wc": wc, "mf_graus": mf}, ruido=ruido)
    try:
        v, _ = P.sintonizar(mod, eixo, variante, alvo)
    except SystemExit:
        return None
    return v


def main():
    mod = P.carregar_modelo()
    an = {e: alvo_do_nativo(mod, e) for e in ("roll", "pitch")}
    linhas = []
    for e in ("roll", "pitch"):
        m = an[e].margens
        linhas.append(f"{P.NOMES[e]}: nativo wc = {m['wc']:.2f} rad/s, MF = {m['mf_graus']:.2f} graus")
    for var, nome_par, fmt in (("lqi", "T_I (s)", "{:.3f}"), ("eso", "omega_o (rad/s)", "{:.2f}")):
        linhas.append(f"\n{var.upper()}: {nome_par} escolhido; linhas: escala do alvo de wc; "
                      f"colunas: escala do piso de MF {ESCALAS}")
        for e in ("roll", "pitch"):
            m = an[e].margens
            linhas.append(f"  {P.NOMES[e]}")
            for sw in ESCALAS:
                cel = []
                for sm in ESCALAS:
                    v = escolher(mod, e, var, m["wc"] * sw, m["mf_graus"] * sm, an[e].ruido)
                    cel.append("  ---  " if v is None else fmt.format(v["parametro"]).rjust(7))
                linhas.append(f"    wc x {sw:5.3f}: " + " ".join(cel))
        # cada eixo com o alvo do outro e com a media
        mr, mp = an["roll"].margens, an["pitch"].margens
        media = {"wc": (mr["wc"] + mp["wc"]) / 2, "mf_graus": (mr["mf_graus"] + mp["mf_graus"]) / 2}
        for rotulo, alvo in (("alvo da rolagem", mr), ("alvo da arfagem", mp), ("alvo médio", media)):
            cel = []
            for e in ("roll", "pitch"):
                v = escolher(mod, e, var, alvo["wc"], alvo["mf_graus"], an[e].ruido)
                cel.append(f"{P.NOMES[e]} " + ("---" if v is None else fmt.format(v["parametro"])))
                if v is not None:
                    cel[-1] += f" (lento {v['lento']:.2f} 1/s)"
            linhas.append(f"  {rotulo:16s}: " + "; ".join(cel))
    texto = "\n".join(linhas)
    print(texto)
    (Path(__file__).resolve().parents[1] / "sensibilidade_sintonia.txt").write_text(texto + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
