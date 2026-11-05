#!/usr/bin/env python3
"""Revisão de 3/11, item 2.3: a linha de base de guinada ressintonizada (nativo_guinada.yaml).

O nativo com os ganhos de guinada do ponto "PID (cascata, ganhos livres), melhor" do
portão (Tabela 13: veredito["yaw"]["relaxados"]["pid"]), e rolagem e arfagem do
AutoTune. A família PID do portão multiplica os ganhos do nativo: kp (P de taxa),
ki_kp (razão I/P), ang_p (P de ângulo) e flte (filtro do erro de taxa, Hz).
Usa o cache da varredura do reprojeto e grava ensaios/controladores/nativo_guinada.yaml.
"""
import pickle
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(RAIZ / "modelagem"))
import projeto as P  # noqa: E402
import validacao as V  # noqa: E402

CACHE = RAIZ / "ensaios" / "resultados" / "verificacao_2026-11-04" / "projeto_portao_cache_v2.pkl"
DESTINO = RAIZ / "ensaios" / "controladores" / "nativo_guinada.yaml"


def main():
    mod = P.carregar_modelo()
    proj = P.projetar(mod)
    pontos = pickle.loads(CACHE.read_bytes())
    P._CTX["mod"] = mod
    ver = P.portao(mod, pontos, proj)
    ponto = ver["yaw"]["relaxados"]["pid"]
    par = ponto["par"]
    g = V.ganhos(mod.p, "ATC", "yaw")
    kp = g["kp"] * par["kp"]
    ki = kp * par["ki_kp"]
    ang_p = g["ang_p"] * par["ang_p"]
    flte = par["flte"]
    met = ponto.get("metricas", {})
    print(f"ponto: {par}")
    print(f"guinada: ATC_RAT_YAW_P {kp:.6f} (nativo {g['kp']:.6f}), _I {ki:.6f} (nativo {g['ki']:.6f}), "
          f"ATC_ANG_YAW_P {ang_p:.6f} (nativo {g['ang_p']:.6f}), FLTE {flte:g} (nativo {g['flte']:g})")
    print(f"previsto no modelo: {met}")

    texto = (RAIZ / "ensaios" / "controladores" / "nativo.yaml").read_text(encoding="utf-8")
    troca = {"ATC_RAT_YAW_P": kp, "ATC_RAT_YAW_I": ki, "ATC_RAT_YAW_FLTE": flte, "ATC_ANG_YAW_P": ang_p}
    linhas = []
    for linha in texto.splitlines():
        chave = linha.strip().split(":")[0]
        if chave in troca and linha.startswith("  "):
            linha = f"  {chave}: {troca[chave]:.6g}"
        linhas.append(linha)
    texto = "\n".join(linhas) + "\n"
    texto = texto.replace('nome: "PID nativo"', 'nome: "PID nativo, guinada ressintonizada"')
    texto = texto.replace("slug: nativo\n", "slug: nativo_guinada\n")
    texto = texto.replace("""descricao: >
  Cascata padrao do ArduPilot. E a referencia contra a qual todo controlador
  proposto e medido.""", f"""descricao: >
  O nativo com os ganhos de guinada do melhor PID em cascata de ganhos livres do
  portao (modelagem/projeto.py, Tabela 13), sob as mesmas restricoes de ruido e de
  margem; rolagem e arfagem ficam com os do AutoTune. Ponto da varredura: {par}.
  E a linha de base de guinada sem a razao k_I = k_P/10 do AutoTune (revisao de
  3/11, item 2.3). Gerado por monografia/verificacao/scripts/nativo_guinada.py.""")
    texto = texto.replace("ganhos_de_interesse:\n", "ganhos_de_interesse:\n  - ATC_RAT_YAW_P\n  - ATC_RAT_YAW_I\n  - ATC_ANG_YAW_P\n  - ATC_RAT_YAW_FLTE\n", 1)
    # so os cenarios do bloco L5 do plano da revisao (run_ensaio.voa_no_cenario)
    texto += ("\n# So os cenarios do bloco L5 do plano da revisao de 3/11: a guinada, o\n"
              "# ruido de RND 20 e os de robustez que o texto compara em guinada.\n"
              "cenarios:\n- guinada\n- ruido/rnd20/torque_impulso\n- saturacao\n"
              "- falha_motor\n- erro_modelo\n")
    DESTINO.write_text(texto, encoding="utf-8")
    print(f"yaml: {DESTINO}")


if __name__ == "__main__":
    main()
