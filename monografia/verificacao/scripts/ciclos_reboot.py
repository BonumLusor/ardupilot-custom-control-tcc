#!/usr/bin/env python3
"""Fase 2 do plano da revisao de 3/11: conta travamentos do reboot do SITL (Cap. 3, 3.6.2).

Cada ciclo sobe um SITL limpo (instancia 1, para nao disputar a porta com ensaios),
conecta e pede o reboot que liga o motor de scripts e o AC_CustomControl:

  antigo: manda o MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN e fecha a conexao em seguida,
          como o orquestrador fazia antes da correcao;
  novo:   run_ensaio.reiniciar() com UMA tentativa (espera o COMMAND_ACK, le ate o
          proprio SITL fechar a conexao e confere o reboot pela linha de comando).

Resultado por ciclo: reiniciou, travou (sem reboot e com o binario a >80 % de CPU) ou
descartou (sem reboot, binario ocioso). Os modos se alternam, para que a carga do
computador pese igual nos dois.

Uso: python ciclos_reboot.py [N por modo, padrao 30]
"""
import sys
import time
from pathlib import Path

# os voos e a saida ficam em ensaios/resultados/verificacao_2026-11-04/
AQUI = Path(__file__).resolve().parents[3] / "ensaios" / "resultados" / "verificacao_2026-11-04"
sys.path.insert(0, str(AQUI.parents[1]))                 # ensaios/
import run_ensaio as R                                     # noqa: E402

INSTANCIA = 1
SAIDA = AQUI / "ciclos_reboot"


def ciclo(modo, i):
    from pymavlink import mavutil
    sitl = R.subir_sitl(5, instancia=INSTANCIA, saida=SAIDA / f"{modo}_{i:02d}.log")
    try:
        m = R.conectar(INSTANCIA, sitl=sitl)
        if modo == "novo":
            try:
                R.reiniciar(m, INSTANCIA, sitl=sitl, tentativas=1)
            except SystemExit:
                pass
        else:
            reboot = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            m.mav.command_long_send(m.target_system, m.target_component, reboot,
                                    0, 1, 0, 0, 0, 0, 0, 0)
            m.close()
        fim = time.monotonic() + 20
        while time.monotonic() < fim:
            if sitl.reiniciou():
                return "reiniciou"
            time.sleep(0.5)
        procs = sitl.processos()
        if not procs:
            return "morreu"
        cpu = sum(p.cpu_percent(interval=2.0) for p in procs)
        return "travou" if cpu > 80 else "descartou"
    except SystemExit as exc:
        return f"erro: {exc}"
    finally:
        R.derrubar_sitl(sitl)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    SAIDA.mkdir(exist_ok=True)
    contagem = {"antigo": {}, "novo": {}}
    with open(AQUI / "ciclos_reboot.txt", "w", encoding="utf-8") as f:
        for i in range(n):
            for modo in ("antigo", "novo"):
                r = ciclo(modo, i)
                contagem[modo][r] = contagem[modo].get(r, 0) + 1
                linha = f"{time.strftime('%H:%M:%S')} ciclo {i + 1:2d} {modo:6s} {r}"
                print(linha, flush=True)
                f.write(linha + "\n")
                f.flush()
        for modo, c in contagem.items():
            linha = f"{modo}: " + ", ".join(f"{k} {v}" for k, v in sorted(c.items())) + f" (de {n})"
            print(linha)
            f.write(linha + "\n")


if __name__ == "__main__":
    main()
