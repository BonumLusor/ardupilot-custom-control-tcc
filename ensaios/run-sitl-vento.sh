#!/bin/bash
# Lançador do SITL PURO com o injetor de vento — para pilotar o ensaio à mão.
#
# Diferente do run-sitl.sh da raiz, este NÃO usa --model JSON e NÃO fala com o
# Gazebo: a física é a interna do SITL, que é a única em que os SIM_WIND_* têm
# efeito real (com --model JSON o SIM_JSON zera o vento internamente).
#
# Para ensaios automatizados use ensaios/run_ensaio.py, que faz tudo isto e
# ainda coleta o log, calcula as métricas e desenha o painel e o GIF.

set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$RAIZ/ardupilot/Tools/autotest:$HOME/.local/bin:$PATH"

# O injetor é um script Lua; o SITL o procura em ./scripts relativo ao cwd.
if [ ! -e "$RAIZ/ardupilot/ArduCopter/scripts/vento.lua" ]; then
  echo "criando o symlink ardupilot/ArduCopter/scripts -> ensaios/lua"
  ln -sfn ../../ensaios/lua "$RAIZ/ardupilot/ArduCopter/scripts"
fi

cd "$RAIZ/ardupilot/ArduCopter"

cat <<'AJUDA'
────────────────────────────────────────────────────────────────────────────
 SITL puro + injetor de vento.  No MAVProxy, comece por (uma vez, com reboot):

   param set SCR_ENABLE 1
   param set SIM_WIND_T 1        # sem isto, a 5 m só chega ~29% do vento
   param set SIM_WIND_TC 0.1     # piso do firmware; o padrão 5 s mata a rajada
   param set SIM_WIND_TURB 0     # desliga a turbulência IIR nativa
   reboot

 Depois escolha o perfil. O WND_MODE é um bitmask somável:

   VENTO (força — move a aeronave, quase não mexe na atitude)
     1 = médio    2 = rajada 1-cosseno    4 = turbulência de Dryden
   TORQUE (rad/s² — é ISTO que exercita a malha de atitude)
     8 = pulso    16 = meio cosseno       32 = degrau sustentado

   param set WND_SPD 8 ; param set WND_DIR 90
   param set WND_GA 10 ; param set WND_GT0 14 ; param set WND_GDUR 3
   param set WND_TSIG 1.5 ; param set WND_TLU 200 ; param set WND_SEED 1
   param set WND_TQX 15 ; param set WND_TQT0 14 ; param set WND_TQDUR 0.3
   param set WND_T0 8            # espera após o armamento (≥5 s: o SITL ignora
                                 # vento nos primeiros 5 s de simulação)
   param set WND_MODE 3          # ou 9 para vento zero + pulso de torque

 O relógio do perfil zera no ARMAMENTO. Para conferir a forma no chão, sem armar,
 use WND_T0 -1 (modo de bancada) e acompanhe com:

   module load graph
   graph SIM_WIND_SPD

 O que foi aplicado vai para o .BIN na mensagem WND (Wn/We/Wd do vento, Tx/Ty/Tz
 do torque), junto de ATT/RATE/PIDR.
────────────────────────────────────────────────────────────────────────────
AJUDA

# -f X: frame X (FRAME_TYPE 1, --model X), o mesmo dos ensaios automatizados e do
# ambiente Gazebo.
exec sim_vehicle.py -v ArduCopter -f X --console "$@"
