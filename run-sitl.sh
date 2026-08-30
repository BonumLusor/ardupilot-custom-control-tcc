#!/bin/bash
# Lançador do ArduPilot SITL — garante o PATH e conecta ao Gazebo via JSON
export PATH="$HOME/faculdade/TCC/ardupilot/Tools/autotest:$HOME/.local/bin:$PATH"

# Pequena folga para o Gazebo subir primeiro (quando os dois sobem juntos).
sleep 4

cd "$HOME/faculdade/TCC/ardupilot/ArduCopter" || exit 1

# Sem --console e sem --map: quem faz mapa/telemetria agora é o QGroundControl.
# O MAVProxy continua neste terminal para você digitar arm/takeoff/etc.
#
# --add-param-file é necessário: o binário SITL resolve os parâmetros do frame
# pelo valor de --model (aqui "JSON"), não pelo -f, então os defaults do
# gazebo-iris (FRAME_CLASS/FRAME_TYPE) não seriam aplicados e o veículo
# reportaria "Frame: UNSUPPORTED" e recusaria armar.
PARM="$HOME/faculdade/TCC/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"

exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --add-param-file "$PARM"