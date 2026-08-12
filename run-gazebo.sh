#!/bin/bash
# Lançador do Gazebo — define as próprias variáveis (não depende do .bashrc)
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/faculdade/TCC/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/faculdade/TCC/ardupilot_gazebo/models:$HOME/faculdade/TCC/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

# Descomente a linha abaixo se o Gazebo abrir tela preta ou travar no WSL:
# export LIBGL_ALWAYS_SOFTWARE=1

exec gz sim -v4 -r iris_runway.sdf