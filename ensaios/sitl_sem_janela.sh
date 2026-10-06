#!/bin/sh
# "Terminal" sem janela para o SITL dos ensaios.
#
# O sim_vehicle.py nao roda o binario direto: chama o run_in_terminal_window.sh
# do ArduPilot, que abre um xterm quando ha DISPLAY. Com SITL_RITW_TERMINAL
# apontando para este arquivo, o run_in_terminal_window.sh chama
#
#     sitl_sem_janela.sh <script com o comando do binario>
#
# e o binario roda aqui, em segundo plano, com stdout e stderr no arquivo que o
# run_ensaio.py indicou em SITL_SAIDA — o sitl.log da pasta do ensaio. Sem
# janela, e com a saida do binario guardada: no xterm ela se perdia junto com a
# janela, inclusive quando o SITL morria.
#
# O <script> e um arquivo temporario em /tmp (ritw-<data>), com o comando na
# ultima linha. Ele e lido e apagado ANTES de o SITL subir: o encerramento do
# ensaio mata este shell com SIGTERM, e uma limpeza deixada para o fim nunca
# rodaria — os arquivos se acumulariam em /tmp, um por ensaio.
cmd=$(tail -n 1 "$1")
rm -f "$1"

# O subshell ( : ; ... ) fica vivo como pai do binario enquanto ele roda. Isso
# importa: o SITL se encerra sozinho quando o processo pai some
# (SITL_State.cpp, _fdm_input_step), e o run_in_terminal_window.sh termina logo
# depois de nos chamar. O ":" impede o shell de trocar o subshell pelo comando.
( : ; eval "$cmd" ) >>"${SITL_SAIDA:-/dev/null}" 2>&1 </dev/null
