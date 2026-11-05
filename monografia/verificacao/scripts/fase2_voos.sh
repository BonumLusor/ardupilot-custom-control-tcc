#!/bin/bash
# Fase 2 do plano da revisao de 3/11: voos de MEDICAO do procedimento (Cap. 3).
# Nao entram nas tabelas: --destino os poe sob verificacao_2026-11-04/medicoes/,
# que validacao.py e tabelas_monografia.py ignoram.
#   1. ATC_INPUT_TC = 0,02 (o dos ensaios): referencia para os tres seguintes
#   2. ATC_INPUT_TC = 0,10 (padrao do Copter): erro de rastreamento do alvo modelado
#   3. leitura antiga da fila (--fila-antiga): atraso do degrau e do retorno
#   4. LOG_BITMASK com os bits 18, 19 e 21 (145151 + 2883584): tamanho do .BIN
cd /home/gustavo/faculdade/TCC
source /home/gustavo/venv-ardupilot/bin/activate
D=ensaios/resultados/verificacao_2026-11-04/medicoes
VOO="python ensaios/run_ensaio.py --cenario rolagem/nominal --controlador nativo --sem-gif"

echo "== 1. ATC_INPUT_TC 0,02 ($(date))"
$VOO --destino $D/input_tc_002
echo "== 2. ATC_INPUT_TC 0,10 ($(date))"
$VOO --destino $D/input_tc_010 --param ATC_INPUT_TC=0.10
echo "== 3. fila antiga ($(date))"
$VOO --destino $D/fila_antiga --fila-antiga
echo "== 4. LOG_BITMASK com os bits 18, 19 e 21 ($(date))"
$VOO --destino $D/log_bits_18_19_21 --param LOG_BITMASK=3028735
echo "== fim ($(date))"
