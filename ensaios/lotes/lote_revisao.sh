#!/bin/bash
# Lote da revisao de 3/11 (plano_revisao_2026-11-04.md, Fase 5), com o firmware congelado.
# Cada voo grava versao.json (hash do binario do SITL); --pular-feitos pula o que ja foi
# voado com o mesmo binario, entao rodar de novo este script retoma sem repetir voo.
# Ordem (P2): L4 primeiro (para, se o determinismo falhar), depois o que o resumo usa.
cd /home/gustavo/faculdade/TCC
source /home/gustavo/venv-ardupilot/bin/activate
L="ensaios/run_ensaio.py --sem-gif --pular-feitos"
ANTIGOS="nativo nativo_lqi nativo_filtro10 cc_pid cc_pid_ff cc_lqi cc_lqi_filtro10 cc_lqi_mesmo_ruido cc_lqi_sem_ff cc_eso cc_eso_filtro10 cc_lqg cc_lqg_filtro10 cc_lqg_mesmo_ruido"
NUCLEO5="nativo cc_lqi cc_eso cc_lqg cc_lqg_mesmo_ruido"
CEN_ANTIGOS=$(find ensaios/cenarios -name '*.yaml' | sed 's|ensaios/cenarios/||; s|\.yaml$||' | grep -v 'torque_impulso\|^comutacao' | sort)

bloco() {   # bloco <descricao> <argumentos do run_ensaio>: registra a falha e segue
    local d="$1"; shift
    python $L "$@" || echo "AVISO: o bloco '$d' terminou com falhas (ver o resumo acima)"
}

echo "== 1. L4, determinismo ($(date))"
bloco "L4" --cenario rolagem/torque_impulso rolagem/torque_degrau --controlador nativo cc_lqg --repeticoes 3
if ! python ensaios/lotes/criterio_l4.py; then
    echo "== PARADA: o criterio de determinismo do L4 falhou ($(date))"
    exit 1
fi

echo "== 2. L5, nativo_guinada ($(date))"
bloco "L5" --controlador nativo_guinada
bloco "L5 sementes" --cenario guinada/torque_estocastico --controlador nativo_guinada --sementes 2-5
bloco "L5 rnd20" --cenario ruido/rnd20/torque_impulso --controlador nativo_guinada --repeticoes 5

echo "== 3. saturacao e grandes angulos ($(date))"
bloco "saturacao" --cenario saturacao grande --controlador $ANTIGOS

echo "== 4. L1 e L1b, torque estocastico, sementes 2 a 5 ($(date))"
bloco "L1" --cenario rolagem/torque_estocastico arfagem/torque_estocastico guinada/torque_estocastico --controlador $NUCLEO5 --sementes 2-5
bloco "L1b" --cenario guinada/torque_estocastico --controlador cc_lqi_mesmo_ruido --sementes 2-5

echo "== 5. L2, ruido RND 20 ($(date))"
bloco "L2 x5" --cenario ruido/rnd20/torque_impulso --controlador nativo cc_lqg cc_lqg_mesmo_ruido --repeticoes 5
bloco "L2 x1" --cenario ruido/rnd20/torque_impulso --controlador cc_lqi cc_eso cc_lqi_mesmo_ruido

echo "== 6. L3, impulso curto ($(date))"
bloco "L3" --cenario rolagem/torque_impulso arfagem/torque_impulso guinada/torque_impulso --controlador $NUCLEO5
bloco "L3 guinada" --cenario guinada/torque_impulso --controlador cc_lqi_mesmo_ruido

echo "== 7. R, o revoo das tabelas e figuras ($(date))"
bloco "R" --cenario $CEN_ANTIGOS --controlador $ANTIGOS

echo "== 8. L7, ablacao do LQG ($(date))"
bloco "L7" --controlador cc_lqg_sem_z cc_lqg_sem_bg

echo "== 9. L8, comutacao ($(date))"
bloco "L8" --cenario comutacao --controlador cc_lqi cc_eso cc_lqg

echo "== fim ($(date))"
