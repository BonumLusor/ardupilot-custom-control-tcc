#!/bin/bash
# Instala os backends AC_CustomControl_LQR e AC_CustomControl_LQG no submodulo
# ardupilot/.
#
# O codigo dos controladores vive neste repositorio (firmware/), e nao num fork
# do ArduPilot: este script liga os arquivos dos backends em
# ardupilot/libraries/AC_CustomControl/ por link simbolico e aplica
# ardupilot.patch: as edicoes do frontend (CUSTOMCONTROL_MAX_TYPES 4, CONT_LQR e
# CONT_LQG, os case em init() e os grupos de parametros "3_" -> CC3_* e
# "4_" -> CC4_*) e a alimentacao direta pura opcional do backend PID de exemplo
# (CC2_FF*), desligada por padrao.
#
# Pode rodar quantas vezes quiser: o que ja esta feito fica como esta.
#
#   firmware/instalar.sh            # instala
#   firmware/instalar.sh --remover  # volta o submodulo ao commit original
#
# Depois de instalar, o proximo sim_vehicle.py (ou o run_ensaio.py) recompila.

set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE="$RAIZ/firmware"
AP="$RAIZ/ardupilot"
DESTINO="$AP/libraries/AC_CustomControl"
ARQUIVOS=(AC_CustomControl_LQR.h AC_CustomControl_LQR.cpp AC_CustomControl_LQG.h AC_CustomControl_LQG.cpp)
PATCH="$FIRMWARE/ardupilot.patch"

patch_aplicado() {
  git -C "$AP" apply --reverse --check "$PATCH" 2>/dev/null
}

if [ "${1:-}" = "--remover" ]; then
  for f in "${ARQUIVOS[@]}"; do
    if [ -L "$DESTINO/$f" ]; then
      rm "$DESTINO/$f"
      echo "removido o link $DESTINO/$f"
    fi
  done
  if patch_aplicado; then
    git -C "$AP" apply --reverse "$PATCH"
    echo "ardupilot.patch desfeito"
  fi
  exit 0
fi

for f in "${ARQUIVOS[@]}"; do
  alvo="$FIRMWARE/$f"
  link="$DESTINO/$f"
  if [ -L "$link" ] && [ "$(readlink -f "$link")" = "$(readlink -f "$alvo")" ]; then
    echo "ok: $link"
  elif [ -e "$link" ]; then
    echo "ERRO: $link existe e nao e o link para $alvo; confira antes de apagar" >&2
    exit 1
  else
    ln -s "$alvo" "$link"
    echo "link: $link -> $alvo"
  fi
done

if patch_aplicado; then
  echo "ok: ardupilot.patch ja aplicado"
elif git -C "$AP" apply --check "$PATCH" 2>/dev/null; then
  git -C "$AP" apply "$PATCH"
  echo "aplicado: ardupilot.patch"
else
  echo "ERRO: ardupilot.patch nao aplica no submodulo (commit diferente de ce9c2d01a4?)" >&2
  exit 1
fi
