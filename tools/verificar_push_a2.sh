#!/usr/bin/env bash
# AC-04 / A2: control posterior al push. No altera datos de trabajo.
set -euo pipefail
RAMA="desarrollo-avance2"
cd "$(dirname "$0")/.."
if [[ "$(git branch --show-current)" != "$RAMA" ]]; then
  echo "[ERROR] Rama incorrecta. Se esperaba $RAMA" >&2
  exit 1
fi
LOCAL="$(git rev-parse HEAD)"
echo "[INFO] Commit local: ${LOCAL}"
git fetch --quiet origin "refs/heads/${RAMA}"
REMOTO="$(git rev-parse FETCH_HEAD)"
echo "[INFO] Commit publicado: ${REMOTO}"
if [[ "$LOCAL" != "$REMOTO" ]]; then
  echo '[ERROR] El remoto no coincide con el ultimo commit local.' >&2
  exit 1
fi
TEMP="$(mktemp -d)"
trap 'rm -rf "$TEMP"' EXIT
# Comprobar el arbol del commit realmente recibido, no el directorio de trabajo.
git archive "$REMOTO" | tar -xf - -C "$TEMP"
python3 "$TEMP/tools/verificar_publicacion_a2.py"
python3 "$TEMP/tools/verificar_paquete_ac03.py"
echo '[OK] Push confirmado: commit local y remoto coinciden; ambas auditorias del arbol remoto aprobaron.'
echo '[PENDIENTE] Abrir GitHub Actions y confirmar el check verde "validar-avance2".'
