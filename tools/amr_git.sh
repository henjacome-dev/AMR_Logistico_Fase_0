#!/usr/bin/env bash
# Gestor local del repositorio AMR. Requiere Git y acceso de escritura remoto.
# Uso: bash amr_git_automatizado.sh instalar
# Tras instalar y recargar ~/.bashrc: amr iniciar | amr guardar "mensaje" | amr estado
set -Eeuo pipefail
REPO="${AMR_REPO:-$HOME/AMR_publicacion_u3s7-entrega-final-2026-11-16}"
RAMa="u3s7-entrega-final-2026-11-16"
SELF="$(readlink -f "${BASH_SOURCE[0]}")"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
ensure_repo() {
  [[ -d "$REPO/.git" ]] || fail "No se encuentra el repositorio Git en: $REPO"
  cd "$REPO"
  [[ "$(git branch --show-current)" == "$RAMa" ]] || fail "No estás en la rama $RAMa. Rama actual: $(git branch --show-current)"
  [[ "$(git remote get-url origin)" == 'https://github.com/henjacome-dev/AMR_Logistico_Fase_0.git' ]] || fail "El remoto origin no coincide con el repositorio previsto. Revísalo antes de publicar."
}

cmd="${1:-ayuda}"
case "$cmd" in
  instalar)
    [[ -d "$REPO/.git" ]] || fail "No está el repositorio en $REPO"
    mkdir -p "$REPO/tools"
    if [[ "$SELF" != "$(readlink -f "$REPO/tools/amr_git.sh" 2>/dev/null || echo '')" ]]; then
      cp "$SELF" "$REPO/tools/amr_git.sh"
      chmod +x "$REPO/tools/amr_git.sh"
    fi
    RC="$HOME/.bashrc"
    if ! grep -q '^# BEGIN AMR GIT GESTOR$' "$RC" 2>/dev/null; then
      cat >> "$RC" <<'EOF'

# BEGIN AMR GIT GESTOR
amr() {
  local repo="${AMR_REPO:-$HOME/AMR_publicacion_u3s7-entrega-final-2026-11-16}"
  [[ -d "$repo/.git" ]] || { echo "Repositorio AMR no encontrado: $repo" >&2; return 1; }
  cd "$repo" || return 1
  bash "$repo/tools/amr_git.sh" "$@"
}
# END AMR GIT GESTOR
EOF
    fi
    printf 'Instalado en %s/tools/amr_git.sh\n' "$REPO"
    printf 'Recarga tu terminal: source ~/.bashrc\n'
    printf 'Comandos: amr iniciar | amr guardar "descripcion" | amr revisar-a2 | amr estado\n'
    printf 'NOTA: el archivo tools/amr_git.sh se guardará en Git en la siguiente publicación.\n'
    ;;
  iniciar)
    ensure_repo
    [[ -z "$(git status --porcelain)" ]] || fail "Hay cambios sin guardar. Ejecuta 'amr estado' y decide si publicarlos con 'amr guardar "mensaje"'."
    git pull --ff-only origin "$RAMa"
    printf '\nActualización de la rama terminada. Directorio: %s\n' "$REPO"
    ;;
  guardar)
    ensure_repo
    msg="${2:-}"
    [[ -n "$msg" ]] || fail 'Indica el mensaje, por ejemplo: amr guardar "docs(a2): añadir informe y comprobante"'
    git fetch origin "refs/heads/$RAMa:refs/remotes/origin/$RAMa"
    git merge-base --is-ancestor "refs/remotes/origin/$RAMa" HEAD || fail "La rama remota avanzó o el historial divergió; primero resuelve la sincronización manualmente."
    git add -A
    if git diff --cached --quiet; then
      printf 'No hay archivos modificados para publicar.\n'; exit 0
    fi
    git diff --cached --check || fail "Git detectó problemas de formato. Corrígelos antes de publicar."
    if git diff --cached --name-only | grep -Eiq '(^|/)(\.env(\.|$)|id_rsa|id_ed25519|credentials\.json|.*\.pem$|.*token.*|.*secret.*|.*blackboard.*|.*acuse.*)'; then
      fail "Se detectó un nombre de archivo potencialmente sensible. Revísalo; no se publicó nada."
    fi
    printf '\nArchivos preparados:\n'
    git diff --cached --stat
    printf '\nCommit: %s\n' "$msg"
    read -r -p "¿Confirmas commit y push a $RAMa? [s/N] " ans
    [[ "$ans" =~ ^[sS]$ ]] || { printf 'Publicación cancelada; archivos siguen preparados en staging.\n'; exit 0; }
    git commit -m "$msg"
    git push origin "$RAMa"
    printf '\nPUBLICACIÓN COMPLETADA. Commit: %s\n' "$(git rev-parse --short HEAD)"
    ;;
  revisar-a2)
    ensure_repo
    printf '=== INVENTARIO A2 EN EL REPOSITORIO ===\n'
    for file in \
      'docs/A2_estado_vigente_20261109.md' \
      'docs/evidencias_A2/A17_cierre_20261109/acta_cierre.md' \
      'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis' \
      'matlab/A2_MAMDANI/A2_F1_tabla_27_reglas.csv' \
      'resultados/A2_R18_equivalencia/evidencia_publicada/A2_R18_comparacion_10_casos.csv' \
      'docs/evidencias_A2/A17_cierre_20261109/tabla_B1_fuente.csv' \
      'docs/evidencias_A2/A17_cierre_20261109/tabla_B2_fuente.csv' \
      'docs/evidencias_A2/A17_cierre_20261109/tabla_B3_fuente.csv' \
      'docs/evidencias_A2/A17_cierre_20261109/tabla_B4_fuente.csv'; do
      if [[ -f "$file" ]]; then printf 'OK      %s\n' "$file"; else printf 'FALTA   %s\n' "$file"; fi
    done
    printf '\n=== REPORTE Y DEMOSTRACIÓN EN docs/entrega_a2/ ===\n'
    if [[ -d docs/entrega_a2 ]]; then
      find docs/entrega_a2 -type f -maxdepth 2 -print
      find docs/entrega_a2 -type f -iname '*.pdf' -print0 | while IFS= read -r -d '' pdf; do
        if command -v pdfinfo >/dev/null 2>&1; then
          printf 'Páginas: '; pdfinfo "$pdf" | awk '/^Pages:/ {print $2}'
        fi
      done
    else
      printf 'No existe docs/entrega_a2/. Revisa si los originales están fuera del repositorio.\n'
    fi
    printf '\n=== TAG REMOTO AVANCE 2 ===\n'
    git ls-remote --tags origin refs/tags/avance2 || true
    printf '\nLos acuses de Blackboard deben conservarse en privado, FUERA del Git público.\n'
    ;;
  estado)
    ensure_repo
    printf 'Repositorio: %s\n' "$REPO"
    printf 'Rama: %s\n' "$(git branch --show-current)"
    git status -sb
    git log -1 --format='%h %s'
    ;;
  ayuda|-h|--help)
    printf 'USO:\n  bash %s instalar\n  amr iniciar\n  amr guardar "mensaje de commit"\n  amr revisar-a2\n  amr estado\n' "$(basename "$0")"
    printf '\nLa publicación siempre exige confirmación. No modifica main ni integracion-final.\n'
    ;;
  *) fail "Comando desconocido: $cmd. Usa 'amr ayuda'.";;
esac
