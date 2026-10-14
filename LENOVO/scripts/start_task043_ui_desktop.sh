#!/usr/bin/env bash
set -Eeuo pipefail
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/task043-ui-desktop.lock"
flock -n 9 || exit 0
ROOT="/home/ps02/grain_robot/staging/lenovo-local-20261010"
DESKTOP_FILE="$HOME/桌面/task043-navigation.desktop"
# GNOME requires executable permission on application desktop files.
if [[ -f "$DESKTOP_FILE" ]]; then
  chmod 755 "$DESKTOP_FILE" 2>/dev/null || true
  gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null || true
fi
export GRAIN_NOETIC_CONTAINER="${GRAIN_NOETIC_CONTAINER:-task043-new-map-relocalization-20261012}"
export TASK043_RUNTIME_CONTAINER="$GRAIN_NOETIC_CONTAINER"
export GRAIN_MAPPING_BASE="${GRAIN_MAPPING_BASE:-$HOME/.cache/grain-diagnostics/GSR-NAV-UI-SOFTWARE-INTEGRATION-043/maps}"
mkdir -p "$GRAIN_MAPPING_BASE"
exec bash "$ROOT/LENOVO/scripts/start_mapping_ui.sh" --task043-gate start
