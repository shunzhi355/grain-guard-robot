#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd -P)"
CONTAINER="${GRAIN_NOETIC_CONTAINER:-codex-fastlio-noetic-20261008}"
IMAGE="${GRAIN_NOETIC_IMAGE:-grain-nav:noetic-x86}"
CONTAINER_PROJECT_ROOT="${GRAIN_PROJECT_ROOT_CONTAINER:-$PROJECT_ROOT}"

exec env GRAIN_NOETIC_CONTAINER="$CONTAINER" GRAIN_NOETIC_IMAGE="$IMAGE" \
  "$SCRIPT_DIR/start_noetic_slam.sh" --container "$CONTAINER" -- \
  env GRAIN_LOCAL_RUNTIME=1 \
      GRAIN_LOCAL_CONFIG="$CONTAINER_PROJECT_ROOT/LENOVO/config/local_robot.operator-live.json" \
      GRAIN_ROBOT_SOCKET="${GRAIN_ROBOT_SOCKET:-/run/grain-robot/control.sock}" \
      PYTHONPATH="$CONTAINER_PROJECT_ROOT/3588/src:$CONTAINER_PROJECT_ROOT/3588:$CONTAINER_PROJECT_ROOT/LENOVO/src" \
      python3 "$CONTAINER_PROJECT_ROOT/3588/scripts/start_ui.py"
