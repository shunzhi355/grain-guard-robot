#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd -P)"
CONTAINER="${GRAIN_NOETIC_CONTAINER:-}"
IMAGE="${GRAIN_NOETIC_IMAGE:-grain-nav:noetic-x86}"
CONTAINER_PROJECT_ROOT="${GRAIN_PROJECT_ROOT_CONTAINER:-/opt/lenovo}"
TASK043_GATE_MODE=""
if [[ "${1:-}" == "--task043-gate" ]]; then
  TASK043_GATE_MODE="${2:?--task043-gate requires check, recover, start or reconnect}"
  shift 2
fi
if [[ -n "$TASK043_GATE_MODE" ]]; then
  case "$TASK043_GATE_MODE" in
    check|recover|start|reconnect) ;;
    *) echo "invalid --task043-gate mode: $TASK043_GATE_MODE" >&2; exit 2 ;;
  esac
  # Qt is installed on the industrial PC. The Golden container owns ROS only.
  # Reuse the local UI entry point and the same state directory as the gate.
  UI_PYTHON="${GRAIN_UI_PYTHON:-${PYTHON:-$PROJECT_ROOT/.venv-ui/bin/python}}"
  [[ -x "$UI_PYTHON" ]] || UI_PYTHON="$PROJECT_ROOT/.venv/bin/python"
  CONFIG_FILE="${GRAIN_LOCAL_CONFIG:-$PROJECT_ROOT/LENOVO/config/local_robot.local-live.json}"
  export GRAIN_MAPPING_BASE="${GRAIN_MAPPING_BASE:-$HOME/.cache/grain-diagnostics/GSR-NAV-UI-SOFTWARE-INTEGRATION-043/maps}"
  export TASK043_RUNTIME_CONTAINER="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
  export GRAIN_MAP_PREVIEW_PYTHON="${GRAIN_MAP_PREVIEW_PYTHON:-$PROJECT_ROOT/.venv/bin/python}"
  STATE_DIR="${TASK043_SOCKET_DIR:-$HOME/.local/state/grain-lenovo}"
  [[ -x "$UI_PYTHON" && -f "$CONFIG_FILE" && -f "$SCRIPT_DIR/local_robot.py" ]] || {
    echo "UI deployment incomplete: Python, local_robot.py or live configuration missing" >&2; exit 1;
  }
  "$UI_PYTHON" -c 'import PySide6' || { echo "host Qt environment unavailable" >&2; exit 1; }
  # The supervisor keeps retrying in the background, so the console does not
  # need the gate's full retry budget: a blocked gate must reach the operator
  # quickly instead of leaving a dead-looking desktop icon for 90 seconds.
  export TASK043_GATE_ATTEMPTS="${TASK043_GATE_ATTEMPTS:-1}"
  export TASK043_SERVICE_WAIT_SEC="${TASK043_SERVICE_WAIT_SEC:-10}"
  if ! bash "$SCRIPT_DIR/task043_navigation_gate.sh" "--$TASK043_GATE_MODE"; then
    # ``--check`` is an explicit diagnostic request and must fail loudly.
    if [[ "$TASK043_GATE_MODE" == check ]]; then
      echo "task043 gate check failed" >&2
      exit 1
    fi
    # The desktop shortcut runs with Terminal=false, so aborting here made the
    # icon look dead whenever the hardware gate was blocked.  Open the operator
    # console anyway so the hardware fault is visible and actionable; the UI
    # itself cannot command motion while the local service is unavailable.
    echo "[task043-ui] startup gate blocked (mode=$TASK043_GATE_MODE); opening the UI in fault-display mode" >&2
  fi
  # Use the existing desktop session; do not install packages or alter X access.
  if [[ -z "${QT_QPA_PLATFORM:-}" && -n "${XDG_RUNTIME_DIR:-}" ]]; then
    wayland_socket="${WAYLAND_DISPLAY:-wayland-0}"
    if [[ -S "$XDG_RUNTIME_DIR/$wayland_socket" ]]; then
      export WAYLAND_DISPLAY="$wayland_socket" QT_QPA_PLATFORM=wayland
    fi
  fi
  exec env -u PYTHONPATH "$UI_PYTHON" "$SCRIPT_DIR/local_robot.py" ui \
    --config "$CONFIG_FILE" --runtime-dir "$STATE_DIR"
fi

if [[ -n "$CONTAINER" ]]; then
  RUNNER=(bash "$SCRIPT_DIR/start_noetic_slam.sh" --container "$CONTAINER")
else
  RUNNER=(bash "$SCRIPT_DIR/start_noetic_slam.sh")
fi
exec env GRAIN_NOETIC_IMAGE="$IMAGE" "${RUNNER[@]}" -- \
  env GRAIN_LOCAL_RUNTIME=1 \
      GRAIN_LOCAL_CONFIG="$CONTAINER_PROJECT_ROOT/LENOVO/config/local_robot.operator-live.json" \
      GRAIN_ROBOT_SOCKET="${GRAIN_ROBOT_SOCKET:-/run/grain-robot/control.sock}" \
      PYTHONPATH="$CONTAINER_PROJECT_ROOT/3588/src:$CONTAINER_PROJECT_ROOT/3588:$CONTAINER_PROJECT_ROOT/LENOVO/src" \
      python3 "$CONTAINER_PROJECT_ROOT/3588/scripts/start_ui.py"
