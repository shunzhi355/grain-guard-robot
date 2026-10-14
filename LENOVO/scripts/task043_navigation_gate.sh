#!/usr/bin/env bash
# One repeatable entry point for task043 startup, reconnect and route recovery.
# It only restores task-owned runtime components and never selects a goal.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd -P)"
STAGING_ROOT="${TASK043_STAGING_ROOT:-$PROJECT_DIR}"
RUNTIME_CONTAINER="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
ADAPTER_CONTAINER="${TASK043_ADAPTER_CONTAINER:-task043-live-nav-adapter-current}"
SOCKET_DIR="${TASK043_SOCKET_DIR:-$HOME/.local/state/grain-lenovo}"
LOG_DIR="${TASK043_LOG_DIR:-$SOCKET_DIR/task043-supervisor}"
IMAGE="${TASK043_ROS_IMAGE:-grain-nav:noetic-x86}"
ATTEMPTS="${TASK043_GATE_ATTEMPTS:-3}"
MODE="check"

usage() {
  cat <<'EOF'
usage: task043_navigation_gate.sh [--check|--recover|--start|--reconnect]

  --check       validate the current route and task-owned runtime only
  --recover     repair the MID360 route and start stopped task-owned services
  --start       recover, then start one ROS adapter if it is not running
  --reconnect   recover, replace the task-owned ROS adapter, then start it
EOF
}

while (($#)); do
  case "$1" in
    --check|--recover|--start|--reconnect) MODE="${1#--}"; shift ;;
    --attempts) ATTEMPTS="${2:?--attempts requires a number}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$ATTEMPTS" =~ ^[1-9][0-9]*$ ]] || { echo "attempts must be a positive integer" >&2; exit 2; }
command -v docker >/dev/null 2>&1 || { echo "docker unavailable" >&2; exit 1; }
command -v ip >/dev/null 2>&1 || { echo "iproute2 unavailable" >&2; exit 1; }
[[ -x "$STAGING_ROOT/LENOVO/scripts/ensure_mapping_network.sh" ]] || {
  echo "missing route recovery script: $STAGING_ROOT/LENOVO/scripts/ensure_mapping_network.sh" >&2
  exit 1
}
[[ -x "$STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh" ]] || {
  echo "missing adapter launcher: $STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh" >&2
  exit 1
}

mkdir -p "$LOG_DIR" "$SOCKET_DIR"
exec 9>"$SOCKET_DIR/task043-navigation-gate.lock"
flock -n 9 || { echo "another task043 gate operation is active" >&2; exit 1; }

log() { printf '[task043-gate] %s\n' "$*"; }
state_of() { docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || true; }

safe_runtime_name() {
  [[ "$1" =~ ^task043-new-map-relocalization-[A-Za-z0-9._-]+$ ||
      "$1" =~ ^codex-fastlio-noetic-[A-Za-z0-9._-]+$ ]]
}

ensure_route() {
  local result
  if [[ "$MODE" == check ]]; then
    # Read-only route check. Recovery modes may assign the configured host
    # address and exact MID360 /32 route through the existing helper.
    if result="$(bash -lc 'set -e; source "$1/LENOVO/deploy/mapping/runtime.sh"; mapping_load_config; mapping_route_is_exact' _ "$STAGING_ROOT" 2>&1)"; then
      log "MID360 route is ready (read-only check)"
      return 0
    fi
    log "MID360 route is not ready: $result"
    return 1
  fi
  if "$STAGING_ROOT/LENOVO/scripts/ensure_mapping_network.sh"; then
    log "MID360 route is ready"
    return 0
  fi
  log "MID360 route check/repair failed"
  return 1
}

ensure_runtime() {
  safe_runtime_name "$RUNTIME_CONTAINER" || {
    log "refusing non-task runtime name: $RUNTIME_CONTAINER"
    return 1
  }
  local state image
  state="$(state_of "$RUNTIME_CONTAINER")"
  if [[ "$state" == running ]]; then
    image="$(docker inspect -f '{{.Config.Image}}' "$RUNTIME_CONTAINER")"
    [[ "$image" == "$IMAGE" ]] || { log "runtime image mismatch: $image"; return 1; }
    log "ROS runtime already running: $RUNTIME_CONTAINER"
    return 0
  fi
  [[ "$MODE" != check ]] || { log "ROS runtime is not running: $RUNTIME_CONTAINER ($state)"; return 1; }
  [[ "$state" == created || "$state" == exited || "$state" == paused ]] || {
    log "ROS runtime is unavailable: $RUNTIME_CONTAINER ($state)"
    return 1
  }
  image="$(docker inspect -f '{{.Config.Image}}' "$RUNTIME_CONTAINER")"
  [[ "$image" == "$IMAGE" ]] || { log "runtime image mismatch: $image"; return 1; }
  docker start "$RUNTIME_CONTAINER" >/dev/null
  log "started task-owned ROS runtime: $RUNTIME_CONTAINER"
}

ensure_local_service() {
  if [[ -S "$SOCKET_DIR/control.sock" ]]; then
    log "local hardware service socket is online"
    return 0
  fi
  [[ "$MODE" != check ]] || { log "local hardware service/socket unavailable"; return 1; }
  log "starting task-owned local hardware service"
  setsid nohup bash -lc "cd '$STAGING_ROOT' && exec .venv/bin/python LENOVO/scripts/local_robot.py serve --config LENOVO/config/local_robot.local-live.json --runtime-dir '$SOCKET_DIR' --live --attended --external-suction-confirmed" \
    >>"$LOG_DIR/local_robot.log" 2>&1 < /dev/null &
  for _ in $(seq 1 30); do
    [[ -S "$SOCKET_DIR/control.sock" ]] && { log "local hardware service socket is ready"; return 0; }
    sleep 1
  done
  log "local hardware service did not create control.sock"
  return 1
}
adapter_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "$ADAPTER_CONTAINER"
}

replace_adapter_if_requested() {
  [[ "$MODE" == reconnect ]] || return 0
  if adapter_running; then
    local label
    label="$(docker inspect -f '{{index .Config.Labels "io.grain.task"}}' "$ADAPTER_CONTAINER" 2>/dev/null || true)"
    [[ "$label" == task043-live-navigation ]] || { log "refusing to replace non-task adapter"; return 1; }
    log "stopping task-owned ROS adapter for reconnect"
    docker stop -t 5 "$ADAPTER_CONTAINER" >/dev/null || docker kill "$ADAPTER_CONTAINER" >/dev/null
  fi
  return 0
}

clear_reconnect_latch() {
  [[ "$MODE" == reconnect ]] || return 0
  [[ -S "$SOCKET_DIR/control.sock" ]] || { log "cannot clear reconnect latch: control.sock missing"; return 1; }
  python3 - "$SOCKET_DIR/control.sock" <<''PY''
import json, socket, sys
path = sys.argv[1]
def request(action, **values):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(5)
        conn.connect(path)
        conn.sendall((json.dumps({"action": action, **values}) + "\n").encode())
        raw = conn.makefile("rb").readline()
    if not raw:
        raise SystemExit("empty local service response")
    reply = json.loads(raw)
    if not reply.get("ok"):
        raise SystemExit(reply.get("error", "local service rejected request"))
    return reply
status = request("status")
chassis = status.get("chassis") or {}
if chassis.get("rc_mode") != "auto" or chassis.get("faults") != 0 or chassis.get("motion_armed"):
    raise SystemExit("reconnect latch reset requires idle auto RC and zero faults")
if status.get("safety_latched") or chassis.get("estop_latched"):
    request("clear_estop", mechanical_reset_confirmed=True)
print("reconnect latch clear check passed")
PY
}
start_adapter() {
  local launcher="$STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh"
  if adapter_running; then
    log "ROS navigation adapter already running: $ADAPTER_CONTAINER"
    return 0
  fi
  [[ "$MODE" != check ]] || { log "ROS navigation adapter is not running"; return 1; }
  setsid nohup bash "$launcher" \
    --container "$RUNTIME_CONTAINER" --name "$ADAPTER_CONTAINER" \
    >>"$LOG_DIR/live-adapter.log" 2>&1 < /dev/null &
  for _ in $(seq 1 30); do
    adapter_running && { log "ROS navigation adapter is running"; return 0; }
    sleep 1
  done
  log "ROS navigation adapter did not become ready"
  return 1
}

runtime_localization_pending() {
  local startup_status
  startup_status="$(docker exec "$RUNTIME_CONTAINER" cat /task043/out/localization_startup_status.json 2>/dev/null || true)"
  [[ -n "$startup_status" ]] || return 1
  python3 -c 'import json,sys; s=json.load(sys.stdin); print("LOCALIZATION_STARTUP="+str(s.get("state"))+" reason="+str(s.get("reason"))); sys.exit(0 if s.get("state") not in ("VALIDATING_STATIC","READY") else 1)' \
    <<<"$startup_status"
}

run_once() {
  ensure_route || return 1
  ensure_runtime || return 1
  ensure_local_service || return 1
  replace_adapter_if_requested || return 1
  clear_reconnect_latch || return 1
  if runtime_localization_pending; then
    log "localization is waiting/blocked; UI is available, navigation is blocked"
    [[ "$MODE" != check ]] || return 1
    return 0
  fi
  # The launcher performs the normalized map, ROS master, publisher, frame,
  # and fresh Odometry gates. It never creates a goal.
  if [[ "$MODE" == check ]]; then
    bash "$STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh" \
      --container "$RUNTIME_CONTAINER" --name "$ADAPTER_CONTAINER" --check
    return 0
  fi
  [[ "$MODE" == recover ]] && return 0
  if ! bash "$STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh" \
    --container "$RUNTIME_CONTAINER" --name "$ADAPTER_CONTAINER" --check; then
    log "localization is not ready; UI may open to display faults, navigation remains blocked"
    docker exec "$RUNTIME_CONTAINER" cat /task043/out/localization_startup_status.json 2>/dev/null || true
    return 0
  fi
  start_adapter
}

report_navigation_admission() {
  # Services may start to display faults. Startup success is not motion admission.
  local python="$STAGING_ROOT/.venv/bin/python"
  PYTHONPATH="$STAGING_ROOT/3588/src:$STAGING_ROOT/3588:$STAGING_ROOT/LENOVO/src" \
    "$python" - "$SOCKET_DIR/control.sock" <<'PY'
import sys, time
from grain_sampling_workflow.robot_bridge import RobotClient
try:
    status = RobotClient(sys.argv[1]).request("status")
except (OSError, RuntimeError, ValueError) as exc:
    print("NAVIGATION_ADMISSION=BLOCKED reason=service_status_unavailable " + str(exc))
    raise SystemExit(1)
pose, chassis = status.get("pose") or {}, status.get("chassis") or {}
reasons = []
if pose.get("localization_valid") is not True:
    reasons.append("localization_invalid")
try:
    if not 0 <= time.monotonic() - pose["odom_at"] <= .3:
        reasons.append("pose_stale")
except (KeyError, TypeError):
    reasons.append("pose_missing")
if chassis.get("rc_mode") != "auto" or chassis.get("rc_valid") is not True:
    reasons.append("RC_not_fresh_auto")
if status.get("safety_latched") or chassis.get("estop_latched"):
    reasons.append("reset_required")
if chassis.get("faults") != 0 or chassis.get("chassis_link") != "online":
    reasons.append("hardware_not_ready")
if reasons:
    print("NAVIGATION_ADMISSION=BLOCKED reasons=" + ",".join(reasons))
    raise SystemExit(1)
print("NAVIGATION_ADMISSION=READY explicit_new_goal_required=yes")
PY
}

for attempt in $(seq 1 "$ATTEMPTS"); do
  log "gate attempt $attempt/$ATTEMPTS mode=$MODE"
  if run_once; then
    log "TASK043_STARTUP=PASS mode=$MODE (services only; no motion authorization)"
    if report_navigation_admission; then
      log "TASK043_GATE=PASS mode=$MODE"
      exit 0
    fi
    [[ "$MODE" != check ]] || exit 1
    log "TASK043_GATE=STARTUP_ONLY navigation blocked; UI may show fault status"
    exit 0
  fi
  [[ "$MODE" != check && "$attempt" -lt "$ATTEMPTS" ]] || break
  log "transient startup failure; rechecking route and task-owned runtime"
  sleep 2
done

echo "TASK043_GATE=BLOCKED mode=$MODE; see $LOG_DIR" >&2
exit 1
