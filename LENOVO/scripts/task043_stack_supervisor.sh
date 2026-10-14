#!/usr/bin/env bash
# Restartable task043 lifecycle.  This supervisor only restores the task-owned
# ROS runtime, local daemon and read-only ROS adapter; it never creates a goal
# or sends a motion command.  A flock makes systemd and @reboot cron harmless
# if both are enabled on an existing installation.
set -Eeuo pipefail

STAGING_ROOT="${TASK043_STAGING_ROOT:-/home/ps02/grain_robot/staging/lenovo-local-20261010}"
RUNTIME_CONTAINER="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
ADAPTER_CONTAINER="${TASK043_ADAPTER_CONTAINER:-task043-live-nav-adapter-current}"
SOCKET_DIR="${TASK043_SOCKET_DIR:-/home/ps02/.local/state/grain-lenovo}"
LOG_DIR="${TASK043_LOG_DIR:-/home/ps02/.local/state/grain-lenovo/task043-supervisor}"
IMAGE="${TASK043_ROS_IMAGE:-grain-nav:noetic-x86}"
INTERVAL="${TASK043_SUPERVISOR_INTERVAL_SEC:-5}"

mkdir -p "$LOG_DIR" "$SOCKET_DIR"
exec 9>"$SOCKET_DIR/task043-supervisor.lock"
flock -n 9 || exit 0

log() { printf '[task043-supervisor] %s\n' "$*"; }
state_of() { docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || true; }

ensure_runtime() {
  # At cold boot Docker may not be ready yet. Wait without starting dependents.
  docker info --format '{{.ServerVersion}}' >/dev/null 2>&1 || {
    log 'Docker is not ready; retrying next cycle'; return 1;
  }
  [[ "$RUNTIME_CONTAINER" =~ ^task043-new-map-relocalization-[A-Za-z0-9._-]+$ ||
     "$RUNTIME_CONTAINER" =~ ^codex-fastlio-noetic-[A-Za-z0-9._-]+$ ]] || {
    log "runtime ownership unknown: $RUNTIME_CONTAINER"; return 1;
  }
  local state image
  state="$(state_of "$RUNTIME_CONTAINER")"
  image="$(docker inspect -f '{{.Config.Image}}' "$RUNTIME_CONTAINER" 2>/dev/null || true)"
  [[ "$image" == "$IMAGE" ]] || { log "runtime image mismatch/unavailable: $image"; return 1; }
  case "$state" in
    running) return 0 ;;
    created|exited)
      docker start "$RUNTIME_CONTAINER" >/dev/null || return 1
      log "started $RUNTIME_CONTAINER" ;;
    *) log "runtime unavailable: $RUNTIME_CONTAINER ($state)"; return 1 ;;
  esac
}

ensure_radar_route() {
  local route_script="$STAGING_ROOT/LENOVO/scripts/ensure_mapping_network.sh"
  [[ -x "$route_script" ]] || { log "route recovery script missing: $route_script"; return 1; }
  "$route_script" >>"$LOG_DIR/network.log" 2>&1 || {
    log 'MID360 route is not ready; runtime recovery is paused'
    return 1
  }
}

ensure_local_service() {
  pgrep -f '[L]ENOVO/scripts/local_robot.py serve' >/dev/null 2>&1 && return 0
  log 'starting local hardware service'
  setsid nohup bash -lc "cd '$STAGING_ROOT' && exec .venv/bin/python LENOVO/scripts/local_robot.py serve --config LENOVO/config/local_robot.local-live.json --runtime-dir '$SOCKET_DIR' --live --attended --external-suction-confirmed" \
    >>"$LOG_DIR/local_robot.log" 2>&1 < /dev/null &
}

adapter_ros_registered() {
  timeout 6 docker exec -i "$ADAPTER_CONTAINER" python3 - <<'PY' 2>/dev/null
import json, os, socket, xmlrpc.client
socket.setdefaulttimeout(2)
c=json.load(open('/run/task043-config/local_robot.json'))
m=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
code, _, graph=m.getSystemState('/task043_adapter_probe')
if code != 1: raise SystemExit(1)
pubs,subs,_=map(dict,graph)
nodes=pubs.get(c['localization_valid_topic'],[])
if len(nodes)!=1 or not nodes[0].startswith('/grain_lenovo_local_navigation_'):
    raise SystemExit(1)
node=nodes[0]
for topic in (c['odom_topic'], c.get('localization_startup_status_topic')):
    if topic and node not in subs.get(topic,[]): raise SystemExit(1)
code,_,uri=m.lookupNode('/task043_adapter_probe',node)
if code!=1: raise SystemExit(1)
code,_,pid=xmlrpc.client.ServerProxy(uri).getPid('/task043_adapter_probe')
raise SystemExit(0 if code==1 and pid>0 else 1)
PY
}

ensure_adapter() {
  # A blocked estimator is a stable diagnostic state, not a reason to keep
  # spawning 60 s Odometry preflights or restarting the task-owned runtime.
  local startup_status
  startup_status="$(docker exec "$RUNTIME_CONTAINER" cat /task043/out/localization_startup_status.json 2>/dev/null || true)"
  if [[ -n "$startup_status" ]]; then
    python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(0 if s.get("state") in ("VALIDATING_STATIC","READY") else 1)' \
      <<<"$startup_status" || return 1
  fi
  local state image owner
  state="$(state_of "$ADAPTER_CONTAINER")"
  if [[ -n "$state" ]]; then
    image="$(docker inspect -f '{{.Config.Image}}' "$ADAPTER_CONTAINER" 2>/dev/null || true)"
    owner="$(docker inspect -f '{{index .Config.Labels "io.grain.task"}}' "$ADAPTER_CONTAINER" 2>/dev/null || true)"
    [[ "$image" == "$IMAGE" && "$owner" == task043-live-navigation ]] || {
      log "adapter ownership/image mismatch: $ADAPTER_CONTAINER"; return 1;
    }
    if [[ "$state" == running ]]; then
      local adapter_started runtime_started
      adapter_started="$(docker inspect -f '{{.State.StartedAt}}' "$ADAPTER_CONTAINER")"
      runtime_started="$(docker inspect -f '{{.State.StartedAt}}' "$RUNTIME_CONTAINER")"
      [[ -n "$adapter_started" && -n "$runtime_started" ]] || return 1
      if [[ ! "$adapter_started" < "$runtime_started" ]] && adapter_ros_registered; then
        return 0
      fi
      # The task-owned adapter lost its ROS session. Stop/reconnect only it;
      # its shutdown and daemon watchdog revoke goals. Never clear a latch.
      log "adapter ROS session stale; reconnecting $ADAPTER_CONTAINER"
      docker stop -t 5 "$ADAPTER_CONTAINER" >/dev/null || return 1
      return 1  # launch through the normal validated path on the next cycle
    fi
  fi
  if pgrep -f "[s]tart_local_navigation_adapter.sh.*--name $ADAPTER_CONTAINER" >/dev/null 2>&1 \
      || pgrep -f "[d]ocker run .*--name $ADAPTER_CONTAINER([[:space:]]|$)" >/dev/null 2>&1; then
    return 0
  fi
  if [[ -n "$(state_of "$ADAPTER_CONTAINER")" ]]; then
    local image="$(docker inspect -f '{{.Config.Image}}' "$ADAPTER_CONTAINER" 2>/dev/null || true)"
    local state="$(state_of "$ADAPTER_CONTAINER")"
    if [[ "$state" == exited && "$image" == "$IMAGE" ]]; then
      docker rm "$ADAPTER_CONTAINER" >/dev/null
    else
      log "adapter unavailable: $ADAPTER_CONTAINER ($state/$image)"; return 1
    fi
  fi
  log "starting $ADAPTER_CONTAINER through validated launcher"
  setsid nohup bash "$STAGING_ROOT/LENOVO/scripts/start_local_navigation_adapter.sh" \
    --container "$RUNTIME_CONTAINER" --name "$ADAPTER_CONTAINER" \
    >>"$LOG_DIR/live-adapter.log" 2>&1 < /dev/null &
}

while :; do
  if ensure_radar_route && ensure_runtime && ensure_local_service; then
    if [[ -S "$SOCKET_DIR/control.sock" ]]; then ensure_adapter || true; fi
  fi
  sleep "$INTERVAL"
done
