#!/usr/bin/env bash
# Start the single ROS1 -> local control.sock adapter after validating the live stack.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd -P)"
CONFIG_FILE="$PROJECT_DIR/LENOVO/config/local_robot.local-live.json"
IMAGE="${GRAIN_NOETIC_IMAGE:-grain-nav:noetic-x86}"
CONTAINER="${GRAIN_NOETIC_CONTAINER:-}"
CONTAINER_NAME="${GRAIN_NAV_ADAPTER_CONTAINER:-task043-live-nav-adapter-current}"
STATE_DIR="${GRAIN_ROBOT_STATE_DIR:-$HOME/.local/state/grain-lenovo}"
RADAR_IP="${MID360_IP:-192.168.1.116}"
CHECK_ONLY=0

while (($#)); do
  case "$1" in
    --config) CONFIG_FILE="${2:?--config requires a path}"; shift 2 ;;
    --container) CONTAINER="${2:?--container requires a name}"; shift 2 ;;
    --name) CONTAINER_NAME="${2:?--name requires a value}"; shift 2 ;;
    --check) CHECK_ONLY=1; shift ;;
    -h|--help) echo "usage: $0 [--check] [--config PATH] [--container NAME] [--name NAME]"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

[[ -f "$CONFIG_FILE" ]] || { echo "missing config file: $CONFIG_FILE" >&2; exit 1; }
command -v docker >/dev/null 2>&1 || { echo "docker unavailable" >&2; exit 1; }
command -v ip >/dev/null 2>&1 || { echo "iproute2 unavailable" >&2; exit 1; }

mapfile -t candidates < <(docker ps --filter "ancestor=$IMAGE" --format '{{.Names}}' \
  | grep -E '^(task043-new-map-relocalization-|codex-fastlio-noetic-)' || true)
if [[ -z "$CONTAINER" ]]; then
  if ((${#candidates[@]} != 1)); then
    printf 'expected exactly one running task-owned Noetic container, found %s\n' "${#candidates[@]}" >&2
    printf 'candidates: %s\n' "${candidates[*]:-none}" >&2
    exit 1
  fi
  CONTAINER="${candidates[0]}"
elif [[ ! " ${candidates[*]} " == *" $CONTAINER "* ]]; then
  echo "requested container is not a running task-owned Noetic runtime: $CONTAINER" >&2
  exit 1
fi

state="$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || true)"
[[ "$state" == running ]] || { echo "Noetic runtime is not running: $CONTAINER ($state)" >&2; exit 1; }
image="$(docker inspect -f '{{.Config.Image}}' "$CONTAINER")"
[[ "$image" == "$IMAGE" ]] || { echo "container image mismatch: $image (expected $IMAGE)" >&2; exit 1; }

PYTHON="${PYTHON:-$PROJECT_DIR/.venv/bin/python}"
[[ -x "$PYTHON" ]] || PYTHON="${PYTHON3:-python3}"
mapfile -t profile < <(PYTHONPATH="$PROJECT_DIR/3588/src:$PROJECT_DIR/3588:$PROJECT_DIR/LENOVO/src" \
  "$PYTHON" - "$CONFIG_FILE" <<'PY'
import json, re, sys
cfg = json.load(open(sys.argv[1], encoding="utf-8"))
if cfg.get("navigation_mode") != "local":
    raise SystemExit("local navigation adapter requires navigation_mode=local")
if cfg.get("localization_health_mode") != "odom_stable":
    raise SystemExit("local navigation adapter requires localization_health_mode=odom_stable")
if cfg.get("require_pointcloud_for_navigation") is not False:
    raise SystemExit("warehouse profile must explicitly allow Odometry-only navigation health")
for key in ("odom_topic", "map_frame", "map_id"):
    if not isinstance(cfg.get(key), str) or not cfg[key]:
        raise SystemExit(f"missing live navigation identity: {key}")
if not re.fullmatch(r"/[A-Za-z0-9_/]+", cfg["odom_topic"]):
    raise SystemExit("invalid Odometry topic")
print(cfg["odom_topic"])
print(cfg["map_frame"])
print(cfg["map_id"])
print(cfg.get("runtime_dir") or "/run/grain-lenovo")
PY
)
if ((${#profile[@]} != 4)); then echo "invalid local navigation profile" >&2; exit 1; fi
ODOM_TOPIC="${profile[0]}"
MAP_FRAME="${profile[1]}"
MAP_ID="${profile[2]}"
CONTAINER_STATE_DIR="${profile[3]}"
[[ "$CONTAINER_STATE_DIR" == /* ]] || { echo "runtime_dir must be absolute inside the container" >&2; exit 1; }

route="$(ip route get "$RADAR_IP" 2>/dev/null | head -n 1)"
route_source="$(awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}' <<<"$route")"
route_device="$(awk '{for(i=1;i<=NF;i++) if($i=="dev") {print $(i+1); exit}}' <<<"$route")"
[[ -n "$route_source" && -n "$route_device" ]] || { echo "no route to MID360 $RADAR_IP" >&2; exit 1; }
ip -o -4 addr show | awk -v ip="$route_source" '{split($4,a,"/"); if(a[1]==ip) found=1} END{exit !found}' \
  || { echo "MID360 route selected a non-local source address: $route_source" >&2; exit 1; }
echo "MID360 route: $RADAR_IP via $route_device src $route_source"
[[ -S "$STATE_DIR/control.sock" ]] || { echo "local hardware service socket missing: $STATE_DIR/control.sock" >&2; exit 1; }

PYTHONPATH="$PROJECT_DIR/3588/src:$PROJECT_DIR/3588:$PROJECT_DIR/LENOVO/src" \
  "$PYTHON" - "$STATE_DIR/control.sock" "$MAP_ID" "$MAP_FRAME" <<'PY'
import sys
from grain_sampling_workflow.robot_bridge import RobotClient
status = RobotClient(sys.argv[1]).request("status")
if status.get("runtime") != "lenovo-local":
    raise SystemExit("control.sock is not served by the Lenovo local hardware service")
if status.get("test_navigation_mode") is True:
    raise SystemExit("hardware service is still in operator mode; adapter was not started")
if status.get("safety_latched") is not False:
    print("hardware safety latch active: restoring read-only pose inputs; motion remains blocked")
chassis = status.get("chassis") or {}
if chassis.get("motion_armed") is not False:
    raise SystemExit("hardware service reports motion already armed or unknown")
slam = status.get("slam") or {}
if slam.get("map_id") != sys.argv[2]:
    raise SystemExit("hardware service map identity/frame does not match the live adapter profile")
if slam.get("frame") not in (None, sys.argv[3]):
    raise SystemExit("hardware service map frame differs from the live adapter profile")
print("hardware service identity/mode/map preflight passed; Odometry frame is checked separately")
PY

ROS_MASTER_URI_VALUE="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
ROS_IP_OVERRIDE="${ROS_IP:-}"
ROS_IP_VALUE=""
for attempt in $(seq 1 30); do
  info="$(docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/noetic/setup.bash; ROS_MASTER_URI="$1" rosnode list >/dev/null && ROS_MASTER_URI="$1" rostopic info "$2"' \
    _ "$ROS_MASTER_URI_VALUE" "$ODOM_TOPIC" 2>/dev/null || true)"
  hosts="$(sed -nE 's#.*http://([0-9.]+):[0-9]+/.*#\1#p' <<<"$info" | sort -u)"
  host_count="$(grep -c . <<<"$hosts" || true)"
  if [[ "$host_count" == 1 ]]; then
    ROS_IP_VALUE="${ROS_IP_OVERRIDE:-$hosts}"
    ip -o -4 addr show | awk -v ip="$ROS_IP_VALUE" '{split($4,a,"/"); if(a[1]==ip) found=1} END{exit !found}' \
      || { echo "ROS_IP $ROS_IP_VALUE is not local; active Odometry publisher advertises $hosts" >&2; exit 1; }
    break
  fi
  sleep 2
done
[[ -n "$ROS_IP_VALUE" ]] || { echo "ROS master/Odometry publisher not ready after 60 seconds: $ODOM_TOPIC" >&2; exit 1; }

lidar_info="$(docker exec "$CONTAINER" bash -lc \
  'source /opt/ros/noetic/setup.bash; ROS_MASTER_URI="$1" rostopic info /livox/lidar' \
  _ "$ROS_MASTER_URI_VALUE" 2>/dev/null || true)"
grep -q 'livox_lidar_publisher' <<<"$lidar_info" || { echo "MID360 driver is not publishing /livox/lidar" >&2; exit 1; }
grep -q 'laserMapping' <<<"$lidar_info" || { echo "Golden is not subscribed to /livox/lidar" >&2; exit 1; }

if ! docker exec -i "$CONTAINER" bash -s -- "$ROS_MASTER_URI_VALUE" "$ODOM_TOPIC" "$MAP_FRAME" <<'REMOTE_CHECK'
set -e
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="$1"
python3 - "$2" "$3" <<'PY'
import math, sys
import rospy
from nav_msgs.msg import Odometry
topic, frame = sys.argv[1:]
rospy.init_node("task043_navigation_preflight", anonymous=True, disable_signals=True)
msg = rospy.wait_for_message(topic, Odometry, timeout=6.0)
age = (rospy.Time.now() - msg.header.stamp).to_sec()
q = msg.pose.pose.orientation
norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
p = msg.pose.pose.position
if not 0 <= age <= 0.3:
    raise SystemExit(f"stale Odometry age={age:.3f}s")
if msg.header.frame_id.lstrip("/") != frame.lstrip("/"):
    raise SystemExit(f"frame mismatch: {msg.header.frame_id!r} expected {frame!r}")
if not all(math.isfinite(v) for v in (p.x, p.y, p.z, norm)) or not 0.9 <= norm <= 1.1:
    raise SystemExit("Odometry position/quaternion invalid")
print(f"fresh Odometry: frame={msg.header.frame_id} x={p.x:.3f} y={p.y:.3f} age={age:.3f}s")
PY
REMOTE_CHECK
then
  echo "live Odometry preflight failed" >&2
  exit 1
fi

echo "ROS graph: master=$ROS_MASTER_URI_VALUE ROS_IP=$ROS_IP_VALUE odom=$ODOM_TOPIC map=$MAP_ID/$MAP_FRAME"
if (( CHECK_ONLY )); then echo "preflight passed; no adapter or motion request started"; exit 0; fi

mkdir -p "$STATE_DIR"
exec 9>"$STATE_DIR/local-navigation-adapter.lock"
flock -n 9 || { echo "another local navigation adapter launcher is active" >&2; exit 1; }
if docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "adapter container already running: $CONTAINER_NAME" >&2; exit 1
fi
if docker inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  owned="$(docker inspect -f '{{index .Config.Labels "io.grain.task"}}' "$CONTAINER_NAME")"
  state="$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME")"
  if [[ "$owned" != task043-live-navigation || "$state" != exited ]]; then
    echo "adapter name occupied by a container not safe to replace: $CONTAINER_NAME ($owned/$state)" >&2; exit 1
  fi
  docker rm "$CONTAINER_NAME" >/dev/null
fi
registered="$(docker exec "$CONTAINER" bash -lc \
  'source /opt/ros/noetic/setup.bash; rosnode list' 2>/dev/null \
  | grep '^/grain_lenovo_local_navigation' || true)"
while IFS= read -r node; do
  [[ -n "$node" ]] || continue
  ping_output="$(docker exec "$CONTAINER" bash -lc \
      'source /opt/ros/noetic/setup.bash; timeout 4 rosnode ping -c 1 "$1"' _ "$node" \
      2>&1 || true)"
  if ! grep -qE 'ERROR:|connection refused|Communication with node' <<<"$ping_output"; then
    echo "a local navigation adapter is already alive: $node" >&2
    exit 1
  fi
  echo "ignoring stale ROS registration for exited adapter: $node" >&2
done <<<"$registered"

# Directory bind avoids Docker's file-vs-directory mount failure on stale paths.
CONFIG_STAGE_DIR="$STATE_DIR/navigation-adapter-config"
mkdir -p "$CONFIG_STAGE_DIR"
temporary="$CONFIG_STAGE_DIR/local_robot.json.tmp.$$"
install -m 600 "$CONFIG_FILE" "$temporary"
mv -f "$temporary" "$CONFIG_STAGE_DIR/local_robot.json"

exec docker run --rm --name "$CONTAINER_NAME" \
  --label io.grain.task=task043-live-navigation --network host \
  -e ROS_MASTER_URI="$ROS_MASTER_URI_VALUE" -e ROS_IP="$ROS_IP_VALUE" \
  -v "$PROJECT_DIR:/opt/lenovo:ro" \
  -v "$STATE_DIR:$CONTAINER_STATE_DIR" \
  --mount "type=bind,source=$CONFIG_STAGE_DIR,target=/run/task043-config,readonly" \
  "$IMAGE" bash -lc \
  'source /opt/ros/noetic/setup.bash; exec python3 /opt/lenovo/LENOVO/scripts/local_navigation_ros1.py --config /run/task043-config/local_robot.json --allow-point-navigation'
