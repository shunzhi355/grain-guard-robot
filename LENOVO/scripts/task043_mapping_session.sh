#!/usr/bin/env bash
# Isolated Task043 mapping session. Reuses the current ROS Master/Livox only.
set -Eeuo pipefail
BASE="${GRAIN_MAPPING_BASE:-$HOME/.cache/grain-diagnostics/GSR-NAV-UI-SOFTWARE-INTEGRATION-043/maps}"
RUNTIME="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
CONTAINER="${TASK043_MAPPING_CONTAINER:-task043-mapping-session}"
IMAGE="${TASK043_ROS_IMAGE:-grain-nav:noetic-x86}"
POINTER="$BASE/active_mapping.json"
state() { docker inspect -f '{{.State.Status}}' "$1" 2>/dev/null || true; }
source_for() {
  docker inspect -f '{{json .Mounts}}' "$RUNTIME" |
    python3 -c 'import json,sys; print(next(x["Source"] for x in json.load(sys.stdin) if x["Destination"]==sys.argv[1]))' "$1"
}
active_root() { [[ -f "$POINTER" ]] && python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map_root"])' "$POINTER"; }
stop_task043_localization() {
  if docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode list 2>/dev/null | grep -Eq "/task043_newmap/laserMapping([_0-9]|$)"'; then
    docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode kill /task043_newmap/laserMapping' >/dev/null || return 1
    for _ in $(seq 1 20); do
      docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; ! rosnode list 2>/dev/null | grep -Eq "/task043_newmap/laserMapping([_0-9]|$)"' && return 0
      sleep .5
    done
    echo 'task043 Golden did not stop; refusing mapping overlap' >&2
    return 1
  fi
}
require_runtime() {
  [[ "$(state "$RUNTIME")" == running ]] || { echo "task043 ROS runtime is not running" >&2; return 1; }
  [[ "$(docker inspect -f '{{.Config.Image}}' "$RUNTIME")" == "$IMAGE" ]] || { echo "runtime image mismatch" >&2; return 1; }
  docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; rosparam list >/dev/null; rostopic info /livox/lidar >/dev/null; rostopic info /livox/imu >/dev/null; test "$(rostopic info /livox/lidar | grep -c /livox_lidar_publisher2)" = 1'
}
write_params() {
  cat >"$1/mapping.yaml" <<'YAML'
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  time_sync_en: false
  time_offset_lidar_to_imu: 0.0
preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.5
  timestamp_unit: 3
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  fov_degree: 360
  det_range: 100.0
  extrinsic_est_en: false
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1,0,0,0,1,0,0,0,1]
cube_side_length: 1000.0
publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: true
  scan_bodyframe_pub_en: false
pcd_save:
  pcd_save_en: true
  interval: 100
YAML
}
case "${1:-}" in
start)
  stop_task043_localization
  require_runtime
  [[ -z "$(state "$CONTAINER")" ]] || { echo "mapping session already exists: $CONTAINER" >&2; exit 1; }
  raw_name="${2:-new_map}"
  slug="$(python3 - "$raw_name" <<'PY'
import re,sys
s=re.sub(r'[^A-Za-z0-9_-]+','_',sys.argv[1].strip()).strip('_-')
if not s: raise SystemExit('enter a valid map name')
print(s[:48])
PY
)"
  session="${slug}_$(date +%Y%m%d_%H%M%S)"
  MAP_ROOT="$BASE/$session"; PCD="$MAP_ROOT/PCD"
  [[ ! -e "$MAP_ROOT" ]] || { echo "map session directory already exists: $MAP_ROOT" >&2; exit 1; }
  mkdir -p "$PCD" "$(dirname "$POINTER")"; write_params "$MAP_ROOT"
  python3 - "$POINTER" "$MAP_ROOT" <<'PY'
import json,os,sys,tempfile
path,root=sys.argv[1:]; fd,tmp=tempfile.mkstemp(dir=os.path.dirname(path),prefix='.active_mapping.')
with os.fdopen(fd,'w') as f: json.dump({'map_root':root,'state':'mapping_start','origin':'mapping_start','frame':'camera_init'},f); f.write('\n')
os.replace(tmp,path)
PY
  deps="$(source_for /task037/deps)"; ws="$(source_for /task037/golden_runtime/ws)"
  overlay="$(source_for /task037/ros_overlay)"; livox="$(source_for /work/livox_ws)"
  [[ -n "$deps" && -n "$ws" && -n "$overlay" && -n "$livox" ]] || { echo 'runtime mounts incomplete' >&2; exit 1; }
  # The Golden binary emits RC030 trace files on its first scan. The mapping
  # session must prove this writable path before accepting the UI start.
  test -d "$MAP_ROOT" -a -w "$MAP_ROOT" || { echo "mapping root is not writable: $MAP_ROOT" >&2; exit 1; }
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  docker run -d --name "$CONTAINER" --label io.grain.task=task043-mapping \
    --network host --mount "type=bind,source=$deps,target=/task037/deps,readonly" \
    --mount "type=bind,source=$ws,target=/task037/golden_runtime/ws,readonly" \
    --mount "type=bind,source=$overlay,target=/task037/ros_overlay,readonly" \
    --mount "type=bind,source=$livox,target=/work/livox_ws,readonly" \
    --mount "type=bind,source=$MAP_ROOT,target=/task043/map" \
    --mount "type=bind,source=$PCD,target=/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD" \
    "$IMAGE" bash -lc '
      set -Eeuo pipefail
      export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=192.168.1.200
      export ROS_HOME=/task043/map/ros_home RC030_TRACE_DIR=/task043/map/rc030_trace
      mkdir -p "$ROS_HOME" "$RC030_TRACE_DIR" /workspace/rootcause027_evidence/B
      test -w "$ROS_HOME" && test -w "$RC030_TRACE_DIR" && test -w /workspace/rootcause027_evidence/B
      source /opt/ros/noetic/setup.bash; source /work/livox_ws/devel/setup.bash; source /task037/golden_runtime/ws/devel/setup.bash
      export CMAKE_PREFIX_PATH=/task037/golden_runtime/ws/devel:/work/livox_ws/devel:/opt/ros/noetic
      export ROS_PACKAGE_PATH=/task037/golden_runtime/ws/src:/work/livox_ws/src:/opt/ros/noetic/share
      export LD_LIBRARY_PATH=/task037/deps:/task037/ros_overlay/lib:/task037/golden_runtime/ws/devel/lib:/work/livox_ws/devel/lib:/opt/ros/noetic/lib
      rosparam load /task043/map/mapping.yaml
      exec /task037/golden_runtime/ws/devel/lib/sfast_lio/sfastlio_mapping __name:=task043_mapping \
        /Odometry:=/task043_mapping/Odometry /path:=/task043_mapping/path \
        /cloud_registered:=/task043_mapping/cloud_registered \
        /cloud_registered_body:=/task043_mapping/cloud_registered_body \
        /cloud_effected:=/task043_mapping/cloud_effected /Laser_map:=/task043_mapping/Laser_map
    ' >"$MAP_ROOT/mapping.log" 2>&1
  for _ in $(seq 1 30); do
    [[ "$(state "$CONTAINER")" == running ]] || { docker logs "$CONTAINER" 2>&1 | tee -a "$MAP_ROOT/mapping.log" || true; echo "mapping process exited during startup; container retained as $CONTAINER" >&2; exit 1; }
    if docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /task043_mapping >/dev/null 2>&1' ; then break; fi
    sleep .5
  done
  docker exec "$RUNTIME" bash -lc 'source /opt/ros/noetic/setup.bash; rosnode ping -c 1 /task043_mapping >/dev/null 2>&1' || { echo "mapping node did not register; inspect $MAP_ROOT/mapping.log and docker logs $CONTAINER" >&2; exit 1; }
  (nohup docker logs -f --tail all "$CONTAINER" >"$MAP_ROOT/mapping.log" 2>&1 < /dev/null &)
  echo "mapping started: $CONTAINER map_root=$MAP_ROOT"
  ;;
stop)
  [[ "$(state "$CONTAINER")" == running ]] || { echo 'mapping session is not running' >&2; exit 1; }
  docker stop -t 15 "$CONTAINER" >/dev/null
  echo 'mapping stopped without map save'
  ;;
save)
  [[ "$(state "$CONTAINER")" == running ]] || { echo 'mapping session is not running' >&2; exit 1; }
  MAP_ROOT="$(active_root)"; [[ -n "$MAP_ROOT" ]] || { echo 'active mapping root missing' >&2; exit 1; }
  PCD="$MAP_ROOT/PCD"
  docker kill --signal=INT "$CONTAINER" >/dev/null
  for _ in $(seq 1 120); do
    session_state="$(state "$CONTAINER")"
    [[ "$session_state" == exited || -z "$session_state" ]] && break
    sleep .5
  done
  [[ "$(state "$CONTAINER")" != running ]] || { echo 'mapping did not finish saving within 60 seconds' >&2; exit 1; }
  [[ -s "$PCD/GlobalMap.pcd" && -s "$PCD/GlobalMap_ikdtree.pcd" ]] || { docker logs "$CONTAINER" 2>&1 | tee -a "$MAP_ROOT/mapping.log" || true; echo 'mapping stopped without both map files' >&2; exit 1; }
  docker rm "$CONTAINER" >/dev/null 2>&1 || true
  python3 - "$POINTER" "$MAP_ROOT" <<'PY'
import json,os,sys,tempfile
path,root=sys.argv[1:]; value={'map_root':root,'state':'saved','origin':'mapping_start','frame':'camera_init','initial_pose':[0,0,0,0,0,0,1]}
fd,tmp=tempfile.mkstemp(dir=os.path.dirname(path),prefix='.active_mapping.')
with os.fdopen(fd,'w') as f: json.dump(value,f,indent=2); f.write('\n')
os.replace(tmp,path)
PY
  echo "map saved: $PCD"
  ;;
status)
  root="$(active_root || true)"
  if [[ -n "$root" && -d "$root/PCD" ]]; then
    printf 'MAP_ROOT=%s\n' "$root"
    [[ "$(state "$CONTAINER")" == running ]] && echo 'MAPPING=RUNNING' || echo 'MAPPING=STOPPED'
    [[ -s "$root/PCD/GlobalMap.pcd" ]] && echo 'MAP_PCD=READY' || echo 'MAP_PCD=WAITING'
  else echo 'MAPPING=IDLE'; fi
  ;;
*) echo "usage: $0 start [NAME]|stop|save|status" >&2; exit 2 ;;
esac
