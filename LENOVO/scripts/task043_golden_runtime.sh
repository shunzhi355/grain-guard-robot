#!/usr/bin/env bash
# Task043 container entrypoint. Only ROS/Livox/Golden, never a motor endpoint.
set -Eeuo pipefail
OUT=/task043/out
export ROS_DISTRO=noetic ROS_VERSION=1 ROS_MASTER_URI=http://127.0.0.1:11311
export RADAR_NETDEV=eno1 MID360_IP=192.168.1.116 RADAR_HOST_IP=192.168.1.200 ROS_ROUTE_TARGET=1.1.1.1 RADAR_PING_REQUIRED=0
source "$OUT/live_ros_network_preflight.sh"
export ROS_IP
export ROS_HOME="$OUT/ros_home" ROS_LOG_DIR="$OUT/ros_log" RC030_TRACE_DIR="$OUT/rc030_trace"
source /opt/ros/noetic/setup.bash
source /work/livox_ws/devel/setup.bash
source /task037/golden_runtime/ws/devel/setup.bash
export CMAKE_PREFIX_PATH=/task037/golden_runtime/ws/devel:/work/livox_ws/devel:/opt/ros/noetic
export ROS_PACKAGE_PATH=/task037/golden_runtime/ws/src:/work/livox_ws/src:/opt/ros/noetic/share
export LD_LIBRARY_PATH=/task037/deps:/task037/ros_overlay/lib:/task037/golden_runtime/ws/devel/lib:/work/livox_ws/devel/lib:/opt/ros/noetic/lib
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" "$RC030_TRACE_DIR"
# Never create a second master or receiver in a host-network container.
if timeout 3 rosparam list >/dev/null 2>&1; then
  echo 'existing ROS Master ownership conflict; refusing a duplicate runtime' >&2; exit 1
fi
python3 - "$OUT/MID360_config.task043.json" <<'PY'
import json,subprocess,sys
c=json.load(open(sys.argv[1]))['MID360']['host_net_info']
ports={int(v) for k,v in c.items() if k.endswith('_port')}
for line in subprocess.check_output(['ss','-H','-lun'],text=True).splitlines():
    endpoint=line.split()[3]
    try: port=int(endpoint.rsplit(':',1)[1])
    except ValueError: continue
    if port in ports: raise SystemExit('MID360 UDP port occupied: '+endpoint)
PY
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  set +e
  # Guard first invalidates readiness and stops its own Golden child.
  for ((i=${#PIDS[@]}-1;i>=0;i--)); do
    kill -TERM "${PIDS[i]}" 2>/dev/null
    wait "${PIDS[i]}" 2>/dev/null
  done
}
trap cleanup EXIT
trap 'exit 0' INT TERM
roscore >"$OUT/roscore.log" 2>&1 & PIDS+=("$!")
master_ready=0
for i in $(seq 1 80); do
  if rosparam list >/dev/null 2>&1; then master_ready=1; break; fi
  sleep .25
done
[[ "$master_ready" == 1 ]] || { echo 'ROS Master timeout' >&2; exit 1; }
rosparam load "$OUT/driver_params.yaml"
/work/livox_ws/devel/lib/livox_ros_driver2/livox_ros_driver2_node 100000000000000 __name:=livox_lidar_publisher2 >"$OUT/livox.log" 2>&1 & PIDS+=("$!")
# Replaces blind sleep3 + Golden start. This guard stays BLOCKED after failure,
# preventing systemd/Docker from repeatedly restarting a divergent estimator.
python3 -u "$OUT/task043_localization_startup.py" --config "$OUT/localization_startup_config.json" --out "$OUT" >"$OUT/localization_startup.log" 2>&1 & PIDS+=("$!")
wait "${PIDS[2]}"
