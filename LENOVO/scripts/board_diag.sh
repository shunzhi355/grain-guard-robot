#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../deploy/mapping/runtime.sh
source "$PROJECT_ROOT/deploy/mapping/runtime.sh"
mapping_load_config || exit 1
mapping_source_ros1 || true

pass() { echo "[PASS] $*"; }
fail() { echo "[FAIL] $*"; }
blocked() { echo "[BLOCKED] $*"; }

echo "=== Mapping board diagnostics (read-only) ==="
echo "RADAR_NETDEV=$RADAR_NETDEV"
echo "RADAR_HOST_IP=$RADAR_HOST_IP"
echo "MID360_IP=$MID360_IP"
echo "LIVOX_WS=$LIVOX_WS"
echo "SFAST_WS=$SFAST_WS"

bash "$SCRIPT_DIR/check_mapping_env.sh" --runtime || true

if command -v rostopic >/dev/null 2>&1 && timeout 3 rostopic list >/dev/null 2>&1; then
    pass "ROS master reachable"
    for topic in "$LIVOX_LIDAR_TOPIC" "$LIVOX_IMU_TOPIC" /Odometry /cloud_registered /Laser_map; do
        if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
            sample="$(timeout 4 rostopic hz "$topic" 2>/dev/null | grep 'average rate' | tail -n 1 || true)"
            [ -n "$sample" ] && pass "$topic: $sample" || blocked "$topic exists but no frequency sample was captured"
        else
            fail "$topic not present"
        fi
    done
else
    blocked "ROS master is not reachable; topic checks skipped"
fi

ps -ef | grep -E 'roscore|rosmaster|roslaunch|livox|sfast|fastlio' | grep -v grep || true
echo "=== Diagnostics complete; no configuration was changed ==="
