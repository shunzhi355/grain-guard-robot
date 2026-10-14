#!/bin/bash
# ================================================================
# Grain Sampling Robot — one-click launcher
# Starts: roscore → livox_radar → sfast_lio → chassis → mock → camera → UI
# Usage:  bash start.sh
# ================================================================

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../deploy/mapping/runtime.sh
source "$PROJECT_ROOT/deploy/mapping/runtime.sh"
mapping_load_config
mapping_source_ros1

bash "$SCRIPT_DIR/check_mapping_env.sh" --preflight
bash "$SCRIPT_DIR/ensure_mapping_network.sh"
bash "$SCRIPT_DIR/check_mapping_env.sh" --runtime
mapping_source_workspace "$LIVOX_WS"
mapping_source_workspace "$SFAST_WS"

cd "$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT/src:${PYTHONPATH:-}"

# PID tracking directory — used for clean shutdown
PID_DIR="/tmp/grain_sampling_pids"
mkdir -p "$PID_DIR"

# Cleanup handler — kill all tracked processes on script exit
cleanup() {
    echo "[start] Shutting down..."
    for pidfile in "$PID_DIR"/*.pid; do
        [ -f "$pidfile" ] && kill "$(cat "$pidfile")" 2>/dev/null || true
    done
}
trap cleanup EXIT

# 1. Start roscore if not running
if ! pgrep roscore > /dev/null; then
    echo "[start] Launching roscore..."
    roscore & sleep 3
fi
echo "[start] roscore ready"

# 2. Start Livox Mid-360 radar driver (bypass sudo, network already configured)
echo "[start] Launching Livox radar driver..."
if pgrep -f livox_ros_driver2 > /dev/null 2>&1; then
    pkill -f livox_ros_driver2 2>/dev/null || true
    sleep 1
fi
setsid nohup roslaunch "$LIVOX_PACKAGE" "$LIVOX_LAUNCH" > /tmp/livox.log 2>&1 &
echo $! > "$PID_DIR/livox.pid"
disown
echo "[start] Waiting for Livox lidar and IMU data..."
mapping_wait_for_topic "$LIVOX_LIDAR_TOPIC"
mapping_wait_for_topic "$LIVOX_IMU_TOPIC"

# 3. Start S-FAST_LIO relocalization (uses sfast_lio package in fastlio2_ws)
echo "[start] Launching S-FAST_LIO relocalization..."
if pgrep -f fastlio_mapping > /dev/null 2>&1; then
    pkill -f fastlio_mapping 2>/dev/null || true
    sleep 1
fi
setsid nohup roslaunch "$SFAST_PACKAGE" "$SFAST_RELOCALIZATION_LAUNCH" > /tmp/sfastlio.log 2>&1 &
echo $! > "$PID_DIR/sfastlio.pid"
disown
echo "[start] Waiting for /Odometry..."
mapping_wait_for_topic "/Odometry" 20

# 4. Start chassis motor driver (needs sudo for PWM)
echo "[start] Launching motor_driver..."
if pgrep -f motor_driver > /dev/null 2>&1; then
    pkill -f motor_driver 2>/dev/null || true
    sleep 1
fi
echo orangepi | sudo -S setsid nohup python3 /home/orangepi/dipan/motor_driver.py daemon > /tmp/motor_driver.log 2>&1 &
echo $! > "$PID_DIR/motor_driver.pid"
disown
sleep 2
echo "[start] motor_driver running (UDP 127.0.0.1:8765)"

# 5. Start goal_controller (chassis navigation)
echo "[start] Launching goal_controller..."
if pgrep -f goal_controller > /dev/null 2>&1; then
    pkill -f goal_controller 2>/dev/null || true
    sleep 1
fi
setsid nohup python3 /home/orangepi/dipan/goal_controller.py __name:=goal_controller > /tmp/gc.log 2>&1 &
echo $! > "$PID_DIR/goal_controller.pid"
disown

# Stop-on-obstacle safety
if pgrep -f stop_on_obstacle > /dev/null 2>&1; then
    pkill -f stop_on_obstacle 2>/dev/null || true
    sleep 1
fi
setsid nohup python3 -m grain_sampling_workflow.stop_on_obstacle > /tmp/stop_on_obstacle.log 2>&1 &
echo $! > "$PID_DIR/stop_on_obstacle.pid"
disown
sleep 1
echo "[start] goal_controller running"

# 6. Start mock_robot (fake sensor data — for UI dev, disable on real hardware)
echo "[start] Launching mock_robot..."
if pgrep -f mock_robot > /dev/null 2>&1; then
    pkill -f mock_robot 2>/dev/null || true
    sleep 1
fi
setsid nohup python3 -c "
import rospy, sys
sys.path.insert(0, '.')
from mock_robot.mock_robot_node import MockRobot
m = MockRobot()
rospy.spin()
" > /tmp/mock_robot.log 2>&1 &
echo $! > "$PID_DIR/mock_robot.pid"
disown
sleep 2
echo "[start] mock_robot running"

# 7. Start RTMP streamer (push /dev/video0 → cloud relay)
echo "[start] Launching RTMP streamer..."
if pgrep -f rtmp_streamer > /dev/null 2>&1; then
    pkill -f rtmp_streamer 2>/dev/null || true
    sleep 1
fi
setsid nohup python3 src/grain_sampling_camera/rtmp_streamer.py > /tmp/rtmp_streamer.log 2>&1 &
echo $! > "$PID_DIR/rtmp_streamer.pid"
disown
sleep 2
echo "[start] RTMP streamer → rtmp://124.220.41.27:41935/live/1"

# 8. Start UI on HDMI display
export DISPLAY=:0
echo "[start] Launching UI..."
setsid nohup python3 src/grain_sampling_ui/main.py > /tmp/ui.log 2>&1 &
echo $! > "$PID_DIR/ui.pid"
disown
