#!/bin/bash
# ================================================================
# Board Diagnostic Script — P0 issues: Livox config + Odometry
# Run on RK3588 board: bash board_diag.sh
# ================================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
pass() { echo -e "${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
info() { echo -e "       $*"; }

echo "============================================"
echo " Grain Sampling Robot — Board Diagnostics"
echo " $(date)"
echo "============================================"

# ── 1. Radar connectivity ────────────────────────────────────
echo; echo "── 1. Livox Mid-360 Connectivity ──"

RADAR_IP="192.168.1.116"
if ping -c 1 -W 2 $RADAR_IP >/dev/null 2>&1; then
    pass "Radar reachable at $RADAR_IP"
else
    fail "Radar NOT reachable at $RADAR_IP"
    info "Check: ethernet cable, enP3p49s0 IP (should be 192.168.1.200)"
fi

# ── 2. Livox driver config ───────────────────────────────────
echo; echo "── 2. Livox Driver Configuration ──"

# Find Livox driver config (multiple possible locations)
CONFIG_FILES=$(find /home/orangepi -name "livox_lidar_config.json" -o \
    -name "MID360_config.json" -o -name "livox_config.json" 2>/dev/null)

if [ -z "$CONFIG_FILES" ]; then
    fail "No Livox config JSON found under /home/orangepi"
else
    for cf in $CONFIG_FILES; do
        info "Found: $cf"
        echo "   ─────"
        head -20 "$cf" 2>/dev/null
        echo "   ─────"
        # Check critical IP fields
        CFG_IP=$(grep -oP '"ip_address"\s*:\s*"[^"]*"' "$cf" 2>/dev/null | head -1)
        info "  lidar ip_address: $CFG_IP"
        CFG_HOST=$(grep -oP '"host_ip"\s*:\s*"[^"]*"' "$cf" 2>/dev/null | head -1)
        info "  host_ip: $CFG_HOST"
        CFG_TYPE=$(grep -oP '"lidar_type"\s*:\s*\d+' "$cf" 2>/dev/null | head -1)
        info "  lidar_type: $CFG_TYPE (expect 9 for Mid-360)"
    done
fi

# FastLIO config
MID360_YAML="/home/orangepi/fastlio_ws/src/FAST_LIO/config/mid360.yaml"
if [ -f "$MID360_YAML" ]; then
    info "FastLIO yaml: $MID360_YAML"
    grep -E "lid_topic|imu_topic|odom_frame|map_frame" "$MID360_YAML" 2>/dev/null
else
    warn "FastLIO mid360.yaml not found at $MID360_YAML"
fi

# ── 3. ROS status ────────────────────────────────────────────
echo; echo "── 3. ROS Core Status ──"

source /opt/ros/noetic/setup.bash

if pgrep roscore >/dev/null 2>&1; then
    pass "roscore running"
else
    fail "roscore NOT running"
    info "Start with: roscore &"
fi

# ── 4. Key ROS topics ────────────────────────────────────────
echo; echo "── 4. Key ROS Topics ──"

check_topic() {
    local topic=$1
    local desc=$2
    if rostopic list 2>/dev/null | grep -qF "$topic"; then
        local hz=$(timeout 3 rostopic hz "$topic" 2>/dev/null | tail -1)
        if [ -n "$hz" ]; then
            pass "$topic ($desc) — $hz"
        else
            warn "$topic ($desc) — found but no data (hz check failed)"
        fi
    else
        fail "$topic ($desc) — NOT FOUND"
    fi
}

check_topic "/livox/lidar" "Livox point cloud"
check_topic "/Odometry" "FastLIO odometry"
check_topic "/cloud_registered" "FastLIO registered cloud"
check_topic "/Laser_map" "FastLIO map"
check_topic "/waypoint_task_done" "Nav completion signal"
check_topic "/move_base_simple/goal" "Nav goal topic"

# ── 5. Odometry frame_id ─────────────────────────────────────
echo; echo "── 5. Odometry Frame Check ──"

if rostopic list 2>/dev/null | grep -qF "/Odometry"; then
    FRAME=$(timeout 2 rostopic echo /Odometry -n 1 2>/dev/null | grep "frame_id" | head -1)
    info "Latest /Odometry frame_id: $FRAME"
    if echo "$FRAME" | grep -q "camera_init"; then
        pass "Frame matches expected 'camera_init'"
    else
        warn "Frame is NOT 'camera_init' — navigation goals may be rejected"
    fi
else
    fail "/Odometry not available — check FastLIO"
fi

# ── 6. Process check ─────────────────────────────────────────
echo; echo "── 6. Running Processes ──"

check_proc() {
    if pgrep -f "$1" >/dev/null 2>&1; then
        pass "$2 running (PID: $(pgrep -f "$1" | head -1))"
    else
        warn "$2 NOT running"
    fi
}

check_proc "motor_driver" "motor_driver"
check_proc "goal_controller" "goal_controller"
check_proc "fastlio_mapping\|FAST_LIO\|laserMapping" "FastLIO SLAM"
check_proc "livox_ros_driver\|livox_lidar" "Livox driver"
check_proc "mjpeg_server" "MJPEG camera"

# ── 7. Network interface ─────────────────────────────────────
echo; echo "── 7. Radar Network Interface ──"
if ip addr show enP3p49s0 2>/dev/null | grep -q "192.168.1.200"; then
    pass "enP3p49s0 has radar host IP 192.168.1.200"
else
    warn "enP3p49s0 IP check:"
    ip addr show enP3p49s0 2>/dev/null | grep "inet " || echo "       interface not found"
fi

echo; echo "============================================"
echo " Diagnostics complete."
echo "============================================"
