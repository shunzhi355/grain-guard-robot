#!/bin/bash
# =============================================================================
# env_check.sh — Environment validation script for Grain Sampling Robot
# Platform:  RK3588 / Ubuntu 22.04 / aarch64 / Debian ROS1 1.15.x
# Usage:
#   chmod +x env_check.sh && ./env_check.sh
#   (some checks require sudo for hardware access)
# =============================================================================

set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# ── Counters & Results ──────────────────────────────────────────────────────
PASS=0
FAIL=0
WARN=0
RESULTS=()  # Array of "status|message"

record() {
    local status="$1"   # PASS / FAIL / WARN
    local msg="$2"
    RESULTS+=("$status|$msg")
    case "$status" in
        PASS) ((PASS++));;
        FAIL) ((FAIL++));;
        WARN) ((WARN++));;
    esac
}

# ── Banner ──────────────────────────────────────────────────────────────────
echo -e "${CYAN}===============================================${NC}"
echo -e "${CYAN}  Grain Sampling Robot — Environment Check${NC}"
echo -e "${CYAN}===============================================${NC}"
echo ""
echo -e "  Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "  Hostname:  $(hostname)"
echo -e "  Kernel:    $(uname -r)"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# 1. OS VERSION
# ═══════════════════════════════════════════════════════════════════════════
echo -e "${CYAN}── [1/5] Operating System ────────────────────────${NC}"

if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    echo "  Detected: $PRETTY_NAME"
    if [[ "$VERSION_ID" == "22.04" ]]; then
        record PASS "Ubuntu 22.04 — target platform"
    else
        record WARN "Ubuntu $VERSION_ID detected (22.04 target)"
    fi
else
    record WARN "Cannot determine OS version (/etc/os-release missing)"
fi

# Architecture check (must be aarch64 for RK3588)
ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" ]]; then
    record PASS "Architecture: aarch64 (RK3588)"
else
    record WARN "Architecture: $ARCH (expected aarch64 for RK3588)"
fi

# ═══════════════════════════════════════════════════════════════════════════
# 2. ROS
# ═══════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}── [2/5] Debian ROS1 ──────────────────────────${NC}"

if command -v roscore &>/dev/null && command -v roslaunch &>/dev/null; then
    record PASS "ROS1 commands available"
else
    record FAIL "ROS1 commands unavailable — see deploy/mapping/DEPENDENCIES.md"
fi

# Debian ROS1 may be directly available without /opt/ros/<distro>.
if compgen -G "/opt/ros/*/setup.bash" >/dev/null 2>&1; then
    record PASS "ROS setup.bash discovered under /opt/ros"
else
    record WARN "No /opt/ros setup.bash; direct Debian ROS1 commands may still be valid"
fi

# ═══════════════════════════════════════════════════════════════════════════
# 3. PYTHON & PACKAGES
# ═══════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}── [3/5] Python Environment ───────────────────────${NC}"

if command -v python3 &>/dev/null; then
    PY_VER=$(python3 --version 2>&1)
    echo "  $PY_VER"
    MAJ=$(python3 -c "import sys; print(sys.version_info.major)")
    MIN=$(python3 -c "import sys; print(sys.version_info.minor)")
    if [[ "$MAJ" -ge 3 && "$MIN" -ge 10 ]]; then
        record PASS "Python $MAJ.$MIN+ (>=3.10)"
    else
        record FAIL "Python $MAJ.$MIN (>=3.10 required)"
    fi
else
    record FAIL "python3 not found in PATH"
fi

# Check pip
if command -v pip3 &>/dev/null; then
    record PASS "pip3 available"
else
    record FAIL "pip3 not found"
fi

# Required Python packages
declare -A PKG_MIN_VER=(
    ["PySide6"]="6.5.0"
    ["paho-mqtt"]="1.6.1"
    ["opencv-python"]="4.8.0"
    ["numpy"]="1.24.0"
    ["rospy"]="1.16.0"
    ["pyyaml"]="6.0"
)

for pkg in "${!PKG_MIN_VER[@]}"; do
    min_ver="${PKG_MIN_VER[$pkg]}"
    import_name="$pkg"
    # Handle special import names
    [[ "$pkg" == "opencv-python" ]] && import_name="cv2"
    [[ "$pkg" == "pyyaml" ]] && import_name="yaml"

    if python3 -c "import $import_name" 2>/dev/null; then
        record PASS "Package $pkg installed"
    else
        record FAIL "Package $pkg missing (pip install $pkg>=$min_ver)"
    fi
done

# ═══════════════════════════════════════════════════════════════════════════
# 4. HARDWARE
# ═══════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}── [4/5] Hardware ──────────────────────────────────${NC}"

# PWM (servo/motor control)
if ls /sys/class/pwm/pwmchip* &>/dev/null 2>&1; then
    record PASS "PWM controller(s) detected"
else
    record WARN "No PWM chips found in /sys/class/pwm/ — verify hardware overlay"
fi

# LiDAR via USB (Livox Mid-360)
if lsusb 2>/dev/null | grep -qi "livox\|3d.*lidar\|vl53\|sick\|hesai\|robosense"; then
    record PASS "LiDAR detected via USB"
else
    # Livox Mid-360 typically uses Ethernet, not USB — this is expected
    record WARN "No LiDAR found via lsusb (Livox Mid-360 uses Ethernet — check eth port)"
fi

# Ethernet → LiDAR connectivity
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/deploy/mapping/runtime.sh"
mapping_load_config || true
if ping -I "${RADAR_NETDEV:-eth1}" -c 1 -W 1 "${MID360_IP:-192.168.1.116}" &>/dev/null; then
    record PASS "MID360 (${MID360_IP:-192.168.1.116}) reachable on ${RADAR_NETDEV:-eth1}"
else
    record WARN "MID360 not responding — check mapping network configuration"
fi

# USB camera
VIDEO_DEV=$(ls /dev/video* 2>/dev/null || true)
if [[ -n "$VIDEO_DEV" ]]; then
    DEV_COUNT=$(echo "$VIDEO_DEV" | wc -l)
    record PASS "Camera devices: $DEV_COUNT found ($(echo "$VIDEO_DEV" | tr '\n' ' '))"
else
    record FAIL "No /dev/video* devices found — camera required"
fi

# GPU / NPU (RK3588 specific)
if ls /dev/dri/card* &>/dev/null 2>&1; then
    record PASS "Graphics device(s) found (/dev/dri/card*)"
else
    record WARN "No DRM device found — GPU may not be enabled"
fi

# NPU (Rockchip RK3588)
if [[ -d /sys/class/npu ]]; then
    record PASS "NPU device found"
else
    record WARN "NPU not detected — expected on RK3588"
fi

# ═══════════════════════════════════════════════════════════════════════════
# 5. NETWORK
# ═══════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}── [5/5] Network ──────────────────────────────────${NC}"

# WiFi
WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1)
if [[ -n "$WIFI_IFACE" ]]; then
    record PASS "WiFi interface: $WIFI_IFACE"
else
    record WARN "No WiFi interface detected (optional if using 5G-only)"
fi

# 5G / cellular (typically wwan0 or usb0)
CELL_IFACE=""
for iface in wwan0 usb0 eth1; do
    if ip link show "$iface" &>/dev/null 2>&1; then
        CELL_IFACE="$iface"
        break
    fi
done
if [[ -n "$CELL_IFACE" ]]; then
    record PASS "5G/cellular interface: $CELL_IFACE"
else
    record WARN "No 5G/cellular interface detected (expected wwan0/usb0/eth1)"
fi

# Default route (internet connectivity)
if ip route show default &>/dev/null; then
    record PASS "Default route configured"
else
    record FAIL "No default route — check network config"
fi

# DNS resolution
if host github.com &>/dev/null 2>&1; then
    record PASS "DNS resolution working (github.com reachable)"
else
    record WARN "DNS resolution failed — may be offline"
fi

# ═══════════════════════════════════════════════════════════════════════════
# SUMMARY
# ═══════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════${NC}"
echo -e "  ${CYAN}ENVIRONMENT CHECK SUMMARY${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════${NC}"
echo ""

for entry in "${RESULTS[@]}"; do
    IFS='|' read -r status msg <<< "$entry"
    case "$status" in
        PASS) echo -e "  ${GREEN}[PASS]${NC}  $msg" ;;
        FAIL) echo -e "  ${RED}[FAIL]${NC}  $msg" ;;
        WARN) echo -e "  ${YELLOW}[WARN]${NC}  $msg" ;;
    esac
done

echo ""
echo -e "${CYAN}────────────────────────────────────────────────────${NC}"
echo -e "  Total:  ${GREEN}$PASS passed${NC}  ${YELLOW}$WARN warnings${NC}  ${RED}$FAIL failed${NC}"
echo -e "${CYAN}────────────────────────────────────────────────────${NC}"
echo ""

if [[ $FAIL -eq 0 ]]; then
    echo -e "${GREEN}✓ All critical checks passed — environment is ready.${NC}"
    exit 0
else
    echo -e "${RED}✗ $FAIL critical check(s) failed. Resolve items above before deployment.${NC}"
    exit 1
fi
