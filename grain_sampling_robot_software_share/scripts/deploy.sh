#!/bin/bash
# =============================================================================
# deploy.sh — One-click deployment script for Grain Sampling Robot
# Platform:  RK3588 / Ubuntu 20.04 / ROS Noetic
# Usage:
#   chmod +x deploy.sh && sudo ./deploy.sh
#   (root or sudo required for system package installation)
# =============================================================================

set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ── Logging helpers ─────────────────────────────────────────────────────────
info()    { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()      { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()    { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

step()    { echo; echo -e "${CYAN}═══════════════════════════════════════════${NC}"; \
            echo -e "${CYAN}  Step $1: $2${NC}"; \
            echo -e "${CYAN}═══════════════════════════════════════════${NC}"; }

# ── Preliminary checks ─────────────────────────────────────────────────────
echo -e "${CYAN}===============================================${NC}"
echo -e "${CYAN}  Grain Sampling Robot — One-Click Deployment${NC}"
echo -e "${CYAN}===============================================${NC}"
echo

# Check root/sudo
if [[ $EUID -ne 0 ]]; then
    fail "This script requires root privileges. Run with: sudo ./deploy.sh"
fi
ok "Root privileges confirmed"

# Detect project root (script location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
info "Project root: $PROJECT_ROOT"

# ── Step 1: Install system dependencies ────────────────────────────────────
step 1 "Install system dependencies"

info "Updating apt package index..."
apt-get update -qq

info "Installing build tools and libraries..."
apt-get install -y -qq \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-venv \
    python3-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    wget \
    curl \
    net-tools \
    network-manager \
    usbutils \
    v4l-utils \
    ros-noetic-ros-base \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs \
    > /dev/null

ok "System dependencies installed"

# ── Step 2: Source ROS environment ────────────────────────────────────────
step 2 "Source ROS environment"

if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "/opt/ros/noetic/setup.bash"
    echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc
    ok "ROS Noetic sourced"
else
    fail "ROS Noetic not found at /opt/ros/noetic. Install ros-noetic-ros-base first."
fi

# ── Step 3: Install Python dependencies ────────────────────────────────────
step 3 "Install Python dependencies from requirements.txt"

if [[ ! -f "$PROJECT_ROOT/requirements.txt" ]]; then
    warn "requirements.txt not found — falling back to setup.py extras"
fi

pip3 install --upgrade pip --quiet
pip3 install -r "$PROJECT_ROOT/requirements.txt" --quiet
ok "Python dependencies installed"

# ── Step 4: Build & install Livox ROS driver ──────────────────────────────
step 4 "Build and install Livox ROS driver"

LIVOX_WS="/opt/livox_ros_driver"
if [[ -d "$LIVOX_WS" ]]; then
    info "Livox ROS driver already exists at $LIVOX_WS — skipping clone"
else
    info "Cloning livox_ros_driver..."
    git clone -b master https://github.com/Livox-SDK/livox_ros_driver.git "$LIVOX_WS" 2>/dev/null || {
        warn "Git clone failed; check network or proxy. Livox driver must be installed manually."
        warn "Skipping Livox driver build..."
    }
fi

if [[ -d "$LIVOX_WS/src" ]]; then
    cd "$LIVOX_WS"
    catkin build --quiet
    echo "source $LIVOX_WS/devel/setup.bash" >> /etc/bash.bashrc
    ok "Livox ROS driver built and installed"

    # Copy project-specific Livox config
    mkdir -p "$LIVOX_WS/src/livox_ros_driver/config"
    cp "$PROJECT_ROOT/config/livox_config.json" "$LIVOX_WS/src/livox_ros_driver/config/"
    ok "Livox configuration deployed"
fi

# ── Step 5: Install this project (editable) ────────────────────────────────
step 5 "Install grain-sampling-robot-software"

cd "$PROJECT_ROOT"
pip3 install -e . --quiet
ok "Project installed (editable mode)"

# Verify entry points
info "Verifying entry points..."
if python3 -c "import grain_sampling_ui" 2>/dev/null; then
    ok "  grain_sampling_ui module importable"
else
    warn "  grain_sampling_ui not importable — check PYTHONPATH"
fi
if python3 -c "import mock_robot" 2>/dev/null; then
    ok "  mock_robot module importable"
else
    warn "  mock_robot not importable — check PYTHONPATH"
fi

# ── Step 6: Verify installation ────────────────────────────────────────────
step 6 "Final verification"

PASS=0
FAIL=0

echo ""
echo -e "  ${CYAN}Checking runtime environment...${NC}"

# Python version
PY_VER=$(python3 --version 2>&1 | awk '{print $2}')
python3 -c "import sys; ver=sys.version_info; exit(0 if ver.major==3 and ver.minor>=10 else 1)" && \
    { ok "  Python $PY_VER (>=3.10)"; ((PASS++)); } || \
    { fail "  Python $PY_VER (<3.10 required)"; ((FAIL++)); }

# ROS
if ros --version 2>/dev/null; then
    ok "  ROS Noetic installed"
    ((PASS++))
else
    fail "  ROS not found"
    ((FAIL++))
fi

# Key Python packages
for pkg in PySide6 paho-mqtt rospy numpy cv2; do
    python3 -c "import $pkg" 2>/dev/null && \
        { ok "  Package $pkg OK"; ((PASS++)); } || \
        { warn "  Package $pkg missing"; ((FAIL++)); }
done

# Entry points
for ep in grain-sampling-ui mock-robot; do
    command -v "$ep" &>/dev/null && \
        { ok "  Entry point $ep available"; ((PASS++)); } || \
        { warn "  Entry point $ep not in PATH"; ((FAIL++)); }
done

echo ""
echo -e "${CYAN}═══════════════════════════════════════════${NC}"
echo -e "  Results:  ${GREEN}$PASS passed${NC}  ${RED}$FAIL failed${NC}"
echo -e "${CYAN}═══════════════════════════════════════════${NC}"

if [[ $FAIL -gt 0 ]]; then
    warn "Deployment completed with $FAIL warnings — review above."
    exit 0
else
    ok "Deployment completed successfully!"
    exit 0
fi
