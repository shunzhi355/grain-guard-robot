#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime.sh
source "$DEPLOY_DIR/runtime.sh"
mapping_load_config

SDK_URL="https://github.com/Livox-SDK/Livox-SDK2.git"
SDK_REV="08f523c930b2f0ba1e98a6afaa8d7476bf479908"
DRIVER_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"
DRIVER_REV="4a1def929e5b59c7a8122d19fce6efba581ce9f7"
MODE="${1:---check}"

missing=()
for cmd in git cmake make gcc g++ python3 catkin_make rospack; do
    command -v "$cmd" >/dev/null 2>&1 || missing+=("$cmd")
done
mapping_source_ros1 || missing+=("ROS1 runtime")
if [ "$(uname -m)" != "aarch64" ]; then
    echo "[FAIL] expected aarch64, found $(uname -m)" >&2
    exit 1
fi
if [ "${#missing[@]}" -ne 0 ]; then
    printf '[FAIL] missing prerequisite: %s\n' "${missing[@]}" >&2
    exit 1
fi
echo "[PASS] prerequisites present; Ubuntu 22.04 + Debian ROS1 remains board-verification-required"

if [ "$MODE" = "--check" ]; then
    exit 0
fi
[ "$MODE" = "--execute" ] || { echo "usage: $0 [--check|--execute]" >&2; exit 2; }

mkdir -p "$LIVOX_WS/src"
clone_or_verify() {
    local url="$1" revision="$2" destination="$3"
    if [ ! -d "$destination/.git" ]; then
        git clone "$url" "$destination"
    fi
    [ -z "$(git -C "$destination" status --porcelain=v1)" ] || {
        echo "[FAIL] dependency worktree is dirty: $destination" >&2; return 1;
    }
    git -C "$destination" cat-file -e "${revision}^{commit}" 2>/dev/null || git -C "$destination" fetch --depth 1 origin "$revision"
    git -C "$destination" checkout --detach "$revision"
    [ "$(git -C "$destination" rev-parse HEAD)" = "$revision" ]
}

SDK_DIR="$LIVOX_WS/src/Livox-SDK2"
DRIVER_DIR="$LIVOX_WS/src/livox_ros_driver2"
clone_or_verify "$SDK_URL" "$SDK_REV" "$SDK_DIR"
clone_or_verify "$DRIVER_URL" "$DRIVER_REV" "$DRIVER_DIR"

cmake -S "$SDK_DIR" -B "$SDK_DIR/build"
cmake --build "$SDK_DIR/build" --parallel "$(nproc)"
sudo cmake --install "$SDK_DIR/build"

python3 "$DEPLOY_DIR/generate_driver2_config.py" \
    --application "$MAPPING_APP_CONFIG" \
    --template "$DRIVER_DIR/config/MID360_config.json" \
    --output "$DRIVER_DIR/config/MID360_config.json"

(cd "$DRIVER_DIR" && ./build.sh ROS1)
mapping_source_workspace "$LIVOX_WS"
rospack find livox_ros_driver2 >/dev/null
[ -f "$(rospack find livox_ros_driver2)/launch_ROS1/msg_MID360.launch" ]
echo "[PASS] pinned Livox SDK2 and ROS1 Driver2 built and verified"
