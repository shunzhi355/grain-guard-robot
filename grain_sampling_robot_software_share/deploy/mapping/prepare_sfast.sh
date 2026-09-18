#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime.sh
source "$DEPLOY_DIR/runtime.sh"
mapping_load_config

UPSTREAM_URL="https://github.com/zlwang7/S-FAST_LIO.git"
UPSTREAM_REV="93946196081ff8e6f665a6ddbd8024f65711edab"
PATCH_FILE="$DEPLOY_DIR/patches/sfast-lio-livox-driver2.patch"
SOURCE_DIR="$SFAST_WS/src/S-FAST_LIO"
BUILD_JOBS="${MAPPING_BUILD_JOBS:-2}"
MODE="${1:---check}"

for cmd in git python3 catkin_make rospack; do
    command -v "$cmd" >/dev/null 2>&1 || { echo "[FAIL] missing prerequisite: $cmd" >&2; exit 1; }
done
mapping_source_ros1
[ -f "$PATCH_FILE" ] || { echo "[FAIL] patch missing: $PATCH_FILE" >&2; exit 1; }

if [ "$MODE" = "--check" ]; then
    echo "[PASS] S-FAST_LIO preparation prerequisites present"
    exit 0
fi
[ "$MODE" = "--execute" ] || { echo "usage: $0 [--check|--execute]" >&2; exit 2; }

mkdir -p "$SFAST_WS/src"
if [ ! -d "$SOURCE_DIR/.git" ]; then
    git clone "$UPSTREAM_URL" "$SOURCE_DIR"
fi
[ -z "$(git -C "$SOURCE_DIR" status --porcelain=v1)" ] || {
    echo "[FAIL] S-FAST_LIO worktree is dirty: $SOURCE_DIR" >&2; exit 1;
}
git -C "$SOURCE_DIR" cat-file -e "${UPSTREAM_REV}^{commit}" 2>/dev/null || git -C "$SOURCE_DIR" fetch --depth 1 origin "$UPSTREAM_REV"
git -C "$SOURCE_DIR" checkout --detach "$UPSTREAM_REV"
git -C "$SOURCE_DIR" apply --check "$PATCH_FILE"
git -C "$SOURCE_DIR" apply "$PATCH_FILE"

mapping_source_workspace "$LIVOX_WS"
(cd "$SFAST_WS" && catkin_make -j"$BUILD_JOBS" -l"$BUILD_JOBS")
[ -x "$SFAST_WS/devel/lib/sfast_lio/sfastlio_mapping" ]
echo "[PASS] pinned S-FAST_LIO adapted to livox_ros_driver2 and built"
