#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime.sh
source "$DEPLOY_DIR/runtime.sh"
mapping_load_config

UPSTREAM_URL="https://github.com/zlwang7/S-FAST_LIO.git"
UPSTREAM_REV="93946196081ff8e6f665a6ddbd8024f65711edab"
PATCH_FILE="$DEPLOY_DIR/patches/sfast-lio-livox-driver2.patch"
BUILD_PATCH_FILE="$DEPLOY_DIR/patches/sfast-lio-message-generation-order.patch"
JAMMY_PATCH_FILE="$DEPLOY_DIR/patches/sfast-lio-jammy-cxx17.patch"
SOPHUS_URL="https://github.com/strasdat/Sophus.git"
SOPHUS_REV="a621ff2e56c56c839a6c40418d42c3c254424b5c"
SOPHUS_PATCH_FILE="$DEPLOY_DIR/patches/sophus-a621ff-jammy-eigen34.patch"
SOURCE_DIR="$SFAST_WS/src/S-FAST_LIO"
SOPHUS_DIR="$SFAST_WS/deps/Sophus"
SOPHUS_BUILD_DIR="$SFAST_WS/build_deps/Sophus"
BUILD_JOBS="${MAPPING_BUILD_JOBS:-2}"
MODE="${1:---check}"

for cmd in git python3 catkin_make rospack; do
    command -v "$cmd" >/dev/null 2>&1 || { echo "[FAIL] missing prerequisite: $cmd" >&2; exit 1; }
done
mapping_source_ros1
[ -f "$PATCH_FILE" ] || { echo "[FAIL] patch missing: $PATCH_FILE" >&2; exit 1; }
[ -f "$BUILD_PATCH_FILE" ] || { echo "[FAIL] build patch missing: $BUILD_PATCH_FILE" >&2; exit 1; }
[ -f "$JAMMY_PATCH_FILE" ] || { echo "[FAIL] Jammy patch missing: $JAMMY_PATCH_FILE" >&2; exit 1; }
[ -f "$SOPHUS_PATCH_FILE" ] || { echo "[FAIL] Sophus patch missing: $SOPHUS_PATCH_FILE" >&2; exit 1; }

if [ "$MODE" = "--check" ]; then
    echo "[PASS] S-FAST_LIO preparation prerequisites present"
    exit 0
fi
[ "$MODE" = "--execute" ] || { echo "usage: $0 [--check|--execute]" >&2; exit 2; }

mkdir -p "$SFAST_WS/src"
mkdir -p "$SFAST_WS/deps"
if [ ! -d "$SOPHUS_DIR/.git" ]; then
    git clone "$SOPHUS_URL" "$SOPHUS_DIR"
fi
if [ "$(git -C "$SOPHUS_DIR" rev-parse HEAD)" != "$SOPHUS_REV" ]; then
    [ -z "$(git -C "$SOPHUS_DIR" status --porcelain=v1)" ] || {
        echo "[FAIL] Sophus revision differs and worktree is dirty: $SOPHUS_DIR" >&2; exit 1;
    }
    git -C "$SOPHUS_DIR" cat-file -e "${SOPHUS_REV}^{commit}" 2>/dev/null || git -C "$SOPHUS_DIR" fetch --depth 1 origin "$SOPHUS_REV"
    git -C "$SOPHUS_DIR" checkout --detach "$SOPHUS_REV"
fi
sophus_status="$(git -C "$SOPHUS_DIR" status --porcelain=v1)"
if [ -n "$sophus_status" ]; then
    unexpected_sophus_paths="$(printf '%s\n' "$sophus_status" | cut -c4- | grep -Ev '^(CMakeLists\.txt|sophus/so2\.cpp)$' || true)"
    [ -z "$unexpected_sophus_paths" ] || {
        printf '[FAIL] unexpected Sophus changes:\n%s\n' "$unexpected_sophus_paths" >&2; exit 1;
    }
fi
if ! git -C "$SOPHUS_DIR" apply --reverse --check "$SOPHUS_PATCH_FILE" 2>/dev/null; then
    git -C "$SOPHUS_DIR" apply --check "$SOPHUS_PATCH_FILE"
    git -C "$SOPHUS_DIR" apply "$SOPHUS_PATCH_FILE"
fi
cmake -S "$SOPHUS_DIR" -B "$SOPHUS_BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SOPHUS_BUILD_DIR" --parallel "$BUILD_JOBS"
sudo cmake --install "$SOPHUS_BUILD_DIR"

if [ ! -d "$SOURCE_DIR/.git" ]; then
    git clone "$UPSTREAM_URL" "$SOURCE_DIR"
fi
if [ "$(git -C "$SOURCE_DIR" rev-parse HEAD)" != "$UPSTREAM_REV" ]; then
    [ -z "$(git -C "$SOURCE_DIR" status --porcelain=v1)" ] || {
        echo "[FAIL] S-FAST_LIO revision differs and worktree is dirty: $SOURCE_DIR" >&2; exit 1;
    }
    git -C "$SOURCE_DIR" cat-file -e "${UPSTREAM_REV}^{commit}" 2>/dev/null || git -C "$SOURCE_DIR" fetch --depth 1 origin "$UPSTREAM_REV"
    git -C "$SOURCE_DIR" checkout --detach "$UPSTREAM_REV"
fi
if ! git -C "$SOURCE_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
    [ -z "$(git -C "$SOURCE_DIR" status --porcelain=v1)" ] || {
        echo "[FAIL] S-FAST_LIO has changes beyond the approved patch: $SOURCE_DIR" >&2; exit 1;
    }
    git -C "$SOURCE_DIR" apply --check "$PATCH_FILE"
    git -C "$SOURCE_DIR" apply "$PATCH_FILE"
fi
if ! git -C "$SOURCE_DIR" apply --reverse --check "$BUILD_PATCH_FILE" 2>/dev/null; then
    git -C "$SOURCE_DIR" apply --check "$BUILD_PATCH_FILE"
    git -C "$SOURCE_DIR" apply "$BUILD_PATCH_FILE"
fi
if grep -q -- '-std=c++14' "$SOURCE_DIR/CMakeLists.txt"; then
    sed -i 's/-std=c++14/-std=c++17/g' "$SOURCE_DIR/CMakeLists.txt"
fi
if grep -q -- '-std=c++0x' "$SOURCE_DIR/CMakeLists.txt"; then
    sed -i 's/-std=c++0x/-std=c++17/g' "$SOURCE_DIR/CMakeLists.txt"
fi
grep -q -- '-std=c++17' "$SOURCE_DIR/CMakeLists.txt" || {
    echo "[FAIL] S-FAST_LIO CMake is not configured for C++17" >&2; exit 1;
}
if ! grep -q 'set_property(TARGET sfastlio_mapping PROPERTY CXX_STANDARD 17)' "$SOURCE_DIR/CMakeLists.txt"; then
    sed -i '/add_dependencies(sfastlio_mapping/a set_property(TARGET sfastlio_mapping PROPERTY CXX_STANDARD 17)' "$SOURCE_DIR/CMakeLists.txt"
fi
if ! grep -q 'set_property(TARGET fastlio_mapping_re PROPERTY CXX_STANDARD 17)' "$SOURCE_DIR/CMakeLists.txt"; then
    sed -i '/add_dependencies(fastlio_mapping_re/a set_property(TARGET fastlio_mapping_re PROPERTY CXX_STANDARD 17)' "$SOURCE_DIR/CMakeLists.txt"
fi

mapping_source_workspace "$LIVOX_WS"
(cd "$SFAST_WS" && catkin_make -j"$BUILD_JOBS" -l"$BUILD_JOBS")
[ -x "$SFAST_WS/devel/lib/sfast_lio/sfastlio_mapping" ]
echo "[PASS] pinned S-FAST_LIO adapted to livox_ros_driver2 and built"
