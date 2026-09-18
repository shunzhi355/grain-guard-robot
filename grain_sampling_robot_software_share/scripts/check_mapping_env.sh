#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../deploy/mapping/runtime.sh
source "$PROJECT_ROOT/deploy/mapping/runtime.sh"

MODE="${1:---preflight}"
[ "$MODE" = "--preflight" ] || [ "$MODE" = "--runtime" ] || {
    echo "usage: $0 [--preflight|--runtime]" >&2
    exit 2
}

pass_count=0
fail_count=0
blocked_count=0
pass() { echo "[PASS] $*"; pass_count=$((pass_count + 1)); }
fail() { echo "[FAIL] $*"; fail_count=$((fail_count + 1)); }
blocked() { echo "[BLOCKED] $*"; blocked_count=$((blocked_count + 1)); }
exists() { [ -e "$1" ] && pass "$2: $1" || fail "$2 missing: $1"; }

mapping_load_config || fail "mapping configuration could not be loaded"

if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    [ "${VERSION_ID:-}" = "22.04" ] && pass "OS Ubuntu 22.04" || blocked "target OS is Ubuntu 22.04; found ${PRETTY_NAME:-unknown}"
else
    blocked "cannot identify OS"
fi
[ "$(uname -m)" = "aarch64" ] && pass "architecture aarch64" || blocked "target architecture is aarch64; found $(uname -m)"

if mapping_source_ros1; then
    pass "ROS1 command environment available"
else
    blocked "ROS1 commands unavailable"
fi
for command_name in roscore roslaunch rostopic rospack; do
    command -v "$command_name" >/dev/null 2>&1 && pass "$command_name available" || blocked "$command_name unavailable"
done

ip link show "$RADAR_NETDEV" >/dev/null 2>&1 && pass "radar interface exists: $RADAR_NETDEV" || fail "radar interface missing: $RADAR_NETDEV"
carrier="$(cat "/sys/class/net/$RADAR_NETDEV/carrier" 2>/dev/null || true)"
[ "$carrier" = "1" ] && pass "radar interface carrier detected" || blocked "radar interface carrier is not detected"

exists "$LIVOX_WS" "Livox workspace"
exists "$SFAST_WS" "S-FAST_LIO workspace"
exists "$SFAST_MAPPING_EXECUTABLE" "S-FAST_LIO mapping executable"
exists "$SFAST_CONFIG" "S-FAST_LIO MID360 config"

if [ -d "$LIVOX_WS" ] && mapping_source_workspace "$LIVOX_WS"; then
    driver_path="$(rospack find "$LIVOX_PACKAGE" 2>/dev/null || true)"
    [ -n "$driver_path" ] && pass "Livox package: $driver_path" || fail "Livox package unavailable: $LIVOX_PACKAGE"
    if [ -n "$driver_path" ]; then
        exists "$driver_path/launch_ROS1/$LIVOX_LAUNCH" "Livox ROS1 launch"
        exists "$driver_path/config/MID360_config.json" "Livox native MID360 config"
    fi
fi

if [ "$MODE" = "--runtime" ]; then
    ip -4 addr show dev "$RADAR_NETDEV" 2>/dev/null | grep -Eq "(^|[[:space:]])inet ${RADAR_HOST_IP//./\.}/" &&
        pass "radar host IP is assigned to $RADAR_NETDEV" || fail "radar host IP $RADAR_HOST_IP is not assigned to $RADAR_NETDEV"
    mapping_route_is_exact && pass "MID360 route has exact dev/src" || fail "MID360 route is not exact for $RADAR_NETDEV source $RADAR_HOST_IP"
    ping -I "$RADAR_NETDEV" -c 1 -W 2 "$MID360_IP" >/dev/null 2>&1 && pass "MID360 reachable at $MID360_IP" || blocked "MID360 unreachable at $MID360_IP"
fi

echo "SUMMARY PASS=$pass_count FAIL=$fail_count BLOCKED=$blocked_count"
[ "$fail_count" -eq 0 ] || exit 1
[ "$blocked_count" -eq 0 ] || exit 2
exit 0
