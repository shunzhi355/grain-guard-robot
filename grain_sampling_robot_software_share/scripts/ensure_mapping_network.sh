#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=../deploy/mapping/runtime.sh
source "$PROJECT_ROOT/deploy/mapping/runtime.sh"
mapping_load_config

ip link show "$RADAR_NETDEV" >/dev/null 2>&1 || { echo "[FAIL] radar interface missing: $RADAR_NETDEV" >&2; exit 1; }
[ "$(cat "/sys/class/net/$RADAR_NETDEV/carrier" 2>/dev/null || true)" = "1" ] || {
    echo "[BLOCKED] no carrier on radar interface: $RADAR_NETDEV" >&2
    exit 2
}

if ! ip -4 addr show dev "$RADAR_NETDEV" | grep -Eq "(^|[[:space:]])inet ${RADAR_HOST_IP//./\.}/"; then
    sudo -n ip addr replace "$RADAR_HOST_IP/32" dev "$RADAR_NETDEV" || {
        echo "[BLOCKED] passwordless sudo is required to assign the temporary radar host address" >&2
        exit 2
    }
fi

if ! mapping_route_is_exact; then
    sudo -n ip route replace "$MID360_IP/32" dev "$RADAR_NETDEV" src "$RADAR_HOST_IP" || {
        echo "[BLOCKED] passwordless sudo is required to install the MID360 host route" >&2
        exit 2
    }
fi

mapping_route_is_exact || { echo "[FAIL] exact MID360 route validation failed" >&2; exit 1; }
echo "[PASS] $(ip route get "$MID360_IP" | head -n 1)"
