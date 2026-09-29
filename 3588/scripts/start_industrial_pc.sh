#!/usr/bin/env bash
# 3588 production runtime: GRICP + local chassis/mechanism daemon, no ROS.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="${GRAIN_HARDWARE_CONFIG:-$ROOT_DIR/config/industrial_pc.env}"
[[ -f "$CONFIG_FILE" ]] || { echo "Missing 3588 config: $CONFIG_FILE" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONFIG_FILE"
export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

[[ "${CHASSIS_BACKEND:-serial}" == serial ]] || { echo "CHASSIS_BACKEND must be serial" >&2; exit 1; }
[[ -r "$CHASSIS_SERIAL_PORT" && -w "$CHASSIS_SERIAL_PORT" ]] || {
    echo "STM32 serial device unavailable: $CHASSIS_SERIAL_PORT" >&2; exit 1;
}
[[ "$CHASSIS_SERIAL_PORT" != "$X2P_PORT" ]] || {
    echo "STM32 and X2P cannot share a serial device" >&2; exit 1;
}
for path in "$GRICP_TLS_CERT" "$GRICP_TLS_KEY" "$GRICP_TLS_CA"; do
    [[ -r "$path" ]] || { echo "TLS file unavailable: $path" >&2; exit 1; }
done

exec python3 -m grain_sampling_interhost.server \
    --bind-ip "$GRICP_BIND_IP" --peer-ip "$GRICP_ALLOWED_PEER" \
    --cert "$GRICP_TLS_CERT" --key "$GRICP_TLS_KEY" --ca "$GRICP_TLS_CA" \
    --ipc "$GRAIN_ROBOT_SOCKET"
