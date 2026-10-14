#!/usr/bin/env bash
# Explicit localization initialization; never clears ESTOP, arms or creates goals.
set -Eeuo pipefail
RUNTIME="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
[[ "$RUNTIME" =~ ^task043-new-map-relocalization-[A-Za-z0-9._-]+$ ]] || { echo 'unknown runtime owner' >&2; exit 1; }
[[ "$(docker inspect -f '{{.Config.Image}}' "$RUNTIME")" == grain-nav:noetic-x86 ]] || { echo 'runtime image mismatch' >&2; exit 1; }
[[ "$(docker inspect -f '{{.State.Status}}' "$RUNTIME")" == running ]] || { echo 'start task043 runtime first' >&2; exit 1; }
case "${1:-}" in
  --map-origin-confirmed)
    [[ "$#" == 1 ]] || exit 2
    # Owner explicitly confirms original mapping position, heading and mounting.
    args=(--x 0 --y 0 --z 0 --yaw 0) ;;
  --pose-confirmed)
    [[ "$#" == 5 ]] || { echo 'usage: --pose-confirmed X Y Z YAW_RAD (native Golden body pose in camera_init)' >&2; exit 2; }
    args=(--x "$2" --y "$3" --z "$4" --yaw "$5") ;;
  *) echo 'choose --map-origin-confirmed ONLY at the original mapping start/heading; otherwise --pose-confirmed X Y Z YAW_RAD' >&2; exit 2 ;;
esac
docker exec "$RUNTIME" python3 /task043/out/task043_localization_startup.py \
  --config /task043/out/localization_startup_config.json --out /task043/out \
  --confirm-initial-pose --stationary-confirmed "${args[@]}"
echo 'Keep the robot stationary through raw-input readiness and 60 s static R2. No goal is resumed.'
