#!/usr/bin/env bash
# Single, ownership-aware entry point for the Lenovo UI/Noetic mapping runtime.
set -Eeuo pipefail

CONTAINER="${GRAIN_NOETIC_CONTAINER:-}"
CHECK=0
COMMAND=()
while (($#)); do
  case "$1" in
    --check) CHECK=1; shift ;;
    --container) CONTAINER="${2:?--container requires a name}"; shift 2 ;;
    --) shift; COMMAND=("$@"); break ;;
    *) COMMAND+=("$1"); shift ;;
  esac
done

if [[ -z "$CONTAINER" ]]; then
  echo "Noetic container is not configured (set GRAIN_NOETIC_CONTAINER)" >&2
  exit 2
fi
command -v docker >/dev/null 2>&1 || { echo "docker is unavailable" >&2; exit 2; }
state="$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || true)"
[[ "$state" == "running" ]] || { echo "task-owned Noetic container is not running: $CONTAINER ($state)" >&2; exit 1; }
image="$(docker inspect -f '{{.Config.Image}}' "$CONTAINER")"
expected="${GRAIN_NOETIC_IMAGE:-grain-nav:noetic-x86}"
[[ "$image" == "$expected" ]] || { echo "container image mismatch: $image (expected $expected)" >&2; exit 1; }

if (( CHECK )); then
  docker exec "$CONTAINER" bash -lc 'source /opt/ros/noetic/setup.bash && command -v roslaunch >/dev/null && command -v rostopic >/dev/null'
  exit $?
fi
if ((${#COMMAND[@]} == 0)); then
  echo "usage: $0 --check [--container NAME] | -- [command...]" >&2
  exit 2
fi
quoted=""
printf -v quoted '%q ' "${COMMAND[@]}"
exec docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash && exec $quoted"
