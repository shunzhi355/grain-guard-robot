#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

if [[ -f /home/neardi/mechanism_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/neardi/mechanism_ws/devel/setup.bash
fi

cd "$ROOT_DIR"
exec python3 -u scripts/test_bin_doors_terminal.py
