#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
export PYTHONPATH="$ROOT_DIR/src:$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

if [[ -f /home/neardi/mechanism_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/neardi/mechanism_ws/devel/setup.bash
fi

cd "$ROOT_DIR"

# Only one interactive door tester may own the hardware at a time.
exec 9>/tmp/grain-bin-door-terminal.lock
if ! flock -n 9; then
    echo "三仓测试终端已经打开，请使用现有窗口。"
    read -r -p "按回车退出…" || true
    exit 1
fi

LOG_DIR=$(mktemp -d /tmp/bin-door-terminal-XXXXXX)
CORE_PID=""
NODE_PID=""
cleanup() {
    if [[ -n "$NODE_PID" ]]; then
        timeout 5 rosservice call /mechanism/emergency_stop || true
        kill -TERM "$NODE_PID" 2>/dev/null || true
        wait "$NODE_PID" 2>/dev/null || true
    fi
    if [[ -n "$CORE_PID" ]]; then
        kill -TERM "$CORE_PID" 2>/dev/null || true
        wait "$CORE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

echo "三仓独立测试终端，日志：$LOG_DIR"
if ! timeout 3 rosparam list >/dev/null 2>&1; then
    echo "正在启动 ROS…"
    roscore >"$LOG_DIR/roscore.log" 2>&1 9>&- &
    CORE_PID=$!
    for _ in $(seq 1 20); do
        if timeout 2 rosparam list >/dev/null 2>&1; then break; fi
        sleep 0.5
    done
    timeout 3 rosparam list >/dev/null
fi

if ! rosservice list | grep -q '^/mechanism/open_bin/deep$'; then
    if rosnode list | grep -qE '^/mechanism_node($|_)'; then
        echo "现有机构节点服务不完整，请先检查机构节点。"
        exit 1
    fi
    echo "正在启动机构服务并初始化电调中位…"
    X2P_PORT='' python3 -u -m grain_sampling_workflow.mechanism_node \
        >"$LOG_DIR/mechanism.log" 2>&1 9>&- &
    NODE_PID=$!
    for _ in $(seq 1 30); do
        if rosservice list | grep -q '^/mechanism/open_bin/deep$'; then break; fi
        if ! kill -0 "$NODE_PID" 2>/dev/null; then
            cat "$LOG_DIR/mechanism.log"
            exit 1
        fi
        sleep 0.5
    done
fi

python3 -u scripts/test_bin_doors_terminal.py
