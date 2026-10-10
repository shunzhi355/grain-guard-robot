#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/neardi/mechanism_ws/devel/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export PYTHONPATH="$PROJECT_DIR/src:$PROJECT_DIR:${PYTHONPATH:-}"

if ! systemctl is-active --quiet grain-sampling.service; then
    echo "grain-sampling.service 未运行，请先启动底层服务。"
    read -r -p "按 Enter 关闭窗口。"
    exit 1
fi

cd "$PROJECT_DIR"
mkdir -p log/post-sampling
LOG_FILE="log/post-sampling/$(date +%Y%m%d-%H%M%S).log"
echo "本次日志：$PROJECT_DIR/$LOG_FILE"
set +e
python3 scripts/test_post_sampling_devices.py "$@" 2>&1 | tee "$LOG_FILE"
status=${PIPESTATUS[0]}
set -e
read -r -p "测试结束，按 Enter 关闭窗口。"
exit "$status"
