#!/bin/bash
# RK3588 工控机部署脚本 — X2P 伺服升降 + 机制服务重启验证
# 用法（已同步源码后，板端执行）：
#   bash scripts/deploy_x2p_board.sh
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "=== 1. 安装 pyserial ==="
pip3 install pyserial --user 2>&1 | tail -2

echo "=== 2. py_compile 验证 ==="
python3 -m py_compile \
  src/grain_sampling_devices/mechanism_driver.py \
  src/grain_sampling_devices/x2p_lift.py \
  src/grain_sampling_workflow/mechanism_node.py \
  src/grain_sampling_workflow/rc_control.py \
  src/grain_sampling_ui/main.py \
  src/grain_sampling_workflow/slam_bridge.py
echo "COMPILE_OK"

echo "=== 3. 验证 x2p 可导入 ==="
PYTHONPATH="$ROOT_DIR/src${PYTHONPATH:+:$PYTHONPATH}" python3 -c "from x2p import X2PDrive, MotionController, ControllerConfig; print('x2p import OK')"

echo "=== 4. 重启 mechanism_node（X2P 自动启用，默认 /dev/ttyS0）==="
# ROS1 环境探测：优先 ROS_SETUP_BASH，其次 /opt/ros/noetic，最后系统拆分包安装。
# 本工控机（LPA3588 / RK3588）没有 /opt/ros/noetic/setup.bash，硬编码 source 会直接失败。
ROS_SETUP_BASH="${ROS_SETUP_BASH:-}"
if [[ -n "$ROS_SETUP_BASH" ]]; then
  [[ -f "$ROS_SETUP_BASH" ]] || { echo "ROS_SETUP_BASH 不存在：$ROS_SETUP_BASH" >&2; exit 1; }
  # shellcheck disable=SC1090
  source "$ROS_SETUP_BASH"
elif [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
elif command -v roscore >/dev/null 2>&1 && python3 -c "import rospy, roslaunch" 2>/dev/null; then
  echo "[deploy] 使用系统自带 ROS1（本机无 /opt/ros/noetic/setup.bash）"
else
  echo "ROS1 不可用：请安装 ros-core、python3-rospy、python3-roslaunch" >&2
  exit 1
fi
if [[ -f "$HOME/mechanism_ws/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/mechanism_ws/devel/setup.bash"
fi
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export PYTHONPATH="$ROOT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
export X2P_PORT="${X2P_PORT:-/dev/ttyS0}"
# 可选覆盖：export X2P_RPM=30 X2P_DURATION=2.0
pkill -f grain_sampling_workflow.mechanism_node 2>/dev/null || true
sleep 2
nohup python3 -m grain_sampling_workflow.mechanism_node > /tmp/mechanism_node.log 2>&1 &
sleep 6

echo "=== 5. mechanism_node 日志 ==="
tail -15 /tmp/mechanism_node.log

echo "=== 6. 服务计数（期望 13）==="
rosservice list 2>/dev/null | grep -cE '^/mechanism/'

echo ""
echo "=== 部署完成 ==="
echo "实机验证："
echo "  rosservice call /mechanism/press '{}'   # 下压"
echo "  rosservice call /mechanism/lift '{}'    # 提升"
echo "  若方向反：改板端 x2p 配置 forward_sign=-1"
echo "  上电后如不在物理最高点，先停服务再做手动微调："
echo "    sudo systemctl stop grain-sampling"
echo "    python3 scripts/manual_lift_adjust.py --status"
echo "    python3 scripts/manual_lift_adjust.py up 0.5"
echo "  详见 项目完整运行流程说明.md 第十四节。"
echo "  [WARN] 垂直轴需硬件急停+上下限位+防坠装置"
