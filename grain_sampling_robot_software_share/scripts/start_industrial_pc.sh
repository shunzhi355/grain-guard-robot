#!/usr/bin/env bash
# 临滴科技工控机启动脚本
#
# 接线约定：
#   USB-TTL  -> FS-iA10B i-BUS（通常 /dev/ttyUSB0）
#   USB-RS485 -> X2P 伺服（通常 /dev/ttyUSB1，建议改成 udev 稳定链接）
#   LPB3588 /dev/i2c-2 -> PCA9685（机构 CH0~CH6，底盘左 CH10 / 右 CH9）
#
# 使用：
#   bash scripts/start_industrial_pc.sh
#
# 可在执行前覆盖设备路径，例如：
#   RC_SERIAL_PORT=/dev/rc_receiver X2P_PORT=/dev/x2p_lift \
#   PCA9685_I2C_DEVICE=/dev/i2c-2 bash scripts/start_industrial_pc.sh

set -e
set -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
CONFIG_FILE="${GRAIN_HARDWARE_CONFIG:-$ROOT_DIR/config/industrial_pc.env}"
[[ -f "$CONFIG_FILE" ]] || { echo "硬件配置文件不存在：$CONFIG_FILE" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONFIG_FILE"
LOG_ROOT="${GRAIN_ROBOT_LOG_DIR:-$ROOT_DIR/log/hardware}"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"
export PYTHONUNBUFFERED=1
export PYTHONFAULTHANDLER=1
exec > >(tee -a "$LOG_DIR/startup.log") 2>&1
printf 'session=%s started=%s\n' "$LOG_DIR" "$(date -Is)"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export PYTHONPATH="$SRC_DIR:$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

say() { printf '[industrial-pc] %s\n' "$*"; }
fail() { say "错误：$*" >&2; exit 1; }

# 项目使用 ROS1 rospy。兼容 ROS 官方 /opt 布局和 Ubuntu/Debian 拆分包布局。
ROS_SETUP_BASH="${ROS_SETUP_BASH:-}"
if [[ -n "$ROS_SETUP_BASH" ]]; then
    [[ -f "$ROS_SETUP_BASH" ]] || fail "ROS_SETUP_BASH 不存在：$ROS_SETUP_BASH"
    # shellcheck disable=SC1090
    source "$ROS_SETUP_BASH"
    ROS_ENV_DESCRIPTION="$ROS_SETUP_BASH"
elif [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    ROS_ENV_DESCRIPTION="/opt/ros/noetic/setup.bash"
elif command -v roscore >/dev/null 2>&1 && python3 -c "import rospy, roslaunch"; then
    ROS_ENV_DESCRIPTION="Ubuntu/Debian system ROS1"
else
    fail "ROS1 不可用：请安装 ros-core、python3-rospy、python3-roslaunch"
fi

# mechanism_node 只需要消息包；若现场有独立工作空间则加载它。
if [[ -n "${MECHANISM_WS_SETUP:-}" ]]; then
    [[ -f "$MECHANISM_WS_SETUP" ]] || fail "MECHANISM_WS_SETUP 不存在：$MECHANISM_WS_SETUP"
    # shellcheck disable=SC1090
    source "$MECHANISM_WS_SETUP"
elif [[ -f "$HOME/mechanism_ws/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/mechanism_ws/devel/setup.bash"
fi

python3 -c "import rospy, serial; from mechanism_node.srv import MoveLift, SetGrain" ||
    fail "缺少 rospy、pyserial 或 mechanism_node 消息包；请安装依赖并加载 MECHANISM_WS_SETUP"
[[ "$RC_SERIAL_PORT" != "$X2P_PORT" ]] || fail "遥控器和伺服不能使用同一串口"
[[ ! -e "$X2P_PORT" || ! "$RC_SERIAL_PORT" -ef "$X2P_PORT" ]] ||
    fail "遥控器和伺服路径指向同一设备"
[[ -r "$RC_SERIAL_PORT" && -w "$RC_SERIAL_PORT" ]] ||
    fail "遥控器串口不可读写：$RC_SERIAL_PORT（检查设备名与串口组权限）"
I2C_DEVICE="${PCA9685_I2C_DEVICE:-/dev/i2c-$PCA9685_I2C_BUS}"
[[ -e "$I2C_DEVICE" ]] ||
    fail "I2C 设备不存在：$I2C_DEVICE；LPB3588 实机使用 /dev/i2c-2，请核实设备树和设备节点"
[[ -r "$I2C_DEVICE" && -w "$I2C_DEVICE" ]] ||
    fail "I2C 设备不可读写：$I2C_DEVICE；请核实总线号和 i2c 组权限"

if [[ -e "$X2P_PORT" ]]; then
    say "X2P 伺服串口：$X2P_PORT"
else
    say "警告：X2P 串口暂未找到：$X2P_PORT（机构节点会保留服务，接入后可重连）"
fi

start_once() {
    local pattern="$1"; shift
    local logfile="$1"; shift
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        say "已运行：$pattern"
        return 0
    fi
    setsid nohup "$@" >"$LOG_DIR/$logfile" 2>&1 < /dev/null &
    local child_pid=$!
    disown || true
    sleep 1
    kill -0 "$child_pid" 2>/dev/null || fail "进程启动失败：$pattern；查看 $LOG_DIR/$logfile"
    say "已启动：$pattern（日志 $LOG_DIR/$logfile）"
}

if ! pgrep -f 'rosco[r]e' >/dev/null 2>&1; then
    setsid nohup roscore >"$LOG_DIR/roscore.log" 2>&1 < /dev/null &
    disown || true
    sleep 3
fi

python3 -c "import rospy; rospy.get_master().getPid()" || fail "ROS master 尚不可用，请检查 roscore.log"

start_once 'grain_sampling_workflow.mechanism_node' mechanism_node.log \
    python3 -m grain_sampling_workflow.mechanism_node
start_once 'motor_driver.py daemo[n]' motor_driver.log \
    python3 "$ROOT_DIR/dipan/motor_driver.py" daemon --host 127.0.0.1 --port 8765 --timeout 0.3
start_once 'grain_sampling_workflow.rc_node' rc_node.log \
    python3 -m grain_sampling_workflow.rc_node
start_once 'cmd_vel_to_moto[r]' cmd_vel_to_motor.log \
    python3 "$ROOT_DIR/dipan/cmd_vel_to_motor.py"

say "启动完成：ROS=$ROS_ENV_DESCRIPTION，RC=$RC_SERIAL_PORT，X2P=$X2P_PORT，PCA9685=${PCA9685_I2C_DEVICE:-/dev/i2c-$PCA9685_I2C_BUS} 地址=$PCA9685_I2C_ADDRESS"
say "查看日志：$LOG_DIR；停止节点可使用 pkill 或 systemctl stop 对应服务"
