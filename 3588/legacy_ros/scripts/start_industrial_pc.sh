#!/usr/bin/env bash
# 临滴科技工控机启动脚本
#
# 接线约定：
#   USB1 -> FT232 USB-TTL -> STM32 USART2（115200 8N1，CHASSIS_SERIAL_PORT）
#   FS-iA10B i-BUS -> STM32 USART1；手动控制与底盘 PWM 由单片机负责
#   USB-RS485 -> X2P 伺服（通常 /dev/ttyUSB1，建议改成 udev 稳定链接）
#   本机 I2C2 -> PCA9685（已通过模块断电/上电对照确认）
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
if [[ "$CHASSIS_BACKEND" == serial ]]; then
    [[ "$CHASSIS_SERIAL_PORT" != "$X2P_PORT" ]] || fail "底盘和伺服不能使用同一串口"
    [[ ! -e "$X2P_PORT" || ! "$CHASSIS_SERIAL_PORT" -ef "$X2P_PORT" ]] || fail "底盘和伺服路径指向同一设备"
    [[ -r "$CHASSIS_SERIAL_PORT" && -w "$CHASSIS_SERIAL_PORT" ]] || fail "底盘串口不可读写：$CHASSIS_SERIAL_PORT；请检查USB-TTL连接及/dev/serial/by-id/设备节点"
    if pgrep -f 'motor_driver.py daemo[n]|cmd_vel_to_moto[r]|grain_sampling_workflow.rc_nod[e]' >/dev/null; then
        fail "旧底盘/遥控节点仍在运行，请先运行 scripts/stop_industrial_pc.sh"
    fi
else
    fail "本工控机只支持 CHASSIS_BACKEND=serial；底盘由 STM32 控制"
fi
I2C_DEVICE="${PCA9685_I2C_DEVICE:-/dev/i2c-$PCA9685_I2C_BUS}"
if pgrep -f '^python3( -u)? /tmp/pca[^ /]*\.py($| )' >/dev/null; then
    fail "PCA9685 独立测试仍在运行，请先停止测试并确认输出关闭"
fi
python3 -m grain_sampling_devices.tp_i2c "$I2C_DEVICE" || fail "PCA9685 控制器身份不符，拒绝启动机构"
[[ -e "$I2C_DEVICE" ]] ||
    fail "I2C 设备不存在：$I2C_DEVICE；本机实测为 I2C2，请核实设备树和设备节点"
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
if [[ "$CHASSIS_BACKEND" == serial ]]; then
    start_once 'grain_sampling_workflow.chassis_nod[e]' chassis_serial.log \
        python3 -m grain_sampling_workflow.chassis_node
    start_once 'goal_controller.p[y]' goal_controller.log \
        python3 "$ROOT_DIR/dipan/goal_controller.py" _chassis_backend:=serial

fi

say "底盘后端=$CHASSIS_BACKEND，底盘串口=$CHASSIS_SERIAL_PORT（USART2: 115200 8N1）"

say "启动完成：ROS=$ROS_ENV_DESCRIPTION，RC=$RC_SERIAL_PORT，X2P=$X2P_PORT，PCA9685=${PCA9685_I2C_DEVICE:-/dev/i2c-$PCA9685_I2C_BUS} 地址=$PCA9685_I2C_ADDRESS"
say "查看日志：$LOG_DIR；停止节点可使用 pkill 或 systemctl stop 对应服务"
