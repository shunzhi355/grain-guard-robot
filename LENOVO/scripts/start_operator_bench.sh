#!/usr/bin/env bash
# One-command Lenovo USB-serial/USB-RS485 operator-navigation bench session.
# This launcher never starts a task, clears a stop, moves the base or replaces
# an existing daemon/UI. The operator must remain at the real emergency stop.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
CONFIG_FILE="$PROJECT_DIR/LENOVO/config/local_robot.operator-live.json"
PYTHON="$PROJECT_DIR/.venv/bin/python"
ATTENDED=0
SUCTION_CONFIRMED=0
DAEMON_PID=""
UI_PID=""
SOCKET=""
LOG_DIR=""

usage() {
    cat <<'EOF'
用法：bash LENOVO/scripts/start_operator_bench.sh [--config 绝对路径] [--attended --external-suction-confirmed]

在联想图形桌面一键启动本机直连硬件服务和完整取样 UI，导航到位由操作员确认。
不启动任务、不自动到位、不复位机构、不清除急停、不启动雷达或自主导航。
不关闭或抢占现有服务；请先安全结束旧会话后再启动。
升降转速、串口身份等均读取配置文件，不使用旧 3588 脚本的 800 rpm 默认值。

未带两个确认参数时，在终端分别交互确认现场监护/急停和外部吸粮处理。
SSH 启动图形 UI 时需先设置 DISPLAY 和必要的 XAUTHORITY。
关闭 UI 后会发送停止请求并关闭本脚本启动的硬件服务；日志保存在运行目录的 bench/ 下。
EOF
}

while (($#)); do
    case "$1" in
        --config)
            (($# >= 2)) || { echo "--config 缺少路径" >&2; exit 2; }
            CONFIG_FILE="$2"
            shift 2
            ;;
        --attended) ATTENDED=1; shift ;;
        --external-suction-confirmed) SUCTION_CONFIRMED=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

[[ -x "$PYTHON" ]] || { echo "缺少专用 Python：$PYTHON" >&2; exit 1; }
[[ -f "$CONFIG_FILE" ]] || { echo "缺少配置：$CONFIG_FILE" >&2; exit 1; }
CONFIG_FILE="$(realpath -e -- "$CONFIG_FILE")"
[[ -n "${DISPLAY:-}" && "${QT_QPA_PLATFORM:-xcb}" != offscreen ]] || {
    echo "需要联想图形桌面 DISPLAY；不能在无显示环境启动实机 UI。" >&2
    exit 1
}
command -v fuser >/dev/null || { echo "缺少 fuser（psmisc）；无法检查串口占用。" >&2; exit 1; }
# Only the small read-only probes need a source path. Never export it to the
# UI: local_robot.py must load installed PySide6 before the project Qt shims.
# The shared workflow package must precede Lenovo's package of the same name.
PROBE_PYTHONPATH="$PROJECT_DIR/3588/src:$PROJECT_DIR/3588:$PROJECT_DIR/LENOVO/src${PYTHONPATH:+:$PYTHONPATH}"

# No physical I/O: validate stable USB identities and the operator-only mode.
PREFLIGHT="$(PYTHONPATH="$PROBE_PYTHONPATH" "$PYTHON" - "$CONFIG_FILE" <<'PY'
import sys
from grain_sampling_local.config import LocalConfig
c = LocalConfig.load(sys.argv[1])
c.live_preflight()
if c.navigation_mode != 'operator':
    raise SystemExit('此一键脚本仅支持 operator 人工到位模式')
for value in (c.stm32_port, c.x2p_port, str(c.socket_path), str(c.directory), str(c.lift_rpm)):
    if '\n' in value:
        raise SystemExit('配置路径不能包含换行')
    print(value)
PY
)" || exit 1
mapfile -t SETTINGS <<< "$PREFLIGHT"
STM32_PORT="$(realpath -e -- "${SETTINGS[0]}")"
X2P_PORT="$(realpath -e -- "${SETTINGS[1]}")"
SOCKET="${SETTINGS[2]}"
STATE_DIR="${SETTINGS[3]}"
LIFT_RPM="${SETTINGS[4]}"

if [[ -e "$STATE_DIR/owner.lock" ]] && ! flock -n "$STATE_DIR/owner.lock" -c true; then
    echo "已有本机硬件服务持有独占锁；不会启动第二份。" >&2
    exit 1
fi
if fuser -s "$STM32_PORT" 2>/dev/null || fuser -s "$X2P_PORT" 2>/dev/null; then
    echo "STM32 或 X2P 串口已被使用；不会替换现有联调服务。" >&2
    exit 1
fi
if [[ -S "$SOCKET" ]] && PYTHONPATH="$PROBE_PYTHONPATH" "$PYTHON" - "$SOCKET" <<'PY' >/dev/null 2>&1
import sys
from grain_sampling_workflow.robot_bridge import RobotClient
RobotClient(sys.argv[1]).request('status')
PY
then
    echo "已有本机硬件服务在运行；不会启动第二份。" >&2
    exit 1
fi

if (( ! ATTENDED )); then
    [[ -t 0 ]] || { echo "无交互终端时必须显式传入 --attended" >&2; exit 2; }
    read -r -p "现场有人监护，行程已清空，硬件急停和防坠可用？输入 YES 确认：" answer
    [[ "$answer" == YES ]] || { echo "未确认现场安全，取消启动。" >&2; exit 2; }
fi
if (( ! SUCTION_CONFIRMED )); then
    [[ -t 0 ]] || { echo "无交互终端时必须显式传入 --external-suction-confirmed" >&2; exit 2; }
    read -r -p "吸粮风机由现行固件/现场外部控制且人员已确认？输入 YES 确认：" answer
    [[ "$answer" == YES ]] || { echo "未确认外部吸粮处理，取消启动。" >&2; exit 2; }
fi

umask 077
mkdir -p -- "$STATE_DIR"
exec 9>"$STATE_DIR/operator-bench.lock"
flock -n 9 || { echo "另一份一键联调脚本正在运行。" >&2; exit 1; }
# Repeat under the launcher lock; never interrupt a concurrent starter.
if ! flock -n "$STATE_DIR/owner.lock" -c true; then
    echo "确认期间硬件服务已启动；取消启动。" >&2
    exit 1
fi
if fuser -s "$STM32_PORT" 2>/dev/null || fuser -s "$X2P_PORT" 2>/dev/null; then
    echo "确认期间串口变为占用；取消启动。" >&2
    exit 1
fi
LOG_DIR="$STATE_DIR/bench/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p -- "$LOG_DIR"

stop_owned_process() {
    local pid="$1" label="$2" tries
    [[ -n "$pid" ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
        for ((tries=0; tries<50; tries++)); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
    fi
    if kill -0 "$pid" 2>/dev/null; then
        echo "$label 未在 5 秒内退出（PID $pid）；未强杀，请现场核查并使用硬件急停。" >&2
    else
        wait "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    local result=$?
    trap - EXIT INT TERM
    if [[ -n "$DAEMON_PID" ]] && kill -0 "$DAEMON_PID" 2>/dev/null; then
        # A normal UI close also stops; an abnormal exit gets the same stop request.
        if ! "$PYTHON" - "$SOCKET" <<'PY' >>"$LOG_DIR/launcher.log" 2>&1
import json, socket, sys
with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
    conn.settimeout(2)
    conn.connect(sys.argv[1])
    conn.sendall(b'{"action":"estop"}\n')
    reply = json.loads(conn.makefile('rb').readline())
    if not reply.get('ok'):
        raise RuntimeError(reply.get('error', 'stop rejected'))
PY
        then
            echo "停止请求未获确认；立即检查硬件急停和机构状态。" >&2
        fi
    fi
    stop_owned_process "$UI_PID" UI
    stop_owned_process "$DAEMON_PID" "硬件服务"
    echo "联调会话结束；日志：$LOG_DIR"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

QT_LIB_DIR="$PROJECT_DIR/.local-qt-deps/extracted/usr/lib/x86_64-linux-gnu"
if [[ -d "$QT_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$QT_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export QT_QPA_PLATFORM=xcb
export GRAIN_SAMPLING_UI_RC_PUBLISH=0
export GRAIN_UI_LOG_DIR="$LOG_DIR/ui-sessions"
echo "启动联想 USB 直连联调：STM32=$STM32_PORT，X2P=$X2P_PORT，升降=$LIFT_RPM r/min"
echo "人工到位模式；本脚本不会自动创建任务或控制底盘行驶。日志：$LOG_DIR"
cd "$PROJECT_DIR"
env -u PYTHONPATH "$PYTHON" -u LENOVO/scripts/local_robot.py serve --config "$CONFIG_FILE" \
    --live --attended --external-suction-confirmed >"$LOG_DIR/daemon.log" 2>&1 &
DAEMON_PID=$!

ready=0
for ((attempt=0; attempt<100; attempt++)); do
    if ! kill -0 "$DAEMON_PID" 2>/dev/null; then
        echo "硬件服务启动失败：" >&2
        tail -n 30 "$LOG_DIR/daemon.log" >&2
        exit 1
    fi
    if [[ -S "$SOCKET" ]] && PYTHONPATH="$PROBE_PYTHONPATH" "$PYTHON" - "$SOCKET" <<'PY' >>"$LOG_DIR/readiness.log" 2>&1
import sys
from grain_sampling_workflow.robot_bridge import RobotClient
s = RobotClient(sys.argv[1]).request('status')
if s.get('runtime') != 'lenovo-local' or s.get('simulation') or not s.get('test_navigation_mode'):
    raise SystemExit('wrong runtime, simulation or navigation mode')
PY
    then
        ready=1
        break
    fi
    sleep 0.1
done
if (( ! ready )); then
    echo "硬件服务未在 10 秒内就绪：$LOG_DIR/daemon.log" >&2
    if [[ -s "$LOG_DIR/readiness.log" ]]; then
        echo "就绪检查错误：" >&2
        tail -n 15 "$LOG_DIR/readiness.log" >&2
    fi
    exit 1
fi

env -u PYTHONPATH "$PYTHON" -u LENOVO/scripts/local_robot.py ui --config "$CONFIG_FILE" \
    >"$LOG_DIR/ui-stdout.log" 2>&1 &
UI_PID=$!
echo "UI 已启动；关闭窗口将停止本次硬件服务。不要关闭现场急停监护。"
set +e
wait "$UI_PID"
UI_RESULT=$?
set -e
UI_PID=""
if (( UI_RESULT != 0 )); then
    echo "UI 异常退出（$UI_RESULT）：" >&2
    tail -n 30 "$LOG_DIR/ui-stdout.log" >&2
fi
exit "$UI_RESULT"
