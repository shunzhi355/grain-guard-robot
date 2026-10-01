#!/usr/bin/env bash
# One-command, ROS-free fake-navigation bench session on the 3588.
# Opening this session never starts a task or acknowledges a fake arrival.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
CONFIG_FILE="${GRAIN_HARDWARE_CONFIG:-$PROJECT_DIR/config/industrial_pc.env}"
RPM="${GRAIN_BENCH_LIFT_RPM:-800}"
DISPLAY="${DISPLAY:-:0}"
RUNTIME_DIR=""
DAEMON_PID=""
FAKE_TERMINAL_PID=""
UI_PID=""

usage() {
    cat <<'EOF'
用法：bash scripts/start_fake_navigation_bench.sh [--rpm 1-800]

在 3588 桌面打开假导航人工确认终端和真实机构 UI，并启动本机硬件守护进程。
默认升降转速为当前联调值 800 r/min；可显式传入 --rpm 30 等较低转速。
本脚本不会自动开始任务、发送假到位、清除急停或复位机构。
启动前会自动关闭同一项目的旧联调会话。旧 UI 窗口已关、工单已停止或停在静止等待阶段时，
会归档残留的本地测试工单；运动中、故障或状态不明时拒绝切换。
关闭 UI 后，本脚本会停止自己启动的进程。日志保存在 3588/log/bench/。
EOF
}

while (($#)); do
    case "$1" in
        --rpm)
            (($# >= 2)) || { echo "--rpm 缺少数值" >&2; exit 2; }
            RPM="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数：$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ "$RPM" =~ ^[0-9]+$ ]] && ((RPM >= 1 && RPM <= 800)) || {
    echo "升降转速必须是 1–800 r/min 的整数" >&2
    exit 2
}
[[ -f "$CONFIG_FILE" ]] || { echo "缺少配置：$CONFIG_FILE" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONFIG_FILE"
export PYTHONPATH="$PROJECT_DIR/src:$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
export DISPLAY

for program in python3 openssl lxterminal fuser; do
    command -v "$program" >/dev/null || { echo "缺少程序：$program" >&2; exit 1; }
done
[[ "${CHASSIS_BACKEND:-serial}" == serial ]] || {
    echo "仅支持 STM32 串口底盘后端" >&2; exit 1;
}
[[ -r "$CHASSIS_SERIAL_PORT" && -w "$CHASSIS_SERIAL_PORT" ]] || {
    echo "STM32 串口不可访问：$CHASSIS_SERIAL_PORT" >&2; exit 1;
}
[[ -r "$X2P_PORT" && -w "$X2P_PORT" ]] || {
    echo "X2P 串口不可访问：$X2P_PORT" >&2; exit 1;
}
[[ "$CHASSIS_SERIAL_PORT" != "$X2P_PORT" ]] || {
    echo "STM32 与 X2P 不能共用串口" >&2; exit 1;
}
if systemctl is-active --quiet grain-sampling.service 2>/dev/null; then
    echo "旧 grain-sampling.service 正在运行；先安全结束旧任务和服务。" >&2
    exit 1
fi
archive_stale_task=0

# Only replace processes launched from this project.  The persisted task
# marker is deliberately not removed: it prevents a one-key restart from
# silently aborting a live or interrupted mechanical workflow.
collect_old_bench_pids() {
    local cmdline_file pid cwd process_name
    old_supervisors=()
    old_ui=()
    old_fake_terminals=()
    old_daemons=()
    for cmdline_file in /proc/[0-9]*/cmdline; do
        [[ -r "$cmdline_file" ]] || continue
        pid="${cmdline_file#/proc/}"
        pid="${pid%/cmdline}"
        [[ "$pid" != "$$" ]] || continue
        cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null)" || continue
        [[ "$cwd" == "$PROJECT_DIR" ]] || continue
        process_argv=()
        mapfile -d '' -t process_argv < "$cmdline_file" || continue
        process_name="${process_argv[0]:-}"
        case "${process_name##*/}" in
            bash)
                [[ "${process_argv[1]:-}" == scripts/start_fake_navigation_bench.sh
                    || "${process_argv[1]:-}" == "$PROJECT_DIR/scripts/start_fake_navigation_bench.sh" ]] \
                    && old_supervisors+=("$pid")
                ;;
            python3*)
                if [[ "${process_argv[1]:-}" == -m
                    && "${process_argv[2]:-}" == grain_sampling_interhost.server ]]; then
                    old_daemons+=("$pid")
                elif [[ "${process_argv[1]:-}" == scripts/start_ui.py
                    || "${process_argv[1]:-}" == "$PROJECT_DIR/scripts/start_ui.py" ]]; then
                    old_ui+=("$pid")
                elif [[ "${process_argv[1]:-}" == scripts/fake_navigation_terminal.py
                    || "${process_argv[1]:-}" == "$PROJECT_DIR/scripts/fake_navigation_terminal.py" ]]; then
                    old_fake_terminals+=("$pid")
                fi
                ;;
        esac
    done
    return 0
}

collect_old_bench_pids
if ((${#old_supervisors[@]} + ${#old_ui[@]} + ${#old_fake_terminals[@]} + ${#old_daemons[@]})); then
    task_marker="$(getent passwd "$(id -u)" | cut -d: -f6)/.grain_robot/current_task.json"
    if [[ -f "$task_marker" ]]; then
        if ((${#old_ui[@]} != 1 || ${#old_daemons[@]} != 1)); then
            echo "未完成工单对应的旧 UI/守护进程不唯一，拒绝自动清理。" >&2
            exit 1
        fi
        python3 "$PROJECT_DIR/scripts/bench_stale_task_guard.py" \
            --ui-pid "${old_ui[0]}" --daemon-pid "${old_daemons[0]}" \
            --marker "$task_marker" --log-root "$PROJECT_DIR/log/bench" || exit 1
        archive_stale_task=1
    fi
    echo "清理同一项目的旧联调会话（仅停止进程，保留日志和任务数据）..."
    # The supervisor's EXIT trap shuts down its own UI, fake terminal and
    # daemon.  Orphans are stopped only after the supervisor has had time.
    for pid in "${old_supervisors[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for _ in {1..50}; do
        collect_old_bench_pids
        ((${#old_supervisors[@]} == 0)) && break
        sleep 0.1
    done
    if ((${#old_supervisors[@]})); then
        echo "旧联调会话未退出；不会强制结束正在运行的进程。" >&2
        exit 1
    fi
    collect_old_bench_pids
    for pid in "${old_ui[@]}" "${old_fake_terminals[@]}" "${old_daemons[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    for _ in {1..50}; do
        collect_old_bench_pids
        ((${#old_ui[@]} + ${#old_fake_terminals[@]} + ${#old_daemons[@]} == 0)) && break
        sleep 0.1
    done
    if ((${#old_ui[@]} + ${#old_fake_terminals[@]} + ${#old_daemons[@]})); then
        echo "旧联调进程未退出；不会强制抢占串口。" >&2
        exit 1
    fi
    if ((archive_stale_task)); then
        archive_dir="$(dirname "$task_marker")/abandoned_bench_tasks"
        mkdir -p "$archive_dir"
        archive_path="$archive_dir/current_task-$(date +%Y%m%d-%H%M%S)-$$.json"
        mv -- "$task_marker" "$archive_path"
        echo "旧本地测试任务已结束；恢复记录：$archive_path"
    fi
fi
if fuser -s "$CHASSIS_SERIAL_PORT" 2>/dev/null \
    || fuser -s "$X2P_PORT" 2>/dev/null; then
    echo "硬件串口仍被占用；不会抢占其他服务。" >&2
    exit 1
fi

umask 077
LOG_DIR="$PROJECT_DIR/log/bench/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$LOG_DIR"
RUNTIME_DIR="$(mktemp -d /tmp/grain-bench.XXXXXX)"
CONTROL_SOCKET="$RUNTIME_DIR/control.sock"
FAKE_SOCKET="$RUNTIME_DIR/fake_navigation.sock"

cleanup() {
    local result=$?
    trap - EXIT INT TERM
    for pid in "$UI_PID" "$FAKE_TERMINAL_PID" "$DAEMON_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    for pid in "$UI_PID" "$FAKE_TERMINAL_PID" "$DAEMON_PID"; do
        if [[ -n "$pid" ]]; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    if [[ "$RUNTIME_DIR" == /tmp/grain-bench.* ]]; then
        rm -f -- "$CONTROL_SOCKET" "$FAKE_SOCKET" \
            "$RUNTIME_DIR/server.crt" "$RUNTIME_DIR/server.key"
        rmdir -- "$RUNTIME_DIR" 2>/dev/null || true
    fi
    echo "联调会话已退出；日志：$LOG_DIR"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

openssl req -x509 -newkey rsa:2048 -nodes \
    -keyout "$RUNTIME_DIR/server.key" -out "$RUNTIME_DIR/server.crt" \
    -days 1 -subj /CN=grain-bench >"$LOG_DIR/certificate.log" 2>&1

echo "启动本机假导航联调；升降设定 $RPM r/min；日志：$LOG_DIR"
echo "真实机构已启用，但任务和每个假到位必须由现场人员在 UI/终端操作。"
echo "重启工控机不会使机构机械回零；开始任务前应确认实际起点和运动范围。"
cd "$PROJECT_DIR"
GRAIN_LIFT_RPM="$RPM" python3 -m grain_sampling_interhost.server \
    --bind-ip 127.0.0.1 --peer-ip 127.0.0.1 \
    --cert "$RUNTIME_DIR/server.crt" --key "$RUNTIME_DIR/server.key" \
    --ca "$RUNTIME_DIR/server.crt" --ipc "$CONTROL_SOCKET" \
    >"$LOG_DIR/daemon.log" 2>&1 &
DAEMON_PID=$!

for _ in {1..100}; do
    [[ -S "$CONTROL_SOCKET" ]] && break
    if ! kill -0 "$DAEMON_PID" 2>/dev/null; then
        echo "守护进程启动失败：" >&2
        tail -n 30 "$LOG_DIR/daemon.log" >&2
        exit 1
    fi
    sleep 0.1
done
[[ -S "$CONTROL_SOCKET" ]] || {
    echo "守护进程启动超时：$LOG_DIR/daemon.log" >&2
    exit 1
}

GRAIN_ROBOT_SOCKET="$CONTROL_SOCKET" python3 -c '
from grain_sampling_workflow.robot_bridge import RobotClient
c = RobotClient().request("status")["chassis"]
print("底盘状态：RC=%s，有效=%s，故障=%s，急停=%s" %
      (c["rc_mode"], c["rc_valid"], c["faults"], c["estop_latched"]))
if c["rc_mode"] != "auto" or not c["rc_valid"] or c["faults"] != 0 or c["estop_latched"]:
    print("当前不满足实机任务条件；界面仅准备就绪，安全检查不会被绕过。")
'

lxterminal --no-remote --title="假导航人工确认" \
    --working-directory="$PROJECT_DIR" \
    --command="env GRAIN_FAKE_NAV_SOCKET=$FAKE_SOCKET python3 scripts/fake_navigation_terminal.py" \
    >"$LOG_DIR/fake-terminal.log" 2>&1 &
FAKE_TERMINAL_PID=$!
for _ in {1..100}; do
    [[ -S "$FAKE_SOCKET" ]] && break
    if ! kill -0 "$FAKE_TERMINAL_PID" 2>/dev/null; then
        echo "假导航终端启动失败：$LOG_DIR/fake-terminal.log" >&2
        exit 1
    fi
    sleep 0.1
done
[[ -S "$FAKE_SOCKET" ]] || {
    echo "假导航终端启动超时：$LOG_DIR/fake-terminal.log" >&2
    exit 1
}

GRAIN_ROBOT_SOCKET="$CONTROL_SOCKET" \
GRAIN_FAKE_NAV_SOCKET="$FAKE_SOCKET" \
GRAIN_SAMPLING_UI_FAKE_NAVIGATION=1 \
GRAIN_SAMPLING_UI_ENABLE_MECHANISM=1 \
GRAIN_SAMPLING_UI_SKIP_MAPPING=1 \
GRAIN_SAMPLING_UI_RC_PUBLISH=0 \
python3 scripts/start_ui.py >"$LOG_DIR/ui-stdout.log" 2>&1 &
UI_PID=$!
echo "UI 已启动。关闭 UI 后会停止本次守护进程；测试中不要关闭窗口。"
set +e
wait "$UI_PID"
UI_RESULT=$?
set -e
if ((UI_RESULT != 0)); then
    echo "UI 异常退出（$UI_RESULT）：$LOG_DIR/ui-stdout.log" >&2
    tail -n 30 "$LOG_DIR/ui-stdout.log" >&2
fi
exit "$UI_RESULT"
