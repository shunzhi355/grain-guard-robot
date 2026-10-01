#!/usr/bin/env python3
"""Read-only guard before abandoning a closed fake-navigation UI session.

This deliberately accepts only the local bench task waiting at an arrival
prompt.  It never clears a task record or sends a motion/stop command.
"""

from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import sys
import time
from pathlib import Path


def fail(message: str) -> None:
    raise RuntimeError(message)


def ipc_request(path: str, action: str, **values: object) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(2.0)
        connection.connect(path)
        connection.sendall(
            (json.dumps({"action": action, **values}) + "\n").encode("utf-8")
        )
        with connection.makefile("rb") as stream:
            data = stream.readline(1024 * 1024)
    if not data:
        fail("旧守护进程没有响应")
    reply = json.loads(data)
    if not reply.get("ok"):
        fail(f"旧守护进程拒绝只读检查：{reply.get('error', 'unknown error')}")
    return reply


def daemon_socket(pid: int) -> str:
    argv = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
    argv = [item.decode("utf-8") for item in argv if item]
    if "--ipc" not in argv:
        fail("无法确定旧守护进程的本机控制套接字")
    index = argv.index("--ipc")
    if index + 1 >= len(argv):
        fail("旧守护进程的 IPC 参数不完整")
    path = argv[index + 1]
    if not path.startswith("/tmp/grain-bench.") or not path.endswith("/control.sock"):
        fail("旧守护进程不是一键假导航联调会话")
    return path


def ui_window_exists(pid: int) -> bool:
    result = subprocess.run(
        ["wmctrl", "-lp"], capture_output=True, text=True, check=True, timeout=2.0
    )
    return any(
        len(parts := line.split(maxsplit=4)) >= 3 and parts[2] == str(pid)
        for line in result.stdout.splitlines()
    )


def ui_log(log_root: Path, pid: int) -> Path:
    matches = []
    for path in log_root.glob("*/ui-stdout.log"):
        try:
            first = path.open(encoding="utf-8", errors="replace").readline()
        except OSError:
            continue
        if f"-{pid}.log" in first:
            matches.append(path)
    if len(matches) != 1:
        fail("无法唯一对应旧 UI 日志，不能判断任务是否停在安全等待阶段")
    return matches[0]


def check(args: argparse.Namespace) -> None:
    marker = json.loads(args.marker.read_text(encoding="utf-8"))
    if marker.get("source") != "local" or not marker.get("task_id"):
        fail("仅允许自动收尾本地假导航工单")
    if ui_window_exists(args.ui_pid):
        fail("旧 UI 窗口仍打开；请在 UI 中结束任务，不能从启动脚本中断")

    log = ui_log(args.log_root, args.ui_pid)
    lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
    transitions = [
        (index, match.group(1))
        for index, line in enumerate(lines)
        if (match := re.search(r"FSM: SamplingState\.\w+ -> SamplingState\.(\w+)", line))
    ]
    if not transitions or transitions[-1][1] not in {"ARRIVED_PROMPT", "STOPPED", "COMPLETED"}:
        fail("旧工单仍可能执行流程动作，拒绝自动结束")
    last_transition = transitions[-1][0]
    if any(
        "[ERROR]" in line and "Report failed: cloud error" not in line
        for line in lines[last_transition:]
    ):
        fail("到位等待后出现错误日志，拒绝自动结束")
    last_press = max(
        (index for index, line in enumerate(lines) if "Mechanism press begin" in line),
        default=-1,
    )
    last_return = max(
        (index for index, line in enumerate(lines) if "Mechanism lift success" in line),
        default=-1,
    )
    if last_press > last_return:
        fail("上次下压没有已记录的完整回程，拒绝覆盖原点")
    mechanism_used = any(
        re.search(r"Mechanism (?!set_grain\b)\w+ begin", line)
        for line in lines
    )

    path = daemon_socket(args.daemon_pid)
    for _ in range(2):
        status = ipc_request(path, "status")
        chassis = status.get("chassis") or {}
        if (
            chassis.get("motion_armed") is not False
            or chassis.get("chassis_link") != "online"
            or chassis.get("estop_latched") is not False
        ):
            fail("底盘不满足静止、在线、无急停条件")
        # MechanismRuntime serializes actions with lift_health. A short IPC
        # timeout therefore rejects a still-running mechanism operation. If
        # navigation stopped before any actuation, RC may now be unavailable;
        # in that case the server intentionally blocks even this health call.
        if mechanism_used:
            ipc_request(path, "mechanism", name="lift_health")
        time.sleep(0.5)
    print(f"旧本地任务 {marker['task_id']} 已无 UI 窗口，处于非动作终态且静止检查通过")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ui-pid", type=int, required=True)
    parser.add_argument("--daemon-pid", type=int, required=True)
    parser.add_argument("--marker", type=Path, required=True)
    parser.add_argument("--log-root", type=Path, required=True)
    args = parser.parse_args()
    try:
        check(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"不能自动结束旧工单：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
