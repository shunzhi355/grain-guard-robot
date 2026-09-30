"""Operator-confirmed, zero-motion navigation substitute for 3588 bench tests.

Only the UI selects this bridge with an explicit environment flag.  It never
arms the chassis, sends a GRICP frame, or publishes a velocity command.
"""
from __future__ import annotations

import json
import math
import os
import socket
import threading
import time
import uuid

from grain_sampling_workflow.robot_bridge import RobotBridge


DEFAULT_SOCKET = "/run/grain-robot/fake_navigation.sock"


class FakeNavigationBridge(RobotBridge):
    def __init__(self, *args, terminal_path: str | None = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.terminal_path = terminal_path or os.getenv("GRAIN_FAKE_NAV_SOCKET", DEFAULT_SOCKET)
        self._cancelled = threading.Event()

    def _stationary(self) -> None:
        status = self.client.request("status")
        chassis = status.get("chassis") or {}
        if status.get("lenovo_online"):
            raise RuntimeError("联想导航已连接，禁止启用假导航")
        if (chassis.get("chassis_link") != "online"
                or chassis.get("rc_mode") != "auto"
                or chassis.get("motion_armed")
                or chassis.get("estop_latched")
                or chassis.get("faults")):
            raise RuntimeError("底盘未满足静止联调条件：串口在线、自动档、未授权运动、无急停/故障")

    def record_start_position(self) -> tuple[float, float] | None:
        # A bench test has no Lenovo pose.  This coordinate is displayed only
        # at the final fake return prompt; it is never sent to the chassis.
        return (0.0, 0.0)

    def call_navigate(self, x: float, y: float) -> bool:
        self._cancelled.clear()
        try:
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("假导航目标坐标无效")
            self._stationary()
            goal_id = str(uuid.uuid4())
            request = {"type": "fake_nav_goal", "goal_id": goal_id,
                       "x_m": float(x), "y_m": float(y)}
            deadline = time.monotonic() + 300.0
            if not hasattr(socket, "AF_UNIX"):
                raise RuntimeError("假导航终端需要 3588 上的 Linux Unix Socket")
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
                conn.settimeout(1.0)
                conn.connect(self.terminal_path)
                conn.sendall(json.dumps(request, allow_nan=False).encode("utf-8") + b"\n")
                conn.settimeout(0.25)
                raw = bytearray()
                while b"\n" not in raw:
                    if self._cancelled.is_set():
                        raise RuntimeError("假导航已取消")
                    if time.monotonic() >= deadline:
                        raise TimeoutError("等待终端确认到位超时（300 秒）")
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("假导航终端已断开")
                    raw.extend(chunk)
                    if len(raw) > 4096:
                        raise ValueError("假导航终端响应过长")
            response = json.loads(raw.split(b"\n", 1)[0])
            if not isinstance(response, dict) or response.get("goal_id") != goal_id:
                raise ValueError("假导航确认与当前目标不匹配")
            if response.get("approved") is not True or self._cancelled.is_set():
                raise RuntimeError("操作员未确认到位")
            self._stationary()  # Recheck immediately before the FSM can run mechanisms.
            self.last_error = ""
            return True
        except (OSError, RuntimeError, ValueError, TimeoutError, json.JSONDecodeError) as exc:
            self.last_error = str(exc)
            return False

    def cancel_goal(self) -> bool:
        self._cancelled.set()
        return super().cancel_goal()

    def call_emergency_stop(self) -> bool:
        self._cancelled.set()
        return super().call_emergency_stop()

    def shutdown(self):
        self._cancelled.set()
