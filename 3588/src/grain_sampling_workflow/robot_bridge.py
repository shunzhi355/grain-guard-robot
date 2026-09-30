"""ROS-free workflow API over the 3588 robot daemon's local socket."""
from __future__ import annotations

import json
import os
import socket
import time
from typing import Any


class RobotClient:
    def __init__(self, path: str | None = None):
        self.path = path or os.getenv("GRAIN_ROBOT_SOCKET", "/run/grain-robot/control.sock")

    def request(self, action: str, **values: Any) -> dict:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
            conn.settimeout(180.0 if action == "mechanism" else 10.0)
            conn.connect(self.path)
            conn.sendall(json.dumps({"action": action, **values}, allow_nan=False).encode() + b"\n")
            with conn.makefile("rb") as stream:
                raw = stream.readline(1024 * 1024)
        if not raw:
            raise ConnectionError("robot daemon did not respond")
        reply = json.loads(raw)
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "robot daemon rejected request"))
        return reply


class RobotBridge:
    NAV_TIMEOUT_SEC = 120.0

    def __init__(self, node_name: str = "robot_bridge", client: RobotClient | None = None):
        self.client = client or RobotClient()
        self.last_error = ""

    def _request(self, action: str, **values: Any) -> bool:
        try:
            self.client.request(action, **values)
            self.last_error = ""
            return True
        except (OSError, RuntimeError, ValueError) as exc:
            self.last_error = str(exc)
            return False

    def call_navigate(self, x: float, y: float) -> bool:
        try:
            initial_status = self.client.request("status")
            test_mode = initial_status.get("test_navigation_mode") is True
            if test_mode:
                response = self.client.request("test_goal", pose={"x_m": float(x),
                    "y_m": float(y)})
                goal_id = response["goal_id"]
                deadline = time.monotonic() + self.NAV_TIMEOUT_SEC
                while time.monotonic() < deadline:
                    status = self.client.request("status")
                    nav = status.get("navigation") or {}
                    if nav.get("goal_id") == goal_id:
                        if nav.get("result") == "SUCCEEDED" and nav.get("test_only") is True:
                            self.last_error = ""
                            return True
                        if nav.get("result") in ("FAILED", "CANCELED"):
                            self.last_error = nav.get("message", "test navigation stopped")
                            return False
                    time.sleep(0.2)
                self.client.request("cancel")
                self.last_error = "operator arrival confirmation timed out"
                return False
            ready_deadline = time.monotonic() + 15
            map_id = None
            status = initial_status
            while time.monotonic() < ready_deadline:
                slam = status.get("slam") or {}
                pose = status.get("pose") or {}
                if pose.get("localization_valid") and slam.get("map_id"):
                    map_id = slam["map_id"]
                    break
                time.sleep(0.2)
                status = self.client.request("status")
            if not map_id:
                raise RuntimeError("Lenovo map or localization is not ready")
            response = self.client.request("goal", goal={"frame_id": "map",
                "map_id": map_id,
                "pose": {"x_m": float(x), "y_m": float(y), "yaw_rad": 0.0}})
            goal_id = response["goal_id"]
            deadline = time.monotonic() + self.NAV_TIMEOUT_SEC
            while time.monotonic() < deadline:
                status = self.client.request("status")
                nav = status.get("navigation") or {}
                if not status.get("lenovo_online"):
                    self.last_error = "Lenovo navigation connection lost"
                    return False
                if nav.get("goal_id") == goal_id:
                    outcome = nav.get("result")
                    if outcome == "SUCCEEDED":
                        self.last_error = ""
                        return True
                    if outcome in ("FAILED", "CANCELED"):
                        self.last_error = nav.get("message", outcome)
                        return False
                time.sleep(0.2)
            self.client.request("cancel")
            self.last_error = "navigation timed out"
            return False
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
            return False

    def record_start_position(self) -> tuple[float, float] | None:
        try:
            pose = self.client.request("status").get("pose") or {}
            return (float(pose["x_m"]), float(pose["y_m"]))
        except (OSError, RuntimeError, ValueError, KeyError):
            return None

    def cancel_goal(self) -> bool:
        return self._request("cancel")

    def call_emergency_stop(self) -> bool:
        return self._request("estop")

    def publish_cmd_vel(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> bool:
        if linear_mps or angular_rps:
            self.last_error = "3588 does not originate navigation velocity"
            return False
        return self.cancel_goal()

    def shutdown(self):
        pass

    def _mechanism(self, name: str, **args: Any) -> bool:
        return self._request("mechanism", name=name, args=args)

    def call_set_grain(self, grain: str) -> bool:
        return self._request("mechanism", name="set_grain", grain=grain)

    def call_move_lift(self, direction: str, distance_cm: float) -> bool:
        return self._mechanism("move_lift", direction=direction, distance_cm=distance_cm)

    def call_lift_health(self) -> bool:
        # A precise health check is implemented by the mechanism daemon.
        return self._mechanism("lift_health")

    def call_open_bin(self, depth_level: int) -> bool:
        return self._mechanism("open_bin", depth=("shallow", "mid", "deep")[int(depth_level)])

    def call_close_bin(self, depth_level: int) -> bool:
        return self._mechanism("close_bin", depth=("shallow", "mid", "deep")[int(depth_level)])

    def call_hold_bin_open(self, depth_level: int) -> bool:
        return self._mechanism("hold_bin_open", depth=("shallow", "mid", "deep")[int(depth_level)])


for _name in ("clamp", "unclamp", "tighten", "untighten", "press", "lift",
              "start_suction", "stop_suction", "convey", "start_convey",
              "stop_convey", "close_all_bins"):
    setattr(RobotBridge, "call_" + _name,
            lambda self, name=_name: self._mechanism(name))
