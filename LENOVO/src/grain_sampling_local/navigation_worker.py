"""Recoverable local navigation worker, independent of ROS and UART.

Registration and sensor input continue under a safety latch. Neither registration
nor reconnect clears a latch, arms a lease, recreates or resubmits a goal.
"""
from __future__ import annotations
import json
import socket
import time


class NavigationSocketClient:
    """Reuse the existing local protocol; bound stalls below the motion lease.

    This socket carries only navigator requests. UI/emergency stop keep their
    independent connections. A failed exchange discards the connection and
    never retries a velocity frame or restores a goal.
    """
    def __init__(self, path, *, timeout_s=0.12):
        self.path = path
        self.timeout_s = timeout_s
        self.conn = None
        self.stream = None

    def close(self):
        if self.stream is not None:
            self.stream.close()
        if self.conn is not None:
            self.conn.close()
        self.stream = self.conn = None

    def request(self, action, **values):
        if action not in ("nav_register", "nav_inputs", "nav_velocity", "nav_fault"):
            raise ValueError("navigation-only local connection")
        try:
            if self.conn is None:
                self.conn = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.conn.settimeout(self.timeout_s)
                self.conn.connect(self.path)
                self.stream = self.conn.makefile("rb")
            self.conn.sendall(json.dumps({"action": action, **values}, allow_nan=False).encode() + b"\n")
            raw = self.stream.readline(1024 * 1024)
            if not raw:
                raise ConnectionError("local navigation service disconnected")
            reply = json.loads(raw)
            if not isinstance(reply, dict) or not reply.get("ok"):
                raise RuntimeError(reply.get("error", "local navigation request rejected")
                                   if isinstance(reply, dict) else "invalid local navigation reply")
            return reply
        except (OSError, RuntimeError, ValueError):
            self.close()
            raise


class NavigationWorker:
    def __init__(self, client, navigator):
        self.client, self.navigator = client, navigator
        self.token = None
        self.sequence = 0
        self.last_error = ""

    def _request(self, action, **values):
        self.sequence += 1
        return self.client.request(action, token=self.token, sequence=self.sequence, **values)

    def tick(self, snapshot):
        try:
            if self.token is None:
                self.token = self.client.request("nav_register")["token"]
                self.sequence = 0
                self.navigator.reset()
            reply = self._request("nav_inputs", inputs=snapshot)
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
            self.token = None
            self.navigator.reset()
            return None
        goal = reply.get("goal")
        if reply.get("latched") or not goal:
            self.navigator.reset()
            self.last_error = ""
            return reply
        try:
            forward, turn, arrived = self.navigator.step(goal, snapshot)
        except (RuntimeError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
            self.navigator.reset()
            try:
                self._request("nav_fault", reason=str(exc)[:256])
            except (OSError, RuntimeError, ValueError):
                self.token = None  # daemon watchdog independently revokes motion
            return reply
        try:
            self._request("nav_velocity", goal_id=goal["goal_id"], created_at=time.monotonic(),
                          forward=forward, turn=turn, arrived=arrived)
            self.last_error = ""
        except (OSError, RuntimeError, ValueError) as exc:
            # A rejected command stops/revokes at the service. Keep sending
            # sensor inputs so the UI can inspect and explicitly reset it.
            self.last_error = str(exc)
            self.navigator.reset()
        return reply
