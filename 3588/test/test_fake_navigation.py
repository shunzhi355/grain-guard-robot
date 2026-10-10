"""The bench fake navigator must never issue a chassis command."""
from __future__ import annotations

import json
import socket
import threading

import pytest

from grain_sampling_workflow.fake_navigation import FakeNavigationBridge


class Client:
    def __init__(self, **overrides):
        self.calls = []
        self.status = {"ok": True, "lenovo_online": False, "chassis": {
            "chassis_link": "online", "rc_mode": "auto", "motion_armed": False,
            "estop_latched": False, "faults": 0,
        }}
        self.status["chassis"].update(overrides)

    def request(self, action, **values):
        self.calls.append(action)
        assert action == "status"  # In particular: never send goal or motion.
        return self.status


def test_fake_navigation_rejects_unsafe_chassis(tmp_path):
    client = Client(motion_armed=True)
    bridge = FakeNavigationBridge(client=client, terminal_path=str(tmp_path / "nav.sock"))
    assert bridge.call_navigate(1, 2) is False
    assert "运动授权" in bridge.last_error
    assert client.calls == ["status"]


@pytest.mark.parametrize(("field", "value", "message"), [
    ("estop_latched", True, "急停仍锁定"),
    ("rc_mode", "manual", "自动档"),
    ("faults", 1, "故障码 1"),
])
def test_fake_navigation_explains_safety_interlock(tmp_path, field, value, message):
    client = Client(**{field: value})
    bridge = FakeNavigationBridge(client=client, terminal_path=str(tmp_path / "nav.sock"))
    assert bridge.call_navigate(1, 2) is False
    assert message in bridge.last_error
    assert client.calls == ["status"]


def test_fake_navigation_allows_chassis_obstacle_when_stationary(tmp_path):
    client = Client(obstacle_stop=True, obstacle=True)
    bridge = FakeNavigationBridge(client=client, terminal_path=str(tmp_path / "nav.sock"))
    assert bridge.call_navigate(1, 2) is False  # No operator terminal is running.
    assert "障碍物" not in bridge.last_error
    assert client.calls == ["status"]


def test_fake_navigation_requires_a_running_terminal(tmp_path):
    client = Client()
    bridge = FakeNavigationBridge(client=client, terminal_path=str(tmp_path / "nav.sock"))
    assert bridge.call_navigate(1, 2) is False
    assert client.calls == ["status"]


@pytest.mark.parametrize("approved", [True, False])
def test_fake_navigation_socket_response_without_hardware(monkeypatch, approved):
    from grain_sampling_workflow import fake_navigation

    class FakeSocket:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def settimeout(self, *_):
            pass

        def connect(self, *_):
            pass

        def sendall(self, raw):
            self.request = json.loads(raw)

        def recv(self, *_):
            return json.dumps({"goal_id": self.request["goal_id"],
                               "approved": approved}).encode() + b"\n"

    monkeypatch.setattr(fake_navigation.socket, "AF_UNIX", 1, raising=False)
    monkeypatch.setattr(fake_navigation.socket, "socket", lambda *_: FakeSocket())
    client = Client()
    bridge = FakeNavigationBridge(client=client, terminal_path="test.sock")
    assert bridge.call_navigate(1.25, -2.5) is approved
    assert client.calls == (["status", "status"] if approved else ["status"])


@pytest.mark.skipif(not hasattr(socket, "AF_UNIX"), reason="Unix sockets unavailable")
@pytest.mark.parametrize("approved", [True, False])
def test_fake_navigation_operator_confirmation(tmp_path, approved):
    path = str(tmp_path / "nav.sock")
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(path)
    listener.listen(1)
    received = []

    def operator():
        conn, _ = listener.accept()
        with conn:
            raw = bytearray()
            while b"\n" not in raw:
                raw.extend(conn.recv(4096))
            request = json.loads(raw)
            received.append(request)
            conn.sendall(json.dumps({"goal_id": request["goal_id"],
                                     "approved": approved}).encode() + b"\n")

    worker = threading.Thread(target=operator, daemon=True)
    worker.start()
    client = Client()
    bridge = FakeNavigationBridge(client=client, terminal_path=path)
    try:
        assert bridge.call_navigate(1.25, -2.5) is approved
        worker.join(timeout=2)
        assert not worker.is_alive()
        assert received[0]["type"] == "fake_nav_goal"
        assert received[0]["x_m"] == 1.25
        assert client.calls == (["status", "status"] if approved else ["status"])
    finally:
        listener.close()


def test_fake_navigation_refuses_real_lenovo_session(tmp_path):
    client = Client()
    client.status["lenovo_online"] = True
    bridge = FakeNavigationBridge(client=client, terminal_path=str(tmp_path / "nav.sock"))
    assert bridge.call_navigate(0, 0) is False
    assert "联想导航已连接" in bridge.last_error
