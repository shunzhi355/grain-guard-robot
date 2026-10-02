"""GRICP framing and the 3588 motion safety boundary."""
import os
import time
import io
from types import SimpleNamespace

import pytest

from grain_sampling_interhost.chassis_controller import ChassisController
from grain_sampling_interhost.protocol import (MOTION, MessageType, ProtocolError,
                                                Frame, crc32c, decode, decode_motion,
                                                encode, json_payload)
from grain_sampling_interhost.server import RobotServer
from grain_sampling_workflow.robot_bridge import RobotBridge
from grain_sampling_workflow.robot_bridge import RobotClient


def test_mechanism_ipc_timeout_covers_full_x2p_cycle(monkeypatch):
    from grain_sampling_workflow import robot_bridge

    class Socket:
        timeout = None

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def settimeout(self, value):
            self.timeout = value

        def connect(self, path):
            assert path == "/tmp/test-control.sock"

        def sendall(self, data):
            assert b'"action": "mechanism"' in data

        def makefile(self, *_):
            return io.BytesIO(b'{"ok": true}\n')

    conn = Socket()
    monkeypatch.setattr(robot_bridge.socket, "AF_UNIX", 1, raising=False)
    monkeypatch.setattr(robot_bridge.socket, "socket", lambda *_: conn)
    assert RobotClient(path="/tmp/test-control.sock").request("mechanism")["ok"]
    assert conn.timeout >= 600.0


class FakeLink:
    def __init__(self):
        self.commands = []
        self.serial = self

    def poll_mode(self):
        return "auto"

    def stream_control(self, kind):
        self.commands.append(("control", kind))

    def stream_effort(self, forward, turn):
        self.commands.append(("effort", forward, turn))

    def mechanism_command(self, command, device):
        self.commands.append(("mechanism", command, device))

    def close(self):
        pass


def test_crc_and_authenticated_motion_frame():
    assert crc32c(b"123456789") == 0xE3069283
    key = os.urandom(32)
    payload = MOTION.pack(123, 300, 0, -800, 100, 1, 0)
    frame = encode(MessageType.MOTION_COMMAND, 99, 7, payload, key=key)
    parsed = decode(frame, key=key, udp=True)
    assert parsed.session_id == 99
    assert decode_motion(parsed.payload) == (123, 300, 0, -800, 100)
    tampered = bytearray(frame)
    tampered[-1] ^= 1
    with pytest.raises(ProtocolError, match="HMAC"):
        decode(bytes(tampered), key=key, udp=True)


def test_motion_rejects_sideways_and_out_of_range():
    with pytest.raises(ProtocolError):
        decode_motion(MOTION.pack(1, 0, 1, 0, 100, 1, 0))
    with pytest.raises(ProtocolError):
        decode_motion(MOTION.pack(1, 301, 0, 0, 100, 1, 0))


def test_chassis_conversion_and_200ms_timeout():
    fake = FakeLink()
    controller = ChassisController(timeout_s=0.2)
    controller.link = fake
    controller.mode = "auto"
    controller.start()
    try:
        epoch = controller.arm("goal-1")
        controller.command(epoch, 150, 400)
        deadline = time.monotonic() + 0.15
        while ("effort", 500, 500) not in fake.commands and time.monotonic() < deadline:
            time.sleep(0.005)
        assert ("effort", 500, 500) in fake.commands
        time.sleep(0.25)
        assert controller.status()["motion_armed"] is False
        assert controller.status()["last_stop_reason"] == "motion timeout"
        with pytest.raises(RuntimeError):
            controller.command(epoch, 150, 400)
    finally:
        controller.close()


def test_estop_never_rearms_from_new_velocity():
    fake = FakeLink()
    controller = ChassisController()
    controller.link = fake
    controller.mode = "auto"
    epoch = controller.arm("goal-1")
    controller.estop()
    with pytest.raises(RuntimeError):
        controller.command(epoch, 10, 10)
    with pytest.raises(RuntimeError):
        controller.arm("goal-2")


def test_stm32_reboot_revokes_motion():
    fake = FakeLink()
    fake.mode_telemetry = SimpleNamespace(
        status={"boot": 1, "flags": 4, "faults": 0, "rc_age_ms": 20},
        received_at=time.monotonic())
    controller = ChassisController()
    controller.link = fake
    controller.mode = "auto"
    controller.start()
    try:
        time.sleep(0.06)
        controller.arm("goal-1")
        fake.mode_telemetry.status = {"boot": 2, "flags": 4, "faults": 0, "rc_age_ms": 20}
        fake.mode_telemetry.received_at = time.monotonic()
        deadline = time.monotonic() + 0.2
        while controller.status()["motion_armed"] and time.monotonic() < deadline:
            time.sleep(0.005)
        assert controller.status()["motion_armed"] is False
        assert controller.status()["last_stop_reason"] == "STM32 rebooted"
    finally:
        controller.close()


def test_rc_fault_recovery_does_not_restart_previous_goal():
    fake = FakeLink()
    fake.mode_telemetry = SimpleNamespace(
        status={"boot": 1, "flags": 4, "faults": 0, "rc_age_ms": 20},
        received_at=time.monotonic())
    controller = ChassisController()
    controller.link = fake
    controller.mode = "auto"
    controller.start()
    try:
        time.sleep(0.06)
        old_epoch = controller.arm("goal-1")
        controller.command(old_epoch, 150, 0)
        fake.mode_telemetry.status = {"boot": 1, "flags": 4, "faults": 1, "rc_age_ms": 20}
        fake.mode_telemetry.received_at = time.monotonic()
        deadline = time.monotonic() + 0.2
        while controller.status()["motion_armed"] and time.monotonic() < deadline:
            time.sleep(0.005)
        assert controller.status()["motion_armed"] is False
        assert controller.status()["last_stop_reason"] == "STM32 fault"
        fake.mode_telemetry.status = {"boot": 1, "flags": 4, "faults": 0, "rc_age_ms": 20}
        fake.mode_telemetry.received_at = time.monotonic()
        with pytest.raises(RuntimeError, match="previous goal stopped"):
            controller.arm("goal-1")
        with pytest.raises(RuntimeError):
            controller.command(old_epoch, 150, 0)
        controller.arm("goal-2")
    finally:
        controller.close()


def test_stale_stm32_telemetry_reopens_serial_without_rearming(monkeypatch):
    """USB re-enumeration may leave an open fd that never raises I/O errors."""
    from grain_sampling_interhost import chassis_controller as chassis_module

    monkeypatch.setattr(chassis_module, "TELEMETRY_RECONNECT_TIMEOUT_S", 0.06)
    monkeypatch.setattr(chassis_module, "ChassisSerial", lambda link: link)
    old = FakeLink()
    old.closed = False
    old.close = lambda: setattr(old, "closed", True)
    replacements = []

    def open_replacement():
        link = FakeLink()
        replacements.append(link)
        return link

    controller = ChassisController(serial_factory=open_replacement)
    controller.link = old
    controller.start()
    try:
        deadline = time.monotonic() + 0.5
        while not old.closed and time.monotonic() < deadline:
            time.sleep(0.01)
        assert old.closed
        assert controller.status()["motion_armed"] is False
        deadline = time.monotonic() + 0.8
        while not replacements and time.monotonic() < deadline:
            time.sleep(0.01)
        assert replacements
    finally:
        controller.close()


def test_nav_result_requires_zero_frames_and_stop():
    class Chassis:
        def __init__(self):
            self.stops = []
        def set_obstacle(self, blocked):
            pass
        def stop(self, reason):
            self.stops.append(reason)

    class Mechanism:
        estop_latched = False

    class FakeSession:
        id = 9
        rx_seq = 0
        goal_id = "g1"
        goal_accepted = True
        zero_frames = 2
        stop_seen = True

    chassis = Chassis()
    server = RobotServer(bind_ip="127.0.0.1", allowed_peer="127.0.0.1",
        cert="", key="", ca="", ipc_path="", chassis=chassis, mechanism=Mechanism())
    session = FakeSession()
    payload = json_payload({"goal_id": "g1", "result": "SUCCEEDED",
                            "final_position_error_m": 0.1})
    server._tcp_frame(session, Frame(MessageType.NAV_RESULT, 9, 1, 0, payload))
    assert chassis.stops == ["navigation result"]
    assert server.state["navigation"]["result"] == "FAILED"


def test_clear_estop_requires_healthy_idle_chassis_and_does_not_resume_goal():
    class Chassis:
        def __init__(self):
            self.faults = 0
            self.resets = 0

        def set_obstacle(self, blocked):
            pass

        def status(self):
            return {"chassis_link": "online", "rc_mode": "auto",
                    "rc_valid": True, "faults": self.faults,
                    "motion_armed": False}

        def clear_estop(self):
            self.resets += 1

    class Mechanism:
        def __init__(self):
            self.resets = []
            self.pending_origin = False

        def requires_mechanical_reset(self):
            return self.pending_origin

        def reset(self, grain, *, mechanical_reset_confirmed=False):
            self.resets.append((grain, mechanical_reset_confirmed))

    chassis = Chassis()
    mechanism = Mechanism()
    server = RobotServer(bind_ip="127.0.0.1", allowed_peer="127.0.0.1",
        cert="", key="", ca="", ipc_path="", chassis=chassis, mechanism=mechanism)
    chassis.faults = 1
    with pytest.raises(RuntimeError, match="healthy chassis"):
        server.local_request({"action": "clear_estop"})
    assert chassis.resets == 0

    chassis.faults = 0
    server.test_goal_id = "old-goal"
    with pytest.raises(RuntimeError, match="no active goal"):
        server.local_request({"action": "clear_estop"})
    assert chassis.resets == 0

    server.test_goal_id = None
    mechanism.pending_origin = True
    with pytest.raises(RuntimeError, match="incomplete lift return"):
        server.local_request({"action": "clear_estop"})
    assert chassis.resets == 0

    mechanism.pending_origin = False
    assert server.local_request({"action": "clear_estop", "grain": "稻谷"}) == {"ok": True}
    assert chassis.resets == 1
    assert mechanism.resets == [("稻谷", False)]
    assert server.state["navigation"] is None


def test_ui_waits_for_final_nav_result_not_status_label(monkeypatch):
    class Client:
        status_calls = 0

        def request(self, action, **values):
            if action == "goal":
                return {"ok": True, "goal_id": "g1"}
            assert action == "status"
            self.status_calls += 1
            if self.status_calls == 1:
                return {"ok": True, "slam": {"map_id": "map-1"},
                        "pose": {"localization_valid": True}}
            if self.status_calls == 2:
                return {"ok": True, "lenovo_online": True,
                        "navigation": {"goal_id": "g1", "state": "SUCCEEDED"}}
            return {"ok": True, "lenovo_online": True,
                    "navigation": {"goal_id": "g1", "result": "SUCCEEDED"}}

    monkeypatch.setattr(time, "sleep", lambda _: None)
    client = Client()
    assert RobotBridge(client=client).call_navigate(1.0, 2.0)
    assert client.status_calls == 3


def test_bridge_clear_estop_uses_explicit_local_action():
    class Client:
        def __init__(self):
            self.actions = []

        def request(self, action, **values):
            self.actions.append((action, values))
            return {"ok": True}

    client = Client()
    assert RobotBridge(client=client).call_clear_estop()
    assert client.actions == [("clear_estop", {"mechanical_reset_confirmed": True})]
