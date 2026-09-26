"""Protocol and fail-stop regression tests; no ROS or physical serial port."""
import binascii
import struct
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices.chassis_serial import ChassisSerial, ChassisError
from grain_sampling_workflow.chassis_node import ChassisNode, normalized_effort


class MCU:
    """Wire-level responder with firmware's enable-token semantics."""
    def __init__(self):
        self.rx = bytearray()
        self.epoch = 7
        self.sent = []
        self.reject = False
        self.silent = False
        self.short = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        frame = p.Parser().feed(data)[0]
        self.sent.append(frame)
        if self.short:
            return 1
        if self.silent:
            return len(data)
        if frame.kind == p.HELLO:
            kind, payload = p.HELLO_ACK, struct.pack("<IIB", 42, self.epoch, 1)
        else:
            if frame.kind in (p.AUTO_ARM, p.AUTO_STOP):
                self.epoch += 1
            kind = p.NACK if self.reject else p.ACK
            payload = struct.pack("<BBII", frame.kind, 3 if self.reject else 0,
                                  frame.sequence, self.epoch)
        self.rx.extend(p.encode(kind, frame.session, len(self.sent), payload))
        return len(data)

    def close(self):
        self.closed = True


def test_crc_reference_and_fixed_hello_vector():
    assert binascii.crc_hqx(b"123456789", 0xffff) == 0x29b1
    frame = p.encode(p.HELLO, 0x12345678, 1)
    assert frame[:14] == bytes.fromhex("a5 5a 01 01 00 00 78 56 34 12 01 00 00 00")


def test_parser_fragmentation_noise_bad_crc_and_coalescing():
    good = p.encode(p.HELLO, 123, 2)
    bad = bytearray(good)
    bad[-1] ^= 0xff
    parser = p.Parser()
    assert parser.feed(b"garbage" + bad + good[:7]) == []
    frames = parser.feed(good[7:] + good)
    assert len(frames) == 2
    assert frames[0].session == 123
    assert not parser.buffer


def test_status_layout_matches_c_offsets():
    payload = bytearray(52)
    struct.pack_into("<III", payload, 0, 42, 100, 8)
    payload[12:16] = bytes((3, 2, 5, 0))
    struct.pack_into("<hhHH", payload, 34, -250, 500, 1438, 1625)
    struct.pack_into("<II", payload, 42, 17, 19)
    payload[50] = 1
    status = p.decode_status(payload)
    assert status["epoch"] == 8
    assert (status["left"], status["right"]) == (-250, 500)
    assert (status["pwm_left"], status["pwm_right"]) == (1438, 1625)
    assert (status["crc_errors"], status["rejects"]) == (17, 19)


def test_handshake_arm_ack_updates_token_before_effort():
    mcu = MCU()
    link = ChassisSerial(mcu)
    link.request(p.HELLO)
    link.token_request(p.AUTO_ARM)
    link.effort(-500, 250)
    assert link.boot == 42
    assert struct.unpack("<Ihh", mcu.sent[-1].payload) == (8, -500, 250)
    link.close()
    assert mcu.sent[-1].kind == p.AUTO_STOP
    assert mcu.closed


@pytest.mark.parametrize("failure", ["reject", "silent", "short"])
def test_link_failures_are_reported(failure):
    mcu = MCU()
    link = ChassisSerial(mcu, timeout=0.001)
    link.request(p.HELLO)
    setattr(mcu, failure, True)
    with pytest.raises(ChassisError):
        link.token_request(p.AUTO_ARM)


def test_foreign_session_status_is_ignored():
    mcu = MCU()
    link = ChassisSerial(mcu)
    mcu.rx.extend(p.encode(p.STATUS, link.session ^ 1, 1, bytes(52)))
    assert link.receive() == []
    assert link.status is None


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_command_rejected(value):
    with pytest.raises(ValueError):
        normalized_effort(value, 0)


def test_effort_sign_and_saturation():
    assert normalized_effort(-0.5, 0.25) == (-500, 250)
    assert normalized_effort(3, -2) == (1000, -1000)


def node_stub():
    node = ChassisNode.__new__(ChassisNode)
    node.lock = threading.RLock()
    node.armed = True
    node.obstacle = False
    node.command = (400, 0)
    node.command_time = time.monotonic()
    node.arm_time = node.command_time
    node.connected_time = node.command_time
    node.link = MagicMock()
    node.link.status = {"mode": 2, "faults": 0, "flags": 5}
    node.link.status_time = node.command_time
    node.String = lambda **kw: kw
    node.Empty = lambda: None
    node.TriggerResponse = lambda **kw: SimpleNamespace(**kw)
    node.mode_pub = MagicMock()
    node.status_pub = MagicMock()
    node.cancel_pub = MagicMock()
    return node


def test_stale_navigation_stops_even_with_healthy_status(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.command_time -= 0.21
    node.tick()
    assert not node.armed and node.command is None
    node.link.send.assert_called_with(p.AUTO_STOP)
    node.link.effort.assert_not_called()
    node.cancel_pub.publish.assert_called_once()


def test_manual_takeover_cancels_and_drops_commands(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.link.status["mode"] = 1
    node.tick()
    node.set_command(1, 0)
    assert not node.armed and node.command is None
    node.link.effort.assert_not_called()


def test_arm_requires_healthy_status_and_does_not_reuse_motion():
    node = node_stub()
    result = node.service(p.AUTO_ARM)
    assert result.success and node.armed and node.command is None
    node.link.effort.assert_called_once_with(0, 0)
    node.link.token_request.assert_called_once_with(p.AUTO_ARM)


def test_obstacle_blocks_arm_and_does_not_resume():
    node = node_stub()
    node.on_obstacle(SimpleNamespace(data=True))
    assert not node.armed and node.command is None
    assert not node.service(p.AUTO_ARM).success
    node.on_obstacle(SimpleNamespace(data=False))
    assert not node.armed


def test_disconnect_cancels_without_replaying():
    node = node_stub()
    link = node.link
    node.disconnect()
    assert node.link is None and node.command is None and not node.armed
    link.close.assert_called_once()
    node.cancel_pub.publish.assert_called_once()


def test_status_loss_detected_with_fresh_navigation(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.link.status_time -= 0.21
    node.connected_time -= 0.21
    with pytest.raises(ChassisError, match="status timeout"):
        node.tick()
    node.link.effort.assert_not_called()


def test_mcu_reboot_detected():
    mcu = MCU()
    link = ChassisSerial(mcu)
    link.request(p.HELLO)
    payload = bytearray(52)
    struct.pack_into("<I", payload, 0, 43)
    mcu.rx.extend(p.encode(p.STATUS, link.session, 2, payload))
    with pytest.raises(ChassisError, match="restarted"):
        link.receive()


def test_short_write_during_disconnect_still_clears_connection():
    node = node_stub()
    node.link.close.side_effect = ChassisError("short serial write")
    node.disconnect()
    assert node.link is None and not node.armed and node.command is None


def test_goal_controller_serial_output_preserves_effort_units(monkeypatch):
    import importlib.util
    from pathlib import Path
    twist = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
    for name, module in {
        "rospy": MagicMock(),
        "geometry_msgs.msg": SimpleNamespace(Twist=twist, PoseStamped=object),
        "nav_msgs.msg": SimpleNamespace(Odometry=object),
        "std_msgs.msg": SimpleNamespace(Bool=object, Empty=object, String=object),
        "tf.transformations": SimpleNamespace(euler_from_quaternion=MagicMock()),
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[1] / "dipan" / "goal_controller.py"
    spec = importlib.util.spec_from_file_location("serial_goal_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    goal = module.GoalController.__new__(module.GoalController)
    goal.effort_pub = MagicMock()
    goal.cmd_debug_pub = MagicMock()
    goal.sock = MagicMock()
    goal.send_motor_command(0.4, -0.12)
    sent = goal.effort_pub.publish.call_args.args[0]
    assert (sent.linear.x, sent.angular.z) == (0.4, -0.12)
    goal.send_stop()
    sent = goal.effort_pub.publish.call_args.args[0]
    assert (sent.linear.x, sent.angular.z) == (0, 0)
    goal.sock.sendto.assert_not_called()


def test_missing_effort_ack_does_not_block_next_setpoint():
    mcu = MCU()
    link = ChassisSerial(mcu)
    link.request(p.HELLO)
    link.token_request(p.AUTO_ARM)
    mcu.silent = True
    link.clock = lambda: (_ for _ in ()).throw(AssertionError("must not wait"))
    first = link.effort(200, 0)
    assert link.effort(100, 50) == first + 1
    assert struct.unpack("<Ihh", mcu.sent[-1].payload) == (8, 100, 50)
    mcu.short = True
    with pytest.raises(ChassisError, match="short serial write"):
        link.effort(0, 0)


def test_new_goal_renews_session_before_arm():
    mcu = MCU()
    link = ChassisSerial(mcu)
    link.request(p.HELLO)
    previous = link.session
    link.renew_session()
    link.token_request(p.AUTO_ARM)
    assert link.session != previous
    assert [f.kind for f in mcu.sent] == [p.HELLO, p.HELLO, p.AUTO_ARM]


def test_stop_service_reports_sent_not_acknowledged():
    node = node_stub()
    result = node.service(p.AUTO_STOP)
    assert result.success and "not confirmed" in result.message
    node.link.send.assert_called_once_with(p.AUTO_STOP)
    node.link.request.assert_not_called()
