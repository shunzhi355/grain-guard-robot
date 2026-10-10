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


def test_commands_share_sequence_without_handshake_or_ack():
    mcu = MCU()
    mcu.silent = True
    link = ChassisSerial(mcu)
    assert link.stream_effort(-500, 250) == 1
    assert link.mechanism_command(1, 6) == 2
    assert struct.unpack("<hh", mcu.sent[0].payload) == (-500, 250)
    link.close()
    assert [f.sequence for f in mcu.sent] == [1, 2, 3]
    assert mcu.sent[-1].kind == p.STREAM_CONTROL
    assert mcu.closed


def test_short_write_is_reported_without_waiting_for_reply():
    mcu = MCU()
    mcu.short = True
    link = ChassisSerial(mcu)
    with pytest.raises(ChassisError, match="short serial write"):
        link.stream_control(p.CLEAR_ESTOP)


def test_unsolicited_nack_does_not_control_next_command():
    mcu = MCU()
    link = ChassisSerial(mcu)
    mcu.reject = True
    link.stream_effort(100, 0)
    assert link.poll_mode() == ""
    link.stream_effort(200, 0)
    assert len(mcu.sent) == 2
    assert link.mode_telemetry.status is None


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
    node.link.poll_mode.return_value = ""
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
    node.link.stream_control.assert_called_with(p.AUTO_STOP)
    node.link.stream_effort.assert_not_called()
    node.cancel_pub.publish.assert_called_once()


def test_missing_feedback_does_not_block_stream(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.link.status["mode"] = 1
    node.tick()
    assert node.armed
    node.link.stream_effort.assert_called_once_with(400, 0)


def test_arm_requires_healthy_status_and_does_not_reuse_motion():
    node = node_stub()
    result = node.service(p.AUTO_ARM)
    assert result.success and node.armed and node.command is None
    node.link.stream_effort.assert_called_once_with(0, 0)
    node.link.request.assert_not_called()


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
    node.tick()
    node.link.stream_effort.assert_called_once_with(400, 0)


def test_mcu_reboot_is_passive_status_not_a_command_response():
    mcu = MCU()
    link = ChassisSerial(mcu)
    payload = bytearray(52)
    struct.pack_into("<I", payload, 0, 43)
    mcu.rx.extend(p.encode(p.STATUS, 0, 2, payload))
    link.poll_mode()
    assert link.mode_telemetry.status["boot"] == 43
    assert mcu.sent == []


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
    path = Path(__file__).resolve().parents[2] / "LENOVO" / "legacy" / "goal_controller.py"
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
    mcu.silent = True
    mcu.read = lambda n: pytest.fail("TX must not read or wait for ACK")
    first = link.stream_effort(200, 0)
    assert link.stream_effort(100, 50) == first + 1
    assert struct.unpack("<hh", mcu.sent[-1].payload) == (100, 50)
    mcu.short = True
    with pytest.raises(ChassisError, match="short serial write"):
        link.stream_effort(0, 0)


def test_new_connection_gets_fresh_sender_without_hello():
    mcu = MCU()
    link = ChassisSerial(mcu)
    link.stream_effort(100, 0)
    replacement = ChassisSerial(mcu)
    replacement.stream_effort(200, 0)
    assert replacement.session != link.session
    assert [f.kind for f in mcu.sent] == [p.STREAM_EFFORT, p.STREAM_EFFORT]


def test_stop_service_reports_sent_not_acknowledged():
    node = node_stub()
    result = node.service(p.AUTO_STOP)
    assert result.success and "unconfirmed" in result.message
    node.link.stream_control.assert_called_once_with(p.AUTO_STOP)
    node.link.request.assert_not_called()


def test_one_way_frames_need_no_reads_or_handshake():
    mcu = MCU()
    mcu.silent = True
    mcu.read = lambda n: (_ for _ in ()).throw(AssertionError("unexpected read"))
    link = ChassisSerial(mcu)
    link.stream_effort(200, -50)
    link.close()
    assert [f.kind for f in mcu.sent] == [p.STREAM_EFFORT, p.STREAM_CONTROL]
    assert struct.unpack("<hh", mcu.sent[0].payload) == (200, -50)
    assert mcu.sent[1].payload == bytes([p.AUTO_STOP])


def test_navigation_zero_immediately_sends_stop_without_disarming_local_gate():
    node = node_stub()
    node.set_command(0, 0)
    node.link.stream_control.assert_called_once_with(p.AUTO_STOP)
    assert node.armed and node.command == (0, 0)


def test_zero_navigation_is_not_reissued_as_arming_motion(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.command = (0, 0)
    node.tick()
    node.link.stream_control.assert_called_once_with(p.AUTO_STOP)
    node.link.stream_effort.assert_not_called()


def test_arrival_idle_preserves_workflow_status(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = node_stub()
    node.command = (0, 0)
    node.command_time -= 0.21
    node.tick()
    assert not node.armed
    node.link.stream_control.assert_called_once_with(p.AUTO_STOP)
    node.cancel_pub.publish.assert_not_called()


@pytest.mark.parametrize("ending", ["arrived", "cancel", "odom_timeout", "obstacle"])
def test_navigation_to_one_way_wire_and_stop(monkeypatch, ending):
    import importlib.util
    from pathlib import Path

    class Stamp:
        def __init__(self, value=0): self.value = value
        @staticmethod
        def now(): return Stamp(10)
        def __sub__(self, other): return Stamp(self.value - other.value)
        def __eq__(self, other): return self.value == other.value
        def to_sec(self): return self.value

    twist = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
    ros = MagicMock()
    ros.get_param.side_effect = lambda name, default=None: default
    ros.Time = Stamp
    ros.ServiceException = RuntimeError
    monkeypatch.setenv("CHASSIS_BACKEND", "serial")
    for name, module in {
        "rospy": ros,
        "geometry_msgs.msg": SimpleNamespace(Twist=twist, PoseStamped=object),
        "nav_msgs.msg": SimpleNamespace(Odometry=object),
        "std_msgs.msg": SimpleNamespace(Bool=object, Empty=object, String=lambda data: data),
        "std_srvs.srv": SimpleNamespace(Trigger=object),
        "tf.transformations": SimpleNamespace(euler_from_quaternion=lambda q: (0, 0, 0)),
        "serial": SimpleNamespace(),
    }.items(): monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).resolve().parents[2] / "LENOVO" / "legacy" / "goal_controller.py"
    spec = importlib.util.spec_from_file_location("navigation_integration", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    goal = module.GoalController()
    node = node_stub()
    wire = MCU()
    wire.silent = True
    wire.read = lambda n: (_ for _ in ()).throw(AssertionError("one-way must not read"))
    node.link = ChassisSerial(wire)
    goal.effort_pub = SimpleNamespace(publish=node.on_effort)
    goal.arm_chassis = lambda: node.service(p.AUTO_ARM)
    goal.pose = (0, 0, 0)
    goal.odom_frame = "map"
    goal.last_odom_receive = Stamp(10)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id="map"), pose=SimpleNamespace(
        position=SimpleNamespace(x=2, y=0), orientation=SimpleNamespace(x=0,y=0,z=0,w=1)))
    try:
        goal.goal_callback(msg)
        goal.control_step()
        node.tick()
        assert wire.sent[-1].kind == p.STREAM_EFFORT
        assert struct.unpack("<hh", wire.sent[-1].payload)[0] > 0
        if ending == "arrived":
            goal.pose = (2, 0, 0)
            goal.control_step()
            assert goal.state == "ARRIVED"
        elif ending == "cancel":
            goal.cancel_callback(None)
            node.stop()  # Both nodes subscribe to /cancel_goal.
            assert goal.goal is None
        elif ending == "odom_timeout":
            goal.last_odom_receive = Stamp(9)
            goal.control_step()
            assert goal.state == "FAULT"
        else:
            goal.obstacle_callback(SimpleNamespace(data=True))
            node.on_obstacle(SimpleNamespace(data=True))
            goal.control_step()
        assert wire.sent[-1].kind == p.STREAM_CONTROL
        assert wire.sent[-1].payload == bytes([p.AUTO_STOP])
        assert all(f.kind in (p.STREAM_EFFORT, p.STREAM_CONTROL) for f in wire.sent)
    finally:
        goal.sock.close()
