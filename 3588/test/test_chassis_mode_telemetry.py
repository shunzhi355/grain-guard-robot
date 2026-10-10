"""Passive mode feedback must not change the production motion protocol."""
import ast
import json
import struct
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices.chassis_serial import ChassisSerial
from grain_sampling_devices.chassis_telemetry import ModeTelemetry
from grain_sampling_workflow.chassis_node import ChassisNode


def report(mode=1, flags=4, age=0, boot=42, sequence=1, session=0):
    payload = bytearray(p.STATUS_STRUCT.size)
    struct.pack_into("<III", payload, 0, boot, 1000, 1)
    payload[13:15] = bytes((mode, flags))
    struct.pack_into("<I", payload, 20, age)
    return p.encode(p.STATUS, session, sequence, payload)


@pytest.mark.parametrize("session", [0, 123456])
def test_passive_reports_accept_manual_and_auto_without_host_session(session):
    clock = [0.0]
    telemetry = ModeTelemetry(lambda: clock[0])
    assert telemetry.mode() == ""
    telemetry.feed(report(session=session))
    assert telemetry.mode() == "manual"
    telemetry.feed(report(mode=2, sequence=2, session=session))
    assert telemetry.mode() == "auto"
    clock[0] = 1.0
    assert telemetry.mode() == ""


@pytest.mark.parametrize("mode,flags,age", [(0, 4, 0), (3, 4, 0), (2, 0, 0), (2, 4, 300)])
def test_invalid_or_lost_rc_is_unknown(mode, flags, age):
    telemetry = ModeTelemetry()
    telemetry.feed(report(mode, flags, age))
    assert telemetry.mode() == ""


def test_fragmentation_crc_noise_and_wrong_packet_length():
    telemetry = ModeTelemetry()
    good = report(mode=2)
    bad = bytearray(good)
    bad[-1] ^= 1
    telemetry.feed(b"noise" + bad + p.encode(p.STATUS, 0, 1, bytes(51)) + good[:9])
    assert telemetry.mode() == ""
    telemetry.feed(good[9:])
    assert telemetry.mode() == "auto"


def test_actual_c_firmware_status_vector():
    # Captured from MDK-ARM/tests/test_mode_telemetry.c, using chassis_status().
    vector = bytes.fromhex(
        "a55a010a340000000000010000002a000000f4010000020000000101040000000000"
        "00000000ffffffffdc05dc05e80300000000dc05dc05000000000000000001007af3"
    )
    telemetry = ModeTelemetry()
    telemetry.feed(vector)
    assert telemetry.mode() == "manual"
    assert telemetry.status["ch8"] == 1000
    assert telemetry.status["boot"] == 42


def test_duplicate_out_of_order_wrap_and_reboot():
    clock = [0.0]
    telemetry = ModeTelemetry(lambda: clock[0])
    telemetry.feed(report(sequence=0xffffffff))
    clock[0] = 0.8
    telemetry.feed(report(sequence=0xffffffff))
    telemetry.feed(report(sequence=0xfffffffe, mode=2))
    clock[0] = 1.1
    assert telemetry.mode() == ""  # Duplicate reports do not refresh receipt time.
    telemetry.feed(report(sequence=0, mode=2))
    assert telemetry.mode() == "auto"
    telemetry.feed(report(sequence=1, boot=43))
    assert telemetry.mode() == "manual"


def test_serial_never_waits_for_report_and_bounds_each_read():
    port = MagicMock()
    port.in_waiting = 0
    link = ChassisSerial(port)
    assert link.poll_mode() == ""
    port.read.assert_not_called()
    port.write.assert_not_called()
    port.in_waiting = 10000
    port.read.return_value = report(mode=2)
    assert link.poll_mode() == "auto"
    port.read.assert_called_once_with(4096)
    port.write.assert_not_called()


@pytest.mark.parametrize("mode", ["manual", "auto", ""])
def test_node_publishes_actual_mode_without_motion_or_handshake(monkeypatch, mode):
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace())
    node = ChassisNode.__new__(ChassisNode)
    node.lock = threading.RLock()
    node.link = MagicMock()
    node.link.poll_mode.return_value = mode
    node.armed = False
    node.String = lambda **kw: SimpleNamespace(**kw)
    node.mode_pub = MagicMock()
    node.status_pub = MagicMock()
    node.tick()
    assert node.mode_pub.publish.call_args.args[0].data == mode
    status = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert status["rc_mode"] == mode and not status["execution_confirmed"]
    node.link.stream_effort.assert_not_called()
    node.link.stream_control.assert_not_called()
    node.link.request.assert_not_called()


def ui_methods():
    """Execute actual pure UI handlers with widget doubles (no PySide required)."""
    path = Path(__file__).resolve().parents[1] / "src/grain_sampling_ui/main.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MainWindow")
    names = {"_on_rc_mode_mirror", "_expire_rc_mode_mirror", "_update_manual_mode_ui"}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    clock = [0.0]
    namespace = {"time": SimpleNamespace(monotonic=lambda: clock[0]), "QColor": lambda value: value}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    window = SimpleNamespace(_rc_control=None, _rc_mode_mirror=None, _rc_mode_updated_at=float("-inf"),
                             _manual_btn=MagicMock(), _mode_dot=MagicMock(),
                             _mode_value=MagicMock(), _update_dot=MagicMock())
    window._update_manual_mode_ui = lambda: namespace["_update_manual_mode_ui"](window)
    return window, namespace, clock


@pytest.mark.parametrize("mode,label", [("manual", "手动"), ("auto", "自动"), ("", "未知"), ("invalid", "未知")])
def test_ui_only_valid_modes_can_display_automatic(mode, label):
    window, methods, clock = ui_methods()
    methods["_on_rc_mode_mirror"](window, mode)
    window._mode_value.setText.assert_called_with(label)


def test_ui_detects_disappearing_ros_publisher():
    window, methods, clock = ui_methods()
    methods["_on_rc_mode_mirror"](window, "auto")
    clock[0] = 1.5
    methods["_expire_rc_mode_mirror"](window)
    window._mode_value.setText.assert_called_with("未知")
    assert window._rc_mode_mirror is None
