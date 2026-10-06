"""Protocol vectors and real runtime -> shared UART -> PWM mirror regression."""
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import MagicMock

import pytest

from grain_sampling_devices import chassis_protocol as wire
from grain_sampling_devices import mechanism_protocol as mcu
from grain_sampling_devices.chassis_serial import ChassisSerial, ChassisError
from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_interhost.chassis_controller import ChassisController
from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from grain_sampling_interhost.server import RobotServer
from grain_sampling_workflow.mechanism_config import get_grain_params
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.state_machine import SamplingAction, SamplingState, SamplingStateMachine


class SerialRecorder:
    in_waiting = 0

    def __init__(self):
        self.writes = []
        self.closed = False
        self.short = False

    def write(self, packet):
        assert not self.closed
        self.writes.append(packet)
        return 1 if self.short else len(packet)

    def close(self):
        self.closed = True

    def frames(self):
        return wire.Parser().feed(b"".join(self.writes))


@pytest.mark.parametrize("sequence,command,device,expected", [
    (1, 1, 1, "A5 5A 01 36 02 00 78 56 34 12 01 00 00 00 01 01 AA DB"),
    (2, 1, 6, "A5 5A 01 36 02 00 78 56 34 12 02 00 00 00 01 06 AD 65"),
    (3, 1, 7, "A5 5A 01 36 02 00 78 56 34 12 03 00 00 00 01 07 2C 30"),
    (4, 1, 2, "A5 5A 01 36 02 00 78 56 34 12 04 00 00 00 01 02 C8 A8"),
    (4, 2, 2, "A5 5A 01 36 02 00 78 56 34 12 04 00 00 00 02 02 9B FD"),
    (5, 1, 3, "A5 5A 01 36 02 00 78 56 34 12 05 00 00 00 01 03 49 FD"),
    (6, 2, 3, "A5 5A 01 36 02 00 78 56 34 12 06 00 00 00 02 03 FA 66"),
    # The shared chat's STOP_ALL example has an incorrect CRC (EA 6B).
    (4, 3, 0, "A5 5A 01 36 02 00 78 56 34 12 04 00 00 00 03 00 E8 EE"),
])
def test_final_firmware_vectors_without_ack(sequence, command, device, expected):
    port = SerialRecorder()  # No read() method: sending must not wait for ACK.
    link = ChassisSerial(port)
    link.session, link.sequence = 0x12345678, sequence - 1
    assert link.mechanism_command(command, device) == sequence
    assert port.writes == [bytes.fromhex(expected)]


@pytest.mark.parametrize("command,device", [(0, 1), (4, 1), (1, 0), (1, 8),
    (2, 8), (3, 1), (3, -1), (True, 1), (1, False), (1, 1.0)])
def test_invalid_command_never_writes(command, device):
    port = SerialRecorder()
    link = ChassisSerial(port)
    with pytest.raises(ValueError):
        link.mechanism_command(command, device)
    assert port.writes == []
    assert link.sequence == 0


@pytest.fixture
def stack():
    port = SerialRecorder()
    chassis = ChassisController()
    chassis.link = ChassisSerial(port)
    chassis.mode = "auto"
    local = MechanismController(mock_mode=True)
    runtime = MechanismRuntime(local, serial_command=chassis.mechanism_command)
    yield port, chassis, local, runtime
    runtime.close()
    chassis.close()


@pytest.mark.parametrize("action,args,payloads,pca_channel,pca_value", [
    ("tighten", {}, [(1, 1)], 6, 1300),
    ("untighten", {}, [(2, 1)], 6, 1900),
    ("convey", {}, [(1, 2)], 0, 1200),
    ("start_convey", {}, [(1, 2)], 1, 1200),
    ("stop_convey", {}, [(2, 2)], 0, 1500),
    ("open_bin", {"depth": "shallow"}, [], 2, 1200),
    ("hold_bin_open", {"depth": "mid"}, [], 3, 1200),
    ("hold_bin_open", {"depth": "deep"}, [], 4, 1200),
    ("close_bin", {"depth": "deep"}, [], 4, 1800),
    ("open_bin_default", {}, [], 3, 1200),
    ("close_all_bins", {}, [], 2, 1800),
])
def test_runtime_sends_semantic_actions_and_retains_pca(
    stack, action, args, payloads, pca_channel, pca_value
):
    port, _, local, runtime = stack
    runtime.execute(action, **args)
    frames = port.frames()
    assert [(f.kind, tuple(f.payload)) for f in frames] == [(0x36, p) for p in payloads]
    assert local.pca9685.register_history[pca_channel][-1] == pca_value
    if action in ("start_convey", "convey", "stop_convey"):
        assert local.pca9685.register_history[0] == local.pca9685.register_history[1]
    if action == "hold_bin_open":
        for channel in {2, 3, 4} - {pca_channel}:
            assert local.pca9685.register_history[channel][-1] == 1800
    if action == "close_all_bins":
        assert all(local.pca9685.register_history[ch][-1] == 1800 for ch in (2, 3, 4))


def test_bin_doors_use_pca_even_without_stm32_link(stack):
    port, chassis, local, runtime = stack
    chassis.link = None
    runtime.execute("hold_bin_open", depth="shallow")
    runtime.execute("close_all_bins")
    assert port.writes == []
    assert [local.pca9685.register_history[ch][0] for ch in (2, 3, 4)] == [1200, 1800, 1800]
    assert all(local.pca9685.register_history[ch][-1] == 1800 for ch in (2, 3, 4))


@pytest.mark.parametrize("command,device", [(1, 3), (2, 3), (1, 4), (2, 5)])
def test_production_uart_rejects_bin_commands(stack, command, device):
    port, chassis, _, _ = stack
    with pytest.raises(ValueError, match="I2C PCA9685"):
        chassis.mechanism_command(command, device)
    assert port.writes == []


def test_bin_i2c_failure_latches_estop_without_bin_serial_start(stack, monkeypatch):
    port, _, local, runtime = stack

    def fail(**kwargs):
        raise OSError("I2C failed")

    monkeypatch.setattr(local, "hold_bin_open", fail)
    with pytest.raises(OSError, match="I2C failed"):
        runtime.execute("hold_bin_open", depth="mid")
    assert [f.payload for f in port.frames()] == [b"\x03\x00"]
    assert runtime.estop_latched and local._stop_flag.is_set()


@pytest.mark.parametrize("grain", ["稻谷", "黄豆"])
def test_formal_workflow_serial_conveys_only_after_sampling_for_grain_duration(stack, grain):
    port, _, local, runtime = stack
    bridge = MagicMock()

    def open_bin(depth):
        runtime.execute("hold_bin_open", depth=("shallow", "mid", "deep")[depth])
        return True

    def start_convey():
        runtime.execute("start_convey")
        return True

    def stop_convey():
        runtime.execute("stop_convey")
        return True

    def close_all_bins():
        runtime.execute("close_all_bins")
        return True

    bridge.call_hold_bin_open.side_effect = open_bin
    bridge.call_start_convey.side_effect = start_convey
    bridge.call_stop_convey.side_effect = stop_convey
    bridge.call_close_all_bins.side_effect = close_all_bins
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.DISCHARGE_WASTE
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._run_async = lambda fn: fn()
    orch._log_sampling_event = lambda *args: None
    orch.sampling_duration_sec = 7
    orch._grain = grain
    convey_sec = get_grain_params(grain)["convey_duration"]
    orch.set_convey_duration(convey_sec)
    waits = []

    def wait(seconds):
        waits.append(seconds)
        frames = [f.payload for f in port.frames()]
        if seconds == 5.0:
            assert fsm.current_state == SamplingState.OPEN_BIN
            assert frames == []
        elif seconds == 7:
            assert fsm.current_state == SamplingState.FORMAL_SAMPLING
            assert frames == []
        elif seconds == convey_sec:
            assert fsm.current_state == SamplingState.CONVEY_1
            assert frames == [b"\x01\x02"]
        elif seconds == 3.0:
            assert frames == [b"\x01\x02"]
            assert all(local.pca9685.register_history[ch][-1] == 1800
                       for ch in (2, 3, 4))
        elif seconds == 0.5:
            assert frames == [b"\x01\x02", b"\x02\x02"]
        return True

    orch._wait_interruptible = wait
    fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
    assert waits == [5.0, 7, convey_sec, 3.0, 0.5]
    assert [f.payload for f in port.frames()] == [
        b"\x01\x02", b"\x02\x02",
    ]
    assert fsm.current_state == SamplingState.CONVEY_DONE


def test_legacy_timed_convey_sends_serial_stop_after_grain_duration(stack, monkeypatch):
    port, _, local, runtime = stack
    timers = []

    def make_timer(seconds, callback):
        timer = MagicMock()
        timer.seconds = seconds
        timer.callback = callback
        timers.append(timer)
        return timer

    monkeypatch.setattr("grain_sampling_interhost.mechanism_controller.threading.Timer", make_timer)
    runtime.grain = "黄豆"
    runtime.execute("convey")
    assert len(timers) == 1
    assert timers[0].seconds == 90.0
    timers[0].start.assert_called_once()
    assert [f.payload for f in port.frames()] == [b"\x01\x02"]

    timers[0].callback()
    assert [f.payload for f in port.frames()] == [b"\x01\x02", b"\x02\x02"]
    assert local.pca9685.register_history[0][-1] == 1500
    assert local.pca9685.register_history[1][-1] == 1500


def test_manual_stop_cancels_legacy_convey_timer(stack, monkeypatch):
    port, _, _, runtime = stack
    timers = []

    def make_timer(seconds, callback):
        timer = MagicMock()
        timer.callback = callback
        timers.append(timer)
        return timer

    monkeypatch.setattr("grain_sampling_interhost.mechanism_controller.threading.Timer", make_timer)
    runtime.execute("convey")
    runtime.execute("stop_convey")
    timers[0].cancel.assert_called_once()
    timers[0].callback()  # A queued old callback must not send another STOP.
    assert [f.payload for f in port.frames()] == [b"\x01\x02", b"\x02\x02"]


def test_emergency_stop_cancels_legacy_convey_timer(stack, monkeypatch):
    port, _, _, runtime = stack
    timers = []

    def make_timer(seconds, callback):
        timer = MagicMock()
        timer.callback = callback
        timers.append(timer)
        return timer

    monkeypatch.setattr("grain_sampling_interhost.mechanism_controller.threading.Timer", make_timer)
    runtime.execute("convey")
    runtime.emergency_stop()
    timers[0].cancel.assert_called_once()
    timers[0].callback()
    assert [f.payload for f in port.frames()] == [b"\x01\x02", b"\x03\x00"]


def test_timed_convey_stops_local_outputs_without_serial(monkeypatch):
    timers = []

    def make_timer(seconds, callback):
        timer = MagicMock()
        timer.callback = callback
        timers.append(timer)
        return timer

    monkeypatch.setattr("grain_sampling_interhost.mechanism_controller.threading.Timer", make_timer)
    local = MechanismController(mock_mode=True)
    runtime = MechanismRuntime(local)
    try:
        runtime.execute("convey")
        assert local.pca9685.register_history[0][-1] == 1200
        assert local.pca9685.register_history[1][-1] == 1200
        timers[0].callback()
        assert local.pca9685.register_history[0][-1] == 1500
        assert local.pca9685.register_history[1][-1] == 1500
    finally:
        runtime.close()


@pytest.mark.parametrize("action,device,direction,duty", [
    ("clamp", 6, False, 9.5), ("unclamp", 7, True, 6.0),
])
def test_gripper_uses_final_numbering_and_preserves_drv8701(stack, action, device, direction, duty):
    port, _, local, runtime = stack
    runtime.execute(action)
    assert [f.payload for f in port.frames()] == [bytes((1, device))]
    assert local.pca9685.level_history[8][-1] == direction
    assert local.pca9685.duty_history[9][-1] == duty
    assert local.pca9685.level_history[10][-1] is True


@pytest.mark.parametrize("action,args", [("open_bin", {"depth": "invalid"}),
    ("hold_bin_open", {}), ("unknown", {}), ("move_lift", {"direction": "up", "distance_cm": 0})])
def test_rejected_requests_do_not_send_start(stack, action, args):
    port, _, _, runtime = stack
    with pytest.raises(ValueError):
        runtime.execute(action, **args)
    assert port.writes == []


def test_unmapped_suction_keeps_original_pca_path(stack):
    port, _, local, runtime = stack
    runtime.execute("start_suction")
    runtime.execute("stop_suction")
    assert port.writes == []
    assert local.pca9685.register_history[7][-1] == 1500


def test_emergency_stop_and_shutdown_send_all_stop_and_latch(stack):
    port, _, local, runtime = stack
    runtime.execute("start_convey")
    runtime.execute("clamp")
    runtime.emergency_stop()
    assert port.frames()[-1].payload == b"\x03\x00"
    assert local.pca9685.register_history[0][-1] == 1500
    assert local.pca9685.level_history[10][-1] is False
    count = len(port.writes)
    with pytest.raises(RuntimeError, match="latched"):
        runtime.execute("clamp")
    assert len(port.writes) == count
    runtime.close()
    assert port.frames()[-1].payload == b"\x03\x00"


def test_short_write_latches_without_starting_pca_or_replaying(stack):
    port, chassis, local, runtime = stack
    port.short = True
    with pytest.raises(ChassisError, match="short serial write"):
        runtime.execute("clamp")
    assert runtime.estop_latched
    assert chassis.link is None
    assert True not in local.pca9685.level_history[10]
    assert [f.payload for f in port.frames()] == [b"\x01\x06", b"\x03\x00"]
    with pytest.raises(RuntimeError, match="latched"):
        runtime.execute("clamp")


def test_pca_failure_after_serial_start_stops_both(stack, monkeypatch):
    port, _, local, runtime = stack

    def fail(**kwargs):
        raise OSError("I2C failed")

    monkeypatch.setattr(local, "clamp", fail)
    with pytest.raises(OSError, match="I2C failed"):
        runtime.execute("clamp")
    assert [f.payload for f in port.frames()] == [b"\x01\x06", b"\x03\x00"]
    assert runtime.estop_latched and local._stop_flag.is_set()


def test_estop_racing_with_start_leaves_stop_all_last(stack, monkeypatch):
    port, _, local, runtime = stack
    entered = threading.Event()
    release = threading.Event()
    stop_requested = threading.Event()
    original = local.clamp

    def delayed_clamp(**kwargs):
        entered.set()
        assert release.wait(2)
        original(**kwargs)

    def stop():
        stop_requested.set()
        runtime.emergency_stop()

    monkeypatch.setattr(local, "clamp", delayed_clamp)
    with ThreadPoolExecutor(max_workers=2) as pool:
        start = pool.submit(runtime.execute, "clamp")
        try:
            assert entered.wait(2)
            halt = pool.submit(stop)
            assert stop_requested.wait(2)
        finally:
            release.set()
        start.result(timeout=2)
        halt.result(timeout=2)
    assert [f.payload for f in port.frames()] == [b"\x01\x06", b"\x03\x00"]
    assert local.pca9685.level_history[10][-1] is False
    count = len(port.writes)
    runtime.reset("")
    assert len(port.writes) == count  # Reset never restarts a previous action.
    runtime.execute("clamp")
    assert port.frames()[-1].payload == b"\x01\x06"


@pytest.mark.parametrize("state", ["offline", "armed", "estop", "manual"])
def test_chassis_blocks_powered_commands_but_allows_safety_stop(stack, state):
    port, chassis, _, _ = stack
    if state == "offline":
        chassis.link = None
    elif state == "armed":
        chassis.epoch = 3
    elif state == "estop":
        chassis.estop_latched = True
    else:
        chassis.mode = "manual"
    for command, device in [(1, 6)]:
        with pytest.raises(RuntimeError):
            chassis.mechanism_command(command, device)
    assert not port.writes
    if state != "offline":
        chassis.mechanism_command(3, 0)
        assert port.frames()[-1].payload == b"\x03\x00"


def test_shared_lock_and_sequence_for_concurrent_chassis_and_mechanism_writes(stack):
    port, chassis, _, _ = stack
    write = port.write
    busy = threading.Lock()

    def guarded_write(packet):
        assert busy.acquire(blocking=False), "overlapping UART writers"
        try:
            time.sleep(0.001)
            return write(packet)
        finally:
            busy.release()

    port.write = guarded_write
    with ThreadPoolExecutor(max_workers=6) as pool:
        futures = [pool.submit(chassis.mechanism_command, 1, 6) if i % 2 else
                   pool.submit(chassis.stop) for i in range(24)]
        for future in futures:
            future.result(timeout=2)
    frames = port.frames()
    assert len(frames) == 24
    assert len({f.session for f in frames}) == 1 and frames[0].session != 0
    assert [f.sequence for f in frames] == list(range(1, 25))
    assert {f.kind for f in frames} == {wire.STREAM_CONTROL, 0x36}


def test_reconnect_neutralizes_mechanisms_without_replaying_start(monkeypatch):
    from grain_sampling_interhost import chassis_controller as module

    ports = []

    def connect():
        port = SerialRecorder()
        ports.append(port)
        return port

    chassis = ChassisController(serial_factory=connect)
    monkeypatch.setattr(module.time, "sleep", lambda _: chassis.running.clear())
    for _ in range(2):
        chassis.running.set()
        chassis._loop()
        assert [(f.kind, f.payload) for f in ports[-1].frames()] == [
            (wire.STREAM_CONTROL, b"\x06"), (0x36, b"\x03\x00")]
        chassis.mode = "auto"
        chassis.mechanism_command(1, 2)
        with chassis.lock:
            chassis._disconnect_locked()
    assert len(ports) == 2
    assert all(port.closed for port in ports)


def test_production_server_injects_shared_uart(monkeypatch):
    from grain_sampling_interhost import mechanism_controller as module

    local = MechanismController(mock_mode=True)
    monkeypatch.setattr(module, "MechanismController", lambda **_: local)
    chassis = ChassisController()
    port = SerialRecorder()
    chassis.link = ChassisSerial(port)
    chassis.mode = "auto"
    server = RobotServer(bind_ip="127.0.0.1", allowed_peer="127.0.0.1",
        cert="", key="", ca="", ipc_path="", chassis=chassis)
    port.writes.clear()  # Constructor's existing chassis obstacle stop.
    try:
        assert server.local_request({"action": "mechanism", "name": "clamp"}) == {"ok": True}
        assert [f.payload for f in port.frames()] == [b"\x01\x06"]
        assert local.pca9685.level_history[10][-1] is True
    finally:
        server.mechanism.close()
        chassis.close()
