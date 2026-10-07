"""Production return barriers and bin UART behavior without hardware."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from unittest.mock import MagicMock

import pytest

from grain_sampling_devices import mechanism_protocol as mcu
from grain_sampling_devices import chassis_protocol as wire
from grain_sampling_devices.chassis_serial import ChassisSerial
from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.robot_bridge import RobotBridge
from grain_sampling_workflow.state_machine import SamplingAction as A, SamplingState as S, SamplingStateMachine


class Port:
    in_waiting = 0
    def __init__(self):
        self.packets = []
    def write(self, packet):
        self.packets.append(packet)
        return len(packet)
    def frames(self):
        return wire.Parser().feed(b"".join(self.packets))


def short_durations(monkeypatch):
    from grain_sampling_interhost import mechanism_controller as module
    original = module.get_grain_params
    monkeypatch.setattr(module, "get_grain_params", lambda grain: {
        **original(grain), "clamp_duration": 0.002, "unclamp_duration": 0.02,
    })
    monkeypatch.setattr(mcu, "UNCLAMP_DURATION_SEC", 0.02)
    monkeypatch.setattr("grain_sampling_devices.mechanism_driver.PRESS_PAUSE_S", 0)


@pytest.mark.parametrize("grain", ["稻谷", "玉米", "黄豆"])
def test_second_pipe_bottom_releases_before_return_even_if_ui_does_not_wait(monkeypatch, grain):
    short_durations(monkeypatch)
    port = Port()
    controller = MechanismController(mock_mode=True)
    link = ChassisSerial(port)
    runtime = MechanismRuntime(controller, serial_command=link.mechanism_command)
    runtime.set_grain(grain)
    returned_pipes = []

    class Lift:
        config = type("Config", (), {"encoder_forward_sign": 1})()
        counts_per_mm = 100.0
        position = 0
        def read_position(self):
            return self.position
        def stop(self):
            pass
        def move_to_position(self, target, duration_s, tolerance_mm):
            if target == 0:
                assert port.frames()[-1].payload == bytes((mcu.START, mcu.UNCLAMP))
                assert controller._clamp_timer is None
                assert controller.pca9685.level_history[10][-1] is False
                assert controller.pca9685.level_history[8][-1] is True
                returned_pipes.append(fsm.current_pipe_index + 1)
            self.position = target

    controller.lift_drive = Lift()
    class Client:
        def request(self, action, **values):
            assert action == "mechanism"
            runtime.execute(values["name"], **values.get("args", {}))
            return {"ok": True}
    bridge = RobotBridge(client=Client())
    fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
    fsm.set_depth_targets([2.0])
    fsm._state = S.ARRIVED_PROMPT
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch._grain = grain
    orch.enable_mechanism()
    orch._run_async = lambda fn: fn()
    orch._wait_interruptible = lambda seconds: True  # Daemon must enforce the wait.
    orch._log_sampling_event = lambda *args: None
    try:
        fsm.transition(A.CONFIRM_READY)
        assert fsm.current_state == S.ADD_PIPE_PROMPT
        fsm.transition(A.CONFIRM_PIPE_ADDED)
        assert fsm.current_state == S.DISCHARGE_WASTE
        assert returned_pipes == [1, 2]
        assert fsm.current_pipe_index == 2
    finally:
        runtime.close()


def test_return_without_unclamp_is_rejected_before_axis_moves():
    controller = MechanismController(mock_mode=True, lift_drive=MagicMock())
    controller.move_lift = MagicMock()
    runtime = MechanismRuntime(controller)
    try:
        runtime.execute("clamp")
        with pytest.raises(RuntimeError, match="没有松夹"):
            runtime.execute("move_lift", direction="return", distance_cm=20)
        controller.move_lift.assert_not_called()
        assert runtime.estop_latched
    finally:
        runtime.close()


def test_previous_pipe_release_cannot_authorize_next_pipe_return(monkeypatch):
    short_durations(monkeypatch)
    controller = MechanismController(mock_mode=True, lift_drive=MagicMock())
    controller.move_lift = MagicMock()
    runtime = MechanismRuntime(controller)
    try:
        runtime.execute("unclamp")
        runtime.execute("move_lift", direction="down_cycle", distance_cm=20)
        controller.move_lift.reset_mock()
        with pytest.raises(RuntimeError, match="没有松夹"):
            runtime.execute("move_lift", direction="return", distance_cm=20)
        controller.move_lift.assert_not_called()
    finally:
        runtime.close()


def test_second_open_frame_failure_stops_outputs_and_does_not_start_pca():
    port = Port()
    original_write = port.write
    writes = []
    def write(packet):
        writes.append(packet)
        if len(writes) == 2:
            raise OSError("second opening frame failed")
        return original_write(packet)
    port.write = write
    controller = MechanismController(mock_mode=True)
    link = ChassisSerial(port)
    runtime = MechanismRuntime(controller, serial_command=link.mechanism_command)
    try:
        with pytest.raises(OSError, match="second opening"):
            runtime.execute("hold_bin_open", depth="shallow")
        assert [frame.payload for frame in port.frames()] == [b"\x01\x03", b"\x03\x00"]
        assert not any(action == "hold_bin_open" for action, _ in controller.action_history)
        assert runtime.estop_latched
    finally:
        runtime.close()


@pytest.mark.parametrize("stop", [False, True])
def test_return_waits_for_actual_local_timer_and_emergency_can_interrupt(monkeypatch, stop):
    short_durations(monkeypatch)
    monkeypatch.setattr("grain_sampling_devices.mechanism_driver.threading.Timer", lambda *args, **kwargs: MagicMock())
    controller = MechanismController(mock_mode=True, lift_drive=MagicMock())
    controller.move_lift = MagicMock(return_value="returned")
    runtime = MechanismRuntime(controller)
    entered = threading.Event()
    original_wait = runtime._wait_for_unclamp
    def wait():
        entered.set()
        original_wait()
    runtime._wait_for_unclamp = wait
    try:
        runtime.execute("unclamp")
        runtime._unclamp_finish_at = time.monotonic()  # Nominal time elapsed, timer pending.
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(runtime.execute, "move_lift", direction="return", distance_cm=20)
            try:
                assert entered.wait(1)
                assert not future.done()
                controller.move_lift.assert_not_called()
                if stop:
                    runtime.emergency_stop()
                    with pytest.raises(RuntimeError, match="(停止|急停)"):
                        future.result(timeout=2)
                    controller.move_lift.assert_not_called()
                else:
                    controller._finish_clamp(controller._clamp_timer)
                    assert future.result(timeout=2) == "returned"
                    controller.move_lift.assert_called_once_with("return", 20.0)
            finally:
                if not future.done():
                    runtime.emergency_stop()
    finally:
        runtime.close()


def test_waste_button_immediately_emits_two_open_bin_frames():
    # Load installed Qt before the board's compatibility shims under src/.
    script = r'''
import sys
from pathlib import Path
root = Path.cwd()
sys.path = [p for p in sys.path if Path(p or '.').resolve() != root / 'src']
try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    try:
        from PySide2.QtWidgets import QApplication
    except ImportError:
        sys.exit(77)
sys.path.insert(0, str(root / 'src'))
from unittest.mock import MagicMock
from grain_sampling_ui.pages.guidance_page import GuidancePage
from grain_sampling_workflow.state_machine import SamplingState as S, SamplingStateMachine
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.robot_bridge import RobotBridge
from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_devices.chassis_serial import ChassisSerial
from grain_sampling_devices.chassis_protocol import Parser
class Port:
    def __init__(self): self.packets = []
    def write(self, packet):
        self.packets.append(packet)
        return len(packet)
port = Port()
link = ChassisSerial(port)
runtime = MechanismRuntime(MechanismController(mock_mode=True), serial_command=link.mechanism_command)
class Client:
    def request(self, action, **values):
        runtime.execute(values['name'], **values.get('args', {}))
        return {'ok': True}
fsm = SamplingStateMachine()
fsm._state = S.DISCHARGE_WASTE
fsm.current_depth_index = 2
orch = WorkflowOrchestrator(fsm, RobotBridge(client=Client()), cloud_client=MagicMock())
orch.enable_mechanism()
orch._run_async = lambda fn: fn() if fn == orch._handle_open_bin else None
orch._log_sampling_event = lambda *args: None
def wait(seconds):
    assert fsm.current_state == S.OPEN_BIN
    frames = Parser().feed(b''.join(port.packets))
    opening = [b'\x01\x05', b'\x01\x05', b'\x02\x03', b'\x02\x04']
    assert (seconds, [f.payload for f in frames]) in (
        (5.0, opening), (6.5, opening + [b'\x02\x05']))
    assert frames[0].sequence != frames[1].sequence
    return True
orch._wait_interruptible = wait
app = QApplication.instance() or QApplication([])
page = GuidancePage()
page.attach_orchestrator(fsm, orch)
page.show()
app.processEvents()
try:
    button = page._action_buttons['CONFIRM_WASTE_DISCHARGED']
    assert button.isVisible()
    button.click()
    app.processEvents()
    assert fsm.current_state == S.FORMAL_SAMPLING
    assert len(port.packets) == 5
finally:
    page.close()
    runtime.close()
'''
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=Path(__file__).resolve().parents[1],
        env=dict(os.environ, QT_QPA_PLATFORM="offscreen"), capture_output=True,
        text=True, encoding="utf-8", errors="replace", timeout=30,
    )
    if result.returncode == 77:
        pytest.skip("Qt bindings unavailable")
    assert result.returncode == 0, result.stdout + result.stderr
