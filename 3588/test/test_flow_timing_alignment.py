"""Real flow/test entry points share timings; no hardware is energized."""
import importlib.util
from pathlib import Path
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import MagicMock

import pytest

from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_interhost.mechanism_controller import MechanismRuntime


@pytest.mark.parametrize("grain,sampling,convey", [
    ("稻谷", 120, 30), ("玉米", 180, 60), ("黄豆", 90, 90),
])
def test_upper_flow_defaults_order_and_waits(monkeypatch, grain, sampling, convey):
    path = Path(__file__).resolve().parents[1] / "scripts/test_upper_mechanism_flow.py"
    spec = importlib.util.spec_from_file_location("upper_timing_test", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    events = []
    monkeypatch.setattr(module.UpperMechanismTest, "_trigger",
                        lambda self, service, label: events.append(service))
    monkeypatch.setattr(module.UpperMechanismTest, "_move_lift",
                        lambda self, direction, distance: events.append((direction, distance)))
    monkeypatch.setattr(module.UpperMechanismTest, "_sleep",
                        lambda self, seconds, reason: events.append(("wait", seconds)))
    assert module.main(["--dry-run", "--grain", grain]) == 0
    assert events == [
        "/mechanism/lift_health", "/mechanism/clamp", ("wait", 2.5),
        ("down_cycle", 20), ("wait", 0.5),
        "/mechanism/unclamp", ("wait", 1.5),
        ("return", 20), ("wait", 0.5),
        "/mechanism/clamp", ("wait", 2.5), "/mechanism/start_suction",
        ("wait", 3), "/mechanism/hold_bin_open/shallow", ("wait", 0.5),
        ("wait", sampling), "/mechanism/stop_suction",
        "/mechanism/start_convey", ("wait", convey), "/mechanism/stop_convey",
        "/mechanism/close_all_bins", ("wait", 3.5),
    ]


@pytest.mark.parametrize("depth,device", [("shallow", 3), ("mid", 4), ("deep", 5)])
def test_uart_and_pca_advance_one_bin_at_a_time(monkeypatch, depth, device):
    controller = MechanismController(mock_mode=True)
    events = []
    runtime = MechanismRuntime(controller, serial_command=lambda cmd, dev: events.append((cmd, dev)))
    def wait(seconds):
        events.append(("wait", seconds, set(controller._running)))
        return False
    monkeypatch.setattr(controller, "_wait_bin_motion", wait)
    try:
        runtime.execute("hold_bin_open", depth=depth)
        others = [dev for dev in (3, 4, 5) if dev != device]
        assert events == [
            (1, device), (1, device), ("wait", 5, {device - 1}),
            (2, others[0]), ("wait", 3, {others[0] - 1}),
            (2, others[1]), ("wait", 3, {others[1] - 1}),
        ]
        assert not controller._running
    finally:
        runtime.close()


def test_emergency_interrupts_bin_wait_before_next_serial_command(monkeypatch):
    controller = MechanismController(mock_mode=True)
    send = MagicMock()
    runtime = MechanismRuntime(controller, serial_command=send)
    entered = threading.Event()
    def wait(seconds):
        entered.set()
        return controller._stop_flag.wait(seconds)
    monkeypatch.setattr(controller, "_wait_bin_motion", wait)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(runtime.execute, "hold_bin_open", depth="mid")
            try:
                assert entered.wait(1)
                assert not future.done()
                runtime.emergency_stop()
                with pytest.raises(RuntimeError, match="interrupted"):
                    future.result(timeout=1)
                assert [c.args for c in send.call_args_list] == [(1, 4), (1, 4), (3, 0)]
            finally:
                controller.emergency_stop()
    finally:
        runtime.close()
