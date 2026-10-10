"""Local migration tests: real host drivers, simulated byte transports only."""
import json
import runpy
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import serial

from grain_sampling_bench.mechanisms import SerialMechanisms, command_plan
from grain_sampling_bench.runner import run_suite, simulated_stack, wait_for
from grain_sampling_devices import mechanism_protocol as m
from grain_sampling_devices.chassis_serial import ChassisSerial
from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from x2p.registers import Register as R

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def forbid_physical_ports(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("a simulation attempted to open a physical serial port")
    monkeypatch.setattr(serial, "Serial", forbidden)


ACTION_CASES = [(action, {}) for action in (
    "clamp", "unclamp", "tighten", "untighten", "open_bin_default", "convey",
    "start_convey", "stop_convey", "close_all_bins")]
ACTION_CASES += [(action, {"depth": depth}) for depth in m.BIN_DEVICES
                 for action in ("open_bin", "hold_bin_open", "close_bin")]


@pytest.mark.parametrize("action,args", ACTION_CASES)
def test_uart_plan_matches_existing_3588_runtime(action, args):
    recorded = []
    local = MagicMock()
    runtime = MechanismRuntime(local, serial_command=lambda cmd, dev: recorded.append((cmd, dev)))
    try:
        runtime.execute(action, **args)
        assert tuple(recorded) == command_plan(action, **args)
    finally:
        runtime.close()  # cancels any timed-convey callback


@pytest.mark.parametrize("action", ["start_suction", "stop_suction"])
def test_no_uart_command_is_not_claimed_as_supported(action):
    recorded = []
    runtime = MechanismRuntime(MagicMock(), serial_command=lambda *args: recorded.append(args))
    try:
        runtime.execute(action)
        assert recorded == []
        with pytest.raises(NotImplementedError, match="no STM32 frame"):
            command_plan(action)
    finally:
        runtime.close()


@pytest.mark.parametrize("duration", [0, -1, 121, float("nan"), float("inf"), True, "2"])
def test_bad_duration_never_starts_conveyor(duration):
    send = MagicMock()
    runtime = SerialMechanisms(send, lambda: None)
    with pytest.raises(ValueError):
        runtime.execute("convey", duration_s=duration)
    send.assert_not_called()


@pytest.mark.parametrize("action,args", [("wrong", {}), ("open_bin", {"depth": "bad"}),
                                        ("clamp", {"duration_s": 1})])
def test_invalid_action_has_no_writes(action, args):
    send = MagicMock()
    runtime = SerialMechanisms(send, lambda: None)
    with pytest.raises(ValueError):
        runtime.execute(action, **args)
    send.assert_not_called()


def test_integration_suite_and_report(tmp_path):
    report = run_suite(cycles=2, log_path=tmp_path / "motion.log")
    assert report["passed"], [(s["name"], s.get("error")) for s in report["scenarios"] if not s["passed"]]
    assert len(report["scenarios"]) == 18
    assert report["physical_hardware_verified"] is False
    assert report["decision"] == "PENDING_HARDWARE_VALIDATION"
    assert report["source_sha256"]
    json.dumps(report)


def test_estop_cancels_running_lift_and_never_returns(tmp_path):
    with simulated_stack(tmp_path / "motion.log", leg_time_s=2) as (session, stm32, servo):
        with ThreadPoolExecutor(max_workers=1) as pool:
            move = pool.submit(session.move_to, -131072)
            wait_for(lambda: servo.triggers == 1)
            stm32.estop = True
            with pytest.raises(RuntimeError):
                move.result(timeout=2)
        assert session.cancelled.is_set()
        assert servo.registers[int(R.STATUS)] == 1
        assert servo.registers[int(R.POSITION_SEGMENT)] == 0
        assert servo.triggers == 1


def test_cancellation_after_enable_prevents_trigger(tmp_path, monkeypatch):
    with simulated_stack(tmp_path / "motion.log") as (session, stm32, servo):
        original = session.motion._enable_and_verify
        def cancel_after_enable(inputs=0):
            original(inputs)
            session.trip("test cancellation immediately after enable")
        monkeypatch.setattr(session.motion, "_enable_and_verify", cancel_after_enable)
        with pytest.raises(RuntimeError, match="latched"):
            session.move_to(-131072)
        assert servo.triggers == 0
        assert servo.registers[int(R.STATUS)] == 1
        assert servo.registers[int(R.POSITION_SEGMENT)] == 0


def test_cancel_during_timed_convey_cannot_restart_it(tmp_path):
    with simulated_stack(tmp_path / "motion.log") as (session, stm32, servo):
        with ThreadPoolExecutor(max_workers=1) as pool:
            action = pool.submit(session.mechanisms.execute, "convey", duration_s=0.2)
            wait_for(lambda: m.CONVEY in stm32.active)
            session.trip("operator emergency stop")
            with pytest.raises(RuntimeError):
                action.result(timeout=1)
        time.sleep(0.22)
        assert not stm32.active
        starts = [f for f in stm32.frames if f["kind"] == m.RELIABLE_FRAME_TYPE and f["payload"] == [m.START, m.CONVEY]]
        assert len(starts) == 1


def test_stop_not_blocked_by_pending_mechanism_ack(tmp_path):
    with simulated_stack(tmp_path / "motion.log") as (session, stm32, servo):
        stm32.drop_ack = True
        with ThreadPoolExecutor(max_workers=1) as pool:
            action = pool.submit(session.mechanisms.execute, "clamp")
            wait_for(lambda: m.CLAMP in stm32.active)
            session.trip("stop while ACK pending")
            with pytest.raises(RuntimeError):
                action.result(timeout=1)
        assert not stm32.active
        assert session.chassis.estop_latched


@pytest.mark.parametrize("confirm", [False, True])
def test_write_guard_rejection_prevents_any_serial_write(confirm):
    port = MagicMock()
    link = ChassisSerial(port)
    @contextmanager
    def revoked():
        raise RuntimeError("authorization revoked before write")
        yield
    with pytest.raises(RuntimeError, match="revoked"):
        link.mechanism_command(m.START, m.CLAMP, confirm=confirm, write_guard=revoked())
    port.write.assert_not_called()


def test_simulated_read_crc_error_can_retry_without_motion(tmp_path):
    with simulated_stack(tmp_path / "motion.log") as (session, stm32, servo):
        servo.corrupt_next_read = True
        assert session.motion.read_encoder_position() == 0
        assert servo.triggers == 0
        reads = [f for f in servo.frames if f["address"] == R.SERVO_POSITION_ENCODER]
        assert len(reads) == 2


def test_arming_base_blocks_new_mechanism_starts(tmp_path):
    with simulated_stack(tmp_path / "motion.log") as (session, stm32, servo):
        session.arm("test-interlock")
        with pytest.raises(RuntimeError, match="before arming"):
            session.mechanisms.execute("clamp")
        assert not [f for f in stm32.frames if f["kind"] == m.RELIABLE_FRAME_TYPE]


def test_cli_cannot_accept_real_hardware_arguments(tmp_path, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["bench_hardware.py", "--execute", "--port", "COM6",
                                      "--report", str(tmp_path / "report.json")])
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(ROOT / "LENOVO/scripts/bench_hardware.py"), run_name="__main__")
    assert exc.value.code == 2


def test_cli_report_is_simulation_and_no_overwrite(tmp_path, monkeypatch):
    path = tmp_path / "report.json"
    monkeypatch.setattr(sys, "argv", ["bench_hardware.py", "--simulate", "--scenario", "stm32_actions",
                                      "--report", str(path)])
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(ROOT / "LENOVO/scripts/bench_hardware.py"), run_name="__main__")
    assert exc.value.code == 0
    original = path.read_bytes()
    assert json.loads(original)["physical_hardware_verified"] is False
    with pytest.raises(SystemExit) as exc:
        runpy.run_path(str(ROOT / "LENOVO/scripts/bench_hardware.py"), run_name="__main__")
    assert exc.value.code == 2
    assert path.read_bytes() == original
