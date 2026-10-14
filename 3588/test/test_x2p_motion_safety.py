"""Regression tests for encoder-controlled X2P stopping behaviour."""

import sys
from types import ModuleType

import pytest

# pyserial is a board-only dependency in this project.  The profile calculation
# is pure, so provide an import stub when running the Windows unit-test suite.
try:  # pragma: no cover - depends on the developer environment
    import serial  # noqa: F401
except ImportError:  # pragma: no cover
    sys.modules["serial"] = ModuleType("serial")

from x2p.errors import ConfigurationError, SafetyInterlockError
from x2p.motion import (
    MotionController,
    _approach_profile,
    _position_timeout_budget,
)
from x2p.registers import Register


class _StaticSetupDrive:
    def __init__(self) -> None:
        self.registers: dict[int, int] = {}
        self.writes: list[tuple[int, int]] = []
        self.diagnostics = 0

    def diagnostic(self) -> None:
        self.diagnostics += 1

    def write_register(self, address: int, value: int) -> None:
        self.registers[int(address)] = value
        self.writes.append((int(address), value))

    def read_registers(self, address: int) -> list[int]:
        return [self.registers.get(int(address), 0)]


def test_approach_profile_accounts_for_modbus_latency() -> None:
    """A 50 ms loop sleep must not hide the RTU transaction latency."""
    rpm, window_mm, duration_s = _approach_profile(
        command_rpm=167,
        distance_mm=200.0,
        tolerance_mm=2.0,
        screw_lead_mm=5.0,
        motor_revs_per_screw_rev=1.0,
        monitor_interval_s=0.05,
    )

    # 12 r/min = 1 mm/s, so even a complete 1 s RTU timeout consumes only
    # half of the 2 mm tolerance.  The old calculation returned 167 r/min and
    # disabled the approach phase entirely.
    assert rpm == 12
    assert window_mm >= 27.8
    assert duration_s > 39.0


def test_approach_profile_does_not_accelerate_an_already_slow_move() -> None:
    rpm, window_mm, duration_s = _approach_profile(
        command_rpm=10,
        distance_mm=5.0,
        tolerance_mm=2.0,
        screw_lead_mm=5.0,
        motor_revs_per_screw_rev=1.0,
        monitor_interval_s=0.05,
    )

    assert rpm == 10
    assert window_mm == 0.0
    assert duration_s == 6.0


def test_position_timeout_covers_long_return_and_settling() -> None:
    counts_per_mm = 131_072 / 5.0
    timeout_s = _position_timeout_budget(
        pulses=round(195.0 * counts_per_mm),
        rpm=167,
        encoder_counts_per_motor_rev=131_072,
        configured_timeout_s=15.0,
    )

    # The previous fixed 15 s deadline expired after the first zero-speed
    # sample.  The computed budget includes the 14 s travel plus ramp/settle.
    assert timeout_s > 17.0


def test_position_timeout_keeps_existing_floor_for_short_moves() -> None:
    assert _position_timeout_budget(
        pulses=round(5.0 * 131_072 / 5.0),
        rpm=30,
        encoder_counts_per_motor_rev=131_072,
        configured_timeout_s=15.0,
    ) == 15.0


def test_position_setup_reuses_verified_static_parameters(monkeypatch) -> None:
    drive = _StaticSetupDrive()
    controller = MotionController(drive, output=None)
    monkeypatch.setattr(controller, "prepare_off", lambda: None)
    monkeypatch.setattr(
        controller, "_verify_position_configuration", lambda: None
    )
    monkeypatch.setattr(
        controller, "_select_control_path", lambda *, position: 0
    )
    monkeypatch.setattr(
        controller, "_is_disabled_and_stopped", lambda: True
    )

    assert controller._prepare_position_move(167) == 0
    assert drive.diagnostics == 1
    assert (int(Register.POSITION_SOURCE), 1) in drive.writes

    drive.writes.clear()
    assert controller._prepare_position_move(167) == 0

    # The fast leg retains an explicit Pn701 reset/readback, but does not
    # rewrite Pn321/Pn700/Pn703-705/Pn708 or rerun the full diagnostic.
    assert drive.diagnostics == 1
    assert drive.writes == [(int(Register.POSITION_SEGMENT), 0)]


def test_position_fast_setup_invalidates_cache_if_axis_is_not_off(
    monkeypatch,
) -> None:
    drive = _StaticSetupDrive()
    controller = MotionController(drive, output=None)
    controller._position_static_ready = True
    controller._position_static_rpm = 167
    monkeypatch.setattr(
        controller, "_is_disabled_and_stopped", lambda: False
    )

    with pytest.raises(SafetyInterlockError, match="OFF"):
        controller._prepare_position_move(167)

    assert controller._position_static_ready is False


def test_position_segment_readback_rechecks_without_replaying_trigger(monkeypatch) -> None:
    drive = _StaticSetupDrive()
    controller = MotionController(drive, output=None)
    values = iter((0, 1))
    monkeypatch.setattr(drive, "read_registers", lambda address: [next(values)])
    monkeypatch.setattr("x2p.motion.time.sleep", lambda seconds: None)

    drive.write_register(Register.POSITION_SEGMENT, 1)
    controller._verify_position_segment_selected()

    assert drive.writes == [(int(Register.POSITION_SEGMENT), 1)]


def test_position_segment_persistent_mismatch_stops_without_replay(monkeypatch) -> None:
    drive = _StaticSetupDrive()
    controller = MotionController(drive, output=None)
    monkeypatch.setattr(drive, "read_registers", lambda address: [0])
    monkeypatch.setattr("x2p.motion.time.sleep", lambda seconds: None)

    drive.write_register(Register.POSITION_SEGMENT, 1)
    with pytest.raises(ConfigurationError, match="readbacks=\\[0, 0, 0\\]"):
        controller._verify_position_segment_selected()

    assert drive.writes == [(int(Register.POSITION_SEGMENT), 1)]


def test_position_move_logs_failing_phase_before_safe_stop(monkeypatch) -> None:
    messages: list[str] = []
    controller = MotionController(_StaticSetupDrive(), output=messages.append)
    stop_calls: list[bool] = []
    monkeypatch.setattr(
        controller, "_prepare_position_move",
        lambda rpm: (_ for _ in ()).throw(SafetyInterlockError("E37")),
    )
    monkeypatch.setattr(
        controller, "stop", lambda *, verify_off: stop_calls.append(verify_off),
    )

    with pytest.raises(SafetyInterlockError, match="E37"):
        controller._move_pulses(
            "forward", 1000, 30, mode="position", timeout_s=None,
            record_result=False, tolerance_pulses=None,
        )

    assert stop_calls == [True]
    assert any("MOVE_ABORT phase=prepare" in message for message in messages)
