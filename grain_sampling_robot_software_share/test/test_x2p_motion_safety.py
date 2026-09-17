"""Regression tests for encoder-controlled X2P stopping behaviour."""

import sys
from types import ModuleType

# pyserial is a board-only dependency in this project.  The profile calculation
# is pure, so provide an import stub when running the Windows unit-test suite.
try:  # pragma: no cover - depends on the developer environment
    import serial  # noqa: F401
except ImportError:  # pragma: no cover
    sys.modules["serial"] = ModuleType("serial")

from x2p.motion import _approach_profile


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
