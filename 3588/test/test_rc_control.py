"""Tests for RC manual control (rc_control.py).

The mock receiver is configured with ``debounce_samples=1`` and
``debounce_threshold=0.0`` so it reports injected values faithfully; this
keeps the tests focused on :class:`RCControl`'s own CH5 debounce and the
CH1/CH3 stick mapping rather than the receiver's median filter.
"""

from __future__ import annotations

import pytest

from grain_sampling_devices.rc_receiver import MockRCReceiver
from grain_sampling_workflow.rc_control import RCControl

# ── Helpers ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize('pulse,effort', [(1000,-1), (1225,-0.5), (1449,-1/450),
    (1450,0), (1500,0), (1550,0), (1551,1/450), (1775,0.5), (2000,1)])
def test_continuous_deadband_mapping(pulse, effort):
    control = RCControl(None, None)
    assert control._map_linear(pulse) == pytest.approx(effort * 0.3)
    assert control._map_angular(pulse) == pytest.approx(-effort * 0.8)


class RecordingBridge:
    """Duck-typed SamplingBridge that records chassis actions."""

    def __init__(self) -> None:
        self.cancel_calls: int = 0
        self.cmd_vels: list[tuple[float, float]] = []

    def cancel_goal(self) -> bool:
        self.cancel_calls += 1
        return True

    def publish_cmd_vel(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> bool:
        self.cmd_vels.append((linear_mps, angular_rps))
        return True


@pytest.fixture
def receiver() -> MockRCReceiver:
    return MockRCReceiver(debounce_samples=1, debounce_threshold=0.0)


@pytest.fixture
def bridge() -> RecordingBridge:
    return RecordingBridge()


@pytest.fixture
def control(receiver, bridge) -> RCControl:
    return RCControl(receiver, bridge)


def _run(control, receiver, ch5: float, ch1: float = 1500.0, ch3: float = 1500.0,
         times: int = 1) -> None:
    """Inject one full sample set *times* times and tick the control."""
    for _ in range(times):
        receiver.update_all({"CH1": ch1, "CH3": ch3, "CH5": ch5})
        control.tick()


def _establish_manual(control, receiver, ch1: float = 1500.0, ch3: float = 1500.0) -> None:
    """Drive the CH5 switch to manual with ``debounce_samples`` clean ticks."""
    _run(control, receiver, ch5=1000.0, ch1=ch1, ch3=ch3, times=5)


# ── 1. Manual mode: stick mapping + cancel_goal ────────────────────────


def test_manual_mode_maps_sticks_and_cancels_goal(control, receiver, bridge):
    """CH5=1000, CH1=1800, CH3=1150 -> cancel_goal once, forward + left."""
    _establish_manual(control, receiver, ch1=1800.0, ch3=1150.0)

    assert control.mode == RCControl.MODE_MANUAL
    assert control.cancel_goal_calls == 1
    assert bridge.cancel_calls == 1

    # CH1=1800 -> (1800-1550)/450 * 0.3 = +1/6 m/s forward
    linear, angular = control.cmd_vel_history[-1]
    assert linear > 0
    assert linear == pytest.approx(1 / 6)
    # CH3=1150 (<1350) -> LEFT -> angular.z > 0
    assert angular > 0
    assert angular == pytest.approx(0.8 * 2 / 3)

    # Publishing starts only once manual mode is established (5th tick).
    assert len(control.cmd_vel_history) == 1
    assert len(bridge.cmd_vels) == 1

    # Once established, every manual tick keeps publishing.
    _run(control, receiver, ch5=1000.0, ch1=1800.0, ch3=1150.0)
    assert len(control.cmd_vel_history) == 2


# ── 2. Auto mode: hands-off ─────────────────────────────────────────────


def test_auto_mode_does_not_take_over(control, receiver, bridge):
    """CH5=2000 -> no cancel_goal, no cmd_vel regardless of sticks."""
    _run(control, receiver, ch5=2000.0, ch1=2000.0, ch3=1000.0, times=10)

    assert control.mode == RCControl.MODE_AUTO
    assert control.cancel_goal_calls == 0
    assert control.cmd_vel_history == []
    assert bridge.cancel_calls == 0
    assert bridge.cmd_vels == []


# ── 3. Mode-switch debounce ─────────────────────────────────────────────


def test_mode_switch_debounce_filters_transition_values(control, receiver):
    """1350 -> 1400 -> 1500 -> 2000 must not switch the mode."""
    _establish_manual(control, receiver)
    cancels_after_entry = control.cancel_goal_calls

    for transition_ch5 in (1350.0, 1500.0, 1500.0, 2000.0):
        _run(control, receiver, ch5=transition_ch5)

    # Still manual; no extra cancel_goal from the transient readings.
    assert control.mode == RCControl.MODE_MANUAL
    assert control.cancel_goal_calls == cancels_after_entry

    # A single auto sample is not enough either (needs debounce_samples).
    assert control._streak == 1


def test_mode_switch_requires_debounce_samples(control, receiver):
    """Only `debounce_samples` consecutive auto readings switch the mode."""
    _establish_manual(control, receiver)
    cancels_after_entry = control.cancel_goal_calls

    _run(control, receiver, ch5=2000.0, times=4)
    assert control.mode == RCControl.MODE_MANUAL

    _run(control, receiver, ch5=2000.0)  # 5th consecutive auto sample
    assert control.mode == RCControl.MODE_AUTO
    assert control.cancel_goal_calls == cancels_after_entry  # no extra cancel


def test_returning_to_manual_cancels_again(control, receiver, bridge):
    """AUTO -> MANUAL transition cancels the navigation goal once more."""
    _run(control, receiver, ch5=2000.0, times=5)
    assert control.mode == RCControl.MODE_AUTO
    assert control.cancel_goal_calls == 0

    _establish_manual(control, receiver)
    assert control.mode == RCControl.MODE_MANUAL
    assert control.cancel_goal_calls == 1
    assert bridge.cancel_calls == 1


# ── 4. Stick centering stops the robot ──────────────────────────────────


def test_centered_sticks_publish_zero(control, receiver, bridge):
    """Manual mode with CH1/CH3 in the deadband -> cmd_vel = (0, 0)."""
    _establish_manual(control, receiver)

    assert control.mode == RCControl.MODE_MANUAL
    assert control.cmd_vel_history[-1] == (0.0, 0.0)
    assert bridge.cmd_vels[-1] == (0.0, 0.0)

    # Moving one stick out then re-centring stops the chassis again.
    _run(control, receiver, ch5=1000.0, ch1=1800.0, ch3=1500.0)
    assert control.cmd_vel_history[-1][0] > 0
    _run(control, receiver, ch5=1000.0, ch1=1500.0, ch3=1500.0)
    assert control.cmd_vel_history[-1] == (0.0, 0.0)


# ── 5. Deadband boundaries ──────────────────────────────────────────────


def test_deadband_boundaries_map_to_zero(control, receiver):
    """CH1=1500 / CH3=1500 (inside 1450~1550) -> speed 0."""
    _establish_manual(control, receiver)
    _run(control, receiver, ch5=1000.0, ch1=1500.0, ch3=1500.0)

    assert control.cmd_vel_history[-1] == (0.0, 0.0)

    # Inclusive boundaries: exactly 1450 / 1550 are still inside the deadband.
    _run(control, receiver, ch5=1000.0, ch1=1450.0, ch3=1550.0)
    assert control.cmd_vel_history[-1] == (0.0, 0.0)


# ── Extra: mapping direction / magnitude ────────────────────────────────


def test_ch1_reverse_is_negative(control, receiver):
    """CH1 < 1350 -> reverse (linear.x < 0)."""
    _establish_manual(control, receiver)
    _run(control, receiver, ch5=1000.0, ch1=1150.0)
    assert control.cmd_vel_history[-1][0] == pytest.approx(-0.2)


def test_ch1_full_scale_clamps_to_max(control, receiver):
    """CH1 beyond 2000 us clamps to max_linear_mps."""
    _establish_manual(control, receiver)
    _run(control, receiver, ch5=1000.0, ch1=2300.0)
    assert control.cmd_vel_history[-1][0] == pytest.approx(0.3)


def test_ch3_right_turn_is_negative(control, receiver):
    """CH3 > 1750 -> right turn (angular.z < 0)."""
    _establish_manual(control, receiver)
    _run(control, receiver, ch5=1000.0, ch3=1800.0)
    assert control.cmd_vel_history[-1][1] == pytest.approx(-0.8 * 5 / 9)


def test_ch3_full_scale_left_clamps_to_max(control, receiver):
    """CH3 far below 1000 us clamps to +max_angular_rps (left)."""
    _establish_manual(control, receiver)
    _run(control, receiver, ch5=1000.0, ch3=700.0)
    assert control.cmd_vel_history[-1][1] == pytest.approx(0.8)


# ── Extra: stale signals ────────────────────────────────────────────────


class FakeClock:
    """Manual clock so the receiver's stale-timeout is controllable."""

    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t


def test_losing_all_channels_in_manual_mode_publishes_stop():
    """A dead receiver while in manual mode maps to a zero stop."""
    clock = FakeClock()
    receiver = MockRCReceiver(debounce_samples=1, debounce_threshold=0.0, clock=clock)
    bridge = RecordingBridge()
    control = RCControl(receiver, bridge)

    _establish_manual(control, receiver, ch1=1800.0)
    assert control.cmd_vel_history[-1][0] > 0

    clock.t += 1.0  # exceed stale_timeout (0.5 s): read() now returns None
    control.tick()

    assert control.mode == RCControl.MODE_MANUAL  # mode retained
    assert control.cmd_vel_history[-1] == (0.0, 0.0)


def test_ch5_transition_value_keeps_manual_mapping(control, receiver):
    """A transition CH5 value doesn't disturb ongoing manual control."""
    _establish_manual(control, receiver, ch1=1800.0)
    _run(control, receiver, ch5=1500.0, ch1=1800.0)

    assert control.mode == RCControl.MODE_MANUAL
    assert control.cmd_vel_history[-1] == pytest.approx((1 / 6, 0.0))


def test_invalid_params_rejected():
    """Non-positive velocity limits are rejected at construction."""
    receiver = MockRCReceiver()
    bridge = RecordingBridge()
    with pytest.raises(ValueError):
        RCControl(receiver, bridge, max_linear_mps=0.0)
    with pytest.raises(ValueError):
        RCControl(receiver, bridge, max_angular_rps=-1.0)


# ── Extra: properties / receiver API variants / fault tolerance ────────


def test_manual_active_and_last_cmd_vel_properties(control, receiver):
    """``manual_active`` and ``last_cmd_vel`` reflect the latest tick."""
    assert control.manual_active is False
    assert control.last_cmd_vel is None

    _establish_manual(control, receiver, ch1=1800.0, ch3=1150.0)

    assert control.manual_active is True
    assert control.last_cmd_vel == (pytest.approx(1 / 6), pytest.approx(0.8 * 2 / 3))


class ScalarReadReceiver:
    """Duck-typed receiver whose no-arg ``read()`` returns a scalar."""

    def __init__(self, ch1: float, ch3: float, ch5: float) -> None:
        self._values = {"CH1": ch1, "CH3": ch3, "CH5": ch5}

    def read(self, channel=None):
        if channel is None:
            return self._values["CH1"]  # naive single-channel receiver
        return self._values[channel]


def test_single_channel_receiver_api_fallback():
    """Receivers whose no-arg read() is scalar still work via per-channel read."""
    control = RCControl(ScalarReadReceiver(1800.0, 1150.0, 1000.0), RecordingBridge())
    for _ in range(5):
        control.tick()

    assert control.mode == RCControl.MODE_MANUAL
    assert control.cancel_goal_calls == 1
    assert control.last_cmd_vel[0] == pytest.approx(1 / 6)


class FailingPublishBridge(RecordingBridge):
    def publish_cmd_vel(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> bool:
        raise RuntimeError("publisher died")


def test_bridge_publish_exception_is_swallowed(control, receiver):
    """A bridge failure must not kill the control loop."""
    control._bridge = FailingPublishBridge()
    _establish_manual(control, receiver, ch1=1800.0)

    assert control.mode == RCControl.MODE_MANUAL
    assert control.last_cmd_vel[0] == pytest.approx(1 / 6)  # still recorded


class FailingCancelBridge(RecordingBridge):
    def cancel_goal(self) -> bool:
        raise RuntimeError("cancel failed")


def test_bridge_cancel_exception_is_swallowed(receiver):
    """A cancel_goal failure must not prevent the manual takeover."""
    control = RCControl(receiver, FailingCancelBridge())
    _establish_manual(control, receiver, ch1=1750.0)

    assert control.mode == RCControl.MODE_MANUAL
    assert control.cancel_goal_calls == 1
