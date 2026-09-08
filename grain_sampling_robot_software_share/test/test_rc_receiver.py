"""Tests for the RC receiver GPIO pulse-width measurement module.

Covers mock-mode sampling (history, debounce, out-of-range handling),
the change-reporting threshold, staleness, and the sysfs polling logic
via an injectable value reader + fake monotonic clock.  A fake sysfs tree
allows the real :class:`RCReceiver` sampling thread to run on Windows.
"""

import os
import time

import pytest

from grain_sampling_devices.rc_receiver import (
    DEFAULT_RC_PINS,
    MockRCReceiver,
    RCReceiver,
    create_rc_receiver,
)

# ── Helpers ─────────────────────────────────────────────────────────────


class FakeClock:
    """Manual monotonic clock for deterministic timing tests."""

    def __init__(self, start: float = 0.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t


def make_pwm_reader(clock, widths_us, period_s: float = 0.020):
    """Build a value_reader that emulates 50 Hz PWM on each GPIO.

    ``widths_us`` maps a sysfs GPIO number to its high-pulse width.
    """

    def reader(gpio: int) -> int:
        phase_us = (clock.t * 1e6) % (period_s * 1e6)
        return 1 if phase_us < widths_us[gpio] else 0

    return reader


@pytest.fixture
def fake_sysfs(tmp_path):
    """A pre-populated sysfs GPIO tree (gpio34/40/111 with value+direction)."""
    gpio_root = tmp_path / "gpio"
    gpio_root.mkdir()
    (gpio_root / "export").write_text("")
    (gpio_root / "unexport").write_text("")
    for gpio in (34, 40, 111):
        gpio_dir = gpio_root / f"gpio{gpio}"
        gpio_dir.mkdir()
        (gpio_dir / "direction").write_text("in")
        (gpio_dir / "value").write_text("0")
    return str(gpio_root)


# ── Module constants ────────────────────────────────────────────────────


def test_default_pins_match_board_measurement():
    assert DEFAULT_RC_PINS == {"CH1": 34, "CH3": 40, "CH5": 111}


# ── Mock receiver: sampling ─────────────────────────────────────────────


def test_mock_update_shorthand_and_last_value():
    receiver = MockRCReceiver()
    assert receiver.update(1500) == 1500.0
    assert receiver.last_value == 1500.0


def test_mock_update_explicit_channel():
    receiver = MockRCReceiver()
    assert receiver.update("CH3", 1200) == 1200.0
    assert receiver.read("CH3") == 1200.0
    # CH1 was never updated -> nothing reported yet
    assert receiver.read("CH1") is None


def test_mock_update_all():
    receiver = MockRCReceiver()
    receiver.update_all({"CH1": 1500, "CH3": 1200, "CH5": 1700})
    assert receiver.read() == {"CH1": 1500.0, "CH3": 1200.0, "CH5": 1700.0}
    assert receiver.latest == {"CH1": 1500.0, "CH3": 1200.0, "CH5": 1700.0}


def test_mock_records_pulse_history():
    receiver = MockRCReceiver()
    for value in (1500, 1520, 1490, 1510, 1505):
        receiver.update("CH1", value)
    history = receiver.history["CH1"]
    assert len(history) == 5
    assert history[-1] == 1505.0
    assert receiver.history["CH3"] == []
    assert receiver.raw_values["CH1"] == 1505.0


# ── Debounce ────────────────────────────────────────────────────────────


def test_median_debounce_filters_instant_spike():
    receiver = MockRCReceiver(debounce_samples=5, debounce_threshold=50)
    for _ in range(5):
        receiver.update("CH1", 1500)
    assert receiver.last_value == 1500.0

    # A single spike does not move the median -> not reported
    receiver.update("CH1", 2300)
    assert receiver.last_value == 1500.0

    # Only after the window fills with the new level does it get reported
    for _ in range(4):
        receiver.update("CH1", 2300)
    assert receiver.last_value == 2300.0


def test_debounce_threshold_blocks_small_changes():
    receiver = MockRCReceiver(debounce_samples=5, debounce_threshold=50)
    for _ in range(5):
        receiver.update("CH1", 1500)
    assert receiver.last_value == 1500.0

    # 20us drift < 50us threshold -> stays at 1500
    for _ in range(5):
        receiver.update("CH1", 1520)
    assert receiver.last_value == 1500.0

    # 50us drift >= 50us threshold -> reported
    for _ in range(5):
        receiver.update("CH1", 1550)
    assert receiver.last_value == 1550.0


def test_on_change_callback_fires_only_on_reported_changes():
    calls = []
    receiver = MockRCReceiver(
        debounce_samples=3,
        debounce_threshold=50,
        on_change=lambda channel, value: calls.append((channel, value)),
    )
    receiver.update("CH1", 1500)
    assert calls == [("CH1", 1500.0)]

    receiver.update("CH1", 1500)  # identical -> no change
    receiver.update("CH1", 2000)  # median still 1500 -> no change
    assert calls == [("CH1", 1500.0)]

    receiver.update("CH1", 2000)
    receiver.update("CH1", 2000)  # median now 2000 -> reported
    assert calls == [("CH1", 1500.0), ("CH1", 2000.0)]


# ── Out-of-range handling ───────────────────────────────────────────────


def test_out_of_range_samples_are_dropped_and_counted():
    receiver = MockRCReceiver()
    receiver.update("CH1", 500)  # below 700us lower bound
    assert receiver.read("CH1") is None
    assert receiver.history["CH1"] == []
    assert receiver.out_of_range_counts["CH1"] == 1

    receiver.update("CH1", 2400)  # above 2300us upper bound
    assert receiver.read("CH1") is None
    assert receiver.out_of_range_counts["CH1"] == 2

    # Valid sample after anomalies -> reports normally, count unchanged
    receiver.update("CH1", 1500)
    assert receiver.read("CH1") == 1500.0
    assert receiver.out_of_range_counts["CH1"] == 2


@pytest.mark.parametrize("boundary", [700, 2300])
def test_boundary_pulse_widths_are_accepted(boundary):
    receiver = MockRCReceiver()
    assert receiver.update("CH5", boundary) == float(boundary)
    assert receiver.out_of_range_counts["CH5"] == 0


def test_out_of_range_never_enters_control_value_path():
    receiver = MockRCReceiver()
    receiver.update("CH1", 1500)
    assert receiver.last_value == 1500.0
    # A burst of glitches must not corrupt the reported value
    for _ in range(10):
        receiver.update("CH1", 60)
    assert receiver.last_value == 1500.0


# ── Staleness / safety ──────────────────────────────────────────────────


def test_stale_channel_returns_none_after_timeout():
    clock = FakeClock(0.0)
    receiver = MockRCReceiver(clock=clock, stale_timeout=0.3)
    receiver.update("CH1", 1500)
    assert receiver.read("CH1") == 1500.0

    clock.t = 0.4  # beyond timeout -> stale
    assert receiver.read("CH1") is None

    clock.t = 0.1
    assert receiver.read("CH1") == 1500.0


def test_fresh_samples_refresh_staleness():
    clock = FakeClock(0.0)
    receiver = MockRCReceiver(clock=clock, stale_timeout=0.3)
    receiver.update("CH1", 1500)
    clock.t = 0.29
    assert receiver.read("CH1") == 1500.0
    receiver.update("CH1", 1501)  # new sample keeps the channel alive
    clock.t = 0.5
    assert receiver.read("CH1") == 1500.0


def test_invalid_channel_raises():
    receiver = MockRCReceiver()
    with pytest.raises(ValueError):
        receiver.update("CH2", 1500)
    assert receiver.read("CH2") is None


def test_reset_clears_all_state():
    receiver = MockRCReceiver()
    receiver.update_all({"CH1": 1500, "CH3": 1200, "CH5": 1700})
    receiver.update("CH1", 600)  # one out-of-range sample
    receiver.reset()
    assert receiver.read("CH1") is None
    assert receiver.history["CH1"] == []
    assert receiver.out_of_range_counts["CH1"] == 0
    assert receiver.last_value is None


# ── Sysfs polling logic (injected reader + fake clock) ──────────────────


def test_polling_measures_pulse_widths_from_simulated_pwm(fake_sysfs):
    clock = FakeClock(0.0)
    widths = {34: 1500.0, 40: 1200.0, 111: 1700.0}
    receiver = RCReceiver(
        rc_pins=DEFAULT_RC_PINS,
        auto_start=False,
        sysfs_dir=fake_sysfs,
        clock=clock,
        value_reader=make_pwm_reader(clock, widths),
        poll_interval=0.0001,
        debounce_samples=3,
        debounce_threshold=50,
    )
    assert receiver.is_available

    # 60 ms of simulated time (3 full 50 Hz frames) at 0.1 ms steps
    for _ in range(600):
        receiver._poll_once()
        clock.t += 0.0001

    ch1 = receiver.read("CH1")
    ch3 = receiver.read("CH3")
    ch5 = receiver.read("CH5")
    assert ch1 is not None and 1350.0 <= ch1 <= 1650.0
    assert ch3 is not None and 1050.0 <= ch3 <= 1350.0
    assert ch5 is not None and 1550.0 <= ch5 <= 1850.0

    receiver.stop()


def test_polling_drops_nonsense_widths(fake_sysfs):
    """Samples wider than one frame are ignored by the edge detector."""
    clock = FakeClock(0.0)

    def weird_reader(gpio: int) -> int:
        # Emits pulses far longer than the 20ms frame -> not valid RC.
        return 1 if clock.t < 0.1 else 0

    receiver = RCReceiver(
        rc_pins=DEFAULT_RC_PINS,
        auto_start=False,
        sysfs_dir=fake_sysfs,
        clock=clock,
        value_reader=weird_reader,
        poll_interval=0.001,
    )
    for _ in range(50):
        receiver._poll_once()
        clock.t += 0.001
    assert receiver.read("CH1") is None
    assert receiver.history["CH1"] == []
    receiver.stop()


def test_receiver_unavailable_without_sysfs(tmp_path):
    missing = str(tmp_path / "no_such_gpio")
    receiver = RCReceiver(rc_pins=DEFAULT_RC_PINS, auto_start=False, sysfs_dir=missing)
    assert receiver.is_available is False
    receiver.start()  # must not crash
    assert receiver.read("CH1") is None
    receiver.stop()


def test_sampling_thread_runs_and_stops(fake_sysfs):
    """The background thread measures a real 50 Hz PWM and stops cleanly."""
    widths = {34: 1500.0, 40: 1200.0, 111: 1700.0}

    def real_pwm_reader(gpio: int) -> int:
        phase_us = (time.perf_counter() * 1e6) % 20000.0
        return 1 if phase_us < widths[gpio] else 0

    receiver = RCReceiver(
        rc_pins=DEFAULT_RC_PINS,
        auto_start=True,
        sysfs_dir=fake_sysfs,
        value_reader=real_pwm_reader,
        poll_interval=0.0002,
        debounce_samples=3,
        debounce_threshold=50,
    )
    assert receiver._thread is not None
    assert receiver._thread.is_alive()

    # Let the thread measure ~25 frames (0.5 s) of real PWM.
    time.sleep(0.5)

    for channel, expected in (("CH1", 1500), ("CH3", 1200), ("CH5", 1700)):
        value = receiver.read(channel)
        assert value is not None, f"{channel} never measured a pulse"
        assert abs(value - expected) <= 400, (
            f"{channel}: measured {value}us, expected ~{expected}us"
        )

    receiver.stop()
    assert receiver._thread is None


def test_start_is_idempotent(fake_sysfs):
    receiver = RCReceiver(
        rc_pins=DEFAULT_RC_PINS,
        auto_start=True,
        sysfs_dir=fake_sysfs,
        poll_interval=0.01,
    )
    first_thread = receiver._thread
    receiver.start()
    assert receiver._thread is first_thread
    receiver.stop()


# ── Factory ─────────────────────────────────────────────────────────────


def test_factory_returns_mock_without_sysfs():
    if os.path.isdir("/sys/class/gpio"):
        pytest.skip("real sysfs present - factory would return RCReceiver")
    receiver = create_rc_receiver()
    assert isinstance(receiver, MockRCReceiver)
    receiver.close()


def test_factory_survives_unknown_kwargs():
    """RCReceiver-only kwargs must not break the mock fallback path."""
    if os.path.isdir("/sys/class/gpio"):
        pytest.skip("real sysfs present - would exercise the RCReceiver path")
    receiver = create_rc_receiver(poll_interval=0.001, debounce_threshold=80)
    assert isinstance(receiver, MockRCReceiver)
    assert receiver.update(1500) == 1500.0
    # 20us drift stays under the 80us threshold -> value unchanged
    for _ in range(4):
        receiver.update(1520)
    assert receiver.last_value == 1500.0
