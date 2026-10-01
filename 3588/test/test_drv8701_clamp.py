"""DRV8701E clamp input sequence; all checks run without I2C hardware."""

from __future__ import annotations

from contextlib import nullcontext
import time
from types import SimpleNamespace

from grain_sampling_devices import mechanism_driver as md

from grain_sampling_devices.mechanism_driver import (
    CLAMP_EN_CHANNEL,
    CLAMP_NS_CHANNEL,
    CLAMP_PH_CHANNEL,
    FULL_ON_OFF_BIT,
    LED0_ON_L,
    MechanismController,
    PCA9685,
)


def test_pca9685_static_level_and_legacy_duty_registers(monkeypatch):
    pca = PCA9685()
    writes = []
    monkeypatch.setattr(pca, "_transaction", nullcontext)
    monkeypatch.setattr(pca, "write_register", lambda register, value: writes.append((register, value)))
    base = LED0_ON_L + 4 * CLAMP_EN_CHANNEL

    pca.set_level(CLAMP_EN_CHANNEL, False)
    assert writes[0] == (base + 3, FULL_ON_OFF_BIT)
    assert (base + 1, 0) in writes

    writes.clear()
    pca.set_duty_cycle(CLAMP_EN_CHANNEL, md.CLAMP_EN_DUTY_CLOSE_PERCENT)
    assert writes[-2:] == [(base + 2, 0x85), (base + 3, 0x01)]  # 389/4096
    assert (base + 3, FULL_ON_OFF_BIT) not in writes

    writes.clear()
    pca.set_duty_cycle(CLAMP_EN_CHANNEL, md.CLAMP_EN_DUTY_OPEN_PERCENT)
    assert writes[-2:] == [(base + 2, 0xF6), (base + 3, 0x00)]  # 246/4096
    assert (base + 3, FULL_ON_OFF_BIT) not in writes

    writes.clear()
    pca.set_level(CLAMP_PH_CHANNEL, True)
    ph_base = LED0_ON_L + 4 * CLAMP_PH_CHANNEL
    assert writes[-1] == (ph_base + 3, 0)
    assert (ph_base + 1, FULL_ON_OFF_BIT) in writes


def test_pca9685_open_sleeps_clamp_before_setting_continuous_pwm(monkeypatch):
    pca = PCA9685()
    events = []
    monkeypatch.setattr(md, "fcntl", SimpleNamespace(ioctl=lambda *_: None))
    monkeypatch.setattr(md, "validate_tp_bus", lambda *_: None)
    monkeypatch.setattr(md.os, "open", lambda *_: 123)
    monkeypatch.setattr(md.os, "close", lambda *_: None)
    monkeypatch.setattr(pca, "set_level", lambda ch, high: events.append(("level", ch, high)))
    monkeypatch.setattr(pca, "set_frequency", lambda hz: events.append(("frequency", hz)))
    monkeypatch.setattr(pca, "all_stop", lambda: events.append(("neutral",)))
    monkeypatch.setattr(pca, "set_duty_cycle", lambda ch, pct: events.append(("duty", ch, pct)))

    pca.open()
    assert events == [
        ("level", CLAMP_NS_CHANNEL, False),
        ("frequency", md.DEFAULT_FREQUENCY_HZ),
        ("neutral",),
        ("level", CLAMP_PH_CHANNEL, False),
        ("duty", CLAMP_EN_CHANNEL, 9.5),
    ]
    pca.close()


class RecordingPCA:
    def __init__(self):
        self.events = []

    def set_level(self, channel, high):
        self.events.append(("level", channel, high))

    def set_duty_cycle(self, channel, percent):
        self.events.append(("duty", channel, percent))

    def channel_stop(self, channel):
        self.events.append(("neutral", channel))


def test_clamp_direction_wake_and_timed_sleep():
    pca = RecordingPCA()
    controller = MechanismController(pca9685=pca, mock_mode=True)
    controller.clamp(duration=0.02)
    assert pca.events[:4] == [
        ("level", CLAMP_NS_CHANNEL, False),
        ("level", CLAMP_PH_CHANNEL, False),
        ("duty", CLAMP_EN_CHANNEL, 9.5),
        ("level", CLAMP_NS_CHANNEL, True),
    ]
    deadline = time.monotonic() + 1
    while CLAMP_EN_CHANNEL in controller._running and time.monotonic() < deadline:
        time.sleep(0.005)
    assert pca.events[4:5] == [("level", CLAMP_NS_CHANNEL, False)]

    pca.events.clear()
    controller.unclamp()
    assert ("level", CLAMP_PH_CHANNEL, True) in pca.events
    before_stop = len(pca.events)
    controller.emergency_stop()
    assert pca.events[before_stop:before_stop + 1] == [
        ("level", CLAMP_NS_CHANNEL, False)
    ]
    assert ("duty", CLAMP_EN_CHANNEL, 6.0) in pca.events
    assert all(event[2] > 0 for event in pca.events if event[0] == "duty")
    assert CLAMP_EN_CHANNEL not in controller._running


def test_old_clamp_timer_cannot_sleep_new_direction():
    pca = RecordingPCA()
    controller = MechanismController(pca9685=pca, mock_mode=True)
    controller.clamp(duration=0.02)
    controller.unclamp(duration=0.15)
    time.sleep(0.05)
    assert CLAMP_EN_CHANNEL in controller._running
    assert pca.events[-1] == ("level", CLAMP_NS_CHANNEL, True)
    before_shutdown = len(pca.events)
    controller.shutdown()
    assert pca.events[before_shutdown] == ("level", CLAMP_NS_CHANNEL, False)
