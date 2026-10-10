"""Supervised X2P speed override is bounded and leaves the default intact."""
from __future__ import annotations

import pytest

from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from grain_sampling_devices.mechanism_driver import MechanismController


class StubController:
    lift_rpm = 30

    def __init__(self):
        self.opened = False

    def open(self):
        self.opened = True

    def init_escs(self, hold_s):
        assert hold_s == 3.0

    def close(self):
        self.opened = False


def test_supervised_speed_override_is_800_rpm(monkeypatch):
    monkeypatch.setenv("GRAIN_LIFT_RPM", "800")
    monkeypatch.setenv("X2P_PORT", "")
    controller = StubController()
    runtime = MechanismRuntime(controller=controller)
    runtime.start()
    assert controller.lift_rpm == 800
    assert controller.opened
    runtime.close()


def test_speed_override_rejects_excess_before_open(monkeypatch):
    monkeypatch.setenv("GRAIN_LIFT_RPM", "801")
    controller = StubController()
    with pytest.raises(ValueError, match="1..800"):
        MechanismRuntime(controller=controller).start()
    assert not controller.opened


def test_speed_override_updates_x2p_position_limit(monkeypatch):
    monkeypatch.setenv("GRAIN_LIFT_RPM", "800")
    monkeypatch.setenv("X2P_PORT", "/dev/test-x2p")
    observed = {}

    class Drive:
        def read_position(self):
            return 0

    def build_drive(**kwargs):
        observed.update(kwargs)
        return Drive()

    monkeypatch.setattr(
        "grain_sampling_interhost.mechanism_controller.build_x2p_lift_drive",
        build_drive,
    )
    controller = StubController()
    runtime = MechanismRuntime(controller=controller)
    runtime.start()
    assert controller.lift_rpm == 800
    assert observed["rpm"] == 800
    runtime.close()


def test_default_x2p_position_limit_unchanged(monkeypatch):
    monkeypatch.delenv("GRAIN_LIFT_RPM", raising=False)
    monkeypatch.delenv("X2P_RPM", raising=False)
    monkeypatch.setenv("X2P_PORT", "/dev/test-x2p")
    observed = {}

    class Drive:
        def read_position(self):
            return 0

    def build_drive(**kwargs):
        observed.update(kwargs)
        return Drive()

    monkeypatch.setattr(
        "grain_sampling_interhost.mechanism_controller.build_x2p_lift_drive",
        build_drive,
    )
    controller = StubController()
    runtime = MechanismRuntime(controller=controller)
    runtime.start()
    assert controller.lift_rpm == 30
    assert observed["rpm"] == 500
    runtime.close()


def test_default_speed_unchanged(monkeypatch):
    monkeypatch.delenv("GRAIN_LIFT_RPM", raising=False)
    monkeypatch.setenv("X2P_PORT", "")
    controller = StubController()
    MechanismRuntime(controller=controller).start()
    assert controller.lift_rpm == 30


def test_doubled_rpm_halves_commanded_move_duration():
    class Drive:
        def __init__(self):
            self.durations = []

        def move_distance(self, direction, distance_mm, duration_s, *, tolerance_mm):
            self.durations.append(duration_s)

    drive = Drive()
    controller = MechanismController(mock_mode=True, lift_drive=drive)
    controller.move_lift("down", 5.0)
    controller.lift_rpm = 60
    controller.move_lift("down", 5.0)
    controller.lift_rpm = 300
    controller.move_lift("down", 5.0)
    assert drive.durations == [20.0, 10.0, 2.0]
