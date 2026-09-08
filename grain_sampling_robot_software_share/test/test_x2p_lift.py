"""X2P 伺服升降适配测试（x2p_lift.py + mechanism press/lift 集成）。

全程 mock，不触碰真实串口：
- :func:`build_x2p_lift_drive` 在 x2p 缺失/串口异常时的降级行为
- MechanismController 注入 mock lift_drive 后 press/lift 走 X2P
- 未注入 lift_drive 时 press/lift 保持占位（既有行为）
"""

from __future__ import annotations

import pytest

from grain_sampling_devices.base_adapter import DeviceError
from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_devices.x2p_lift import _LiftDrive, build_x2p_lift_drive


# ── build_x2p_lift_drive 降级行为 ──────────────────────────────────────


def test_build_x2p_lift_drive_missing_package_raises(monkeypatch):
    """x2p 包不可导入时抛 DeviceError（而不是 ImportError）。"""
    import builtins

    real_import = builtins.__import__

    def _block(module_name, *args, **kwargs):
        if module_name == "x2p" or module_name.startswith("x2p."):
            raise ImportError("no module named x2p")
        return real_import(module_name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", _block)
    with pytest.raises(DeviceError, match="无法导入 x2p"):
        build_x2p_lift_drive(port="/dev/ttyUSB0")


def test_build_x2p_lift_drive_serial_failure_raises(monkeypatch):
    """x2p 存在但串口初始化失败时抛 DeviceError。"""
    import builtins

    real_import = builtins.__import__

    class _FakeSerial:
        def __init__(self, *a, **kw):
            raise OSError("no such device")

    class _FakeDrive:
        def __init__(self, port, slave):
            raise OSError("cannot open /dev/ttyUSB0")

    class _FakeConfig:
        port = "/dev/ttyUSB0"
        slave = 2
        limits = type("L", (), {"max_rpm": 120, "max_duration_s": None,
                                "max_move_pulses": 200000, "max_distance_mm": 300.0,
                                "stop_timeout_s": 2.0, "position_timeout_s": 10.0})()

        def validate(self):
            pass

    class _FakeMotion:
        def __init__(self, drive, config):
            raise OSError("init failed")

    class _FakeX2P:
        def __init__(self):
            self.ControllerConfig = _FakeConfig
            self.MotionController = _FakeMotion
            self.X2PDrive = _FakeDrive

    fake = _FakeX2P()

    def _import(module_name, *args, **kwargs):
        if module_name == "x2p":
            return fake
        if module_name == "x2p.drive":
            return type("D", (), {"X2PDrive": _FakeDrive})()
        return real_import(module_name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", _import)
    with pytest.raises(DeviceError, match="初始化失败"):
        build_x2p_lift_drive(port="/dev/ttyUSB0")


# ── _LiftDrive 包装 ────────────────────────────────────────────────────


class _FakeX2PController:
    """模仿 x2p.MotionController 的最小鸭子类型。"""

    def __init__(self):
        self.speed_calls: list[tuple] = []
        self.stop_calls = 0
        self.closed = False
        self.drive = _FakeDrive()

    def run_speed(self, direction, rpm, duration_s):
        self.speed_calls.append((direction, rpm, duration_s))
        return object()

    def stop(self):
        self.stop_calls += 1


class _FakeDrive:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def test_lift_drive_wraps_controller():
    drive = _LiftDrive(_FakeX2PController(), rpm=30, duration_s=2.0)
    # 语义方向 up → 驱动器 forward
    result = drive.run_speed("up")
    assert drive._controller.speed_calls == [("forward", 30, 2.0)]
    assert result is not None

    # 语义方向 down → 驱动器 reverse
    drive.run_speed("down", rpm=45, duration_s=3.5)
    assert drive._controller.speed_calls[1] == ("reverse", 45, 3.5)

    drive.stop()
    assert drive._controller.stop_calls == 1


def test_lift_drive_close_closes_serial():
    ctrl = _FakeX2PController()
    drive = _LiftDrive(ctrl, rpm=30, duration_s=2.0)
    drive.close()
    assert ctrl.drive.closed is True


# ── MechanismController press/lift 集成 ────────────────────────────────


def test_press_lift_use_lift_drive_when_injected(mock_mechanism):
    """注入 lift_drive 后，press/lift 调用 run_speed + stop，不再写 I2C。"""
    fake = _FakeX2PController()
    mock_mechanism.lift_drive = _LiftDrive(fake, rpm=30, duration_s=2.0)

    mock_mechanism.press()  # press=下压 → down → reverse
    assert fake.speed_calls == [("reverse", 30, 2.0)]
    assert fake.stop_calls == 1

    mock_mechanism.lift(duration=3.0)  # lift=提升 → up → forward
    assert fake.speed_calls == [("reverse", 30, 2.0), ("forward", 30, 3.0)]
    assert fake.stop_calls == 2

    # 未写 PCA9685 通道（register_history 应保持空）
    assert mock_mechanism.pca9685.register_history == {}


def test_press_lift_placeholder_without_lift_drive(mock_mechanism):
    """未注入 lift_drive 时 press/lift 走既有通道行为（Mock 写 CH2）。"""
    mock_mechanism.press()
    mock_mechanism.lift()
    # MockMechanismController 未覆盖 press/lift → 基类 _act 写 bin_shallow(CH2)
    assert mock_mechanism.pca9685.register_history[2] == [1200, 1900]  # open/close
    names = [name for name, _ in mock_mechanism.action_history]
    assert "press" in names
    assert "lift" in names


def test_press_lift_rpm_duration_attributes(mock_mechanism):
    """默认 lift_rpm/lift_duration 可现场标定覆盖。"""
    assert mock_mechanism.lift_rpm == 30
    assert mock_mechanism.lift_duration == 2.0

    mock_mechanism.lift_rpm = 50
    mock_mechanism.lift_duration = 4.0
    fake = _FakeX2PController()
    mock_mechanism.lift_drive = _LiftDrive(fake, rpm=50, duration_s=4.0)
    mock_mechanism.lift()
    assert fake.speed_calls == [("forward", 50, 4.0)]


def test_close_closes_lift_drive(monkeypatch):
    """close() 关闭 X2P 串口。"""
    fake = _FakeX2PController()
    from grain_sampling_devices.mechanism_driver import _RecordingPCA9685

    ctrl = MechanismController(pca9685=_RecordingPCA9685(), mock_mode=True,
                               lift_drive=_LiftDrive(fake, rpm=30, duration_s=2.0))
    ctrl.close()
    assert fake.drive.closed is True
