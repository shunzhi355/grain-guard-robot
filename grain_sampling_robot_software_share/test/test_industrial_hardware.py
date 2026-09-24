"""Industrial PC wiring and shared-chip regressions; no physical hardware needed."""
import os
import socket
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, call

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from dipan import motor_driver as motor
from grain_sampling_devices import mechanism_driver as md
from grain_sampling_devices import rc_receiver as rc
from grain_sampling_devices.ibus_receiver import IBusRCReceiver, build_ibus_frame, parse_ibus_frame
from grain_sampling_workflow.rc_control import RCControl


@pytest.fixture
def chip(monkeypatch):
    """One register bank shared by two independently opened driver instances."""
    registers = bytearray(256)
    registers[md.MODE1] = md.MODE1_ALLCALL
    registers[md.MODE2] = md.MODE2_OUTDRV
    registers[md.PRESCALE] = 30
    writes, closed, cursors = [], [], {}

    def open_device(*args):
        fd = 100 + len(cursors)
        cursors[fd] = 0
        return fd

    def write_device(fd, payload):
        cursors[fd] = payload[0]
        if len(payload) > 1:
            writes.append(bytes(payload))
            registers[payload[0]:payload[0] + len(payload) - 1] = payload[1:]
        return len(payload)

    def read_device(fd, size):
        return bytes(registers[cursors[fd]:cursors[fd] + size])

    fake_os = SimpleNamespace(open=open_device, close=closed.append,
                              write=write_device, read=read_device, O_RDWR=2, environ={})
    fake_fcntl = SimpleNamespace(ioctl=MagicMock(), flock=MagicMock(), LOCK_EX=2, LOCK_UN=8)
    monkeypatch.setattr(md, "os", fake_os)
    monkeypatch.setattr(md, "fcntl", fake_fcntl)
    return SimpleNamespace(registers=registers, writes=writes, closed=closed,
                           fcntl=fake_fcntl, os=fake_os)


def test_second_open_preserves_every_output_and_frequency(chip):
    first = md.PCA9685().open()
    first.set_pwm(0, 1200)
    first.set_pwm(8, 1750)
    before = bytes(chip.registers)
    chip.writes.clear()
    second = md.PCA9685().open()
    assert bytes(chip.registers) == before
    assert chip.writes == []  # No oscillator sleep/restart and no all-off.
    assert second.frequency_hz == md.prescale_to_frequency(chip.registers[md.PRESCALE])
    first.close()
    second.close()


def test_chassis_and_mechanism_initialization_are_isolated(chip):
    chassis = motor.DifferentialMotorDriver(backend="pca9685", pca9685=md.PCA9685().open())
    chassis.set_left_right(0.5, -0.5)
    chassis_before = {
        channel: bytes(chip.registers[md.LED0_ON_L + 4 * channel:md.LED0_ON_L + 4 * channel + 4])
        for channel in (9, 10)
    }
    mechanism = md.MechanismController(pca9685=md.PCA9685().open())
    mechanism.init_escs(hold_s=0)
    assert {
        channel: bytes(chip.registers[md.LED0_ON_L + 4 * channel:md.LED0_ON_L + 4 * channel + 4])
        for channel in (9, 10)
    } == chassis_before
    assert mechanism.action_history[-1][1]["channels"] == list(range(7))
    mechanism_before = bytes(chip.registers[md.LED0_ON_L:md.LED0_ON_L + 4 * 7])
    chassis.stop()
    chassis.off()
    assert bytes(chip.registers[md.LED0_ON_L:md.LED0_ON_L + 4 * 7]) == mechanism_before
    mechanism.close()
    chassis.close()


def test_frequency_change_rejected_while_esc_active(chip):
    driver = md.PCA9685().open()
    driver.set_pwm(9, 1500)
    before = bytes(chip.registers)
    with pytest.raises(RuntimeError, match="outputs are active"):
        driver.set_frequency(100)
    assert bytes(chip.registers) == before
    driver.close()


def test_failed_initialization_releases_descriptor(chip):
    chip.fcntl.ioctl.side_effect = OSError("no adapter")
    driver = md.PCA9685()
    with pytest.raises(OSError):
        driver.open()
    assert driver.fd is None
    assert chip.closed == [100]


def test_pwm_is_one_transaction_using_actual_frequency(chip):
    driver = md.PCA9685().open()
    chip.writes.clear()
    chip.fcntl.flock.reset_mock()
    counts = driver.set_pwm(8, 1500)
    expected = round(1500 * driver.frequency_hz * 4096 / 1e6)
    assert counts == expected
    assert chip.writes == [bytes((md.LED0_ON_L + 4 * 8, 0, 0, expected & 255, expected >> 8))]
    assert chip.fcntl.flock.call_args_list == [call(driver.fd, 2), call(driver.fd, 8)]
    driver.close()


def test_short_write_releases_transaction_lock(chip):
    driver = md.PCA9685().open()
    chip.os.write = lambda fd, payload: 2
    with pytest.raises(RuntimeError, match="short I2C channel write"):
        driver.set_pwm(8, 1500)
    assert driver._io_depth == 0
    assert chip.fcntl.flock.call_args == call(driver.fd, chip.fcntl.LOCK_UN)
    driver.close()


def test_i2c_environment_overrides(chip):
    chip.os.environ.update(PCA9685_I2C_BUS="3", PCA9685_I2C_ADDRESS="0x41",
                           PCA9685_I2C_DEVICE="/dev/i2c-lvds")
    driver = md.PCA9685()
    assert (driver.bus, driver.address, driver.device) == (3, 0x41, "/dev/i2c-lvds")


def test_linux_motor_default_requires_pca9685(monkeypatch):
    monkeypatch.setattr(motor, "os", SimpleNamespace(name="posix", environ={}))
    factory = MagicMock(side_effect=RuntimeError("I2C unavailable"))
    monkeypatch.setattr(md, "PCA9685", factory)
    with pytest.raises(RuntimeError, match="I2C unavailable"):
        motor.DifferentialMotorDriver()
    factory.assert_called_once()


def test_motor_neutral_and_off_use_only_ch10_ch9():
    pca = MagicMock()
    driver = motor.DifferentialMotorDriver(backend="pca9685", pca9685=pca)
    assert driver.stop() == (0, 0)
    assert pca.channel_off.call_args_list == [call(10), call(9)]
    driver.off()
    assert pca.channel_off.call_args_list == [call(10), call(9), call(10), call(9)]
    pca.all_off.assert_not_called()


def test_daemon_timeout_hard_disables_pwm(monkeypatch):
    driver = MagicMock()
    driver.set_cmd_normalized.return_value = (1, 1, 1750, 1750)
    sock = MagicMock()
    sock.recvfrom.side_effect = [BlockingIOError, (b"cmd 1 0", ("127.0.0.1", 12)),
                                socket.timeout, KeyboardInterrupt]
    monkeypatch.setattr(motor, "build_driver", lambda args: driver)
    monkeypatch.setattr(motor.socket, "socket", lambda *args: sock)
    monkeypatch.setattr(motor.signal, "signal", lambda *args: None)
    monkeypatch.setattr(motor.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(motor.time, "monotonic", MagicMock(side_effect=[0, 0.1, 0.2, 0.6, 0.7]))
    args = SimpleNamespace(host="127.0.0.1", port=8765, timeout=0.3, arm_seconds=3)
    with pytest.raises(KeyboardInterrupt):
        motor.run_daemon(args)
    driver.stop.assert_not_called()
    assert driver.off.call_count == 3  # Startup, command timeout, final shutdown.
    driver.close.assert_called_once()
    sock.close.assert_called_once()


def test_zero_track_is_disabled_while_nonzero_track_is_driven():
    pca = MagicMock()
    driver = motor.DifferentialMotorDriver(
        backend="pca9685", pca9685=pca, start_boost=False
    )

    assert driver.set_left_right(0.0, 0.2) == (0, 1450)
    pca.channel_off.assert_called_once_with(10)
    pca.set_pwm.assert_called_once_with(9, 1450)


def receiver(serial_port=None, **kwargs):
    return IBusRCReceiver(serial_port=serial_port or MagicMock(), auto_start=False,
                         debounce_samples=1, **kwargs)


def feed(rx, data):
    rx._buffer.extend(data)
    rx._consume_frames()


def test_ibus_fragmented_noisy_stream_maps_receiver_ch8_to_mode():
    rx = receiver()
    frame = build_ibus_frame({3: 1900, 1: 1200, 8: 1000})
    feed(rx, b"noise" + frame[:1])
    feed(rx, frame[1:17])
    assert rx.read("CH1") is None
    feed(rx, frame[17:])
    assert rx.read() == {"CH1": 1900.0, "CH3": 1200.0, "CH5": 1000.0}
    rx.stop()


def test_ibus_bad_checksum_and_invalid_controls_do_not_refresh_signal():
    rx = receiver()
    frame = bytearray(build_ibus_frame({3: 1800}))
    frame[-1] ^= 1
    assert parse_ibus_frame(bytes(frame)) is None
    feed(rx, frame + build_ibus_frame({3: 65535, 1: 1800, 8: 1000}))
    assert rx.read() == {"CH1": None, "CH3": None, "CH5": None}
    feed(rx, build_ibus_frame({3: 1900, 1: 1500, 8: 1000}))
    assert rx.read("CH1") == 1900
    rx.stop()


def test_ibus_stale_signal_stops_manual_chassis():
    now = [10.0]
    rx = receiver(clock=lambda: now[0])
    bridge = MagicMock()
    control = RCControl(rx, bridge, debounce_samples=1)
    feed(rx, build_ibus_frame({3: 2000, 1: 1500, 8: 1000}))
    control.tick()
    assert control.last_cmd_vel[0] > 0
    now[0] += 0.6
    control.tick()
    assert control.last_cmd_vel == (0.0, 0.0)
    rx.stop()


def test_usb_disconnect_immediately_discards_old_throttle():
    serial_port = MagicMock()
    serial_port.read.side_effect = OSError("USB unplugged")
    rx = receiver(serial_port)
    feed(rx, build_ibus_frame({3: 2000, 8: 1000}))
    assert rx.read("CH1") == 2000
    rx._read_loop()
    assert not rx.is_available
    assert rx.read("CH1") is None
    serial_port.close.assert_called_once()
    rx.stop()


def test_linux_receiver_default_does_not_fall_back_to_gpio(monkeypatch):
    monkeypatch.setattr(rc, "os", SimpleNamespace(name="posix", environ={}))
    monkeypatch.setattr(IBusRCReceiver, "_open_serial", MagicMock(side_effect=OSError("USB missing")))
    with pytest.raises(OSError, match="USB missing"):
        rc.create_rc_receiver(auto_start=False)


def test_serial_factory_passes_configured_usb_device(monkeypatch):
    fake_serial_module = SimpleNamespace(Serial=MagicMock(return_value=MagicMock()),
                                        EIGHTBITS=8, PARITY_NONE="N", STOPBITS_ONE=1)
    monkeypatch.setitem(sys.modules, "serial", fake_serial_module)
    monkeypatch.setenv("RC_SERIAL_PORT", "/dev/rc_receiver")
    rx = rc.create_rc_receiver(backend="ibus", auto_start=False)
    assert fake_serial_module.Serial.call_args.kwargs == dict(
        port="/dev/rc_receiver", baudrate=115200, bytesize=8, parity="N", stopbits=1, timeout=0.05)
    rx.stop()
