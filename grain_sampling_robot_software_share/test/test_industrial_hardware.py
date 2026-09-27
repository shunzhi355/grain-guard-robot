"""Industrial PC wiring and shared-chip regressions; no physical hardware needed."""
import os
import argparse
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


def test_production_cli_matches_standalone_linear_pwm():
    parser = argparse.ArgumentParser()
    motor.add_common_args(parser)
    production = motor.build_driver(parser.parse_args(["--backend", "mock"]))
    standalone = motor.DifferentialMotorDriver(
        backend="mock", start_boost=False, deadband=0.0
    )
    for linear, angular in [(0, 0), (0.01, 0), (0.1, 0), (0.5, 0),
                            (1, 0), (-0.1, 0), (-1, 0), (0, 0.2), (0.3, -0.1)]:
        assert production.set_cmd_normalized(linear, angular) == standalone.set_cmd_normalized(linear, angular)
    assert production.set_left_right(0.1, 0.1) == (1515, 1515)


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


def test_open_reinitializes_clock_and_neutral_like_original(chip):
    first = md.PCA9685().open()
    first.set_pwm(0, 1200)
    first.set_pwm(8, 1750)
    unused = bytes(chip.registers[38:70])
    chip.writes.clear()
    second = md.PCA9685().open()
    assert bytes([md.PRESCALE, 133]) in chip.writes
    assert list(chip.registers[6:10]) == [0, 0, 51, 1]
    assert bytes(chip.registers[38:70]) == unused
    assert second.frequency_hz == 50.0
    first.close()
    second.close()


def test_original_calibration_and_target_frequency_counts(chip):
    assert md.OSCILLATOR_HZ == 27_545_088
    driver = md.PCA9685().open()
    assert (driver.device, driver.address) == ('/dev/i2c-2', 0x40)
    assert chip.registers[md.PRESCALE] == 133
    assert chip.registers[md.MODE1] & 0x7f == 0x21
    assert chip.registers[md.MODE2] == 0x04
    for ch in range(8):
        assert driver.set_pwm(ch, 1500) == 307
        base = md.LED0_ON_L + ch * 4
        assert list(chip.registers[base:base + 4]) == [0, 0, 51, 1]


def test_original_initialization_always_wakes_oscillator(chip):
    driver = md.PCA9685().open()
    chip.writes.clear()
    driver.set_frequency(50)
    assert chip.writes == [
        bytes([md.MODE1, 0x31]), bytes([md.PRESCALE, 133]),
        bytes([md.MODE1, 0xa1]), bytes([md.MODE1, 0xa1]),
        bytes([md.MODE2, 4]),
    ]


def test_original_mode2_preserves_bits_and_sets_outdrv(chip):
    chip.registers[md.MODE2] = 0
    driver = md.PCA9685().open()
    assert chip.registers[md.MODE2] == 4
    driver.close()


def test_original_write_uses_four_register_transactions(chip):
    driver = md.PCA9685().open()
    chip.writes.clear()
    driver.set_pwm(0, 1900)
    assert chip.writes == [bytes([6, 0]), bytes([7, 0]),
                           bytes([8, 133]), bytes([9, 1])]


def test_init_failure_returns_all_channels_to_neutral(chip):
    controller = md.MechanismController(pca9685=md.PCA9685().open())
    original_set = controller.pca9685.set_pwm
    def fail_ch3(channel, pulse):
        if channel == 3:
            raise OSError('simulated NACK')
        return original_set(channel, pulse)
    controller.pca9685.set_pwm = fail_ch3
    with pytest.raises(RuntimeError, match='failed to write pulse'):
        controller.init_escs(hold_s=0)
    assert controller._stop_flag.is_set()
    for ch in range(8):
        assert list(chip.registers[6 + ch * 4:10 + ch * 4]) == [0, 0, 51, 1]


def test_real_fan_uses_ch7_and_stop_keeps_neutral_pwm(chip):
    controller = md.MechanismController(pca9685=md.PCA9685().open())
    controller.fan()
    assert not chip.registers[9 + 7 * 4] & 16
    controller.set_stop(7)
    assert list(chip.registers[34:38]) == [0, 0, 51, 1]


def test_mechanism_init_and_shutdown_only_touch_configured_channels(chip):
    mechanism = md.MechanismController(pca9685=md.PCA9685().open())
    unused_start = md.LED0_ON_L + 4 * 8
    unused_before = bytes(chip.registers[unused_start:md.LED0_ON_L + 4 * 16])
    mechanism.init_escs(hold_s=0)
    assert mechanism.action_history[-1][1]["channels"] == list(range(8))
    mechanism.pca9685.all_off()
    for channel in range(8):
        base = md.LED0_ON_L + 4 * channel
        assert list(chip.registers[base:base + 4]) == [0, 0, 51, 1]
    assert bytes(chip.registers[unused_start:md.LED0_ON_L + 4 * 16]) == unused_before
    mechanism.close()


def test_shutdown_and_close_keep_every_channel_neutral(chip):
    controller = md.MechanismController(pca9685=md.PCA9685().open())
    controller.init_escs(hold_s=0)
    controller.set_pulse(5, 1900)
    controller.set_stop(2)
    chip.writes.clear()
    controller.close()
    assert controller.pca9685.fd is None
    for ch in range(8):
        assert list(chip.registers[6 + ch * 4:10 + ch * 4]) == [0, 0, 51, 1]
    assert not any(len(p) == 5 and p[4] & 16 for p in chip.writes)


def test_stop_ignores_grain_specific_stop_override(chip):
    controller = md.MechanismController(pca9685=md.PCA9685().open())
    controller.stop_value = 1400
    controller.actuate(0, 'stop')
    assert list(chip.registers[6:10]) == [0, 0, 51, 1]


def test_original_write_failure_propagates(chip):
    driver = md.PCA9685().open()
    chip.os.write = lambda fd, payload: 0
    with pytest.raises(RuntimeError, match='short I2C write'):
        driver.set_pwm(0, 1900)
    assert driver._io_depth == 0


def test_original_frequency_change_reprograms_prescaler(chip):
    driver = md.PCA9685().open()
    driver.set_frequency(100)
    assert driver.frequency_hz == 100
    assert chip.registers[md.PRESCALE] == md.frequency_to_prescale(100)


def test_failed_initialization_releases_descriptor(chip):
    chip.fcntl.ioctl.side_effect = OSError("no adapter")
    driver = md.PCA9685()
    with pytest.raises(OSError):
        driver.open()
    assert driver.fd is None
    assert chip.closed == [100]


def test_pwm_register_writes_share_transaction_lock(chip):
    driver = md.PCA9685().open()
    chip.writes.clear()
    chip.fcntl.flock.reset_mock()
    assert driver.set_pwm(8, 1500) == 307
    assert chip.writes == [bytes([38, 0]), bytes([39, 0]),
                           bytes([40, 51]), bytes([41, 1])]
    assert chip.fcntl.flock.call_args_list == [call(driver.fd, 2), call(driver.fd, 8)]
    driver.close()


def test_short_write_releases_transaction_lock(chip):
    driver = md.PCA9685().open()
    chip.os.write = lambda fd, payload: 1
    with pytest.raises(RuntimeError, match="short I2C write"):
        driver.set_pwm(8, 1500)
    assert driver._io_depth == 0
    assert chip.fcntl.flock.call_args == call(driver.fd, chip.fcntl.LOCK_UN)
    driver.close()


def test_i2c_environment_overrides(chip):
    chip.os.environ.update(PCA9685_I2C_BUS="3", PCA9685_I2C_ADDRESS="0x41",
                           PCA9685_I2C_DEVICE="/dev/i2c-lvds")
    driver = md.PCA9685()
    assert (driver.bus, driver.address, driver.device) == (3, 0x41, "/dev/i2c-lvds")


def test_linux_motor_requires_explicit_legacy_backend(monkeypatch):
    monkeypatch.setattr(motor, "os", SimpleNamespace(name="posix", environ={}))
    with pytest.raises(ValueError, match="explicit sysfs or mock"):
        motor.DifferentialMotorDriver()


def test_removed_backend_cannot_open_i2c(monkeypatch):
    factory = MagicMock()
    monkeypatch.setattr(md, "PCA9685", factory)
    with pytest.raises(ValueError, match="STM32 serial"):
        motor.DifferentialMotorDriver(backend="pca9685")
    factory.assert_not_called()


def test_daemon_timeout_returns_neutral(monkeypatch):
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
    assert driver.stop.call_count == 3  # Arming, command timeout, final shutdown.
    driver.close.assert_called_once()
    sock.close.assert_called_once()


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
