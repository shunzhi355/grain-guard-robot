#!/usr/bin/env python3
"""PCA9685 command-line driver for Orange Pi 5 Max.

Default hardware configuration:
  I2C bus:  /dev/i2c-2 (I2C2_M4, physical pins 11/13)
  Address:  0x40
  VCC:      3.3 V

No third-party Python module is required. The driver talks to Linux i2c-dev
directly. Run with sudo unless the user has permission to access /dev/i2c-2.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence


I2C_SLAVE = 0x0703

MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
PRESCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_AUTO_INCREMENT = 0x20
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01
MODE2_OUTDRV = 0x04

FULL_ON_OFF_BIT = 0x10
CHANNEL_COUNT = 16
COUNTS_PER_CYCLE = 4096
OSCILLATOR_HZ = 25_000_000.0
DEFAULT_FREQUENCY_HZ = 50.0


def parse_int(value: str) -> int:
    return int(value, 0)


def require_range(name: str, value: float, minimum: float, maximum: float) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}, got {value}")


def frequency_to_prescale(frequency_hz: float) -> int:
    if frequency_hz <= 0:
        raise ValueError("frequency must be greater than zero")
    prescale = int(round(OSCILLATOR_HZ / (COUNTS_PER_CYCLE * frequency_hz) - 1.0))
    if not 3 <= prescale <= 255:
        raise ValueError(
            f"frequency {frequency_hz:g} Hz is outside the PCA9685 prescale range"
        )
    return prescale


def prescale_to_frequency(prescale: int) -> float:
    return OSCILLATOR_HZ / (COUNTS_PER_CYCLE * (prescale + 1))


@dataclass(frozen=True)
class ChannelState:
    on_count: int
    off_count: int
    full_on: bool
    full_off: bool

    @property
    def active_counts(self) -> int:
        if self.full_off:
            return 0
        if self.full_on:
            return COUNTS_PER_CYCLE
        return (self.off_count - self.on_count) % COUNTS_PER_CYCLE


class PCA9685:
    def __init__(self, bus: int = 2, address: int = 0x40) -> None:
        require_range("I2C address", address, 0x03, 0x77)
        self.bus = bus
        self.address = address
        self.device = f"/dev/i2c-{bus}"
        self.fd: Optional[int] = None

    def __enter__(self) -> "PCA9685":
        try:
            self.fd = os.open(self.device, os.O_RDWR)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"{self.device} does not exist. Check the i2c2-m4 overlay and reboot."
            ) from exc
        except PermissionError as exc:
            raise RuntimeError(
                f"permission denied for {self.device}; run this command with sudo"
            ) from exc

        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
        except OSError:
            self.close()
            raise
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _require_open(self) -> int:
        if self.fd is None:
            raise RuntimeError("I2C device is not open")
        return self.fd

    def write_register(self, register: int, value: int) -> None:
        require_range("register", register, 0x00, 0xFF)
        require_range("register value", value, 0x00, 0xFF)
        written = os.write(self._require_open(), bytes((register, value)))
        if written != 2:
            raise RuntimeError(f"short I2C write: expected 2 bytes, wrote {written}")

    def read_register(self, register: int) -> int:
        require_range("register", register, 0x00, 0xFF)
        fd = self._require_open()
        if os.write(fd, bytes((register,))) != 1:
            raise RuntimeError("failed to select PCA9685 register")
        data = os.read(fd, 1)
        if len(data) != 1:
            raise RuntimeError("short I2C read from PCA9685")
        return data[0]

    def set_frequency(self, frequency_hz: float) -> float:
        prescale = frequency_to_prescale(frequency_hz)
        old_mode = self.read_register(MODE1)
        sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP
        awake_mode = (old_mode & ~MODE1_SLEEP) | MODE1_AUTO_INCREMENT | MODE1_ALLCALL

        self.write_register(MODE1, sleep_mode)
        self.write_register(PRESCALE, prescale)
        self.write_register(MODE1, awake_mode)
        time.sleep(0.005)
        self.write_register(MODE1, awake_mode | MODE1_RESTART)
        self.write_register(MODE2, self.read_register(MODE2) | MODE2_OUTDRV)
        return prescale_to_frequency(prescale)

    def wake(self) -> None:
        mode = self.read_register(MODE1)
        if mode & MODE1_SLEEP:
            mode = (mode & ~MODE1_SLEEP) | MODE1_AUTO_INCREMENT | MODE1_ALLCALL
            self.write_register(MODE1, mode)
            time.sleep(0.005)
            self.write_register(MODE1, mode | MODE1_RESTART)

    def sleep(self) -> None:
        mode = self.read_register(MODE1)
        self.write_register(MODE1, (mode & ~MODE1_RESTART) | MODE1_SLEEP)

    @staticmethod
    def _channel_base(channel: int) -> int:
        require_range("channel", channel, 0, CHANNEL_COUNT - 1)
        return LED0_ON_L + 4 * channel

    def set_pwm(
        self,
        channel: int,
        on_count: int,
        off_count: int,
        *,
        full_on: bool = False,
        full_off: bool = False,
    ) -> None:
        require_range("on_count", on_count, 0, COUNTS_PER_CYCLE - 1)
        require_range("off_count", off_count, 0, COUNTS_PER_CYCLE - 1)
        if full_on and full_off:
            raise ValueError("full_on and full_off cannot both be true")

        base = self._channel_base(channel)
        self.write_register(base, on_count & 0xFF)
        self.write_register(base + 1, ((on_count >> 8) & 0x0F) | (FULL_ON_OFF_BIT if full_on else 0))
        self.write_register(base + 2, off_count & 0xFF)
        self.write_register(base + 3, ((off_count >> 8) & 0x0F) | (FULL_ON_OFF_BIT if full_off else 0))

    def set_duty_cycle(self, channel: int, percent: float) -> None:
        require_range("percent", percent, 0.0, 100.0)
        if percent == 0.0:
            self.set_pwm(channel, 0, 0, full_off=True)
        elif percent == 100.0:
            self.set_pwm(channel, 0, 0, full_on=True)
        else:
            counts = max(1, min(4095, int(round(percent * COUNTS_PER_CYCLE / 100.0))))
            self.set_pwm(channel, 0, counts)

    def set_pulse_us(self, channel: int, pulse_us: float, frequency_hz: float) -> int:
        period_us = 1_000_000.0 / frequency_hz
        if not 0.0 < pulse_us < period_us:
            raise ValueError(
                f"pulse must be greater than 0 and less than the {period_us:g} us period"
            )
        counts = max(
            1,
            min(
                COUNTS_PER_CYCLE - 1,
                int(round(pulse_us * frequency_hz * COUNTS_PER_CYCLE / 1_000_000.0)),
            ),
        )
        self.set_pwm(channel, 0, counts)
        return counts

    def off(self, channel: int) -> None:
        self.set_pwm(channel, 0, 0, full_off=True)

    def all_off(self) -> None:
        for channel in range(CHANNEL_COUNT):
            self.off(channel)

    def channel_state(self, channel: int) -> ChannelState:
        base = self._channel_base(channel)
        on_l = self.read_register(base)
        on_h = self.read_register(base + 1)
        off_l = self.read_register(base + 2)
        off_h = self.read_register(base + 3)
        return ChannelState(
            on_count=on_l | ((on_h & 0x0F) << 8),
            off_count=off_l | ((off_h & 0x0F) << 8),
            full_on=bool(on_h & FULL_ON_OFF_BIT),
            full_off=bool(off_h & FULL_ON_OFF_BIT),
        )

    def status(self, selected_channel: Optional[int] = None) -> str:
        mode1 = self.read_register(MODE1)
        mode2 = self.read_register(MODE2)
        prescale = self.read_register(PRESCALE)
        frequency = prescale_to_frequency(prescale)
        lines = [
            f"device={self.device} address=0x{self.address:02x}",
            f"MODE1=0x{mode1:02x} ({'sleep' if mode1 & MODE1_SLEEP else 'awake'})",
            f"MODE2=0x{mode2:02x}",
            f"PRESCALE=0x{prescale:02x} frequency={frequency:.3f} Hz",
        ]

        channels: Sequence[int]
        channels = range(CHANNEL_COUNT) if selected_channel is None else (selected_channel,)
        for channel in channels:
            state = self.channel_state(channel)
            duty = state.active_counts * 100.0 / COUNTS_PER_CYCLE
            pulse_us = state.active_counts * 1_000_000.0 / (frequency * COUNTS_PER_CYCLE)
            if state.full_off:
                detail = "OFF"
            elif state.full_on:
                detail = "ON 100%"
            else:
                detail = (
                    f"on={state.on_count} off={state.off_count} "
                    f"duty={duty:.3f}% pulse={pulse_us:.1f} us"
                )
            lines.append(f"channel {channel:02d}: {detail}")
        return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Control a PCA9685 through Orange Pi 5 Max I2C2_M4"
    )
    parser.add_argument("--bus", type=int, default=2, help="I2C bus number (default: 2)")
    parser.add_argument(
        "--address",
        type=parse_int,
        default=0x40,
        help="7-bit PCA9685 address (default: 0x40)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init_parser = subparsers.add_parser("init", help="initialize frequency and turn all channels off")
    init_parser.add_argument("frequency", type=float, nargs="?", default=DEFAULT_FREQUENCY_HZ)

    frequency_parser = subparsers.add_parser("frequency", help="set the shared PWM frequency")
    frequency_parser.add_argument("frequency", type=float)

    pulse_parser = subparsers.add_parser("pulse", help="output a pulse width in microseconds")
    pulse_parser.add_argument("channel", type=int)
    pulse_parser.add_argument("microseconds", type=float)
    pulse_parser.add_argument("--frequency", type=float, default=DEFAULT_FREQUENCY_HZ)

    duty_parser = subparsers.add_parser("duty", help="set a channel duty cycle in percent")
    duty_parser.add_argument("channel", type=int)
    duty_parser.add_argument("percent", type=float)
    duty_parser.add_argument("--frequency", type=float, default=DEFAULT_FREQUENCY_HZ)

    raw_parser = subparsers.add_parser("raw", help="set raw 12-bit ON and OFF counts")
    raw_parser.add_argument("channel", type=int)
    raw_parser.add_argument("on_count", type=int)
    raw_parser.add_argument("off_count", type=int)

    off_parser = subparsers.add_parser("off", help="turn one channel fully off")
    off_parser.add_argument("channel", type=int)

    subparsers.add_parser("all-off", help="turn all 16 channels fully off")
    subparsers.add_parser("sleep", help="stop the PCA9685 oscillator")
    subparsers.add_parser("wake", help="wake the PCA9685 oscillator")

    status_parser = subparsers.add_parser("status", help="show controller and channel status")
    status_parser.add_argument("channel", type=int, nargs="?")
    return parser


def run(args: argparse.Namespace) -> None:
    with PCA9685(args.bus, args.address) as driver:
        if args.command == "init":
            actual = driver.set_frequency(args.frequency)
            driver.all_off()
            print(f"PCA9685 initialized at {actual:.3f} Hz; all channels are off.")
        elif args.command == "frequency":
            actual = driver.set_frequency(args.frequency)
            print(f"PCA9685 frequency set to {actual:.3f} Hz.")
        elif args.command == "pulse":
            actual = driver.set_frequency(args.frequency)
            counts = driver.set_pulse_us(args.channel, args.microseconds, actual)
            print(
                f"channel {args.channel}: {args.microseconds:g} us at "
                f"{actual:.3f} Hz ({counts}/4096)."
            )
        elif args.command == "duty":
            actual = driver.set_frequency(args.frequency)
            driver.set_duty_cycle(args.channel, args.percent)
            print(f"channel {args.channel}: {args.percent:g}% at {actual:.3f} Hz.")
        elif args.command == "raw":
            driver.wake()
            driver.set_pwm(args.channel, args.on_count, args.off_count)
            print(
                f"channel {args.channel}: on={args.on_count}, off={args.off_count}."
            )
        elif args.command == "off":
            driver.off(args.channel)
            print(f"channel {args.channel}: off.")
        elif args.command == "all-off":
            driver.all_off()
            print("all PCA9685 channels are off.")
        elif args.command == "sleep":
            driver.sleep()
            print("PCA9685 is sleeping.")
        elif args.command == "wake":
            driver.wake()
            print("PCA9685 is awake.")
        elif args.command == "status":
            print(driver.status(args.channel))
        else:
            raise RuntimeError(f"unsupported command: {args.command}")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
