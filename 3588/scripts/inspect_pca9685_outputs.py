#!/usr/bin/env python3
"""Read PCA9685 output registers without changing any channel settings.

Run beside the ROS-free mechanism daemon while testing CH2-4 and CH8-10.
This reports chip register state; it cannot measure the voltage at a wire or
whether the PCA9685 OE pin is externally disabled.
"""

from __future__ import annotations

import argparse
import os
import time


CHANNELS = (2, 3, 4, 8, 9, 10)
LED0_ON_L = 0x06
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE


def decode_channel(registers: tuple[int, int, int, int], mode2: int,
                   frequency_hz: float) -> str:
    on_l, on_h, off_l, off_h = registers
    inverted = bool(mode2 & 0x10)
    if off_h & 0x10:
        return "HIGH" if inverted else "LOW"
    if on_h & 0x10:
        return "LOW" if inverted else "HIGH"
    on = on_l | ((on_h & 0x0F) << 8)
    off = off_l | ((off_h & 0x0F) << 8)
    duty = (off - on) % 4096 / 4096 * 100
    if inverted:
        duty = 100 - duty
    return f"PWM {duty:.2f}% ({duty / 100 / frequency_hz * 1_000_000:.0f} us high)"


def read_register(fd: int, address: int) -> int:
    if os.write(fd, bytes((address,))) != 1:
        raise OSError(f"failed to select register 0x{address:02x}")
    value = os.read(fd, 1)
    if len(value) != 1:
        raise OSError(f"failed to read register 0x{address:02x}")
    return value[0]


def snapshot(fd: int, oscillator_hz: float) -> str:
    import fcntl

    # The daemon uses the same device-file flock around whole I2C operations.
    fcntl.flock(fd, fcntl.LOCK_EX)
    try:
        mode1 = read_register(fd, MODE1)
        mode2 = read_register(fd, MODE2)
        prescale = read_register(fd, PRESCALE)
        frequency_hz = oscillator_hz / (4096 * (prescale + 1))
        outputs = []
        for channel in CHANNELS:
            base = LED0_ON_L + channel * 4
            registers = tuple(read_register(fd, base + index) for index in range(4))
            outputs.append(f"CH{channel}={decode_channel(registers, mode2, frequency_hz)}")
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
    return (f"MODE1=0x{mode1:02x} MODE2=0x{mode2:02x} "
            f"PWM={frequency_hz:.2f}Hz | " + " | ".join(outputs))


def main() -> int:
    import fcntl
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "src"))
    from utils.sampling_params import (PCA9685_I2C_ADDRESS, PCA9685_I2C_BUS,
                                       PCA9685_I2C_DEVICE, PCA9685_OSCILLATOR_HZ)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", type=float, default=0,
                        help="watch for this many seconds, printing changes")
    parser.add_argument("--interval", type=float, default=0.2,
                        help="seconds between watched reads (default: 0.2)")
    args = parser.parse_args()
    if args.watch < 0 or args.interval < 0.1:
        parser.error("--watch must be >= 0 and --interval must be >= 0.1")

    device = (os.getenv("PCA9685_I2C_DEVICE", "").strip()
              or PCA9685_I2C_DEVICE or f"/dev/i2c-{os.getenv('PCA9685_I2C_BUS', PCA9685_I2C_BUS)}")
    address = int(os.getenv("PCA9685_I2C_ADDRESS", str(PCA9685_I2C_ADDRESS)), 0)
    fd = os.open(device, os.O_RDWR)
    try:
        fcntl.ioctl(fd, 0x0703, address)
        deadline = time.monotonic() + args.watch
        previous = None
        while True:
            current = snapshot(fd, PCA9685_OSCILLATOR_HZ)
            if current != previous:
                print(f"{time.strftime('%H:%M:%S')} {current}", flush=True)
                previous = current
            if not args.watch or time.monotonic() >= deadline:
                break
            time.sleep(args.interval)
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
