#!/usr/bin/env python3
"""Standalone i-BUS receiver to PCA9685 chassis PWM test service."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
for path in (str(PROJECT_ROOT), str(SRC_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from dipan.motor_driver import DifferentialMotorDriver  # noqa: E402
from grain_sampling_devices.ibus_receiver import IBusRCReceiver  # noqa: E402
from grain_sampling_workflow.rc_control import RCControl  # noqa: E402
from utils.sampling_params import RC_MAX_ANGULAR_RPS, RC_MAX_LINEAR_MPS  # noqa: E402


LOG = logging.getLogger("rc_pwm_test")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class DirectMotorBridge:
    """Small RCControl bridge that writes directly to the motor driver."""

    def __init__(self, driver: DifferentialMotorDriver) -> None:
        self.driver = driver
        self.last_result: Optional[tuple[float, float, int, int]] = None

    def cancel_goal(self) -> None:
        left_us, right_us = self.driver.stop()
        self.last_result = (0.0, 0.0, left_us, right_us)

    def publish_cmd_vel(self, linear_mps: float, angular_rps: float) -> None:
        linear = clamp(linear_mps / RC_MAX_LINEAR_MPS, -1.0, 1.0)
        angular = clamp(angular_rps / RC_MAX_ANGULAR_RPS, -1.0, 1.0)
        self.last_result = self.driver.set_cmd_normalized(linear, angular)

    def stop(self) -> None:
        left_us, right_us = self.driver.stop()
        self.last_result = (0.0, 0.0, left_us, right_us)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test i-BUS remote control to PCA9685 CH8/CH9 PWM output"
    )
    parser.add_argument("--port", default=os.environ.get("RC_SERIAL_PORT", "/dev/ttyUSB0"))
    parser.add_argument(
        "--baudrate", type=int, default=int(os.environ.get("RC_SERIAL_BAUDRATE", "115200"))
    )
    parser.add_argument("--tick-hz", type=float, default=20.0)
    parser.add_argument("--arm-seconds", type=float, default=3.0)
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--max-offset-us", type=int, default=250)
    return parser


def run(args: argparse.Namespace) -> int:
    if args.tick_hz <= 0 or args.arm_seconds < 0 or args.report_interval <= 0:
        raise ValueError("tick-hz/report-interval must be > 0 and arm-seconds must be >= 0")

    running = True

    def handle_signal(_signum, _frame) -> None:  # type: ignore[no-untyped-def]
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    driver: Optional[DifferentialMotorDriver] = None
    receiver: Optional[IBusRCReceiver] = None
    try:
        driver = DifferentialMotorDriver(backend="pca9685", max_offset_us=args.max_offset_us)
        bridge = DirectMotorBridge(driver)
        bridge.stop()
        LOG.info("PCA9685 initialized; CH8/CH9 neutral at 1500 us for %.1f s", args.arm_seconds)
        deadline = time.monotonic() + args.arm_seconds
        while running and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        if not running:
            return 0

        receiver = IBusRCReceiver(port=args.port, baudrate=args.baudrate)
        control = RCControl(receiver, bridge)
        LOG.info(
            "RC PWM test ready: %s at %d baud; physical i-BUS CH8 selects manual/auto",
            args.port,
            args.baudrate,
        )

        period = 1.0 / args.tick_hz
        next_report = 0.0
        previous_mode: Optional[str] = None
        while running:
            loop_started = time.monotonic()
            mode = control.tick()
            if mode != RCControl.MODE_MANUAL:
                # Auto, undecided, or lost mode input must never retain a drive command.
                bridge.stop()

            if mode != previous_mode:
                LOG.info("control mode: %s", mode or "waiting-for-valid-switch")
                previous_mode = mode

            now = time.monotonic()
            if now >= next_report:
                values = receiver.read()
                result = bridge.last_result or (0.0, 0.0, 1500, 1500)
                LOG.info(
                    "mode=%s CH1=%s CH3=%s CH8=%s tracks=(%.3f,%.3f) PWM=(%d,%d)us",
                    mode or "waiting",
                    values.get("CH1"),
                    values.get("CH3"),
                    values.get("CH5"),
                    result[0],
                    result[1],
                    result[2],
                    result[3],
                )
                next_report = now + args.report_interval

            remaining = period - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
        return 0
    finally:
        if receiver is not None:
            receiver.stop()
        if driver is not None:
            try:
                driver.stop()
                LOG.info("service stopped; CH8/CH9 returned to 1500 us")
            finally:
                driver.close()


def main(argv: Optional[list[str]] = None) -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        return run(build_parser().parse_args(argv))
    except Exception:
        LOG.exception("RC PWM test failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
