#!/usr/bin/env python3
"""
Differential tracked-base motor driver.

RK3588 (Orange Pi 5 Max) on-chip PWM:
  left  motor ESC signal: physical pin 7  -> pwm3-m3 -> fd8b0030.pwm
  right motor ESC signal: physical pin 16 -> pwm1-m2 -> fd8b0010.pwm

Jetson Orin NX 40-pin PWM (verified on board):
  left  motor ESC signal: physical pin 15 -> PWM1 -> pwmchip0 -> 3280000.pwm
  right motor ESC signal: physical pin 33 -> PWM5 -> pwmchip2 -> 32c0000.pwm

On the industrial PC the default backend is PCA9685 over Linux i2c-dev
(chassis outputs are CH8/CH9).  The historical sysfs PWM backend remains
available for Orange Pi/Jetson deployments.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

# Allow ``python dipan/motor_driver.py`` from the project root without an
# installed package; ROS launchers may still provide PYTHONPATH themselves.
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_SRC_DIR = _PROJECT_ROOT / "src"
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

try:
    from utils.sampling_params import (
        PCA9685_CHASSIS_LEFT,
        PCA9685_CHASSIS_RIGHT,
        PCA9685_I2C_ADDRESS,
        PCA9685_I2C_BUS,
        PCA9685_I2C_DEVICE,
    )
except ImportError:  # direct script execution before PYTHONPATH is set
    PCA9685_CHASSIS_LEFT, PCA9685_CHASSIS_RIGHT = 8, 9
    PCA9685_I2C_ADDRESS, PCA9685_I2C_BUS, PCA9685_I2C_DEVICE = 0x40, 4, ""


PERIOD_NS = 20_000_000  # 50 Hz
NEUTRAL_US = 1500
MIN_US = 1000
MAX_US = 2000
# 2026-08-26 用户要求默认半速（满速 1750us），原 500 全速 2000us 过快.
DEFAULT_MAX_OFFSET_US = 250
DEFAULT_DEADBAND = 0.03
DEFAULT_LEFT_FORWARD_MIN = 0.062
DEFAULT_LEFT_REVERSE_MIN = 0.186
DEFAULT_RIGHT_FORWARD_MIN = 0.054
DEFAULT_RIGHT_REVERSE_MIN = 0.191


# Platform-specific PWM node names.
#
# RK3588 (Orange Pi 5 Max) on-chip PWM:
RK3588_LEFT_NODE = "fd8b0030.pwm"
RK3588_RIGHT_NODE = "fd8b0010.pwm"

# Jetson Orin NX 40-pin PWM (verified on board):
#   left  -> pwmchip0 (Pin 15, PWM1 -> 3280000.pwm)
#   right -> pwmchip2 (Pin 33, PWM5 -> 32c0000.pwm)
JETSON_LEFT_NODE = "3280000.pwm"
JETSON_RIGHT_NODE = "32c0000.pwm"


@dataclass
class MotorConfig:
    name: str
    node: str
    invert_direction: bool = False
    polarity: str = "normal"


def detect_platform() -> str:
    """Detect the PWM platform from /sys/class/pwm.

    Returns "jetson" when pwmchip0 resolves to the 3280000.pwm controller,
    otherwise "rk3588" (the historical default / fallback). A non-Linux host
    (e.g. the Windows dev machine) falls back to "rk3588" since no hardware
    is touched at import time.
    """
    env = os.environ.get("MOTOR_DRIVER_PLATFORM", "").strip().lower()
    if env in ("jetson", "rk3588"):
        return env

    pwm_root = Path("/sys/class/pwm")
    if not pwm_root.is_dir():
        return "rk3588"

    try:
        chip0_real = os.path.realpath(pwm_root / "pwmchip0")
    except OSError:
        chip0_real = ""
    return "jetson" if "3280000.pwm" in chip0_real else "rk3588"


PLATFORM = detect_platform()


def _node(rk3588_node: str, jetson_node: str) -> str:
    return jetson_node if PLATFORM == "jetson" else rk3588_node


LEFT_MOTOR = MotorConfig(
    "left pin7", _node(RK3588_LEFT_NODE, JETSON_LEFT_NODE), invert_direction=True
)
RIGHT_MOTOR = MotorConfig(
    "right pin16", _node(RK3588_RIGHT_NODE, JETSON_RIGHT_NODE), invert_direction=True
)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def write_text(path: Path, value: str) -> None:
    path.write_text(value)


class PwmOutput:
    def __init__(self, config: MotorConfig, *, backend: str = "sysfs",
                 pca9685: object = None, channel: Optional[int] = None) -> None:
        self.config = config
        self.backend = backend
        self._pca9685 = pca9685
        self.channel = channel
        self._mock_pulse: Optional[int] = None
        self.base: Optional[Path] = None
        if backend == "sysfs":
            self.base = self._find_pwm_base(config.node)

    @staticmethod
    def _find_pwm_base(node: str) -> Path:
        pwm_root = Path("/sys/class/pwm")
        for chip in sorted(pwm_root.glob("pwmchip*")):
            try:
                real = os.path.realpath(chip)
            except OSError:
                continue
            if node in real:
                return PwmOutput._ensure_channel(chip)
        raise RuntimeError(
            f"PWM node {node} not found. Check overlays in /boot/orangepiEnv.txt."
        )

    @staticmethod
    def _ensure_channel(chip: Path) -> Path:
        # Normal sysfs PWM layout: pwmchipX/export then pwmchipX/pwm0/...
        channel = chip / "pwm0"
        if channel.exists():
            return channel

        # Some kernels expose controls directly under pwmchipX.
        if (chip / "period").exists() and (chip / "duty_cycle").exists():
            return chip

        export = chip / "export"
        if not export.exists():
            raise RuntimeError(f"{chip} has no export file and no direct PWM controls")

        try:
            write_text(export, "0")
        except OSError as exc:
            if channel.exists():
                return channel
            consumers = sorted((chip / "device").glob("consumer:*"))
            consumer_text = ", ".join(p.name for p in consumers) or "unknown"
            raise RuntimeError(
                f"Cannot export {chip}/pwm0: {exc}. "
                f"It may be busy. Consumers: {consumer_text}"
            ) from exc

        for _ in range(20):
            if channel.exists():
                return channel
            time.sleep(0.02)
        raise RuntimeError(f"{channel} was not created after export")

    def setup(self) -> None:
        if self.backend == "pca9685":
            return
        assert self.base is not None
        enable = self.base / "enable"
        if enable.exists():
            try:
                write_text(enable, "0")
            except OSError:
                pass
        write_text(self.base / "period", str(PERIOD_NS))
        if (self.base / "polarity").exists():
            write_text(self.base / "polarity", self.config.polarity)

    def set_pulse_us(self, pulse_us: int) -> None:
        pulse_us = int(clamp(pulse_us, MIN_US, MAX_US))
        if self.backend == "pca9685":
            if self._pca9685 is None or self.channel is None:
                raise RuntimeError("PCA9685 output is not initialized")
            self._pca9685.set_pwm(self.channel, pulse_us)
            return
        if self.backend == "mock":
            self._mock_pulse = pulse_us
            return
        assert self.base is not None
        self.setup()
        write_text(self.base / "duty_cycle", str(pulse_us * 1000))
        write_text(self.base / "enable", "1")

    def disable(self) -> None:
        if self.backend == "pca9685":
            if self._pca9685 is not None and self.channel is not None:
                self._pca9685.channel_off(self.channel)
            return
        if self.backend == "mock":
            self._mock_pulse = None
            return
        assert self.base is not None
        enable = self.base / "enable"
        if enable.exists():
            write_text(enable, "0")

    def status(self) -> str:
        if self.backend == "pca9685":
            return f"{self.config.name}: PCA9685 CH{self.channel}"
        if self.backend == "mock":
            return f"{self.config.name}: mock pulse={self._mock_pulse}"
        assert self.base is not None
        fields = []
        for name in ("period", "duty_cycle", "polarity", "enable"):
            path = self.base / name
            if path.exists():
                fields.append(f"{name}={path.read_text().strip()}")
        return f"{self.config.name}: {self.base} " + " ".join(fields)


class DifferentialMotorDriver:
    def __init__(
        self,
        max_offset_us: int = DEFAULT_MAX_OFFSET_US,
        deadband: float = DEFAULT_DEADBAND,
        left_forward_min: float = DEFAULT_LEFT_FORWARD_MIN,
        left_reverse_min: float = DEFAULT_LEFT_REVERSE_MIN,
        right_forward_min: float = DEFAULT_RIGHT_FORWARD_MIN,
        right_reverse_min: float = DEFAULT_RIGHT_REVERSE_MIN,
        start_boost: bool = True,
        forward_only: bool = False,
        invert_left: bool = False,
        invert_right: bool = False,
        backend: Optional[str] = None,
        pca9685: object = None,
    ) -> None:
        default_backend = "pca9685" if os.name == "posix" else "mock"
        selected = (backend or os.environ.get("MOTOR_DRIVER_BACKEND", default_backend)).strip().lower()
        if selected not in {"auto", "pca9685", "sysfs", "mock"}:
            raise ValueError("MOTOR_DRIVER_BACKEND must be auto, pca9685, sysfs or mock")
        if selected == "auto":
            # Missing I2C must not silently redirect this wiring to sysfs PWM.
            selected = default_backend
        self.backend = selected
        self._pca9685 = pca9685
        if selected == "pca9685" and self._pca9685 is None:
            from grain_sampling_devices.mechanism_driver import PCA9685
            self._pca9685 = PCA9685(
                bus=PCA9685_I2C_BUS,
                address=PCA9685_I2C_ADDRESS,
                device=PCA9685_I2C_DEVICE,
            ).open()
        left_cfg = MotorConfig(
            f"left CH{PCA9685_CHASSIS_LEFT}" if selected == "pca9685" else LEFT_MOTOR.name,
            LEFT_MOTOR.node, LEFT_MOTOR.invert_direction ^ invert_left
        )
        right_cfg = MotorConfig(
            f"right CH{PCA9685_CHASSIS_RIGHT}" if selected == "pca9685" else RIGHT_MOTOR.name,
            RIGHT_MOTOR.node, RIGHT_MOTOR.invert_direction ^ invert_right
        )
        self.left = PwmOutput(left_cfg, backend=selected, pca9685=self._pca9685,
                              channel=PCA9685_CHASSIS_LEFT)
        self.right = PwmOutput(right_cfg, backend=selected, pca9685=self._pca9685,
                               channel=PCA9685_CHASSIS_RIGHT)
        self.max_offset_us = int(clamp(max_offset_us, 1, 500))
        self.deadband = float(clamp(deadband, 0.0, 0.5))
        self.left_forward_min = float(clamp(left_forward_min, 0.0, 1.0))
        self.left_reverse_min = float(clamp(left_reverse_min, 0.0, 1.0))
        self.right_forward_min = float(clamp(right_forward_min, 0.0, 1.0))
        self.right_reverse_min = float(clamp(right_reverse_min, 0.0, 1.0))
        self.start_boost = start_boost
        self.forward_only = forward_only

    def _speed_to_pulse(self, speed: float, motor: PwmOutput) -> int:
        speed = clamp(speed, -1.0, 1.0)
        if abs(speed) < self.deadband:
            speed = 0.0
        if self.forward_only:
            speed = max(0.0, speed)
        if motor.config.invert_direction:
            speed = -speed
        speed = self._apply_start_boost(speed, motor)
        return int(round(NEUTRAL_US + speed * self.max_offset_us))

    def _apply_start_boost(self, speed: float, motor: PwmOutput) -> float:
        if not self.start_boost or speed == 0.0:
            return speed

        if motor is self.left:
            min_abs = self.left_forward_min if speed > 0.0 else self.left_reverse_min
        else:
            min_abs = self.right_forward_min if speed > 0.0 else self.right_reverse_min

        magnitude = abs(speed)
        boosted = min_abs + magnitude * (1.0 - min_abs)
        return (1.0 if speed > 0.0 else -1.0) * clamp(boosted, 0.0, 1.0)

    def set_left_right(self, left_speed: float, right_speed: float) -> Tuple[int, int]:
        left_pulse = self._speed_to_pulse(left_speed, self.left)
        right_pulse = self._speed_to_pulse(right_speed, self.right)
        self.left.set_pulse_us(left_pulse)
        self.right.set_pulse_us(right_pulse)
        return left_pulse, right_pulse

    def set_cmd_normalized(self, linear: float, angular: float) -> Tuple[float, float, int, int]:
        # Positive angular means turn left: left track slower, right track faster.
        left = linear - angular
        right = linear + angular
        scale = max(1.0, abs(left), abs(right))
        left /= scale
        right /= scale
        left_pulse, right_pulse = self.set_left_right(left, right)
        return left, right, left_pulse, right_pulse

    def set_twist(
        self,
        linear_mps: float,
        angular_rps: float,
        max_track_mps: float,
        track_width_m: float,
    ) -> Tuple[float, float, int, int]:
        left_mps = linear_mps - angular_rps * track_width_m / 2.0
        right_mps = linear_mps + angular_rps * track_width_m / 2.0
        left = left_mps / max_track_mps
        right = right_mps / max_track_mps
        scale = max(1.0, abs(left), abs(right))
        left /= scale
        right /= scale
        left_pulse, right_pulse = self.set_left_right(left, right)
        return left, right, left_pulse, right_pulse

    def stop(self) -> Tuple[int, int]:
        return self.set_left_right(0.0, 0.0)

    def off(self) -> None:
        self.left.disable()
        self.right.disable()

    def close(self) -> None:
        """Release the PCA9685 descriptor (sysfs/mock are no-ops)."""
        if self._pca9685 is not None:
            closer = getattr(self._pca9685, "close", None)
            if closer is not None:
                closer()

    def status(self) -> str:
        return self.left.status() + "\n" + self.right.status()


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--backend", choices=("pca9685", "sysfs", "mock", "auto"), default=None)
    parser.add_argument("--max-offset-us", type=int, default=DEFAULT_MAX_OFFSET_US)
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND)
    parser.add_argument("--left-forward-min", type=float, default=DEFAULT_LEFT_FORWARD_MIN)
    parser.add_argument("--left-reverse-min", type=float, default=DEFAULT_LEFT_REVERSE_MIN)
    parser.add_argument("--right-forward-min", type=float, default=DEFAULT_RIGHT_FORWARD_MIN)
    parser.add_argument("--right-reverse-min", type=float, default=DEFAULT_RIGHT_REVERSE_MIN)
    parser.add_argument("--no-start-boost", action="store_true")
    parser.add_argument("--forward-only", action="store_true")
    parser.add_argument("--invert-left", action="store_true")
    parser.add_argument("--invert-right", action="store_true")


def build_driver(args: argparse.Namespace) -> DifferentialMotorDriver:
    return DifferentialMotorDriver(
        backend=args.backend,
        max_offset_us=args.max_offset_us,
        deadband=args.deadband,
        left_forward_min=args.left_forward_min,
        left_reverse_min=args.left_reverse_min,
        right_forward_min=args.right_forward_min,
        right_reverse_min=args.right_reverse_min,
        start_boost=not args.no_start_boost,
        forward_only=args.forward_only,
        invert_left=args.invert_left,
        invert_right=args.invert_right,
    )


def parse_udp_payload(payload: str) -> Tuple[str, Optional[Tuple[float, float]]]:
    payload = payload.strip()
    if not payload:
        raise ValueError("empty command")
    if payload.lower() == "stop":
        return "stop", None

    if payload.startswith("{"):
        data = json.loads(payload)
        if data.get("stop"):
            return "stop", None
        if "left" in data and "right" in data:
            return "lr", (float(data["left"]), float(data["right"]))
        if "linear" in data and "angular" in data:
            return "cmd", (float(data["linear"]), float(data["angular"]))
        raise ValueError("JSON must contain stop, left/right, or linear/angular")

    parts = payload.split()
    if len(parts) == 3 and parts[0] in ("lr", "cmd"):
        return parts[0], (float(parts[1]), float(parts[2]))
    if len(parts) == 2:
        return "cmd", (float(parts[0]), float(parts[1]))
    raise ValueError("use: stop | cmd LINEAR ANGULAR | lr LEFT RIGHT")


def run_daemon(args: argparse.Namespace) -> int:
    driver = build_driver(args)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Reserve the port before touching outputs; a duplicate daemon must not
        # change the neutral state of an already running chassis.
        sock.bind((args.host, args.port))
        driver.stop()
        time.sleep(args.arm_seconds)
        sock.settimeout(0.05)
        # Drop commands queued during arming; require a fresh command afterward.
        sock.setblocking(False)
        try:
            while True:
                sock.recvfrom(2048)
        except BlockingIOError:
            pass
        sock.settimeout(0.05)
    except BaseException:
        driver.close()
        sock.close()
        raise

    last_command = time.monotonic()
    stopped_by_timeout = False
    running = True

    def handle_signal(signum, frame) -> None:  # type: ignore[no-untyped-def]
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"motor_driver UDP listening on {args.host}:{args.port}", flush=True)
    print("accepted: stop | cmd LINEAR ANGULAR | lr LEFT RIGHT | JSON", flush=True)

    try:
        while running:
            now = time.monotonic()
            if now - last_command > args.timeout:
                if not stopped_by_timeout:
                    driver.stop()
                    stopped_by_timeout = True
                time.sleep(0.02)

            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue

            try:
                mode, values = parse_udp_payload(data.decode("utf-8"))
                if mode == "stop":
                    pulses = driver.stop()
                    reply = {"ok": True, "mode": "stop", "pulses": pulses}
                elif mode == "lr" and values:
                    pulses = driver.set_left_right(values[0], values[1])
                    reply = {"ok": True, "mode": "lr", "input": values, "pulses": pulses}
                elif mode == "cmd" and values:
                    left, right, lp, rp = driver.set_cmd_normalized(values[0], values[1])
                    reply = {
                        "ok": True,
                        "mode": "cmd",
                        "input": values,
                        "tracks": (left, right),
                        "pulses": (lp, rp),
                    }
                else:
                    raise ValueError("invalid command")
                last_command = time.monotonic()
                stopped_by_timeout = False
            except Exception as exc:
                reply = {"ok": False, "error": str(exc)}

            sock.sendto((json.dumps(reply, ensure_ascii=False) + "\n").encode("utf-8"), addr)
    finally:
        try:
            driver.stop()
        finally:
            driver.close()
            sock.close()
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Tracked chassis ESC PWM motor driver")
    subparsers = parser.add_subparsers(dest="command", required=True)

    p_stop = subparsers.add_parser("stop", help="1500us neutral on both motors")
    add_common_args(p_stop)

    p_off = subparsers.add_parser("off", help="disable both PWM outputs")
    add_common_args(p_off)

    p_status = subparsers.add_parser("status", help="show PWM status")
    add_common_args(p_status)

    p_lr = subparsers.add_parser("lr", help="set left/right track speeds in -1..1")
    p_lr.add_argument("left", type=float)
    p_lr.add_argument("right", type=float)
    add_common_args(p_lr)

    p_cmd = subparsers.add_parser("cmd", help="normalized differential command")
    p_cmd.add_argument("linear", type=float, help="-1..1")
    p_cmd.add_argument("angular", type=float, help="-1..1, positive turns left")
    add_common_args(p_cmd)

    p_twist = subparsers.add_parser("twist", help="Twist-like command in m/s and rad/s")
    p_twist.add_argument("linear_mps", type=float)
    p_twist.add_argument("angular_rps", type=float)
    p_twist.add_argument("--max-track-mps", type=float, default=0.5)
    p_twist.add_argument("--track-width-m", type=float, default=0.45)
    add_common_args(p_twist)

    p_daemon = subparsers.add_parser("daemon", help="UDP command server with timeout stop")
    p_daemon.add_argument("--host", default="127.0.0.1")
    p_daemon.add_argument("--port", type=int, default=8765)
    p_daemon.add_argument("--timeout", type=float, default=0.3)
    p_daemon.add_argument("--arm-seconds", type=float, default=3.0)
    add_common_args(p_daemon)

    args = parser.parse_args(argv)

    # PCA9685 uses /dev/i2c-* and works for users in the ``i2c`` group.  Only
    # the legacy sysfs backend requires root; keep that check for Orange Pi
    # while allowing an industrial-PC deployment to run unprivileged.
    if args.command == "daemon" and (args.arm_seconds < 0 or args.timeout <= 0):
        parser.error("--arm-seconds must be >= 0 and --timeout must be > 0")
    requested_backend = (args.backend or os.environ.get(
        "MOTOR_DRIVER_BACKEND", "pca9685" if os.name == "posix" else "mock")).strip().lower()
    sysfs_backend = requested_backend == "sysfs"
    geteuid = getattr(os, "geteuid", None)
    if sysfs_backend and geteuid is not None and geteuid() != 0:
        print("error: run with sudo, sysfs PWM writes need root", file=sys.stderr)
        return 1

    if args.command == "daemon":
        return run_daemon(args)

    driver = build_driver(args)
    try:
        if args.command == "stop":
            pulses = driver.stop()
            print(f"stop: left={pulses[0]}us right={pulses[1]}us")
        elif args.command == "off":
            driver.off()
            print("PWM disabled")
        elif args.command == "status":
            print(driver.status())
        elif args.command == "lr":
            pulses = driver.set_left_right(args.left, args.right)
            print(f"lr: left={args.left:.3f}->{pulses[0]}us right={args.right:.3f}->{pulses[1]}us")
        elif args.command == "cmd":
            left, right, lp, rp = driver.set_cmd_normalized(args.linear, args.angular)
            print(
                f"cmd: linear={args.linear:.3f} angular={args.angular:.3f} "
                f"tracks=({left:.3f},{right:.3f}) pulses=({lp},{rp})us"
            )
        elif args.command == "twist":
            left, right, lp, rp = driver.set_twist(
                args.linear_mps, args.angular_rps, args.max_track_mps, args.track_width_m
            )
            print(
                f"twist: v={args.linear_mps:.3f} w={args.angular_rps:.3f} "
                f"tracks=({left:.3f},{right:.3f}) pulses=({lp},{rp})us"
            )
        else:
            parser.error(f"unknown command {args.command}")
        return 0
    finally:
        driver.close()


if __name__ == "__main__":
    raise SystemExit(main())
