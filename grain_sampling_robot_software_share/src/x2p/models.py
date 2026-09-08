"""Configuration and value objects for motor motion."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

from .errors import ConfigurationError


def _require_config_number(
    name: str,
    value: object,
    *,
    minimum: float,
    strict: bool = False,
) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or (isinstance(value, float) and not math.isfinite(value))
    ):
        raise ConfigurationError(f"{name}必须是有限数字")
    if (strict and value <= minimum) or (not strict and value < minimum):
        operator = "大于" if strict else "不小于"
        raise ConfigurationError(f"{name}必须{operator}{minimum}")


def _require_config_int(
    name: str,
    value: object,
    *,
    minimum: int,
    maximum: int,
) -> None:
    if type(value) is not int:
        raise ConfigurationError(f"{name}必须是整数")
    if not minimum <= value <= maximum:
        raise ConfigurationError(f"{name}必须在{minimum}..{maximum}之间")


def _require_finite_number(name: str, value: object) -> int | float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or (isinstance(value, float) and not math.isfinite(value))
    ):
        raise ValueError(f"{name}必须是有限数字")
    return value


class Direction(Enum):
    FORWARD = 1
    REVERSE = -1

    @classmethod
    def parse(cls, value: str | "Direction") -> "Direction":
        if isinstance(value, cls):
            return value
        normalized = value.strip().lower()
        if normalized in {"forward", "fwd", "+", "正向", "正转"}:
            return cls.FORWARD
        if normalized in {"reverse", "rev", "-", "反向", "反转"}:
            return cls.REVERSE
        raise ValueError(f"无效方向: {value!r}，应为 forward 或 reverse")


@dataclass(frozen=True)
class SafetyLimits:
    max_rpm: int = 30
    max_duration_s: float | None = None
    max_move_pulses: int = 200_000
    max_distance_mm: float = 300.0
    stop_timeout_s: float = 2.0
    position_timeout_s: float = 10.0

    def validate(self) -> None:
        _require_config_int("max_rpm", self.max_rpm, minimum=1, maximum=32767)
        if self.max_duration_s is not None:
            _require_config_number(
                "max_duration_s", self.max_duration_s, minimum=0, strict=True
            )
        _require_config_int(
            "max_move_pulses",
            self.max_move_pulses,
            minimum=1,
            maximum=0x7FFFFFFF,
        )
        _require_config_number(
            "max_distance_mm", self.max_distance_mm, minimum=0, strict=True
        )
        _require_config_number(
            "stop_timeout_s", self.stop_timeout_s, minimum=0, strict=True
        )
        _require_config_number(
            "position_timeout_s",
            self.position_timeout_s,
            minimum=0,
            strict=True,
        )


@dataclass(frozen=True)
class ControllerConfig:
    port: str = "/dev/ttyUSB0"
    slave: int = 2
    pulses_per_mm: float | None = None
    screw_lead_mm: float = 5.0
    motor_revs_per_screw_rev: float = 1.0
    encoder_counts_per_motor_rev: int = 131_072
    encoder_forward_sign: int = 1
    position_tolerance_mm: float = 0.2
    forward_sign: int = 1
    hybrid_mode_di: int = 3
    log_path: str = "x2p_motion.log"
    limits: SafetyLimits = SafetyLimits()

    @classmethod
    def load(cls, path: str | Path | None) -> "ControllerConfig":
        if path is None:
            config = cls()
        else:
            try:
                raw = json.loads(Path(path).read_text(encoding="utf-8"))
                limits = SafetyLimits(**raw.pop("limits", {}))
                config = cls(limits=limits, **raw)
            except (OSError, json.JSONDecodeError, TypeError) as exc:
                raise ConfigurationError(f"无法读取配置 {path}: {exc}") from exc
        config.validate()
        return config

    def validate(self) -> None:
        if not isinstance(self.port, str) or not self.port.strip():
            raise ConfigurationError("port必须是非空字符串")
        _require_config_int("slave", self.slave, minimum=1, maximum=247)
        if type(self.forward_sign) is not int or self.forward_sign not in {-1, 1}:
            raise ConfigurationError("forward_sign只能是1或-1")
        if (
            type(self.hybrid_mode_di) is not int
            or not 3 <= self.hybrid_mode_di <= 4
        ):
            raise ConfigurationError(
                "hybrid_mode_di必须是3或4；"
                "DI1保留给使能，DI2保留给位置触发"
            )
        if self.pulses_per_mm is not None:
            _require_config_number(
                "pulses_per_mm", self.pulses_per_mm, minimum=0, strict=True
            )
        _require_config_number(
            "screw_lead_mm", self.screw_lead_mm, minimum=0, strict=True
        )
        _require_config_number(
            "motor_revs_per_screw_rev",
            self.motor_revs_per_screw_rev,
            minimum=0,
            strict=True,
        )
        _require_config_int(
            "encoder_counts_per_motor_rev",
            self.encoder_counts_per_motor_rev,
            minimum=1,
            maximum=0x7FFFFFFF,
        )
        if (
            type(self.encoder_forward_sign) is not int
            or self.encoder_forward_sign not in {-1, 1}
        ):
            raise ConfigurationError("encoder_forward_sign只能是1或-1")
        _require_config_number(
            "position_tolerance_mm",
            self.position_tolerance_mm,
            minimum=0,
            strict=True,
        )
        if not isinstance(self.log_path, str) or not self.log_path.strip():
            raise ConfigurationError("log_path必须是非空字符串")
        self.limits.validate()


@dataclass(frozen=True)
class MotionResult:
    mode: str
    direction: str
    target: float
    elapsed_s: float
    peak_abs_rpm: int
    final_rpm: int
    start_position: int | None = None
    final_position: int | None = None
    completed: bool = True
    requested_duration_s: float | None = None
    position_error_pulses: int | None = None
    position_error_mm: float | None = None


@dataclass(frozen=True)
class TimedMovePlan:
    distance_mm: float
    requested_duration_s: float
    motor_revolutions: float
    target_pulses: int
    command_rpm: int
    nominal_duration_s: float


def plan_timed_move(
    distance_mm: float,
    duration_s: float,
    *,
    screw_lead_mm: float,
    motor_revs_per_screw_rev: float,
    feedback_counts_per_rev: int,
    max_rpm: int,
) -> TimedMovePlan:
    """Calculate one constant-speed screw move from distance and time."""
    distance_mm = _require_finite_number("distance_mm", distance_mm)
    duration_s = _require_finite_number("duration_s", duration_s)
    screw_lead_mm = _require_finite_number("screw_lead_mm", screw_lead_mm)
    motor_revs_per_screw_rev = _require_finite_number(
        "motor_revs_per_screw_rev", motor_revs_per_screw_rev
    )
    if type(feedback_counts_per_rev) is not int:
        raise ValueError("feedback_counts_per_rev必须是整数")
    if type(max_rpm) is not int:
        raise ValueError("max_rpm必须是整数")
    if distance_mm <= 0:
        raise ValueError("distance_mm必须大于0")
    if duration_s <= 0:
        raise ValueError("duration_s必须大于0")
    if screw_lead_mm <= 0 or motor_revs_per_screw_rev <= 0:
        raise ValueError("丝杆导程和传动比必须大于0")
    if feedback_counts_per_rev <= 0:
        raise ValueError("编码器每圈计数必须大于0")

    screw_revolutions = distance_mm / screw_lead_mm
    motor_revolutions = screw_revolutions * motor_revs_per_screw_rev
    exact_rpm = motor_revolutions * 60 / duration_s
    command_rpm = round(exact_rpm)
    if command_rpm < 1:
        raise ValueError(
            f"计算速度仅{exact_rpm:.3f} r/min，"
            "低于驱动器最小整数速度1 r/min"
        )
    if command_rpm > max_rpm:
        minimum_duration = motor_revolutions * 60 / max_rpm
        raise ValueError(
            f"需要{exact_rpm:.2f} r/min，超过上限{max_rpm} r/min；"
            f"时间至少应为{minimum_duration:.3f}秒"
        )
    target_pulses = round(motor_revolutions * feedback_counts_per_rev)
    nominal_duration = motor_revolutions * 60 / command_rpm
    return TimedMovePlan(
        distance_mm=distance_mm,
        requested_duration_s=duration_s,
        motor_revolutions=motor_revolutions,
        target_pulses=target_pulses,
        command_rpm=command_rpm,
        nominal_duration_s=nominal_duration,
    )


def direction_sign(direction: str | Direction, forward_sign: int = 1) -> int:
    if forward_sign not in {-1, 1}:
        raise ValueError("forward_sign只能是1或-1")
    return Direction.parse(direction).value * forward_sign


def distance_to_pulses(
    distance_mm: float,
    pulses_per_mm: float,
    direction: str | Direction,
    forward_sign: int = 1,
) -> int:
    distance_mm = _require_finite_number("distance_mm", distance_mm)
    pulses_per_mm = _require_finite_number("pulses_per_mm", pulses_per_mm)
    if distance_mm <= 0:
        raise ValueError("distance_mm必须大于0")
    if pulses_per_mm <= 0:
        raise ValueError("pulses_per_mm必须大于0")
    pulses = round(distance_mm * pulses_per_mm)
    if pulses == 0:
        raise ValueError("距离换算后不足1个脉冲")
    return pulses * direction_sign(direction, forward_sign)
