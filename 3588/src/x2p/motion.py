"""Safety-checked motion orchestration."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict
from enum import Enum
from pathlib import Path
from typing import Callable

from .drive import X2PDrive
from .errors import (
    ConfigurationError,
    MotionTimeoutError,
    PositionNotReachedError,
    SafetyInterlockError,
)
from .models import (
    ControllerConfig,
    Direction,
    MotionResult,
    direction_sign,
    distance_to_pulses,
    plan_timed_move,
)
from .protocol import signed16
from .registers import (
    ControlMode,
    DigitalInputFunction,
    DriveStatus,
    Register,
    digital_input_bit,
)


class MotionState(Enum):
    UNKNOWN = "unknown"
    OFF = "off"
    ARMED = "armed"
    RUNNING = "running"
    STOPPING = "stopping"
    FAULT = "fault"


#: 使能核验窗口。RTU 传输每次读写最长约 1 s 且会重试一次，1 s 的旧值
#: 在偶发慢应答时只够采到一个样本，容易把"其实是能上电"误判成超时。
ENABLE_VERIFY_TIMEOUT_S = 3.0

# Internal-position commands need time not only for their constant-speed
# travel, but also for the configured 500 ms acceleration/deceleration ramps
# and three stopped samples.  The old fixed 15 s deadline expired exactly as
# a 195 mm return reached its target, producing a false "not reached" error.
POSITION_SETTLE_MARGIN_S = 3.0


def _finite_number(name: str, value: object) -> int | float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or (isinstance(value, float) and not math.isfinite(value))
    ):
        raise ValueError(f"{name}必须是有限数字")
    return value


def _position_timeout_budget(
    *,
    pulses: int,
    rpm: int,
    encoder_counts_per_motor_rev: int,
    configured_timeout_s: float,
) -> float:
    """Return a safe deadline covering travel, ramps and final settling."""
    travel_s = (
        pulses
        / encoder_counts_per_motor_rev
        * 60.0
        / rpm
    )
    return max(
        float(configured_timeout_s),
        travel_s + POSITION_SETTLE_MARGIN_S,
    )


def _approach_profile(
    *,
    command_rpm: int,
    distance_mm: float,
    tolerance_mm: float,
    screw_lead_mm: float,
    motor_revs_per_screw_rev: float,
    monitor_interval_s: float,
) -> tuple[int, float, float]:
    """Return approach RPM, approach window, and conservative duration."""
    # A loop iteration is not just ``monitor_interval_s``: while the axis is
    # moving it also performs Modbus reads.  The production RTU transport has
    # a 1 s read timeout and an occasional slow reply was observed to stretch
    # one loop to about 1.3 s.  Using the nominal 50 ms sleep here previously
    # classified 167 r/min as an acceptable approach speed; the low-speed
    # phase was therefore disabled and the lift overshot by 13.4 mm.
    #
    # Budget one complete RTU timeout.  At the resulting approach speed the
    # axis travels at most half the allowed tolerance during that interval.
    control_latency_s = max(monitor_interval_s, 1.0)
    cruise_mm_s = (
        command_rpm
        * screw_lead_mm
        / (60 * motor_revs_per_screw_rev)
    )
    max_approach_mm_s = tolerance_mm / (2 * control_latency_s)
    calculated_rpm = math.floor(
        max_approach_mm_s
        * 60
        * motor_revs_per_screw_rev
        / screw_lead_mm
    )
    approach_rpm = min(command_rpm, max(1, calculated_rpm))
    if approach_rpm >= command_rpm:
        return command_rpm, 0.0, distance_mm / cruise_mm_s

    approach_window_mm = min(
        distance_mm,
        max(
            1.0,
            tolerance_mm * 10,
            # Enter the low-speed phase early enough even if one complete
            # control iteration is delayed by an RTU timeout.
            cruise_mm_s * control_latency_s * 2,
        ),
    )
    approach_mm_s = (
        approach_rpm
        * screw_lead_mm
        / (60 * motor_revs_per_screw_rev)
    )
    conservative_duration_s = (
        (distance_mm - approach_window_mm) / cruise_mm_s
        + approach_window_mm / approach_mm_s
    )
    return approach_rpm, approach_window_mm, conservative_duration_s


class MotionController:
    """High-level API for bounded internal-position motion.

    The public ``run_speed`` name is kept for compatibility with the lift
    adapter, but it now converts the requested speed/time pair into a relative
    position command and executes it through the drive's internal position
    table (Pn706/Pn708 + Pn701 trigger).
    """

    def __init__(
        self,
        drive: X2PDrive,
        config: ControllerConfig | None = None,
        *,
        monitor_interval_s: float = 0.10,
        output: Callable[[str], None] | None = print,
        cancel_check: Callable[[], None] | None = None,
    ):
        self.drive = drive
        self.config = config or ControllerConfig()
        self.config.validate()
        if (
            isinstance(monitor_interval_s, bool)
            or not isinstance(monitor_interval_s, (int, float))
            or not math.isfinite(monitor_interval_s)
            or monitor_interval_s <= 0
        ):
            raise ValueError("monitor_interval_s必须是大于0的有限数字")
        self.monitor_interval_s = monitor_interval_s
        self.output = output
        # Optional local supervisor interlock. It must raise when revoked;
        # stop() deliberately never calls it, so cleanup remains possible.
        self.cancel_check = cancel_check
        self.state = MotionState.UNKNOWN
        # Position-table parameters are invariant for the lifetime of this
        # controller. A completed move already performs a verified stop, so
        # subsequent reciprocating legs only need to prove OFF/zero-speed and
        # refresh the dynamic target. Any failure invalidates this cache.
        self._position_static_ready = False
        self._position_forced_inputs = 0
        self._position_static_rpm: int | None = None

    def _check_cancelled(self) -> None:
        if self.cancel_check is not None:
            self.cancel_check()

    def _emit(self, message: str) -> None:
        if self.output is not None:
            self.output(message)

    def _status(self) -> int:
        return self.drive.read_registers(Register.STATUS)[0]

    def _actual_speed(self) -> int:
        return signed16(self.drive.read_registers(Register.ACTUAL_SPEED)[0])

    def _is_disabled_and_stopped(self) -> bool:
        """True when the drive is really de-energised and not turning."""
        return (
            self._status() != DriveStatus.RUN
            and abs(self._actual_speed()) <= 1
        )

    def _check_not_faulted(self) -> None:
        """Fail fast with the E-code when the drive reports a fault."""
        current = self._status()
        if current == DriveStatus.FAULT:
            # 驱动器报警(STATUS=4)：立即读故障码报错，避免空等超时抛出误导性信息
            raise SafetyInterlockError(self._fault_message(current))

    def _wait_disabled(self, timeout_s: float) -> None:
        """Wait for software S-ON to be released and the axis to stand still."""
        last_status = 0
        last_speed = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_not_faulted()
            last_status = self._status()
            last_speed = self._actual_speed()
            if last_status != DriveStatus.RUN and abs(last_speed) <= 1:
                return
            time.sleep(self.monitor_interval_s)
        raise MotionTimeoutError(
            "取消软件使能后仍未停机："
            f"STATUS={last_status}，实际转速={last_speed} r/min；"
            "检查外部S-ON是否仍有效、是否有外力拖动或驱动器报警"
        )

    def _wait_enabled(self, timeout_s: float) -> None:
        """Wait for the X7P state display to enter servo RUN (STATUS=2).

        X7P's documented monitor table ends at Un046.  Address 0x203A is not
        an X7P servo-enable monitor, so its constant zero must not veto S-ON.
        The board test proves 0x3E00 changes 1 -> 2 on S-ON and back to 1 on
        S-OFF, and mode-7 position motion runs while it is 2.
        """
        last_status = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_cancelled()
            self._check_not_faulted()
            last_status = self._status()
            if last_status == DriveStatus.RUN:
                return
            time.sleep(self.monitor_interval_s)
        raise MotionTimeoutError(
            f"伺服使能超时：STATUS={last_status}；"
            "检查DI1/SRV-ON接线、外部急停和驱动器报警"
            + self._enable_chain_hint()
        )

    def check_motion_ready(self) -> dict[str, int]:
        """Validate communication, position configuration and servo enable.

        The check never writes a position or speed command.  It briefly proves
        S-ON through STATUS=2 and always removes software enable before returning,
        making it suitable for the workflow preflight before the clamp moves.
        """
        self.drive.diagnostic()
        self.prepare_off()
        self._verify_position_configuration()
        forced_inputs = self._select_control_path(position=True)
        try:
            self._enable_and_verify(forced_inputs)
            return {
                "status": self._status(),
                "speed_rpm": self._actual_speed(),
                "encoder_position": self.read_encoder_position(),
            }
        finally:
            self.stop(verify_off=True)

    def _enable_chain_hint(self) -> str:
        """附带 SRV-ON 使能链路的现场快照，避免只能靠猜。

        快照按信号流顺序给出：DI1 功能(Pn400) → 强制输入(Pn415) →
        驱动器看到的 DI 状态(Un032 bit0) → 运行状态(STATUS=2)。
        任何一项读不到都不影响报错本身。
        """
        reader = getattr(self.drive, "read_enable_chain", None)
        if reader is None:
            return ""
        try:
            values = reader()
        except Exception:  # noqa: BLE001 - 诊断信息不能掩盖原始报错
            return ""
        if not values:
            return ""
        rendered = "，".join(f"{key}={value}" for key, value in values.items())
        hints = self._enable_chain_hints(values)
        text = f"\n使能链路快照：{rendered}"
        if hints:
            text += "\n判读：" + "；".join(hints)
        return text

    @staticmethod
    def _enable_chain_hints(values: dict[str, object]) -> list[str]:
        """把寄存器快照翻译成下一步该查什么。"""
        forced = values.get("P415_强制输入")
        inputs = values.get("Un032_DI状态")
        status = values.get("STATUS_0x3E00")
        function = values.get("P400_DI1功能")
        hints: list[str] = []
        if isinstance(function, int) and function != DigitalInputFunction.SERVO_ON:
            hints.append(f"Pn400={function}，DI1 未配置为 SRV-ON(1)")
        if isinstance(forced, int) and not forced & 0x01:
            hints.append("Pn415 第0位没有置起，强制位没有写进驱动器")
        if isinstance(inputs, int) and not inputs & 0x01:
            hints.append(
                "Un032 第0位为0：驱动器没把 DI1 当成有效输入，"
                "查 24V/DI 公共端与端子接线"
            )
        if (
            isinstance(inputs, int)
            and inputs & 0x01
            and isinstance(status, int)
            and status != DriveStatus.RUN
        ):
            hints.append(
                f"DI1 已有效但 STATUS={status}未进入RUN(2)："
                "驱动器主动拒绝使能，"
                "查主电源、外部急停/限位和驱动面板 E 码"
            )
        return hints

    def _fault_message(self, status: int) -> str:
        """驱动器报警(STATUS=4)时读 Un100 故障码低字节，生成明确报错。

        低字节即 E 码（实测 E04=0x04/E0b=0x0B/E37=0x37）；替代原先
        "取消软件使能后仍未进入OFF" 这类等待超时的误导性信息。
        """
        try:
            raw = self.drive.read_registers(Register.LAST_FAULT_CODE)[0]
        except Exception as exc:  # noqa: BLE001 - 报警时读码失败也要能报错
            return f"驱动器报警(状态={status})；读故障码失败: {exc!r}"
        code = raw & 0xFF
        return (
            f"驱动器报警(状态={status})，故障码=E{code:02X} "
            f"(Un100=0x{raw:04X})；见X2P手册报警码表"
        )

    def _record(self, result: MotionResult) -> None:
        record = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            **asdict(result),
        }
        log_path = Path(self.config.log_path)
        try:
            with log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as exc:
            self._emit(
                f"警告：运动已经完成，但日志{log_path}写入失败: {exc}"
            )

    def stop(self, *, verify_off: bool = True) -> None:
        """Cancel the active position segment, stop, then remove software S-ON."""
        self.state = MotionState.STOPPING
        errors: list[Exception] = []

        try:
            # In position mode Pn301 is not the active command.  Clearing
            # Pn701 first cancels the internal position segment; Pn301=0 is
            # retained as a harmless compatibility stop for speed-mode drives.
            self.drive.write_register(Register.POSITION_SEGMENT, 0)
        except Exception as exc:
            errors.append(exc)

        try:
            self.drive.set_speed(0)
        except Exception as exc:
            errors.append(exc)

        try:
            deadline = time.monotonic() + self.config.limits.stop_timeout_s
            while time.monotonic() < deadline:
                if abs(self._actual_speed()) <= 1:
                    break
                time.sleep(self.monitor_interval_s)
            else:
                raise MotionTimeoutError("停止超时：实际速度未归零")
        except Exception as exc:
            errors.append(exc)

        try:
            self.drive.servo_off()
        except Exception as exc:
            errors.append(exc)

        if verify_off:
            try:
                self._wait_disabled(self.config.limits.stop_timeout_s)
            except Exception as exc:
                errors.append(exc)

        if errors:
            self.state = MotionState.FAULT
            if len(errors) == 1:
                raise errors[0]
            summary = "; ".join(str(error) for error in errors)
            raise SafetyInterlockError(
                "停机过程中出现多个错误，已尝试全部停机步骤: "
                f"{summary}"
            ) from errors[0]

        self.state = MotionState.OFF

    def prepare_off(self) -> None:
        """Establish an OFF baseline before changing any motion parameters."""
        self.stop(verify_off=True)
        if not self._is_disabled_and_stopped():
            self.state = MotionState.FAULT
            raise SafetyInterlockError(
                "驱动器未处于OFF："
                f"STATUS={self._status()}，"
                f"实际转速={self._actual_speed()} r/min；"
                "检查外部S-ON是否仍然有效"
            )
        self.state = MotionState.ARMED

    def _select_control_path(self, *, position: bool) -> int:
        """Select a direct or Pn001=3 path while OFF and verify C-MODE."""
        mode = self.drive.read_registers(Register.CONTROL_MODE)[0]
        direct_mode = ControlMode.POSITION if position else ControlMode.SPEED
        if mode == direct_mode:
            return 0
        if mode != ControlMode.SPEED_POSITION_AT_ZERO:
            requested = "位置" if position else "速度"
            raise ConfigurationError(
                f"{requested}命令不支持当前Pn001={mode}；"
                f"要求Pn001={int(direct_mode)}或3"
            )
        if abs(self._actual_speed()) > 1:
            raise SafetyInterlockError(
                "Pn001=3只能在零速时切换速度/位置模式"
            )

        di_number = self.config.hybrid_mode_di
        function = self.drive.read_digital_input_function(di_number)
        if function != DigitalInputFunction.CONTROL_MODE:
            raise ConfigurationError(
                f"Pn001=3要求DI{di_number}配置为C-MODE(功能5)，"
                f"当前功能为{function}"
            )

        mode_bit = digital_input_bit(di_number)
        forced_inputs = mode_bit if position else 0
        self.drive.force_digital_inputs(forced_inputs)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            input_active = bool(self.drive.read_digital_inputs() & mode_bit)
            if input_active == position:
                return forced_inputs
            time.sleep(self.monitor_interval_s)

        self.drive.force_digital_inputs(0)
        requested = (
            "有效（位置模式）" if position else "无效（速度模式）"
        )
        raise SafetyInterlockError(
            f"DI{di_number}/C-MODE未变为{requested}；"
            "检查外部端子是否被强制为相反状态"
        )

    def _enable_and_verify(self, additional_forced_inputs: int = 0) -> None:
        self._check_cancelled()
        self.drive.servo_on(additional_forced_inputs)
        self._wait_enabled(ENABLE_VERIFY_TIMEOUT_S)
        self.state = MotionState.RUNNING

    def _wait_speed_command(self, expected_rpm: int) -> None:
        """Require Un001 to prove that the active mode accepted Pn301."""
        deadline = time.monotonic() + 1.0
        monitored = 0
        while time.monotonic() < deadline:
            monitored = signed16(
                self.drive.read_registers(
                    Register.MONITORED_SPEED_COMMAND
                )[0]
            )
            if monitored == expected_rpm:
                return
            time.sleep(self.monitor_interval_s)
        raise MotionTimeoutError(
            f"Pn301已写入{expected_rpm} r/min，但Un001仍为{monitored} r/min；"
            "当前控制模式未接受通讯速度命令"
        )

    def _verify_position_configuration(self) -> None:
        """Verify the drive is configured for internal position + auto mode 2."""
        tuning_mode = self.drive.read_registers(Register.TUNING_MODE)[0]
        command_pulses = self.drive.read_signed32(
            Register.COMMAND_PULSES_PER_REV
        )
        expected_pulses = self.config.encoder_counts_per_motor_rev
        if tuning_mode != 2:
            raise ConfigurationError(
                f"当前Pn002={tuning_mode}，要求自动调整模式2(Pn002=2)；"
                "请先运行 scripts/configure_x2p_position_auto2.py 并断电重启"
            )
        if command_pulses != expected_pulses:
            raise ConfigurationError(
                f"内部位置脉冲单位不匹配：Pn008={command_pulses}，"
                f"控制器配置encoder_counts_per_motor_rev={expected_pulses}。"
                "请先运行 scripts/configure_x2p_position_auto2.py "
                f"--command-pulses-per-rev {expected_pulses}，"
                "否则Pn706位置会按错误比例移动"
            )

    def _invalidate_position_static_setup(self) -> None:
        self._position_static_ready = False
        self._position_static_rpm = None

    def _prepare_position_move(self, rpm: int) -> int:
        """Prepare Pr1, reusing only previously verified invariants.

        Normal completion always calls ``stop(verify_off=True)``. On the next
        leg we still independently read STATUS and actual speed, reset/read
        Pn701, and later read back Pn706 and verify S-ON.
        """
        if self._position_static_ready:
            if not self._is_disabled_and_stopped():
                self._invalidate_position_static_setup()
                self.state = MotionState.FAULT
                raise SafetyInterlockError(
                    "快速换向前驱动器未处于OFF且零速状态"
                )
            self.drive.write_register(Register.POSITION_SEGMENT, 0)
            if self.drive.read_registers(Register.POSITION_SEGMENT)[0] != 0:
                self._invalidate_position_static_setup()
                raise ConfigurationError("Pn701位置段复位读回不一致")
            if self._position_static_rpm != rpm:
                self.drive.write_register(Register.PR1_SPEED, rpm)
                if self.drive.read_registers(Register.PR1_SPEED)[0] != rpm:
                    self._invalidate_position_static_setup()
                    raise ConfigurationError("Pn708位置速度写入读回不一致")
                self._position_static_rpm = rpm
            self.state = MotionState.ARMED
            return self._position_forced_inputs

        self.drive.diagnostic()
        self.prepare_off()
        self._verify_position_configuration()
        forced_inputs = self._select_control_path(position=True)
        self.drive.write_register(Register.MODBUS_NO_SAVE, 0)
        self.drive.write_register(Register.MODBUS_SAVE_POLICY, 1)
        for label, address, value in (
            ("Pn321位置指令来源", Register.POSITION_SOURCE, 1),
            ("Pn700内部位置模式", Register.POSITION_MODE, 7),
            ("Pn701当前段", Register.POSITION_SEGMENT, 0),
            ("Pn703加速时间", Register.POSITION_ACCEL, 500),
            ("Pn704减速时间", Register.POSITION_DECEL, 500),
            ("Pn705S曲线时间", Register.POSITION_S_CURVE, 100),
            ("Pn708位置速度", Register.PR1_SPEED, rpm),
        ):
            self.drive.write_register(address, value)
            readback = self.drive.read_registers(address)[0]
            if readback != value:
                raise ConfigurationError(
                    f"{label}写入读回不一致: {readback}!={value}"
                )
        self._position_forced_inputs = forced_inputs
        self._position_static_rpm = rpm
        self._position_static_ready = True
        return forced_inputs

    def run_speed(
        self,
        direction: str | Direction,
        rpm: int,
        duration_s: float,
    ) -> MotionResult:
        """Run the same timed displacement through internal position mode."""
        limits = self.config.limits
        if type(rpm) is not int:
            raise ValueError("rpm必须是整数")
        duration_s = _finite_number("duration_s", duration_s)
        if not 1 <= rpm <= limits.max_rpm:
            raise ValueError(f"rpm必须在1..{limits.max_rpm}之间")
        if duration_s <= 0:
            raise ValueError("duration_s必须大于0")
        if (
            limits.max_duration_s is not None
            and duration_s > limits.max_duration_s
        ):
            raise ValueError(
                f"duration_s不能超过{limits.max_duration_s}"
            )
        parsed_direction = Direction.parse(direction)
        encoder_counts = self.config.encoder_counts_per_motor_rev
        pulses = round(rpm * duration_s * encoder_counts / 60.0)
        if pulses < 1:
            raise ValueError("按rpm和duration_s换算后的位置脉冲必须大于0")
        if pulses > limits.max_move_pulses:
            raise ValueError(
                f"换算后的位置脉冲{pulses}超过安全上限"
                f"{limits.max_move_pulses}"
            )

        move_result = self._move_pulses(
            direction,
            pulses,
            rpm,
            mode="speed_position",
            timeout_s=None,
            record_result=False,
            tolerance_pulses=max(2, min(50, pulses // 100)),
        )
        signed_rpm = rpm * direction_sign(
            direction, self.config.forward_sign
        )
        result = MotionResult(
            **{
                **asdict(move_result),
                "mode": "position",
                "direction": parsed_direction.name.lower(),
                "target": float(signed_rpm),
                "requested_duration_s": duration_s,
            }
        )
        self._record(result)
        return result

    def move_timed_distance(
        self,
        direction: str | Direction,
        distance_mm: float,
        duration_s: float,
        *,
        tolerance_mm: float | None = None,
    ) -> MotionResult:
        """Move a screw distance through Pn706 internal position mode."""
        limits = self.config.limits
        distance_mm = _finite_number("distance_mm", distance_mm)
        duration_s = _finite_number("duration_s", duration_s)
        if not 0 < distance_mm <= limits.max_distance_mm:
            raise ValueError(
                f"distance_mm必须在0..{limits.max_distance_mm}之间"
            )
        if duration_s <= 0:
            raise ValueError("duration_s必须大于0")
        if (
            limits.max_duration_s is not None
            and duration_s > limits.max_duration_s
        ):
            raise ValueError(
                f"duration_s不能超过{limits.max_duration_s}"
            )
        tolerance = (
            self.config.position_tolerance_mm
            if tolerance_mm is None
            else _finite_number("tolerance_mm", tolerance_mm)
        )
        if not 0 < tolerance <= distance_mm:
            raise ValueError("tolerance_mm必须大于0且不超过移动距离")

        plan = plan_timed_move(
            distance_mm,
            duration_s,
            screw_lead_mm=self.config.screw_lead_mm,
            motor_revs_per_screw_rev=(
                self.config.motor_revs_per_screw_rev
            ),
            feedback_counts_per_rev=(
                self.config.encoder_counts_per_motor_rev
            ),
            max_rpm=limits.max_rpm,
        )
        counts_per_mm = plan.target_pulses / distance_mm
        tolerance_pulses = max(1, round(tolerance * counts_per_mm))
        parsed_direction = Direction.parse(direction)
        move_result = self._move_pulses(
            direction,
            plan.target_pulses,
            plan.command_rpm,
            mode="timed_distance",
            timeout_s=None,
            record_result=False,
            tolerance_pulses=tolerance_pulses,
        )
        target_position = int(move_result.start_position) + (
            direction_sign(direction, self.config.forward_sign)
            * self.config.encoder_forward_sign
            * plan.target_pulses
        )
        final_position = int(move_result.final_position)
        error_pulses = target_position - final_position
        error_mm = error_pulses / counts_per_mm
        self._emit(
            f"MOVE_STOP start={move_result.start_position} "
            f"target={target_position} final={final_position} "
            f"error_mm={error_mm:.4f} tolerance_mm={tolerance}"
        )
        result = MotionResult(
            mode="timed_distance",
            direction=parsed_direction.name.lower(),
            target=distance_mm * parsed_direction.value,
            elapsed_s=move_result.elapsed_s,
            peak_abs_rpm=move_result.peak_abs_rpm,
            final_rpm=move_result.final_rpm,
            start_position=move_result.start_position,
            final_position=move_result.final_position,
            completed=True,
            requested_duration_s=duration_s,
            position_error_pulses=error_pulses,
            position_error_mm=error_mm,
        )
        self._record(result)
        return result

    def read_encoder_position(self) -> int:
        """Return the current signed servo encoder position.

        This deliberately exposes a read-only position primitive to the lift
        adapter.  Paired mechanism strokes can therefore remember an absolute
        origin instead of assuming that two opposite relative moves cancel.
        """
        return self.drive.read_signed32(Register.SERVO_POSITION_ENCODER)

    def move_pulses(
        self,
        direction: str | Direction,
        pulses: int,
        rpm: int,
        *,
        timeout_s: float | None = None,
        tolerance_pulses: int | None = None,
    ) -> MotionResult:
        """Execute one relative move with Pn321=1 internal Pr1 position mode."""
        return self._move_pulses(
            direction,
            pulses,
            rpm,
            mode="position",
            timeout_s=timeout_s,
            record_result=True,
            tolerance_pulses=tolerance_pulses,
        )

    def experimental_move_pulses(
        self,
        direction: str | Direction,
        pulses: int,
        rpm: int,
        *,
        allow_experimental: bool = False,
        timeout_s: float | None = None,
    ) -> MotionResult:
        """Compatibility wrapper for the formerly experimental Pr1 path."""
        del allow_experimental
        return self._experimental_move_pulses(
            direction,
            pulses,
            rpm,
            allow_experimental=True,
            timeout_s=timeout_s,
            record_result=True,
        )

    def _experimental_move_pulses(
        self,
        direction: str | Direction,
        pulses: int,
        rpm: int,
        *,
        allow_experimental: bool,
        timeout_s: float | None,
        record_result: bool,
    ) -> MotionResult:
        del allow_experimental
        return self._move_pulses(
            direction,
            pulses,
            rpm,
            mode="experimental_position",
            timeout_s=timeout_s,
            record_result=record_result,
            tolerance_pulses=None,
        )

    def _verify_position_segment_selected(self) -> None:
        """Verify the one-shot Pn701 trigger without ever writing it twice.

        A busy drive can return an inconsistent first monitor read.  Read-only
        confirmation may recover that case; persistent disagreement must stop
        the move because the trigger may already have executed.
        """
        readbacks: list[int] = []
        for attempt in range(3):
            if attempt:
                time.sleep(0.10)
            self._check_cancelled()
            value = self.drive.read_registers(Register.POSITION_SEGMENT)[0]
            readbacks.append(value)
            if value == 1:
                if attempt:
                    self._emit(f"Pn701只读复核成功: readbacks={readbacks}")
                return
        raise ConfigurationError(
            f"Pn701位置段选择写入读回不一致: readbacks={readbacks}; "
            "触发指令未重发，转入停机流程"
        )

    def _move_pulses(
        self,
        direction: str | Direction,
        pulses: int,
        rpm: int,
        *,
        mode: str,
        timeout_s: float | None,
        record_result: bool,
        tolerance_pulses: int | None,
    ) -> MotionResult:
        """Execute one Pr1 relative move and always remove servo enable."""
        limits = self.config.limits
        if type(pulses) is not int:
            raise ValueError("pulses必须是整数")
        if type(rpm) is not int:
            raise ValueError("rpm必须是整数")
        if not 1 <= pulses <= limits.max_move_pulses:
            raise ValueError(f"pulses必须在1..{limits.max_move_pulses}之间")
        if not 1 <= rpm <= limits.max_rpm:
            raise ValueError(f"rpm必须在1..{limits.max_rpm}之间")
        if timeout_s is None:
            timeout = _position_timeout_budget(
                pulses=pulses,
                rpm=rpm,
                encoder_counts_per_motor_rev=(
                    self.config.encoder_counts_per_motor_rev
                ),
                configured_timeout_s=limits.position_timeout_s,
            )
        else:
            timeout = _finite_number("timeout_s", timeout_s)
        if not 0 < timeout or (
            timeout_s is not None
            and timeout > limits.position_timeout_s
        ):
            raise ValueError(
                f"timeout_s必须在0..{limits.position_timeout_s}之间"
            )
        tolerance = (
            max(2, min(50, pulses // 100))
            if tolerance_pulses is None
            else tolerance_pulses
        )
        if type(tolerance) is not int or not 1 <= tolerance <= pulses:
            raise ValueError("tolerance_pulses必须在1..pulses之间")

        parsed_direction = Direction.parse(direction)
        signed_pulses = pulses * direction_sign(
            direction, self.config.forward_sign
        )
        start = time.monotonic()
        peak = 0
        completed = False
        start_position = 0
        phase = "prepare"
        try:
            self._check_cancelled()
            forced_inputs = self._prepare_position_move(rpm)
            phase = "write_target"
            self.drive.write_signed32(Register.PR1_PULSES, signed_pulses)
            phase = "verify_target"
            if (
                self.drive.read_signed32(Register.PR1_PULSES)
                != signed_pulses
            ):
                raise ConfigurationError("Pn706位置写入读回不一致")

            # Un014 is reconstructed from *command* pulses.  It changes as
            # soon as Pr1 is accepted, even if a brake, torque inhibit, or
            # mechanical disconnection prevents the axis from moving.  Use
            # the actual encoder position (Un022) for all arrival decisions.
            phase = "read_encoder"
            start_position = self.drive.read_signed32(
                Register.SERVO_POSITION_ENCODER
            )
            encoder_pulses = (
                signed_pulses * self.config.encoder_forward_sign
            )
            target_position = start_position + encoder_pulses
            phase = "servo_enable"
            self._enable_and_verify(forced_inputs)
            # Mode 7 executes the segment immediately when Pn701 becomes
            # non-zero.  Board testing proved mode 6 + DI2/CTRG was ignored,
            # while this path moved the encoder on the same 5 mm command.
            phase = "trigger_segment"
            self._check_cancelled()
            self.drive.write_register(Register.POSITION_SEGMENT, 1)
            phase = "verify_segment"
            self._verify_position_segment_selected()
            phase = "monitor_motion"
            deadline = time.monotonic() + timeout
            settled = 0
            while time.monotonic() < deadline:
                self._check_cancelled()
                actual = self._actual_speed()
                peak = max(peak, abs(actual))
                position = self.drive.read_signed32(
                    Register.SERVO_POSITION_ENCODER
                )
                remaining = target_position - position
                self._emit(
                    f"实际转速: {actual} r/min, 位置: {position} pulse, "
                    f"剩余距离: {remaining} pulse"
                )
                at_target = abs(position - target_position) <= tolerance
                settled = settled + 1 if at_target and abs(actual) <= 1 else 0
                if settled >= 3:
                    completed = True
                    break
                time.sleep(self.monitor_interval_s)
        except Exception as exc:
            self._emit(f"MOVE_ABORT phase={phase} error={type(exc).__name__}: {exc}")
            self._invalidate_position_static_setup()
            raise
        finally:
            try:
                self.stop(verify_off=True)
            except Exception as exc:
                self._emit(f"MOVE_STOP_FAILED after_phase={phase} error={type(exc).__name__}: {exc}")
                self._invalidate_position_static_setup()
                raise

        final_position = self.drive.read_signed32(
            Register.SERVO_POSITION_ENCODER
        )
        error_pulses = target_position - final_position
        if abs(error_pulses) > tolerance:
            self._invalidate_position_static_setup()
            raise PositionNotReachedError(
                f"内部位置运动未到位；已执行安全停止；"
                f"目标={target_position} pulse，最终={final_position} pulse，"
                f"误差={error_pulses} pulse，容差={tolerance} pulse"
            )
        if not completed:
            # The deadline can fall between the first and third zero-speed
            # samples.  Safety stop first, then accept the command only when
            # the final encoder position independently proves it arrived.
            self._emit(
                "到位确认窗口结束，安全停止后编码器位置在容差内"
            )
            completed = True
        result = MotionResult(
            mode=mode,
            direction=parsed_direction.name.lower(),
            target=float(signed_pulses),
            elapsed_s=time.monotonic() - start,
            peak_abs_rpm=peak,
            final_rpm=self._actual_speed(),
            start_position=start_position,
            final_position=final_position,
            completed=completed,
            position_error_pulses=error_pulses,
        )
        if record_result:
            self._record(result)
        return result

    def experimental_move_distance(
        self,
        direction: str | Direction,
        distance_mm: float,
        rpm: int,
        *,
        allow_experimental: bool = False,
    ) -> MotionResult:
        distance_mm = _finite_number("distance_mm", distance_mm)
        if distance_mm > self.config.limits.max_distance_mm:
            raise ValueError(
                f"distance_mm不能超过{self.config.limits.max_distance_mm}"
            )
        if self.config.pulses_per_mm is None:
            raise ConfigurationError(
                "尚未配置pulses_per_mm，不能执行毫米距离运动"
            )
        signed = distance_to_pulses(
            distance_mm,
            self.config.pulses_per_mm,
            direction,
            self.config.forward_sign,
        )
        result = self._experimental_move_pulses(
            direction,
            abs(signed),
            rpm,
            allow_experimental=allow_experimental,
            timeout_s=None,
            record_result=False,
        )
        result = MotionResult(
            **{
                **asdict(result),
                "mode": "experimental_distance",
                "target": distance_mm * Direction.parse(direction).value,
            }
        )
        self._record(result)
        return result
