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


def _finite_number(name: str, value: object) -> int | float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or (isinstance(value, float) and not math.isfinite(value))
    ):
        raise ValueError(f"{name}必须是有限数字")
    return value


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
    effective_cycle_s = max(monitor_interval_s, 0.05)
    cruise_mm_s = (
        command_rpm
        * screw_lead_mm
        / (60 * motor_revs_per_screw_rev)
    )
    max_approach_mm_s = tolerance_mm / (2 * effective_cycle_s)
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
            cruise_mm_s * effective_cycle_s * 10,
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
    """High-level API with bounded speed and experimental position motion."""

    def __init__(
        self,
        drive: X2PDrive,
        config: ControllerConfig | None = None,
        *,
        monitor_interval_s: float = 0.05,
        output: Callable[[str], None] | None = print,
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
        self.state = MotionState.UNKNOWN

    def _emit(self, message: str) -> None:
        if self.output is not None:
            self.output(message)

    def _status(self) -> int:
        return self.drive.read_registers(Register.STATUS)[0]

    def _actual_speed(self) -> int:
        return signed16(self.drive.read_registers(Register.ACTUAL_SPEED)[0])

    def _wait_status(self, expected: DriveStatus, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            current = self._status()
            if current == expected:
                return
            if current == DriveStatus.FAULT:
                # 驱动器报警(STATUS=4)：立即读故障码报错，避免空等超时抛出误导性信息
                raise SafetyInterlockError(self._fault_message(current))
            time.sleep(self.monitor_interval_s)
        current = self._status()
        raise MotionTimeoutError(
            f"等待驱动器状态{expected.name}超时，当前status={current}"
        )

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
        """Set speed to zero, wait for standstill, then remove software S-ON."""
        self.state = MotionState.STOPPING
        errors: list[Exception] = []

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
            self.drive.write_register(Register.POSITION_SEGMENT, 0)
        except Exception as exc:
            errors.append(exc)

        try:
            self.drive.servo_off()
        except Exception as exc:
            errors.append(exc)

        if verify_off:
            try:
                self._wait_status(
                    DriveStatus.OFF, self.config.limits.stop_timeout_s
                )
            except MotionTimeoutError:
                errors.append(
                    SafetyInterlockError(
                        "取消软件使能后仍未进入OFF；"
                        "检查外部S-ON是否仍有效"
                    )
                )
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
        if self._status() != DriveStatus.OFF:
            self.state = MotionState.FAULT
            raise SafetyInterlockError(
                "驱动器未处于OFF，检查外部S-ON是否仍然有效"
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
        self.drive.servo_on(additional_forced_inputs)
        self._wait_status(DriveStatus.RUN, 1.0)
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

    def run_speed(
        self,
        direction: str | Direction,
        rpm: int,
        duration_s: float,
    ) -> MotionResult:
        """Run in the hardware-verified speed mode, then return to OFF."""
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
        signed_rpm = rpm * direction_sign(direction, self.config.forward_sign)

        self.drive.diagnostic()
        self.prepare_off()
        start = time.monotonic()
        peak = 0
        try:
            forced_inputs = self._select_control_path(position=False)
            source = self.drive.read_registers(Register.SPEED_SOURCE)[0]
            if source != 0:
                raise ConfigurationError(
                    f"速度命令要求Pn300=0，当前为{source}"
                )
            self.drive.write_register(Register.MODBUS_NO_SAVE, 0)
            self.drive.write_register(Register.MODBUS_SAVE_POLICY, 1)
            self.drive.set_speed(signed_rpm)
            readback = signed16(
                self.drive.read_registers(Register.SPEED_COMMAND)[0]
            )
            if readback != signed_rpm:
                raise ConfigurationError(
                    "Pn301速度指令写入读回不一致: "
                    f"{readback}!={signed_rpm}"
                )
            self._enable_and_verify(forced_inputs)
            self._wait_speed_command(signed_rpm)
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                actual = self._actual_speed()
                peak = max(peak, abs(actual))
                monitored = signed16(
                    self.drive.read_registers(
                        Register.MONITORED_SPEED_COMMAND
                    )[0]
                )
                torque = signed16(
                    self.drive.read_registers(Register.TORQUE_COMMAND)[0]
                )
                self._emit(
                    f"实际转速: {actual} r/min, 速度命令: {monitored} r/min, "
                    f"转矩: {torque / 10:.1f}%"
                )
                time.sleep(self.monitor_interval_s)
        finally:
            self.stop(verify_off=True)

        result = MotionResult(
            mode="speed",
            direction=parsed_direction.name.lower(),
            target=float(signed_rpm),
            elapsed_s=time.monotonic() - start,
            peak_abs_rpm=peak,
            final_rpm=self._actual_speed(),
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
        """Move a screw distance using speed control and encoder feedback."""
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

        parsed_direction = Direction.parse(direction)
        sign = direction_sign(direction, self.config.forward_sign)
        self.drive.diagnostic()
        self.prepare_off()
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
        (
            approach_rpm,
            approach_window_mm,
            conservative_duration_s,
        ) = _approach_profile(
            command_rpm=plan.command_rpm,
            distance_mm=distance_mm,
            tolerance_mm=tolerance,
            screw_lead_mm=self.config.screw_lead_mm,
            motor_revs_per_screw_rev=(
                self.config.motor_revs_per_screw_rev
            ),
            monitor_interval_s=self.monitor_interval_s,
        )

        counts_per_mm = plan.target_pulses / distance_mm
        tolerance_pulses = max(1, round(tolerance * counts_per_mm))
        approach_window_pulses = round(approach_window_mm * counts_per_mm)
        approach_active = (
            approach_rpm < plan.command_rpm
            and approach_window_mm >= distance_mm
        )
        initial_rpm = approach_rpm if approach_active else plan.command_rpm
        signed_rpm = initial_rpm * sign
        signed_approach_rpm = approach_rpm * sign
        encoder_sign = (
            parsed_direction.value * self.config.encoder_forward_sign
        )
        start_position = 0
        target_position = 0
        peak = 0
        start = 0.0
        last_output = 0.0
        arrival_elapsed = 0.0
        previous_progress = 0
        previous_time = 0.0
        try:
            forced_inputs = self._select_control_path(position=False)
            source = self.drive.read_registers(Register.SPEED_SOURCE)[0]
            if source != 0:
                raise ConfigurationError(
                    f"定时距离命令要求Pn300=0，当前为{source}"
                )
            self.drive.write_register(Register.MODBUS_NO_SAVE, 0)
            self.drive.write_register(Register.MODBUS_SAVE_POLICY, 1)
            self.drive.set_speed(signed_rpm)
            readback = signed16(
                self.drive.read_registers(Register.SPEED_COMMAND)[0]
            )
            if readback != signed_rpm:
                raise ConfigurationError(
                    f"Pn301写入读回不一致: {readback}!={signed_rpm}"
                )

            start_position = self.drive.read_signed32(
                Register.SERVO_POSITION_ENCODER
            )
            signed_target_pulses = plan.target_pulses * encoder_sign
            target_position = start_position + signed_target_pulses
            start = time.monotonic()
            previous_time = start
            self._enable_and_verify(forced_inputs)
            self._wait_speed_command(signed_rpm)

            expected_duration_s = max(
                plan.nominal_duration_s, conservative_duration_s
            )
            deadline = start + expected_duration_s + max(
                2.0, expected_duration_s * 0.25
            )
            while time.monotonic() < deadline:
                position = self.drive.read_signed32(
                    Register.SERVO_POSITION_ENCODER
                )
                actual = self._actual_speed()
                peak = max(peak, abs(actual))
                progress = (position - start_position) * encoder_sign
                remaining = plan.target_pulses - progress
                if progress < -tolerance_pulses:
                    raise SafetyInterlockError(
                        "编码器位置向目标反方向变化；"
                        "检查forward_sign和机械方向"
                    )
                if (
                    not approach_active
                    and approach_rpm < plan.command_rpm
                    and tolerance_pulses < remaining <= approach_window_pulses
                ):
                    self.drive.set_speed(signed_approach_rpm)
                    readback = signed16(
                        self.drive.read_registers(Register.SPEED_COMMAND)[0]
                    )
                    if readback != signed_approach_rpm:
                        raise ConfigurationError(
                            "接近目标时的低速指令写入读回不一致: "
                            f"{readback}!={signed_approach_rpm}"
                        )
                    approach_active = True
                    self._emit(
                        f"进入低速接近段: {signed_approach_rpm} r/min, "
                        f"剩余约{remaining / counts_per_mm:.3f} mm"
                    )
                now = time.monotonic()
                if now - last_output >= 0.25:
                    self._emit(
                        f"位置: {position} pulse, 剩余: {remaining} pulse, "
                        f"速度: {actual} r/min"
                    )
                    last_output = now
                if remaining <= tolerance_pulses:
                    if (
                        progress >= plan.target_pulses
                        and progress > previous_progress
                    ):
                        target_fraction = (
                            plan.target_pulses - previous_progress
                        ) / (progress - previous_progress)
                        arrival_time = previous_time + target_fraction * (
                            now - previous_time
                        )
                        arrival_elapsed = arrival_time - start
                    else:
                        arrival_elapsed = now - start
                    break
                previous_progress = progress
                previous_time = now
                time.sleep(self.monitor_interval_s)
            else:
                raise PositionNotReachedError(
                    "编码器位置在允许时间内未到达目标"
                )
        finally:
            self.stop(verify_off=True)

        final_position = self.drive.read_signed32(
            Register.SERVO_POSITION_ENCODER
        )
        error_pulses = target_position - final_position
        error_mm = error_pulses / counts_per_mm
        if abs(error_pulses) > tolerance_pulses:
            raise PositionNotReachedError(
                f"停止后位置误差{error_pulses} pulse/"
                f"{error_mm:.4f} mm，超过容差{tolerance:.4f} mm"
            )

        result = MotionResult(
            mode="timed_distance",
            direction=parsed_direction.name.lower(),
            target=distance_mm * parsed_direction.value,
            elapsed_s=arrival_elapsed,
            peak_abs_rpm=peak,
            final_rpm=self._actual_speed(),
            start_position=start_position,
            final_position=final_position,
            completed=True,
            requested_duration_s=duration_s,
            position_error_pulses=error_pulses,
            position_error_mm=error_mm,
        )
        self._record(result)
        return result

    def experimental_move_pulses(
        self,
        direction: str | Direction,
        pulses: int,
        rpm: int,
        *,
        allow_experimental: bool = False,
        timeout_s: float | None = None,
    ) -> MotionResult:
        return self._experimental_move_pulses(
            direction,
            pulses,
            rpm,
            allow_experimental=allow_experimental,
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
        """Try one Pr1 relative move; this path has not worked on this drive yet."""
        if not allow_experimental:
            raise SafetyInterlockError(
                "内部位置触发尚未通过实机验证；"
                "必须显式设置allow_experimental=True"
            )
        limits = self.config.limits
        if type(pulses) is not int:
            raise ValueError("pulses必须是整数")
        if type(rpm) is not int:
            raise ValueError("rpm必须是整数")
        if not 1 <= pulses <= limits.max_move_pulses:
            raise ValueError(f"pulses必须在1..{limits.max_move_pulses}之间")
        if not 1 <= rpm <= limits.max_rpm:
            raise ValueError(f"rpm必须在1..{limits.max_rpm}之间")
        timeout = (
            limits.position_timeout_s
            if timeout_s is None
            else _finite_number("timeout_s", timeout_s)
        )
        if not 0 < timeout <= limits.position_timeout_s:
            raise ValueError(f"timeout_s必须在0..{limits.position_timeout_s}之间")

        parsed_direction = Direction.parse(direction)
        signed_pulses = pulses * direction_sign(direction, self.config.forward_sign)
        self.drive.diagnostic()
        self.prepare_off()
        start = time.monotonic()
        peak = 0
        completed = False
        start_position = 0
        try:
            forced_inputs = self._select_control_path(position=True)
            self.drive.write_register(Register.MODBUS_NO_SAVE, 0)
            self.drive.write_register(Register.MODBUS_SAVE_POLICY, 1)
            self.drive.write_register(Register.POSITION_SOURCE, 1)
            self.drive.write_register(Register.POSITION_MODE, 6)
            self.drive.write_register(Register.POSITION_SEGMENT, 0)
            self.drive.write_register(Register.POSITION_ACCEL, 500)
            self.drive.write_register(Register.POSITION_DECEL, 500)
            self.drive.write_register(Register.POSITION_S_CURVE, 100)
            self.drive.write_signed32(Register.PR1_PULSES, signed_pulses)
            self.drive.write_register(Register.PR1_SPEED, rpm)
            if self.drive.read_signed32(Register.PR1_PULSES) != signed_pulses:
                raise ConfigurationError("Pn706位置写入读回不一致")

            start_position = self.drive.read_signed32(
                Register.FEEDBACK_COMMAND_PULSES
            )
            target_position = start_position + signed_pulses
            tolerance = max(2, min(50, pulses // 100))
            self.drive.write_register(Register.POSITION_SEGMENT, 1)
            self._enable_and_verify(forced_inputs)
            enabled_inputs = digital_input_bit(1) | forced_inputs
            self.drive.trigger_position(enabled_inputs)
            deadline = time.monotonic() + timeout
            settled = 0
            while time.monotonic() < deadline:
                actual = self._actual_speed()
                peak = max(peak, abs(actual))
                position = self.drive.read_signed32(
                    Register.FEEDBACK_COMMAND_PULSES
                )
                deviation = self.drive.read_signed32(Register.POSITION_DEVIATION)
                self._emit(
                    f"实际转速: {actual} r/min, 位置: {position} pulse, "
                    f"位置偏差: {deviation}"
                )
                at_target = abs(position - target_position) <= tolerance
                settled = settled + 1 if at_target and abs(actual) <= 1 else 0
                if settled >= 3:
                    completed = True
                    break
                time.sleep(self.monitor_interval_s)
            if not completed:
                raise PositionNotReachedError(
                    "实验性位置运动未到位；已执行安全停止"
                )
        finally:
            self.stop(verify_off=True)

        result = MotionResult(
            mode="experimental_position",
            direction=parsed_direction.name.lower(),
            target=float(signed_pulses),
            elapsed_s=time.monotonic() - start,
            peak_abs_rpm=peak,
            final_rpm=self._actual_speed(),
            start_position=start_position,
            final_position=self.drive.read_signed32(
                Register.FEEDBACK_COMMAND_PULSES
            ),
            completed=completed,
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
