"""Drive-specific operations built on the Modbus transport."""

from __future__ import annotations

import time

from .errors import ConfigurationError
from .protocol import ModbusRTUClient
from .registers import (
    DigitalInputFunction,
    Register,
    digital_input_bit,
    digital_input_function_register,
)


class X2PDrive:
    """Thin X2P drive facade.

    ``port`` may also be an injected ``ModbusRTUClient`` for tests.
    """

    def __init__(
        self,
        port: str | ModbusRTUClient,
        slave: int = 2,
        timeout: float = 1.0,  # USB-RS485 occasional slow ack (2026-09, was 0.5)
    ):
        self.client = (
            port
            if isinstance(port, ModbusRTUClient)
            else ModbusRTUClient(port, slave, timeout)
        )

    def __enter__(self) -> "X2PDrive":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        self.client.close()

    def diagnostic(self, value: int = 0x51A3) -> None:
        self.client.diagnostic(value)

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        return self.client.read_registers(address, count)

    def write_register(self, address: int, value: int) -> None:
        self.client.write_register(address, value)

    def write_registers(self, address: int, values: list[int]) -> None:
        self.client.write_registers(address, values)

    def read_signed32(self, address: int) -> int:
        return self.client.read_signed32(address)

    def write_signed32(self, address: int, value: int) -> None:
        self.client.write_signed32(address, value)

    def set_speed(self, rpm: int) -> None:
        self.write_register(Register.SPEED_COMMAND, rpm)

    def read_digital_inputs(self) -> int:
        return self.read_registers(Register.DIGITAL_INPUT_STATUS)[0]

    def read_digital_input_function(self, di_number: int) -> int:
        address = digital_input_function_register(di_number)
        return self.read_registers(address)[0]

    def force_digital_inputs(self, mask: int) -> None:
        if not 0 <= mask <= 0xFF:
            raise ValueError("数字输入强制掩码必须在0x00..0xFF之间")
        self.write_register(Register.FORCE_DIGITAL_INPUTS, mask)

    def servo_on(self, additional_forced_inputs: int = 0) -> None:
        function = self.read_registers(Register.DI1_FUNCTION)[0]
        if function != DigitalInputFunction.SERVO_ON:
            raise ConfigurationError(
                f"DI1并未配置为SRV-ON(Pn400={function})，拒绝强制使能"
            )
        self.write_register(Register.INTERNAL_SERVO_ON, 0)
        self.force_digital_inputs(
            digital_input_bit(1) | additional_forced_inputs
        )

    def servo_off(self) -> None:
        errors: list[Exception] = []
        try:
            self.force_digital_inputs(0)
        except Exception as exc:
            errors.append(exc)
        try:
            self.write_register(Register.INTERNAL_SERVO_ON, 0)
        except Exception as exc:
            errors.append(exc)
        if errors:
            raise errors[0]

    def trigger_position(self, base_forced_inputs: int = 1) -> None:
        function = self.read_registers(Register.DI2_FUNCTION)[0]
        if function != DigitalInputFunction.POSITION_TRIGGER:
            raise ConfigurationError(
                f"DI2并未配置为CTRG(Pn401={function})，拒绝触发位置运动"
            )
        trigger_bit = digital_input_bit(2)
        self.force_digital_inputs(base_forced_inputs & ~trigger_bit)
        self.force_digital_inputs(base_forced_inputs | trigger_bit)
        time.sleep(0.03)
        self.force_digital_inputs(base_forced_inputs & ~trigger_bit)

    def read_position_feedback(self) -> dict[str, int]:
        from .protocol import signed16

        return {
            "speed_rpm": signed16(
                self.read_registers(Register.ACTUAL_SPEED)[0]
            ),
            "encoder_total_ppr": self.read_signed32(Register.FEEDBACK_PULSES),
            "feedback_command_pulse": self.read_signed32(
                Register.FEEDBACK_COMMAND_PULSES
            ),
            "position_deviation": self.read_signed32(
                Register.POSITION_DEVIATION
            ),
            "servo_position_command_pulse": self.read_signed32(
                Register.SERVO_POSITION_COMMAND
            ),
            "servo_position_encoder_ppr": self.read_signed32(
                Register.SERVO_POSITION_ENCODER
            ),
            "positioning_status": self.read_registers(
                Register.POSITIONING_STATUS
            )[0],
        }
