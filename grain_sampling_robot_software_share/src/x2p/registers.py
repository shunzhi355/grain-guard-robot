"""X2P register map used by this project.

Parameter suffixes are decimal offsets from the hexadecimal group base.  For
example Pn321 is 0x0300 + decimal 21, therefore its address is 0x0315.
"""

from enum import IntEnum


class Register(IntEnum):
    CONTROL_MODE = 0x0001  # Pn001; restart required
    COMMAND_PULSES_PER_REV = 0x0008  # Pn008

    SPEED_SOURCE = 0x0300  # Pn300: 0 uses Pn301
    SPEED_COMMAND = 0x0301  # Pn301, signed r/min
    POSITION_SOURCE = 0x0315  # Pn321: 1 internal multi-position

    DI1_FUNCTION = 0x0400  # Pn400: 1 SRV-ON
    DI2_FUNCTION = 0x0401  # Pn401: 11 CTRG; restart required
    FORCE_DIGITAL_INPUTS = 0x040F  # Pn415: bit0 DI1, bit1 DI2

    MODBUS_NO_SAVE = 0x0604  # Pn604
    MODBUS_SAVE_POLICY = 0x0605  # Pn605

    POSITION_MODE = 0x0700  # Pn700
    POSITION_SEGMENT = 0x0701  # Pn701
    POSITION_ACCEL = 0x0703  # Pn703, ms
    POSITION_DECEL = 0x0704  # Pn704, ms
    POSITION_S_CURVE = 0x0705  # Pn705, ms
    PR1_PULSES = 0x0706  # Pn706, signed 32-bit
    PR1_SPEED = 0x0708  # Pn708, r/min

    ACTUAL_SPEED = 0x2000  # Un000, signed r/min
    MONITORED_SPEED_COMMAND = 0x2001  # Un001, signed r/min
    TORQUE_COMMAND = 0x2002  # Un002, signed 0.1%
    FEEDBACK_PULSES = 0x200C  # Un012, signed 32-bit
    FEEDBACK_COMMAND_PULSES = 0x200E  # Un014, signed 32-bit
    POSITION_DEVIATION = 0x2010  # Un016, signed 32-bit
    SERVO_POSITION_COMMAND = 0x2014  # Un020, signed 32-bit
    SERVO_POSITION_ENCODER = 0x2016  # Un022, signed 32-bit
    DIGITAL_INPUT_STATUS = 0x2020  # Un032
    CURRENT_POSITION_SEGMENT = 0x202A  # Un042
    POSITIONING_STATUS = 0x202C  # Un044
    SERVO_ENABLE_STATUS = 0x203A  # Un058

    STATUS = 0x3E00
    LAST_FAULT_CODE = 0x2064  # Un100 最后一次故障码；低字节=E码(E04/E37等)

    INTERNAL_SERVO_ON = 0x3F00  # Fn000
    SOFTWARE_RESET = 0x3F06  # Fn006


class ControlMode(IntEnum):
    POSITION = 0
    SPEED = 1
    SPEED_POSITION_AT_ZERO = 3


class DriveStatus(IntEnum):
    OFF = 1
    RUN = 2
    FAULT = 4  # 驱动器报警/故障态；具体E码读 LAST_FAULT_CODE(0x2064) 低字节


class DigitalInputFunction(IntEnum):
    SERVO_ON = 1
    CONTROL_MODE = 5
    POSITION_TRIGGER = 11


def digital_input_function_register(di_number: int) -> int:
    """Return Pn400..Pn403 for the physical DI number 1..4."""
    if not 1 <= di_number <= 4:
        raise ValueError("数字输入编号必须在1..4之间")
    return int(Register.DI1_FUNCTION) + di_number - 1


def digital_input_bit(di_number: int) -> int:
    """Return the Pn415/Un032 bit mask for physical DI number 1..8."""
    if not 1 <= di_number <= 8:
        raise ValueError("数字输入编号必须在1..8之间")
    return 1 << (di_number - 1)
