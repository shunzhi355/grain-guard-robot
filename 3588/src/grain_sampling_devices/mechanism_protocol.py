"""STM32 mechanism frame format on the existing chassis UART.

Final device numbering from MECHANISM_PROTOCOL.md: both conveyors are device 2;
firmware also defines bins as 3..5 and the two gripper directions as 6/7.
Bin actions are mirrored to the existing RK3588 I2C PCA9685 outputs.
"""

FRAME_TYPE = 0x36
START, STOP, STOP_ALL = 1, 2, 3
TIGHTEN, CONVEY, BIN_SHALLOW, BIN_MID, BIN_DEEP, CLAMP, UNCLAMP = range(1, 8)
BIN_DEVICES = {"shallow": BIN_SHALLOW, "mid": BIN_MID, "deep": BIN_DEEP}

# Local firmware timings from robot/mechanism.h; UART does not carry duration.
BIN_OPEN_DURATION_SEC = 5.0
BIN_CLOSE_DURATION_SEC = 6.0
CLAMP_DURATION_SEC = 3.0
UNCLAMP_DURATION_SEC = 4.0


def command_payload(command: int, device: int) -> bytes:
    """Validate before sending; STOP on a bin actively closes that bin."""
    if type(command) is not int or command not in (START, STOP, STOP_ALL):
        raise ValueError("invalid mechanism command")
    if type(device) is not int:
        raise ValueError("invalid mechanism device")
    if command == STOP_ALL:
        if device != 0:
            raise ValueError("mechanism STOP_ALL requires device 0")
    elif not TIGHTEN <= device <= UNCLAMP:
        raise ValueError("mechanism device must be in 1..7")
    return bytes((command, device))
