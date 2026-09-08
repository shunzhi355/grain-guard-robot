"""Public API for the X2P Modbus RTU controller."""

from .drive import X2PDrive
from .errors import (
    CommunicationError,
    ConfigurationError,
    MotionTimeoutError,
    PositionNotReachedError,
    SafetyInterlockError,
    X2PError,
)
from .models import (
    ControllerConfig,
    Direction,
    MotionResult,
    SafetyLimits,
    TimedMovePlan,
    direction_sign,
    distance_to_pulses,
    plan_timed_move,
)
from .motion import MotionController, MotionState
from .protocol import ModbusRTUClient, add_crc, crc16, signed16

__all__ = [
    "CommunicationError",
    "ConfigurationError",
    "ControllerConfig",
    "Direction",
    "ModbusRTUClient",
    "MotionController",
    "MotionResult",
    "MotionState",
    "MotionTimeoutError",
    "PositionNotReachedError",
    "SafetyInterlockError",
    "SafetyLimits",
    "TimedMovePlan",
    "X2PDrive",
    "X2PError",
    "add_crc",
    "crc16",
    "direction_sign",
    "distance_to_pulses",
    "plan_timed_move",
    "signed16",
]
