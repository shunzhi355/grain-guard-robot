"""兼容转接层 — 统一标定参数已移至项目根目录 ``sampling_params.py``。

现场标定/调整参数请直接编辑项目根目录的 ``sampling_params.py``
（本文件仅做 re-export，保证旧 ``from utils.sampling_params import ...``
引用继续工作）。
"""

from __future__ import annotations

from sampling_params import *  # noqa: F401,F403
from sampling_params import (  # noqa: F401
    CHANNELS,
    PCA9685_I2C_BUS,
    PCA9685_I2C_ADDRESS,
    PCA9685_I2C_DEVICE,
    CLOUD_BASE_URL,
    DEFAULT_GRAIN_PARAMS,
    DEVICE_MAC,
    ENABLE_UNWIRED_CHANNELS,
    GRAIN_MECHANISM_CONFIG,
    PULSE_CLOSE,
    PULSE_MAX_US,
    PULSE_MIN_US,
    PULSE_OPEN,
    PULSE_STOP,
    RC_DEADBAND_HIGH,
    RC_DEADBAND_LOW,
    RC_DEBOUNCE_SAMPLES,
    RC_MAX_ANGULAR_RPS,
    RC_MAX_LINEAR_MPS,
    RC_MODE_RANGES,
    RC_PINS,
    RC_RECEIVER_BACKEND,
    RC_SERIAL_PORT,
    RC_SERIAL_BAUDRATE,
    RC_STICK_CENTER,
    SUPPORTED_GRAINS,
    X2P_DURATION_S,
    X2P_FORWARD_SIGN,
    X2P_PACKAGE_PATH,
    X2P_PORT,
    X2P_POSITION_TOLERANCE_MM,
    X2P_RPM,
    X2P_RETURN_CLEARANCE_MM,
    X2P_SLAVE,
)
