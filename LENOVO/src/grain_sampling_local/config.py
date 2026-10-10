"""Strict, inert configuration and read-only USB inventory."""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class LocalConfig:
    stm32_port: str = ""
    x2p_port: str = ""
    x2p_slave: int = 2
    lift_rpm: int = 30
    forward_sign: int = 1
    navigation_mode: str = "operator"
    suction_policy: str = "required"
    acknowledge_legacy_untighten: bool = False
    runtime_dir: str = ""
    # Optical navigation uses dimensionless effort via the original conversion.
    odom_topic: str = "/Odometry"
    cloud_topic: str = "/cloud_registered_body"
    localization_valid_topic: str = "/grain/localization_valid"
    map_frame: str = "camera_init"
    map_id: str = ""
    ros_image: str = "grain-nav:noetic-x86"

    @classmethod
    def load(cls, path):
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict) or set(value) - {field.name for field in fields(cls)}:
            raise ValueError("unknown/non-object local runtime configuration")
        config = cls(**value)
        config.validate()
        return config

    def validate(self):
        for name in ("stm32_port", "x2p_port", "runtime_dir", "map_frame", "map_id",
                     "odom_topic", "cloud_topic", "localization_valid_topic", "ros_image"):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f"{name} must be a string")
        if self.navigation_mode not in ("operator", "local"):
            raise ValueError("navigation_mode must be operator or local")
        if self.suction_policy not in ("required", "external"):
            raise ValueError("suction_policy must be required or external")
        if type(self.lift_rpm) is not int or not 1 <= self.lift_rpm <= 800:
            raise ValueError("lift_rpm must be an integer in 1..800")
        if type(self.x2p_slave) is not int or not 1 <= self.x2p_slave <= 247:
            raise ValueError("invalid X2P slave")
        # Existing absolute-cycle formulas are calibrated for forward_sign=1.
        if type(self.forward_sign) is not int or self.forward_sign != 1:
            raise ValueError("full workflow requires calibrated forward_sign=1; do not invert without recalibrating absolute lift logic")
        if type(self.acknowledge_legacy_untighten) is not bool:
            raise ValueError("acknowledge_legacy_untighten must be boolean")

    @property
    def directory(self):
        return Path(self.runtime_dir).expanduser() if self.runtime_dir else Path.home() / ".local/state/grain-lenovo"

    @property
    def socket_path(self):
        return self.directory / "control.sock"

    def live_preflight(self):
        self.validate()
        if os.name != "posix":
            raise RuntimeError("live Lenovo runtime requires Linux")
        ports = [Path(port) for port in (self.stm32_port, self.x2p_port)]
        for port in ports:
            if not str(port).startswith("/dev/serial/by-id/"):
                raise ValueError("live ports must use explicit /dev/serial/by-id USB identities")
            if not port.exists() or not os.access(port, os.R_OK | os.W_OK):
                raise RuntimeError(f"serial port absent/inaccessible: {port}")
        if ports[0].resolve() == ports[1].resolve():
            raise ValueError("STM32 and X2P must be different physical serial devices")
        if not self.acknowledge_legacy_untighten:
            raise RuntimeError("confirm that untighten follows the original STOP TIGHTEN UART command")
        if self.suction_policy != "external":
            raise RuntimeError("suction has no existing UART opcode; external suction handling must be explicitly configured")


def inventory():
    from serial.tools import list_ports
    return [{"device": port.device, "description": port.description,
             "vid": port.vid, "pid": port.pid, "serial_number": port.serial_number,
             "location": port.location} for port in list_ports.comports()]
