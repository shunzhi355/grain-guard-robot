"""Strict, inert configuration and read-only USB inventory."""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, fields, replace as dataclass_replace
from pathlib import Path
from utils.sampling_params import X2P_RPM


@dataclass(frozen=True)
class LocalConfig:
    stm32_port: str = ""
    x2p_port: str = ""
    x2p_slave: int = 2
    lift_rpm: int = X2P_RPM
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
    map_version: int = 0
    map_manifest: str = ""
    ros_image: str = "grain-nav:noetic-x86"
    # Normalized point-navigation effort.  These are configuration limits,
    # not physical speed claims; defaults preserve the previously validated
    # commissioning behavior while allowing controlled future measurements.
    point_far_forward: float = 0.55
    point_near_forward: float = 0.40
    point_max_effort_step: float = 0.15
    point_forward_deadzone: float = 0.0
    point_forward_min: float = 0.40
    point_turn_deadzone: float = 0.18
    point_turn_min: float = 0.18
    point_turn_max: float = 0.20
    point_xy_tolerance: float = 0.12
    point_heading_tolerance: float = 0.20
    point_heading_realign_tolerance: float = 0.30
    point_arrival_hysteresis: float = 0.04
    # Keep the strict external health contract by default. ``odom_stable``
    # is an explicit fallback for deployments where Golden has no Bool topic.
    localization_health_mode: str = "external"
    require_pointcloud_for_navigation: bool = True
    localization_startup_status_topic: str = ""
    localization_startup_map_version: str = ""
    localization_startup_manifest_sha256: str = ""

    @classmethod
    def load(cls, path):
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict) or set(value) - {field.name for field in fields(cls)}:
            raise ValueError("unknown/non-object local runtime configuration")
        config = cls(**value)
        if not config.map_manifest:
            config = dataclass_replace(
                config,
                map_manifest=os.environ.get("GRAIN_ACTIVE_MAP_MANIFEST", ""),
            )
        if not config.map_id:
            try:
                from grain_sampling_workflow.map_manifest import load_active_manifest
                manifest = load_active_manifest(config.map_manifest or None)
            except (ImportError, OSError, ValueError):
                manifest = None
            if manifest is not None:
                config = dataclass_replace(
                    config,
                    map_id=manifest.map_id,
                    map_version=manifest.map_version,
                    map_frame=manifest.frame,
                )
        config.validate()
        return config

    def validate(self):
        for name in ("stm32_port", "x2p_port", "runtime_dir", "map_frame", "map_id", "map_manifest",
                     "odom_topic", "cloud_topic", "localization_valid_topic", "ros_image",
                     "localization_startup_status_topic", "localization_startup_map_version",
                     "localization_startup_manifest_sha256"):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f"{name} must be a string")
        if self.localization_startup_status_topic and (not self.localization_startup_map_version
                or len(self.localization_startup_manifest_sha256) != 64):
            raise ValueError("startup gate requires pinned map version and manifest SHA256")
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
        if self.localization_health_mode not in ("external", "odom_stable"):
            raise ValueError("localization_health_mode must be external or odom_stable")
        if type(self.require_pointcloud_for_navigation) is not bool:
            raise ValueError("require_pointcloud_for_navigation must be boolean")
        if type(self.map_version) is not int or self.map_version < 0:
            raise ValueError("map_version must be a non-negative integer")
        for name in ("point_far_forward", "point_near_forward", "point_max_effort_step", "point_turn_deadzone", "point_turn_max", "point_xy_tolerance", "point_heading_tolerance", "point_forward_deadzone", "point_forward_min", "point_turn_min", "point_heading_realign_tolerance", "point_arrival_hysteresis"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be in 0..1")
        if not self.point_forward_deadzone <= self.point_forward_min <= self.point_near_forward:
            raise ValueError("point forward deadzone/min/near limits are inconsistent")
        if not self.point_turn_deadzone <= self.point_turn_min <= self.point_turn_max:
            raise ValueError("point turn deadzone/min/max limits are inconsistent")
        if not self.point_heading_tolerance < self.point_heading_realign_tolerance:
            raise ValueError("realign tolerance must exceed heading tolerance")
        if self.point_max_effort_step <= 0:
            raise ValueError("point_max_effort_step must be positive")
        if self.point_near_forward > self.point_far_forward:
            raise ValueError("point_near_forward must be <= point_far_forward")
        if self.point_turn_deadzone > self.point_turn_max or self.point_xy_tolerance <= 0 or self.point_heading_tolerance <= 0:
            raise ValueError("invalid point-navigation deadzone or tolerance")

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
