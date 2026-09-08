"""
Centralized application configuration for the grain sampling robot.

Provides ``AppConfig`` — a dataclass with typed defaults for all subsystems,
plus a factory method to build from a flat ``dict`` or JSON file.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, ClassVar, Dict, Optional

from utils.sampling_params import CLOUD_BASE_URL, DEVICE_MAC


@dataclass
class AppConfig:
    """Central configuration for all grain-sampling-robot subsystems.

    Every field has a sensible default so the class can be instantiated
    with ``AppConfig()`` and used immediately.  Per-module overrides are
    supported via ``from_dict()`` or ``from_json_file()``.

    Attributes
    ----------
    mqtt_broker : str
        MQTT broker URL in ``mqtt://host:port`` format.
    device_id : str
        Unique robot/device identifier used in MQTT topics and logging.
    rtsp_port : int
        RTSP / HTTP server port for the camera stream.
    camera_device : str
        V4L2 device path for the camera (Linux only).
    livox_config_path : str
        Path to the Livox LiDAR configuration JSON file.
    slice_default_height : float
        Default slice height (Z-axis) in metres for point-cloud extraction.
    biochemical_port : int
        TCP port for the biochemical analyser device.
    physicochemical_port : int
        TCP port for the physicochemical tester device.
    cloud_base_url : str
        Base URL of the cloud API (placeholder by default).
    device_mac : str
        MAC address override; empty string means auto-detect.
    biochemical_host : str
        Host/IP of the biochemical analyser device.
    physicochemical_host : str
        Host/IP of the physicochemical tester device.
    """

    # ── Cloud / MQTT ────────────────────────────────────
    mqtt_broker: str = "mqtt://localhost:1883"
    device_id: str = "robot-001"

    # ── Camera / RTSP ───────────────────────────────────
    rtsp_port: int = 8554
    camera_device: str = "/dev/video0"

    # ── Livox / Point Cloud ─────────────────────────────
    livox_config_path: str = "config/livox_config.json"
    slice_default_height: float = 0.5

    # ── External Devices ────────────────────────────────
    biochemical_port: int = 5001
    physicochemical_port: int = 5002
    biochemical_host: str = "127.0.0.1"
    physicochemical_host: str = "127.0.0.1"

    # ── Cloud API ───────────────────────────────────────
    cloud_base_url: str = CLOUD_BASE_URL
    device_mac: str = DEVICE_MAC

    # ── Camera / RTMP streaming ──────────────────────────
    rtmp_push_url: str = "rtmp://124.220.41.27:41935/live/1"
    camera_device: str = "/dev/video0"

    # ── Class-level helpers ─────────────────────────────
    _field_names: ClassVar[Optional[set]] = None

    # ── Factory methods ─────────────────────────────────

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> AppConfig:
        """Create an ``AppConfig`` from a flat dictionary.

        Unknown keys are silently ignored.  All values retain their
        defaults unless explicitly overridden.

        Parameters
        ----------
        data : dict
            Flat dictionary (e.g. ``{"device_id": "robot-002"}``).

        Returns
        -------
        AppConfig
        """
        valid = cls._valid_field_names()
        kwargs = {k: v for k, v in data.items() if k in valid}
        return cls(**kwargs)

    @classmethod
    def from_json_file(cls, filepath: str) -> AppConfig:
        """Create an ``AppConfig`` from a JSON configuration file.

        The JSON file should contain a flat object with keys matching the
        field names of this class.

        Parameters
        ----------
        filepath : str
            Path to a JSON file.

        Returns
        -------
        AppConfig

        Raises
        ------
        FileNotFoundError
            If the file does not exist.
        json.JSONDecodeError
            If the file is not valid JSON.
        """
        path = Path(filepath).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    @classmethod
    def from_env(cls, prefix: str = "ROBOT_") -> AppConfig:
        """Create an ``AppConfig`` from environment variables.

        Environment variables are expected in the form
        ``{prefix}{FIELD_NAME}``, case-insensitive.  For example,
        ``ROBOT_DEVICE_ID=robot-002`` overrides ``device_id``.

        Parameters
        ----------
        prefix : str
            Prefix for environment variable names (default ``"ROBOT_"``).

        Returns
        -------
        AppConfig
        """
        overrides: Dict[str, Any] = {}
        valid = cls._valid_field_names()
        # Build a name→type map to handle both real types and string
        # annotations (e.g. from ``from __future__ import annotations``).
        type_map: Dict[str, type] = {}
        for f in fields(cls):
            t = f.type
            # Try identity check first, then string matching for lazy annotations
            if t is int or getattr(t, "__qualname__", "") == "int" or str(t) == "int":
                type_map[f.name] = int
            elif t is float or getattr(t, "__qualname__", "") == "float" or str(t) == "float":
                type_map[f.name] = float
            else:
                type_map[f.name] = str

        for key in os.environ:
            if not key.upper().startswith(prefix.upper()):
                continue
            field_name = key[len(prefix):].lower()
            if field_name in valid:
                raw = os.environ[key]
                field_type = type_map.get(field_name, str)
                if field_type is int:
                    overrides[field_name] = int(raw)
                elif field_type is float:
                    overrides[field_name] = float(raw)
                else:
                    overrides[field_name] = raw
        return cls.from_dict(overrides)

    # ── Helpers ─────────────────────────────────────────

    @classmethod
    def _valid_field_names(cls) -> set:
        if cls._field_names is None:
            cls._field_names = {f.name for f in fields(cls)}
        return cls._field_names

    def as_dict(self) -> Dict[str, Any]:
        """Return all current values as a flat dictionary."""
        return {
            f.name: getattr(self, f.name)
            for f in fields(self)
        }
