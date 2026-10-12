"""Small, reusable preflight for the Lenovo navigation session.

This module reports readiness for the already existing local daemon and ROS
adapter.  It deliberately does not open a serial device, start ROS, or send a
motion request; those actions remain owned by the existing launchers/services.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any


@dataclass
class Gate:
    name: str
    state: str
    detail: str


def _gate(name: str, state: str, detail: str) -> Gate:
    return Gate(name, state, detail)


def _socket_status(socket_path: str) -> dict[str, Any]:
    from grain_sampling_workflow.robot_bridge import RobotClient
    return RobotClient(socket_path).request("status")


def _status_gates(status: dict[str, Any], *, require_local: bool) -> list[Gate]:
    gates: list[Gate] = []
    if status.get("runtime") == "lenovo-local":
        gates.append(_gate("local_service", "PASS", "Lenovo local daemon identity matches"))
    else:
        gates.append(_gate("local_service", "BLOCKED", "local daemon is not lenovo-local"))

    if require_local and status.get("test_navigation_mode") is True:
        gates.append(_gate("navigation_mode", "BLOCKED", "daemon is operator mode; local mode is required for motion"))
    elif require_local and status.get("lenovo_online") is not True:
        gates.append(_gate("navigation_mode", "BLOCKED", "local navigation input is not online"))
    else:
        gates.append(_gate("navigation_mode", "PASS", "mode is explicit and compatible with this check"))

    pose = status.get("pose") or {}
    try:
        finite_pose = all(math.isfinite(float(pose[k])) for k in ("x_m", "y_m", "yaw_rad"))
    except (KeyError, TypeError, ValueError):
        finite_pose = False
    valid = pose.get("localization_valid") is True
    if finite_pose and valid:
        gates.append(_gate("localization", "PASS", "latest pose is finite and marked valid"))
    else:
        gates.append(_gate("localization", "BLOCKED", "pose is missing, invalid, or non-finite"))

    slam = status.get("slam") or {}
    if isinstance(slam.get("map_id"), str) and slam["map_id"]:
        gates.append(_gate("map_identity", "PASS", f"active map_id={slam['map_id']}"))
    else:
        gates.append(_gate("map_identity", "BLOCKED", "no active map identity"))

    chassis = status.get("chassis") or {}
    if chassis.get("rc_mode") == "auto" and chassis.get("faults", 0) == 0:
        gates.append(_gate("rc_and_faults", "PASS", "RC auto and no reported faults"))
    else:
        gates.append(_gate("rc_and_faults", "BLOCKED", f"rc_mode={chassis.get('rc_mode')!r}, faults={chassis.get('faults')}"))
    return gates


def _serial_owner_gates(config: Any) -> list[Gate]:
    if os.name != "posix":
        return [_gate("serial_ownership", "UNKNOWN", "serial ownership is only inspectable on the Lenovo Linux host")]
    fuser = shutil.which("fuser")
    if not fuser:
        return [_gate("serial_ownership", "UNKNOWN", "fuser is unavailable")]
    result = subprocess.run([fuser, "-v", config.stm32_port], capture_output=True, text=True, check=False)
    owners = result.stdout.strip() or result.stderr.strip()
    if result.returncode == 0:
        return [_gate("serial_ownership", "PASS", owners)]
    return [_gate("serial_ownership", "PASS", "STM32 serial has no competing process")]


def check_session(config_path: str, *, require_local: bool = False) -> dict[str, Any]:
    from grain_sampling_local.config import LocalConfig

    config = LocalConfig.load(Path(config_path))
    gates: list[Gate] = []
    try:
        config.validate()
        gates.append(_gate("configuration", "PASS", "configuration schema and safety values are valid"))
    except Exception as exc:
        gates.append(_gate("configuration", "BLOCKED", str(exc)))

    socket_path = str(config.socket_path)
    try:
        status = _socket_status(socket_path)
    except Exception as exc:
        status = {}
        gates.append(_gate("control_socket", "BLOCKED", f"daemon status unavailable: {exc}"))
    else:
        gates.append(_gate("control_socket", "PASS", socket_path))
        gates.extend(_status_gates(status, require_local=require_local))
    gates.extend(_serial_owner_gates(config))
    passed = all(g.state == "PASS" for g in gates)
    return {
        "ok": passed,
        "checked_at": time.time(),
        "config": str(Path(config_path).resolve()),
        "navigation_mode": config.navigation_mode,
        "gates": [asdict(g) for g in gates],
        "status": status,
    }


def format_report(report: dict[str, Any]) -> str:
    return json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
