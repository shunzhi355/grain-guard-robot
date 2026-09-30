"""Poll the local robot daemon for navigation, pose and RC status."""
from __future__ import annotations

import math

from PySide2.QtCore import QThread, Signal
from PySide2.QtGui import QImage

from grain_sampling_workflow.robot_bridge import RobotClient
from grain_sampling_ui.safety_status import chassis_safety_message


class RobotEventThread(QThread):
    odometry_updated = Signal(float, float, float)
    mechanism_status_updated = Signal(dict)
    map_updated = Signal(dict)
    connection_changed = Signal(bool)
    error = Signal(str)
    cloud_registered_updated = Signal(QImage)
    rc_mode_updated = Signal(str)
    safety_status_updated = Signal(str)

    def __init__(self, parent=None, client=None):
        super().__init__(parent)
        self.client = client or RobotClient()
        self._running = True
        self._connected = False
        self._last_error = ""
        self._last_safety_message = None

    def run(self):
        while self._running:
            try:
                status = self.client.request("status")
                self._last_error = ""
                connected = bool(status.get("lenovo_online"))
                if connected != self._connected:
                    self._connected = connected
                    self.connection_changed.emit(connected)
                chassis = status.get("chassis") or {}
                self.rc_mode_updated.emit(chassis.get("rc_mode", "unknown"))
                safety_message = chassis_safety_message(chassis)
                if safety_message != self._last_safety_message:
                    self._last_safety_message = safety_message
                    self.safety_status_updated.emit(safety_message)
                pose = status.get("pose") or {}
                if all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                       for k in ("x_m", "y_m", "yaw_rad")):
                    self.odometry_updated.emit(pose["x_m"], pose["y_m"], pose["yaw_rad"])
            except (OSError, RuntimeError, ValueError) as exc:
                if self._connected:
                    self._connected = False
                    self.connection_changed.emit(False)
                message = str(exc)
                if message != self._last_error:
                    self._last_error = message
                    self.error.emit(message)
            self.msleep(100)

    def stop(self):
        self._running = False
        self.wait(1500)

    def get_latest_laser_map(self):
        return None
