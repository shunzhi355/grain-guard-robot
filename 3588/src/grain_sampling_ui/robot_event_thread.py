"""Poll the local robot daemon for navigation, pose and RC status."""
from __future__ import annotations

import math
import os
import json
import time
from pathlib import Path

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
    local_runtime_updated = Signal(dict)

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
                status = (self.client.request("status", client_role="ui")
                          if os.getenv("GRAIN_LOCAL_RUNTIME") == "1"
                          else self.client.request("status"))
                self._last_error = ""
                connected = (status.get("runtime") == "lenovo-local"
                             or bool(status.get("lenovo_online")))
                if connected != self._connected:
                    self._connected = connected
                    self.connection_changed.emit(connected)
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1":
                    self.local_runtime_updated.emit(status)
                chassis = status.get("chassis") or {}
                self.rc_mode_updated.emit(chassis.get("rc_mode", "unknown"))
                safety_message = chassis_safety_message(chassis)
                if status.get("runtime") == "lenovo-local" and status.get("safety_latched"):
                    safety_message = safety_message or f"本机安全锁定：{status.get('last_error', '')}"
                if safety_message != self._last_safety_message:
                    self._last_safety_message = safety_message
                    self.safety_status_updated.emit(safety_message)
                pose = status.get("pose") or {}
                if all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                       for k in ("x_m", "y_m", "yaw_rad")):
                    self.odometry_updated.emit(pose["x_m"], pose["y_m"], pose["yaw_rad"])
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1":
                    self._local_preview()
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

    def _local_preview(self):
        """Bounded same-host preview file; no dual-host map transfer."""
        path = os.getenv("GRAIN_LOCAL_PREVIEW")
        if not path:
            return
        try:
            with Path(path).open("r", encoding="utf-8") as stream:
                value = json.loads(stream.read(200000))
            if not 0 <= time.monotonic() - value["at"] < 1:
                return
            from PySide2.QtGui import QColor, QPainter
            picture = QImage(500, 500, QImage.Format_RGB32)
            picture.fill(QColor("#101923"))
            painter = QPainter(picture)
            painter.setPen(QColor("#59d7cf"))
            for point in value["points"][:3000]:
                if len(point) == 2 and all(isinstance(v, (float, int)) and math.isfinite(v) for v in point):
                    x, y = point
                    px, py = round(250 - y * 20), round(400 - x * 20)
                    if 0 <= px < 500 and 0 <= py < 500:
                        painter.drawPoint(px, py)
            painter.end()
            self.cloud_registered_updated.emit(picture)
        except (OSError, ValueError, KeyError, TypeError):
            return

    def get_latest_laser_map(self):
        return None
