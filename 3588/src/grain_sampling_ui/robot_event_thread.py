"""Poll the local robot daemon for navigation, pose and RC status."""
from __future__ import annotations

import math
import os
import json
import time
import threading
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
    saved_map_updated = Signal(dict)
    navigation_telemetry_updated = Signal(dict)

    def __init__(self, parent=None, client=None):
        super().__init__(parent)
        self.client = client or RobotClient()
        self._running = True
        self._connected = False
        self._last_error = ""
        self._last_safety_message = None
        self._last_local_view = None
        self._last_rc_mode = None
        self._last_rc_emit_at = float("-inf")
        self._telemetry_emit_at = float("-inf")
        self._preview_check_at = 0.0
        self._saved_preview_key = None
        self._saved_preview_loading = False

    def run(self):
        while self._running:
            try:
                status = (self.client.request("status", client_role="ui")
                          if os.getenv("GRAIN_LOCAL_RUNTIME") == "1"
                          else self.client.request("status"))
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1" and status.get("runtime") != "lenovo-local":
                    raise RuntimeError("连接到的不是联想本机硬件服务")
                self._last_error = ""
                connected = (status.get("runtime") == "lenovo-local"
                             or bool(status.get("lenovo_online")))
                if connected != self._connected:
                    self._connected = connected
                    self.connection_changed.emit(connected)
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1":
                    # The socket request is the 100 ms UI heartbeat, but the
                    # Qt widget state rarely changes. Avoid marshalling a
                    # nested dict and repainting pixmaps ten times a second.
                    local_view = (status.get("simulation"),
                                  status.get("test_navigation_mode"),
                                  status.get("lenovo_online"))
                    if local_view != self._last_local_view:
                        self._last_local_view = local_view
                        self.local_runtime_updated.emit(status)
                chassis = status.get("chassis") or {}
                mode = chassis.get("rc_mode", "unknown")
                now = time.monotonic()
                if mode != self._last_rc_mode or now - self._last_rc_emit_at >= 0.5:
                    self._last_rc_mode, self._last_rc_emit_at = mode, now
                    self.rc_mode_updated.emit(mode)
                safety_message = chassis_safety_message(chassis)
                if status.get("runtime") == "lenovo-local" and status.get("safety_latched"):
                    safety_message = safety_message or f"本机安全锁定：{status.get('last_error', '')}"
                if safety_message != self._last_safety_message:
                    self._last_safety_message = safety_message
                    self.safety_status_updated.emit(safety_message)
                pose = status.get("pose") or {}
                if now - self._telemetry_emit_at >= 0.2:
                    self._telemetry_emit_at = now
                    self.navigation_telemetry_updated.emit({"pose": dict(pose),
                        "navigation": status.get("navigation"), "recovery": status.get("recovery"),
                        "motion_blocked": bool(status.get("safety_latched") or chassis.get("estop_latched"))})
                if all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                       for k in ("x_m", "y_m", "yaw_rad")):
                    self.odometry_updated.emit(pose["x_m"], pose["y_m"], pose["yaw_rad"])
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1":
                    self._local_preview()
                    self._saved_preview(status)
            except (OSError, RuntimeError, ValueError) as exc:
                self.navigation_telemetry_updated.emit({"pose": {}, "navigation": None})
                self._last_local_view = None
                self._last_rc_mode = None
                self._last_rc_emit_at = float("-inf")
                if self._connected:
                    self._connected = False
                    self.connection_changed.emit(False)
                message = (f"本机硬件服务未就绪：{exc}"
                           if os.getenv("GRAIN_LOCAL_RUNTIME") == "1" else str(exc))
                if message != self._last_error:
                    self._last_error = message
                    self.error.emit(message)
                if os.getenv("GRAIN_LOCAL_RUNTIME") == "1":
                    # Keep showing the last selected map as an explicitly
                    # unverified snapshot instead of falling back to the legacy
                    # slice UI when the daemon is unavailable.
                    self._saved_preview(None)
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

    def _saved_preview(self, status):
        """Read saved-map identity without blocking the 100 ms safety heartbeat."""
        now = time.monotonic()
        if self._saved_preview_loading or now - self._preview_check_at < 1.0:
            return
        self._preview_check_at = now
        pointer = os.environ.get("GRAIN_SAVED_MAP_PREVIEW")
        if not pointer:
            return
        offline = not status
        slam = (status or {}).get("slam") or {}
        from grain_sampling_workflow.saved_map_preview import preview_paths
        try:
            pointer_value = json.loads(Path(pointer).read_text(encoding="utf-8"))
            candidate = pointer_value.get("candidate") is True
            paths = preview_paths(pointer)
            key = ("offline" if offline else slam.get("map_id"),
                   "" if offline else slam.get("frame"), candidate,
                   tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            key = ("error", str(exc))
            if key != self._saved_preview_key:
                self.saved_map_updated.emit({"error": str(exc)})
                self._saved_preview_key = key
            return
        if key == self._saved_preview_key:
            return
        self._saved_preview_loading = True
        self.saved_map_updated.emit({"error": "正在校验保存地图…"})
        threading.Thread(target=self._load_saved_preview,
                         args=(pointer, dict(slam), key, offline), daemon=True,
                         name="saved-map-preview-loader").start()

    def _load_saved_preview(self, pointer, slam, key, offline=False):
        from grain_sampling_workflow.saved_map_preview import load_saved_preview, preview_paths
        try:
            pointer_value = json.loads(Path(pointer).read_text(encoding="utf-8"))
            candidate = pointer_value.get("candidate") is True
            data = load_saved_preview(pointer, map_id=slam.get("map_id"), frame=slam.get("frame"),
                                      allow_candidate=candidate or offline)
            data["candidate"] = candidate
            data["offline"] = bool(offline)
            data["active_map_match"] = (not offline) and not candidate and data.get("map_id") == slam.get("map_id")
            image = QImage(data["image_path"])
            if image.isNull() or (image.width(), image.height()) != (data["width_px"], data["height_px"]):
                raise ValueError("saved map image unreadable or dimensions differ")
            signature = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in preview_paths(pointer))
            if signature != key[3]:
                raise ValueError("saved map artifacts changed during validation")
            data["image"] = image
            self.saved_map_updated.emit(data)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.saved_map_updated.emit({"error": str(exc)})
        finally:
            self._saved_preview_key = key
            self._saved_preview_loading = False

    def get_latest_laser_map(self):
        return None
