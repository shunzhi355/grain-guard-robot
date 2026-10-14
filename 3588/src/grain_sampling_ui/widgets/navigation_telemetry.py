"""Shared read-only XY goal/pose/distance display for both map pages."""
from __future__ import annotations
import math
import time
from PySide2.QtCore import QTimer, Signal
from PySide2.QtWidgets import QWidget, QVBoxLayout, QLabel


class NavigationTelemetry(QWidget):
    readiness_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pose = {}
        self._goal = None
        self._map_ready = False
        self._motion_blocked = False
        self.ready = False
        self.pose_label = QLabel("实时位置：等待定位")
        self.goal_label = QLabel("目标 XY：未选择")
        self.distance_label = QLabel("距目标（直线）：—")
        self.health_label = QLabel("定位：等待实时数据")
        self.result_label = QLabel("导航：未开始")
        self.recovery_label = QLabel("底盘：等待状态")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        for label in (self.pose_label, self.goal_label, self.distance_label,
                      self.health_label, self.recovery_label, self.result_label):
            label.setWordWrap(True)
            layout.addWidget(label)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(200)

    def set_map_ready(self, ready):
        self._map_ready = bool(ready)
        self.refresh()

    def set_goal(self, x=None, y=None):
        self._goal = None if x is None else (float(x), float(y))
        self.refresh()

    def update_status(self, status):
        self._pose = dict(status.get("pose") or {})
        recovery = status.get("recovery") or {}
        self._motion_blocked = bool(status.get("motion_blocked") or recovery.get("reset_required"))
        if recovery.get("reset_required"):
            message = ("单片机已复位；请点故障复位" if recovery.get("reset_detected")
                       else "锁存；请点故障复位")
        else:
            message = "就绪" if recovery.get("reset_ready") else "等待新鲜通信状态"
        self._text(self.recovery_label, "底盘：" + message)
        nav = status.get("navigation") or {}
        if nav:
            state = nav.get("result") or nav.get("state") or "未知"
            self.set_result(str(state))
        self.refresh()

    def set_result(self, text):
        self._text(self.result_label, "导航：" + text)

    def refresh(self):
        pose = self._pose
        finite = all(isinstance(pose.get(k), (int, float)) and math.isfinite(pose[k])
                     for k in ("x_m", "y_m"))
        stamp = pose.get("odom_at")
        age = time.monotonic() - stamp if isinstance(stamp, (int, float)) else float("inf")
        fresh = finite and pose.get("localization_valid") is True and 0 <= age <= 0.3
        if finite:
            self._text(self.pose_label, f"实时 XY：{pose['x_m']:.3f}, {pose['y_m']:.3f} m")
        else:
            self._text(self.pose_label, "实时位置：等待定位")
        self._text(self.health_label, f"定位：有效 · {age:.2f} s" if fresh else "定位：失效 / STALE")
        if self._goal is None:
            self._text(self.goal_label, "目标 XY：未选择")
            self._text(self.distance_label, "距目标（直线）：—")
        else:
            x, y = self._goal
            self._text(self.goal_label, f"目标 XY：{x:.3f}, {y:.3f} m")
            distance = math.hypot(x - pose["x_m"], y - pose["y_m"]) if fresh else None
            self._text(self.distance_label, f"距目标（直线）：{distance:.3f} m" if distance is not None
                       else "距目标（直线）：—（定位失效）")
        ready = fresh and self._map_ready and not self._motion_blocked
        if ready != self.ready:
            self.ready = ready
            self.readiness_changed.emit(ready)

    @staticmethod
    def _text(label, text):
        if label.text() != text:
            label.setText(text)
