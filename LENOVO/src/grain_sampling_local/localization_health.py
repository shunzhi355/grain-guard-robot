"""Small, deterministic localization-health gate for the ROS adapter."""
from __future__ import annotations

import math


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationHealthGate:
    """Require consecutive good samples before asserting localization health."""

    def __init__(self, *, frame_id="camera_init", required_samples=3,
                 max_pose_jump=1.0, max_yaw_jump=math.radians(75.0),
                 max_sample_age=0.2, max_stream_gap=0.5):
        if not frame_id or required_samples < 1:
            raise ValueError("invalid localization health gate configuration")
        self.frame_id = frame_id.lstrip("/")
        self.required_samples = int(required_samples)
        self.max_pose_jump = float(max_pose_jump)
        self.max_yaw_jump = float(max_yaw_jump)
        self.max_sample_age = float(max_sample_age)
        self.max_stream_gap = float(max_stream_gap)
        self._last = None
        self._last_receive = 0.0
        self._good = 0
        self._valid = False
        self.reason = "initializing"

    @property
    def valid(self):
        return self._valid

    def accept(self, *, now, stamp, x, y, yaw, frame_id):
        frame = str(frame_id).lstrip("/")
        values = (now, stamp, x, y, yaw)
        if not all(_finite(value) for value in values):
            return self._reject("non_finite")
        age = now - stamp
        if age < 0.0 or age > self.max_sample_age:
            return self._reject("stale_or_future")
        if frame != self.frame_id:
            return self._reject("frame_mismatch")
        if self._last is not None:
            last_stamp, last_x, last_y, last_yaw = self._last
            if stamp <= last_stamp:
                return self._reject("timestamp_not_increasing")
            dt = stamp - last_stamp
            if dt <= self.max_stream_gap:
                if math.hypot(x - last_x, y - last_y) > self.max_pose_jump:
                    return self._reject("pose_jump")
                if abs(_wrap(yaw - last_yaw)) > self.max_yaw_jump:
                    return self._reject("yaw_jump")
        self._last = (stamp, x, y, yaw)
        self._last_receive = now
        self._good += 1
        self._valid = self._good >= self.required_samples
        self.reason = "healthy" if self._valid else "warming_up"
        return self._valid

    def tick(self, *, now):
        if not _finite(now) or self._last_receive <= 0.0 or now - self._last_receive > self.max_stream_gap:
            self._valid = False
            self._good = 0
            self.reason = "odometry_timeout"
        return self._valid

    def _reject(self, reason):
        self._valid = False
        self._good = 0
        self.reason = reason
        return False
