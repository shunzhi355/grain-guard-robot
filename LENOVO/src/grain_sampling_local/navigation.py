"""Conservative point controller for commissioning, not an obstacle-avoiding planner.

The chassis remains encoderless: outputs are normalized efforts, not measured
physical velocities. The local daemon independently checks pose, cloud, RC,
authorization, command lifetime and arrival before accepting any movement.
"""
from __future__ import annotations

import math
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PointNavigator:
    def __init__(self):
        self.goal_id = None
        self.zero_count = 0
        self.last = (0.0, 0.0)
        self.stuck = None
        self.started = 0

    def step(self, goal, pose):
        now = time.monotonic()
        if goal["goal_id"] != self.goal_id:
            self.goal_id = goal["goal_id"]
            self.zero_count, self.stuck, self.last = 0, None, (0.0, 0.0)
            self.started = now
        if now - self.started > 120:
            raise RuntimeError("point navigation timed out")
        target = goal["pose"]
        dx, dy = target["x_m"] - pose["x_m"], target["y_m"] - pose["y_m"]
        distance = math.hypot(dx, dy)
        heading = wrap(math.atan2(dy, dx) - pose["yaw_rad"])
        yaw_error = wrap(target["yaw_rad"] - pose["yaw_rad"])
        if distance <= 0.12:
            forward = 0.0
            turn = 0.0 if abs(yaw_error) <= 0.15 else max(-0.2, min(0.2, 0.3 * yaw_error))
        else:
            forward = 0.0 if abs(heading) > math.pi / 4 else (0.4 if distance <= 0.8 else 0.55)
            bound = 0.2 if forward == 0 else 0.12
            turn = max(-bound, min(bound, 0.3 * heading))
        if forward == turn == 0:
            self.last = (0.0, 0.0)
            self.zero_count += 1
            return 0.0, 0.0, self.zero_count >= 3
        self.zero_count = 0
        forward, turn = (max(old - 0.15, min(old + 0.15, value)) for old, value in zip(self.last, (forward, turn)))
        self.last = forward, turn
        if abs(forward) >= 0.05:
            if self.stuck is None or math.hypot(pose["x_m"] - self.stuck[1], pose["y_m"] - self.stuck[2]) >= 0.03:
                self.stuck = (now, pose["x_m"], pose["y_m"])
            elif now - self.stuck[0] > 2:
                raise RuntimeError("point navigation stuck")
        else:
            self.stuck = None
        return forward, turn, False
