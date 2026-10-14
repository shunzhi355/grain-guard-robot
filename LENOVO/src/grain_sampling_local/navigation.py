"""Segmented XY commissioning controller; all outputs are normalized efforts.

Deadzone is an observed breakaway threshold, not a calibrated stable speed.
Running floors are separate configuration. No UART or motion lease is owned here.
"""
from __future__ import annotations
import math
import time


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PointNavigator:
    def __init__(self, *, far_forward=0.55, near_forward=0.40, max_effort_step=0.15,
                 turn_deadzone=0.18, turn_max=0.20, xy_tolerance=0.12,
                 heading_tolerance=0.20, forward_deadzone=0.0, forward_min=None,
                 turn_min=None, heading_realign_tolerance=0.30,
                 arrival_hysteresis=0.04):
        self.far_forward = self._effort(far_forward, "far_forward")
        self.near_forward = self._effort(near_forward, "near_forward")
        self.max_effort_step = self._effort(max_effort_step, "max_effort_step")
        self.forward_deadzone = self._effort(forward_deadzone, "forward_deadzone")
        self.forward_min = self._effort(near_forward if forward_min is None else forward_min, "forward_min")
        self.turn_deadzone = self._effort(turn_deadzone, "turn_deadzone")
        self.turn_min = self._effort(turn_deadzone if turn_min is None else turn_min, "turn_min")
        self.turn_max = self._effort(turn_max, "turn_max")
        self.xy_tolerance = float(xy_tolerance)
        self.heading_tolerance = float(heading_tolerance)
        self.heading_realign_tolerance = float(heading_realign_tolerance)
        self.arrival_hysteresis = float(arrival_hysteresis)
        if not (0 <= self.forward_deadzone <= self.forward_min <= self.near_forward <= self.far_forward):
            raise ValueError("require forward_deadzone <= forward_min <= near_forward <= far_forward")
        if not (0 <= self.turn_deadzone <= self.turn_min <= self.turn_max):
            raise ValueError("require turn_deadzone <= turn_min <= turn_max")
        if (not all(math.isfinite(v) for v in (self.xy_tolerance, self.heading_tolerance,
                                              self.heading_realign_tolerance, self.arrival_hysteresis))
                or self.xy_tolerance <= 0 or self.arrival_hysteresis < 0
                or not 0 < self.heading_tolerance < self.heading_realign_tolerance <= math.pi
                or self.max_effort_step <= 0):
            raise ValueError("invalid point-navigation tolerance or effort step")
        self.reset()

    @classmethod
    def from_config(cls, config):
        return cls(far_forward=config.point_far_forward, near_forward=config.point_near_forward,
                   max_effort_step=config.point_max_effort_step,
                   forward_deadzone=config.point_forward_deadzone, forward_min=config.point_forward_min,
                   turn_deadzone=config.point_turn_deadzone, turn_min=config.point_turn_min,
                   turn_max=config.point_turn_max, xy_tolerance=config.point_xy_tolerance,
                   heading_tolerance=config.point_heading_tolerance,
                   heading_realign_tolerance=config.point_heading_realign_tolerance,
                   arrival_hysteresis=config.point_arrival_hysteresis)

    def reset(self):
        self.goal_id = None
        self.zero_count = 0
        self.last = (0.0, 0.0)
        self.stuck = None
        self.started = 0.0
        self.phase = "ALIGN"

    def _moving_effort(self, desired, previous, minimum, maximum):
        # Reverse direction through zero. Otherwise clamp the slew toward the
        # desired value, then enforce the stable running floor last.
        if previous * desired < 0:
            return 0.0
        value = max(previous - self.max_effort_step,
                    min(previous + self.max_effort_step, desired))
        return math.copysign(min(maximum, max(minimum, abs(value))), desired)

    def step(self, goal, pose):
        now = time.monotonic()
        if goal["goal_id"] != self.goal_id:
            self.reset()
            self.goal_id = goal["goal_id"]
            self.started = now
        if self.phase == "DONE":
            return 0.0, 0.0, True
        if now - self.started > 120:
            raise RuntimeError("point navigation timed out")
        target = goal["pose"]
        dx, dy = target["x_m"] - pose["x_m"], target["y_m"] - pose["y_m"]
        distance = math.hypot(dx, dy)
        heading = wrap(math.atan2(dy, dx) - pose["yaw_rad"])
        if not all(math.isfinite(v) for v in (distance, heading)):
            raise RuntimeError("non-finite navigation pose/goal")
        if distance <= self.xy_tolerance:
            self.phase = "SETTLE"
        if self.phase == "SETTLE":
            # Hold neutral during small pose jitter; arrival STILL requires
            # three consecutive frames inside the original XY tolerance.
            if distance <= self.xy_tolerance + self.arrival_hysteresis:
                self.last, self.stuck = (0.0, 0.0), None
                self.zero_count = self.zero_count + 1 if distance <= self.xy_tolerance else 0
                if self.zero_count >= 3:
                    self.phase = "DONE"
                return 0.0, 0.0, self.phase == "DONE"
            self.phase, self.zero_count = "ALIGN", 0
        if self.phase == "DRIVE":
            if abs(heading) > self.heading_realign_tolerance:
                self.phase = "ALIGN"
        elif abs(heading) <= self.heading_tolerance:
            self.phase = "DRIVE"
        if self.phase == "ALIGN":
            desired = math.copysign(min(self.turn_max, max(self.turn_min, .3 * abs(heading))), heading)
            forward = 0.0
            turn = self._moving_effort(desired, self.last[1], self.turn_min, self.turn_max)
        else:
            desired = self.near_forward if distance <= .8 else self.far_forward
            forward = self._moving_effort(desired, self.last[0], self.forward_min, self.far_forward)
            turn = 0.0
        self.last = forward, turn
        if forward:
            if self.stuck is None or math.hypot(pose["x_m"]-self.stuck[1], pose["y_m"]-self.stuck[2]) >= .03:
                self.stuck = now, pose["x_m"], pose["y_m"]
            elif now - self.stuck[0] > 2:
                raise RuntimeError("point navigation stuck")
        else:
            self.stuck = None
        return forward, turn, False

    @staticmethod
    def _effort(value, name):
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(f"{name} must be finite and in 0..1")
        return value
