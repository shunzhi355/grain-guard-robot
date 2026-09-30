"""Local, ROS-free owner of PCA9685 and X2P mechanism devices."""
from __future__ import annotations

import os
import threading
import logging

from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_devices.x2p_lift import build_x2p_lift_drive
from grain_sampling_workflow.mechanism_config import get_grain_params
from utils.sampling_params import X2P_DURATION_S, X2P_FORWARD_SIGN, X2P_PORT, X2P_RPM, X2P_SLAVE

DEPTHS = ("shallow", "mid", "deep")
logger = logging.getLogger(__name__)


class MechanismRuntime:
    """Exposes only whitelisted mechanism actions to the local workflow."""

    def __init__(self, controller=None):
        self.controller = controller or MechanismController(mock_mode=False)
        self.grain = ""
        self.estop_latched = False
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()

    def start(self):
        # The real reciprocating press defaults to 30 r/min.  Allow a bounded
        # per-run override for supervised bench tests without changing the
        # production calibration or the X2P controller's 500 r/min limit.
        lift_rpm = os.getenv("GRAIN_LIFT_RPM")
        if lift_rpm is not None:
            requested_rpm = int(lift_rpm)
            if not 1 <= requested_rpm <= 300:
                raise ValueError("GRAIN_LIFT_RPM must be in 1..300 r/min")
            self.controller.lift_rpm = requested_rpm
            logger.warning("X2P supervised lift speed override: %d r/min", requested_rpm)
        self.controller.open()
        try:
            self.controller.init_escs(hold_s=3.0)
            port = os.getenv("X2P_PORT", X2P_PORT)
            if port:
                self.controller.lift_drive = build_x2p_lift_drive(
                    port=port, slave=int(os.getenv("X2P_SLAVE", X2P_SLAVE)),
                    rpm=int(os.getenv("X2P_RPM", X2P_RPM)),
                    duration_s=float(os.getenv("X2P_DURATION", X2P_DURATION_S)),
                    forward_sign=int(os.getenv("X2P_FORWARD_SIGN", X2P_FORWARD_SIGN)),
                )
                self.controller.lift_drive.read_position()
        except Exception:
            self.controller.close()
            raise

    def close(self):
        self.controller.close()

    def emergency_stop(self):
        self.estop_latched = True
        self.controller.emergency_stop()

    def set_grain(self, grain: str):
        with self._action_lock:
            if self.estop_latched:
                raise RuntimeError("mechanism emergency stop latched")
            self.controller.set_grain(grain)
            self.grain = grain

    def reset(self, grain: str):
        """Only the trusted local operator workflow may clear mechanism estop."""
        with self._action_lock, self._lock:
            self.controller.set_grain(grain)
            self.controller.reset()
            self.grain = grain
            self.estop_latched = False

    def execute(self, action: str, **args):
        with self._action_lock:
            return self._execute_locked(action, **args)

    def _execute_locked(self, action: str, **args):
        if self.estop_latched:
            raise RuntimeError("mechanism emergency stop latched")
        if action == "lift_health":
            if self.controller.lift_drive is None:
                raise RuntimeError("X2P lift unavailable")
            return self.controller.lift_drive.read_position()
        if action in ("open_bin", "close_bin", "hold_bin_open"):
            depth = args.get("depth")
            if depth not in DEPTHS:
                raise ValueError("invalid bin depth")
            return getattr(self.controller, action)(depth=depth)
        if action == "move_lift":
            direction = args.get("direction")
            distance = float(args.get("distance_cm", 0))
            if direction not in ("up", "down", "down_cycle", "return") or not 0 < distance <= 30:
                raise ValueError("invalid lift movement")
            if self.controller.lift_drive is None:
                raise RuntimeError("X2P lift unavailable")
            return self.controller.move_lift(direction, distance)
        durations = get_grain_params(self.grain)
        actions = {
            "clamp": (self.controller.clamp, "clamp_duration"),
            "unclamp": (self.controller.unclamp, "unclamp_duration"),
            "tighten": (self.controller.tighten, "tighten_duration"),
            "untighten": (self.controller.untighten, "untighten_duration"),
            "convey": (self.controller.convey, "convey_duration"),
            "open_bin_default": (self.controller.open_bin, "open_duration"),
        }
        if action in actions:
            method, key = actions[action]
            return method(duration=durations[key])
        direct = {
            "start_suction": self.controller.fan,
            "stop_suction": lambda: self.controller.actuate(7, "stop"),
            "start_convey": lambda: self.controller.convey(duration=None),
            "stop_convey": lambda: self.controller.convey(duration=None, direction=0),
            "close_all_bins": self.controller.close_all_bins,
            "press": self.controller.press,
            "lift": self.controller.lift,
        }
        if action not in direct:
            raise ValueError("unknown mechanism action")
        return direct[action]()
