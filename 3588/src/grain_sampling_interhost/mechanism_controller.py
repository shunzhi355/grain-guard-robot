"""PCA9685/X2P owner with optional action mirroring over the shared STM32 UART."""
from __future__ import annotations

import os
import threading
import logging

from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_devices import mechanism_protocol as mcu
from grain_sampling_devices.x2p_lift import build_x2p_lift_drive
from grain_sampling_workflow.mechanism_config import get_grain_params
from utils.sampling_params import X2P_DURATION_S, X2P_FORWARD_SIGN, X2P_PORT, X2P_RPM, X2P_SLAVE

DEPTHS = ("shallow", "mid", "deep")
logger = logging.getLogger(__name__)


class MechanismRuntime:
    """Exposes only whitelisted mechanism actions to the local workflow."""

    def __init__(self, controller=None, *, serial_command=None):
        self.controller = controller or MechanismController(mock_mode=False)
        # Inject the chassis owner's locked writer; never open a second port.
        self.serial_command = serial_command
        self.grain = ""
        self.estop_latched = False
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._convey_timer = None

    def start(self):
        # The real reciprocating press defaults to 30 r/min.  Allow a bounded
        # per-run override for supervised bench tests without changing the
        # production calibration. The same bound must reach the X2P motion
        # controller, otherwise it rejects the faster timed position moves.
        lift_rpm = os.getenv("GRAIN_LIFT_RPM")
        requested_rpm = None
        if lift_rpm is not None:
            requested_rpm = int(lift_rpm)
            if not 1 <= requested_rpm <= 800:
                raise ValueError("GRAIN_LIFT_RPM must be in 1..800 r/min")
            self.controller.lift_rpm = requested_rpm
            logger.warning("X2P supervised lift speed override: %d r/min", requested_rpm)
        self.controller.open()
        try:
            self.controller.init_escs(hold_s=3.0)
            port = os.getenv("X2P_PORT", X2P_PORT)
            if port:
                self.controller.lift_drive = build_x2p_lift_drive(
                    port=port, slave=int(os.getenv("X2P_SLAVE", X2P_SLAVE)),
                    rpm=(requested_rpm if requested_rpm is not None
                         else int(os.getenv("X2P_RPM", X2P_RPM))),
                    duration_s=float(os.getenv("X2P_DURATION", X2P_DURATION_S)),
                    forward_sign=int(os.getenv("X2P_FORWARD_SIGN", X2P_FORWARD_SIGN)),
                )
                self.controller.lift_drive.read_position()
        except Exception:
            self.controller.close()
            raise

    def close(self):
        with self._lock:
            self.estop_latched = True
            self._cancel_convey_timer()
            try:
                self._serial_stop_all()
            finally:
                self.controller.close()

    def emergency_stop(self):
        with self._lock:
            self.estop_latched = True
            self._cancel_convey_timer()
            try:
                self._serial_stop_all()
            finally:
                self.controller.emergency_stop()

    def _serial_stop_all(self):
        if self.serial_command is not None:
            try:
                self.serial_command(mcu.STOP_ALL, 0)
            except (OSError, RuntimeError):
                logger.exception("STM32 mechanism STOP_ALL failed; local outputs still stopping")

    def _cancel_convey_timer(self):
        timer = self._convey_timer
        self._convey_timer = None
        if timer is not None:
            timer.cancel()

    def _finish_timed_convey(self, timer):
        # Only the latest timed command may stop the conveyors. A manual
        # start/stop or emergency stop cancels the old callback.
        with self._action_lock:
            with self._lock:
                if self._convey_timer is not timer or self.estop_latched:
                    return
                self._convey_timer = None
            try:
                self._mirrored(
                    lambda: self.controller.convey(duration=None, direction=0),
                    ((mcu.STOP, mcu.CONVEY),),
                )
            except Exception:
                logger.exception("Timed conveyor STOP failed")

    def _mirrored(self, operation, commands):
        # These operations only start PWM/timers, so the lock does not span
        # motion duration. It orders START against emergency STOP_ALL.
        with self._lock:
            if self.estop_latched:
                raise RuntimeError("mechanism emergency stop latched")
            try:
                if self.serial_command is not None:
                    for command, device in commands:
                        self.serial_command(command, device)
                return operation()
            except Exception:
                # Either output may have started. Stop both and require an
                # explicit reset, preventing workflow retries from restarting.
                try:
                    self.emergency_stop()
                except Exception:
                    logger.exception("mechanism cleanup after action failure")
                raise

    def set_grain(self, grain: str):
        with self._action_lock:
            if self.estop_latched:
                raise RuntimeError("mechanism emergency stop latched")
            self.controller.set_grain(grain)
            self.grain = grain

    def requires_mechanical_reset(self) -> bool:
        return self.controller._lift_cycle_origin is not None

    def reset(self, grain: str, *, mechanical_reset_confirmed: bool = False):
        """Only the trusted local operator workflow may clear mechanism estop."""
        with self._action_lock, self._lock:
            self._cancel_convey_timer()
            self.controller.set_grain(grain)
            self.controller.reset(
                mechanical_reset_confirmed=mechanical_reset_confirmed
            )
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
            command = mcu.STOP if action == "close_bin" else mcu.START
            return self._mirrored(
                lambda: getattr(self.controller, action)(depth=depth),
                ((command, mcu.BIN_DEVICES[depth]),),
            )
        if action == "move_lift":
            direction = args.get("direction")
            distance = float(args.get("distance_cm", 0))
            if direction not in ("up", "down", "down_cycle", "return") or not 0 < distance <= 30:
                raise ValueError("invalid lift movement")
            if self.controller.lift_drive is None:
                raise RuntimeError("X2P lift unavailable")
            return self.controller.move_lift(direction, distance)
        durations = get_grain_params(self.grain)
        if action == "convey":
            duration = float(durations["convey_duration"])
            with self._lock:
                self._cancel_convey_timer()
            result = self._mirrored(
                lambda: self.controller.convey(duration=None),
                ((mcu.START, mcu.CONVEY),),
            )
            try:
                timer = threading.Timer(
                    duration, lambda: self._finish_timed_convey(timer)
                )
                timer.daemon = True
                with self._lock:
                    if not self.estop_latched:
                        self._convey_timer = timer
                        timer.start()
            except Exception:
                self.emergency_stop()
                raise
            return result
        actions = {
            "clamp": (self.controller.clamp, "clamp_duration"),
            "unclamp": (self.controller.unclamp, "unclamp_duration"),
            "tighten": (self.controller.tighten, "tighten_duration"),
            "untighten": (self.controller.untighten, "untighten_duration"),
            "open_bin_default": (self.controller.open_bin, "open_duration"),
        }
        if action in actions:
            method, key = actions[action]
            commands = {
                "clamp": ((mcu.START, mcu.CLAMP),),
                "unclamp": ((mcu.START, mcu.UNCLAMP),),
                "tighten": ((mcu.START, mcu.TIGHTEN),),
                # Firmware has no reverse-twist action. Stop MCU tightening
                # and leave the existing PCA9685 reverse action in place.
                "untighten": ((mcu.STOP, mcu.TIGHTEN),),
                "open_bin_default": ((mcu.START, mcu.BIN_MID),),
            }
            return self._mirrored(lambda: method(duration=durations[key]), commands[action])
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
        commands = {
            "start_convey": ((mcu.START, mcu.CONVEY),),
            "stop_convey": ((mcu.STOP, mcu.CONVEY),),
            # STOP_ALL only neutralizes outputs; normal closing needs STOP
            # on each bin so the firmware runs its closing PWM and timer.
            "close_all_bins": tuple((mcu.STOP, device) for device in mcu.BIN_DEVICES.values()),
        }
        if action in commands:
            if action in ("start_convey", "stop_convey"):
                with self._lock:
                    self._cancel_convey_timer()
            return self._mirrored(direct[action], commands[action])
        return direct[action]()
