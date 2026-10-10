"""The existing full sampling/lift workflow with only its UART output enabled."""
from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path

from grain_sampling_devices.mechanism_driver import _BaseMechanismController
from grain_sampling_interhost.mechanism_controller import MechanismRuntime

logger = logging.getLogger(__name__)


class LiftWorkflowController(_BaseMechanismController):
    """Reuse calibrated press/extraction logic, never construct a PCA9685.

    Non-lift methods are not used as outputs: SerialWorkflowRuntime sends the
    existing UART command list instead. Accidental PWM calls fail closed.
    """
    def __init__(self, lift, rpm):
        super().__init__(pca9685=None, mock_mode=False)
        self.lift_drive = lift
        self.lift_rpm = rpm
        self._clamp_timer = None  # MCU owns gripper timing; runtime still waits.

    def _write_hw(self, *args):
        raise RuntimeError("PCA9685 output is forbidden in Lenovo serial-only mode")

    _write_hw_off = _write_hw

    def emergency_stop(self):
        self._stop_flag.set()
        # A running position operation owns this lock and observes cancel_check.
        # Never insert a stop from another thread between enable and trigger.
        if self._lift_motion_lock.acquire(blocking=False):
            try:
                self.lift_drive.stop()
            finally:
                self._lift_motion_lock.release()

    def close(self):
        self._stop_flag.set()
        with self._lift_motion_lock:
            try:
                self.lift_drive.stop()
            finally:
                self.lift_drive.close()


class SerialWorkflowRuntime(MechanismRuntime):
    def __init__(self, lift, *, rpm, serial_command, check, state_path,
                 suction_policy="required", event=None):
        super().__init__(LiftWorkflowController(lift, rpm), serial_command=serial_command)
        if suction_policy not in ("required", "external"):
            raise ValueError("suction_policy must be required or external")
        self.check = check
        self.suction_policy = suction_policy
        self.event = event or (lambda *args, **kwargs: None)
        self.state_path = Path(state_path)
        self.recovery_pending = False
        if self.state_path.exists():
            # Corrupt state must block startup, not silently lose recovery data.
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            if type(state.get("recovery_pending")) is not bool:
                raise ValueError("invalid lift recovery state; inspect mechanically before repairing it")
            self.recovery_pending = state["recovery_pending"]
        if self.recovery_pending:
            self.estop_latched = True
            self.controller._stop_flag.set()

    def start(self):
        # Read only: opening the UI/daemon does not enable or reposition X2P.
        self.controller.lift_drive.read_position()

    def _save_recovery(self, pending):
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump({"recovery_pending": bool(pending)}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.state_path)
        if hasattr(os, "O_DIRECTORY"):
            descriptor = os.open(self.state_path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        self.recovery_pending = bool(pending)

    def requires_mechanical_reset(self):
        return self.recovery_pending or super().requires_mechanical_reset()

    def reset(self, grain, *, mechanical_reset_confirmed=False):
        if self.requires_mechanical_reset() and not mechanical_reset_confirmed:
            raise RuntimeError("incomplete lift operation: physical reset confirmation required")
        super().reset(grain, mechanical_reset_confirmed=mechanical_reset_confirmed)
        self._save_recovery(False)

    def _mirrored(self, operation, commands):
        # Do not invoke the legacy PCA operation. Command ordering/timers and
        # unclamp-before-return still come from the unchanged parent runtime.
        self.check()
        if self.estop_latched:
            raise RuntimeError("mechanism emergency stop latched")
        try:
            for command, device in commands:
                self.serial_command(command, device)
                self.event("stm32_command", command=command, device=device)
            self.check()
        except Exception:
            self.emergency_stop()
            raise

    def _execute_locked(self, action, **args):
        self.check()
        if self.estop_latched:
            raise RuntimeError("mechanism emergency stop latched")
        self.event("mechanism_begin", action=action, args=args)
        if action in ("start_suction", "stop_suction"):
            if self.suction_policy != "external":
                raise RuntimeError("existing 3588 UART flow has no suction opcode; configure/confirm external suction before full workflow")
            self.event("external_suction_required", action=action, serial_command_sent=False)
            return {"external_control": True, "execution_confirmed": False}
        lifting = action in ("move_lift", "press", "lift")
        if lifting:
            self._save_recovery(True)  # write before any possible physical move
        try:
            result = super()._execute_locked(action, **args)
            if lifting and self.controller._lift_cycle_origin is None:
                self._save_recovery(False)
            self.event("mechanism_end", action=action)
            return result
        except Exception:
            self.emergency_stop()
            raise
