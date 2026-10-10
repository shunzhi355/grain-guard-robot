"""Non-ROS, single-owner 3588 to STM32 chassis control.

TX commands share a lock. A separate RX worker dispatches CH8 status and
mechanism replies without holding the TX/state lock.
Loss of fresh velocity commands revokes chassis motion locally.
"""
from __future__ import annotations

import logging
import math
import os
import secrets
import threading
import time

from grain_sampling_devices import chassis_protocol as stm32
from grain_sampling_devices import mechanism_protocol as mechanism
from grain_sampling_devices.chassis_serial import (
    ChassisError, ChassisSerial, DEFAULT_SERIAL_PORT, MechanismDisconnectedError,
    MechanismRejectedError, MechanismTimeoutError,
)

logger = logging.getLogger(__name__)
MECHANISM_RETRY_WINDOW_S = 10.0
MECHANISM_RETRY_DELAY_S = 0.1


class ChassisController:
    def __init__(self, port: str | None = None, *, serial_factory=None,
                 clock=time.monotonic, timeout_s: float = 0.2):
        self.port = port or os.getenv("CHASSIS_SERIAL_PORT", DEFAULT_SERIAL_PORT)
        self.clock = clock
        self.timeout_s = timeout_s
        self.serial_factory = serial_factory or self._open_serial
        self.lock = threading.RLock()
        self._mechanism_idle = threading.Condition(self.lock)
        self.running = threading.Event()
        self.worker: threading.Thread | None = None
        self.receiver: threading.Thread | None = None
        self.link: ChassisSerial | None = None
        self.mode = "unknown"
        self.epoch: int | None = None
        self.goal_id: str | None = None
        self.faulted_goal_id: str | None = None
        self.last_command: float | None = None
        self.effort = (0, 0)
        self.estop_latched = False
        self.obstacle = False
        self.last_stop_reason = "startup"
        self._mcu_boot: int | None = None
        self._last_status_received_at = float("-inf")
        self.last_mechanism_reply: dict | None = None
        self._serial_session: int | None = None
        self._serial_sequence = 0
        self._pending_mechanism = None
        self._ever_connected = False

    def _open_serial(self):
        import serial
        return serial.Serial(self.port, 115200, timeout=0,
                             write_timeout=0.05, exclusive=True)

    def start(self):
        if self.running.is_set():
            return
        self.running.set()
        self.worker = threading.Thread(target=self._loop, daemon=True, name="chassis-tx")
        self.receiver = threading.Thread(target=self._receive_loop, daemon=True, name="chassis-rx")
        self.worker.start()
        self.receiver.start()

    def close(self):
        self.running.clear()
        with self.lock:
            self._pending_mechanism = None
            self._mechanism_idle.notify_all()
            if self.link is not None:
                self.link.fail_pending("controller closed")
        if self.worker is not None:
            self.worker.join(timeout=1)
        if self.receiver is not None:
            self.receiver.join(timeout=1)
        with self.lock:
            self._stop_mechanisms_locked()
            self._stop_locked("shutdown")
            if self.link is not None:
                try:
                    self.link.close()
                except (OSError, RuntimeError):
                    logger.exception("chassis serial close failed")
                self.link = None

    def _stop_locked(self, reason: str):
        self.epoch = None
        self.goal_id = None
        self.effort = (0, 0)
        self.last_command = None
        self.last_stop_reason = reason
        if self.link is not None:
            try:
                self.link.stream_control(stm32.AUTO_STOP)
            except (OSError, RuntimeError):
                logger.exception("STM32 stop write failed")
                self._disconnect_locked()

    def stop(self, reason: str = "stop"):
        with self.lock:
            self._stop_locked(reason)

    def _stop_mechanisms_locked(self):
        if self.link is not None:
            try:
                self.link.mechanism_command(mechanism.STOP_ALL, 0)
            except (OSError, RuntimeError):
                logger.exception("STM32 mechanism stop write failed")

    def mechanism_command(self, command: int, device: int):
        """Keep one request pending across transient USB loss; resend its same ID."""
        mechanism.command_payload(command, device)
        if command == mechanism.STOP_ALL:
            with self.lock:
                self._pending_mechanism = None
                self._mechanism_idle.notify_all()
                if self.link is None:
                    raise RuntimeError("STM32 mechanism serial link unavailable")
                self.link.fail_pending("STOP_ALL canceled pending mechanism action")
                return self.link.mechanism_command(command, device)
        moving = command == mechanism.START or (
            command == mechanism.STOP and device in mechanism.BIN_DEVICES.values()
        )
        token = object()
        with self._mechanism_idle:
            while self._pending_mechanism is not None:
                self._mechanism_idle.wait()
            if self.link is None and not self.running.is_set():
                raise RuntimeError("STM32 mechanism serial link unavailable")
            if moving and (self.estop_latched or self.epoch is not None or
                           self.mode == "manual"):
                raise RuntimeError("STM32 mechanism requires stopped chassis, auto mode and no estop")
            if self.link is not None:
                self._serial_session = self.link.session
                sequence = self.link.reserve_sequence()
                self._serial_sequence = sequence
            else:
                if self._serial_session is None:
                    self._serial_session = secrets.randbelow(0xffffffff) + 1
                self._serial_sequence = (self._serial_sequence + 1) & 0xffffffff
                sequence = self._serial_sequence
            self._pending_mechanism = token
        deadline = self.clock() + MECHANISM_RETRY_WINDOW_S
        attempts = 0
        last_error = "no reply"
        try:
            while self.clock() < deadline:
                with self.lock:
                    if self._pending_mechanism is not token or self.estop_latched:
                        raise RuntimeError("STM32 mechanism action canceled")
                    link = self.link
                    if moving and self.epoch is not None:
                        raise RuntimeError("chassis started moving during mechanism action")
                if link is None and not self.running.is_set():
                    raise MechanismDisconnectedError(
                        f"STM32 mechanism link cannot reconnect: {last_error}"
                    )
                if link is None or (moving and link.mode_telemetry.mode() != "auto"
                                    and self.mode != "auto"):
                    time.sleep(MECHANISM_RETRY_DELAY_S)
                    continue
                attempts += 1
                try:
                    link.mechanism_command(command, device, confirm=True,
                                           sequence=sequence)
                except MechanismRejectedError:
                    raise
                except (MechanismTimeoutError, MechanismDisconnectedError, OSError,
                        RuntimeError) as exc:
                    last_error = str(exc)
                    logger.warning("STM32 mechanism retry command=%d device=%d sequence=%d attempt=%d: %s",
                                   command, device, sequence, attempts, exc)
                    if isinstance(exc, (OSError, ChassisError)) and not isinstance(exc, MechanismTimeoutError):
                        with self.lock:
                            if self.link is link:
                                self._disconnect_locked()
                    time.sleep(MECHANISM_RETRY_DELAY_S)
                    continue
                with self.lock:
                    if self._pending_mechanism is not token:
                        raise RuntimeError("STM32 mechanism action canceled")
                    self.last_mechanism_reply = {
                        "command": command, "device": device,
                        "sequence": sequence, "accepted": True,
                        "attempts": attempts,
                    }
                logger.info("STM32 mechanism accepted command=%d device=%d sequence=%d attempts=%d",
                            command, device, sequence, attempts)
                return sequence
            raise MechanismTimeoutError(
                f"STM32 mechanism retry timeout after {MECHANISM_RETRY_WINDOW_S:.1f}s "
                f"command={command} device={device} sequence={sequence} "
                f"attempts={attempts} last_error={last_error}"
            )
        except Exception as exc:
            with self.lock:
                self.last_mechanism_reply = {
                    "command": command, "device": device,
                    "sequence": sequence, "accepted": False, "error": str(exc),
                    "attempts": attempts,
                }
                if self._pending_mechanism is token:
                    self._stop_mechanisms_locked()
            logger.error("STM32 mechanism unconfirmed command=%d device=%d: %s",
                         command, device, exc)
            raise
        finally:
            with self.lock:
                if self._pending_mechanism is token:
                    self._pending_mechanism = None
                    self._mechanism_idle.notify_all()

    def estop(self):
        with self.lock:
            self._pending_mechanism = None
            self._mechanism_idle.notify_all()
            if self.link is not None:
                self.link.fail_pending("emergency stop")
            self._stop_locked("estop")
            self.estop_latched = True
            if self.link is not None:
                try:
                    self.link.stream_control(stm32.ESTOP)
                except (OSError, RuntimeError):
                    self._disconnect_locked()

    def clear_estop(self, *, confirm_status: bool = False, timeout_s: float = 1.5):
        """Send reset once; optionally require a fresh cleared MCU status.

        CLEAR_ESTOP is one-way, so an accepted write is not proof that the MCU
        accepted the reset. The Lenovo UI uses passive STATUS confirmation;
        the legacy 3588 caller keeps its previous non-waiting behavior.
        """
        with self.lock:
            if self.link is None or self.mode != "auto":
                raise RuntimeError("STM32 link and auto mode required for reset")
            link = self.link
            before, received_before = link.mode_telemetry.snapshot()
            self._last_status_received_at = received_before
            if confirm_status:
                self.estop_latched = True
            self._stop_locked("local estop reset")
            if self.link is None:
                raise RuntimeError("STM32 reset write failed")
            try:
                self.link.stream_control(stm32.CLEAR_ESTOP)
            except (OSError, RuntimeError):
                self._disconnect_locked()
                raise
            if not confirm_status:
                self.estop_latched = False
                return
        deadline = time.monotonic() + timeout_s
        last = before
        while time.monotonic() < deadline:
            with self.lock:
                if self.link is not link:
                    raise RuntimeError("STM32 disconnected during emergency-stop reset")
                state, received = link.mode_telemetry.snapshot()
                if state is not None and received > received_before:
                    last = state
                    if before is not None and state["boot"] != before["boot"]:
                        raise RuntimeError("STM32 rebooted during emergency-stop reset")
                    if (not state["flags"] & 2 and not state["faults"]
                            and link.mode_telemetry.mode() == "auto"):
                        self._last_status_received_at = received
                        self.estop_latched = False
                        return
            time.sleep(0.01)
        raise RuntimeError(
            "STM32 emergency-stop reset not confirmed by a fresh safe status: "
            f"flags={last['flags'] if last else None} "
            f"faults={last['faults'] if last else None}"
        )

    def set_obstacle(self, blocked: bool):
        with self.lock:
            self.obstacle = bool(blocked)
            if blocked:
                self._stop_locked("obstacle")

    def arm(self, goal_id: str) -> int:
        with self.lock:
            if not goal_id or self.link is None or self.mode != "auto":
                raise RuntimeError("chassis unavailable or RC not in auto mode")
            if goal_id == self.faulted_goal_id:
                raise RuntimeError("previous goal stopped by STM32 fault; start a new goal")
            if self.status()["faults"]:
                raise RuntimeError("STM32 fault active")
            if self.estop_latched or self.obstacle:
                raise RuntimeError("safety interlock active")
            self._stop_locked("new authorization")
            if self.link is None:
                raise RuntimeError("STM32 stop write failed")
            self.epoch = secrets.randbelow(0xFFFFFFFF) + 1
            self.goal_id = goal_id
            self.last_command = self.clock()
            return self.epoch

    def command(self, epoch: int, vx_mm_s: int, wz_mrad_s: int):
        with self.lock:
            if epoch != self.epoch or self.epoch is None:
                raise RuntimeError("invalid motion epoch")
            if self.link is None or self.mode != "auto" or self.estop_latched or self.obstacle:
                self._stop_locked("safety interlock")
                raise RuntimeError("chassis safety interlock")
            if not -300 <= vx_mm_s <= 300 or not -800 <= wz_mrad_s <= 800:
                self._stop_locked("speed out of range")
                raise ValueError("speed out of range")
            forward = round(vx_mm_s / 300.0 * 1000)
            turn = round(wz_mrad_s / 800.0 * 1000)
            if not all(math.isfinite(v) and -1000 <= v <= 1000 for v in (forward, turn)):
                self._stop_locked("invalid effort")
                raise ValueError("invalid effort")
            self.effort = (forward, turn)
            self.last_command = self.clock()

    def status(self) -> dict:
        with self.lock:
            age = None if self.last_command is None else int((self.clock() - self.last_command) * 1000)
            state, received_at = (self.link.mode_telemetry.snapshot() if self.link is not None
                                  else (None, float("-inf")))
            if state is None or self.clock() - received_at >= 1.0:
                state = None
            return {"rc_mode": self.mode or "unknown", "rc_valid": self.mode in ("auto", "manual"),
                    "rc_age_ms": state["rc_age_ms"] if state else None,
                    "chassis_link": "online" if self.link is not None else "offline",
                    "chassis_serial_online": self.link is not None,
                    "execution_confirmed": False,
                    "chassis_armed": self.epoch is not None, "motion_armed": self.epoch is not None,
                    "motion_epoch": self.epoch or 0, "estop_latched": self.estop_latched,
                    "obstacle_stop": self.obstacle, "obstacle": self.obstacle,
                    "last_motion_age_ms": age, "last_stop_reason": self.last_stop_reason,
                    "last_mechanism_reply": self.last_mechanism_reply,
                    "mechanism_pending": self._pending_mechanism is not None,
                    "faults": state["faults"] if state else None}

    def _disconnect_locked(self):
        link = self.link
        self.link = None
        self.mode = "unknown"
        self.epoch = None
        self.goal_id = None
        self.effort = (0, 0)
        self.last_command = None
        self._mcu_boot = None
        self._last_status_received_at = float("-inf")
        if link is not None:
            self._ever_connected = True
            self._serial_session = link.session
            self._serial_sequence = link.sequence
            link.fail_pending("serial disconnected")
            try:
                link.serial.close()
            except OSError:
                pass

    def _receive_loop(self):
        """The only UART reader; RX cannot delay command writes or vice versa."""
        while self.running.is_set():
            link = self.link
            if link is not None:
                try:
                    link.poll_mode()
                except (OSError, RuntimeError, ValueError, TypeError) as exc:
                    with self.lock:
                        # A read from the old handle may finish after reconnect.
                        if self.link is link:
                            logger.error("chassis serial receive failure: %s", exc)
                            self._stop_locked("serial receive failure")
                            self._disconnect_locked()
            time.sleep(0.01 if link is not None else 0.05)

    def _loop(self):
        while self.running.is_set():
            started = self.clock()
            with self.lock:
                try:
                    if self.link is None:
                        first_connection = not self._ever_connected
                        if self._serial_session is None:
                            self._serial_session = secrets.randbelow(0xffffffff) + 1
                        self.link = ChassisSerial(self.serial_factory(), clock=self.clock,
                                                  session=self._serial_session,
                                                  sequence=self._serial_sequence)
                        self.link.stream_control(stm32.AUTO_STOP)
                        if first_connection and self._pending_mechanism is None:
                            self.link.mechanism_command(mechanism.STOP_ALL, 0)
                        self._ever_connected = True
                    # Read cached reports only; this worker never reads the UART.
                    mode = self.link.mode_telemetry.mode()
                    status, received_at = self.link.mode_telemetry.snapshot()
                    if (status is not None and self.clock() - received_at < 1.0
                            and received_at > self._last_status_received_at):
                        self._last_status_received_at = received_at
                        if self._mcu_boot is not None and status["boot"] != self._mcu_boot:
                            self._stop_locked("STM32 rebooted")
                        self._mcu_boot = status["boot"]
                        if status["faults"] and self.epoch is not None:
                            self.faulted_goal_id = self.goal_id
                            self._stop_locked("STM32 fault")
                        if status["flags"] & 2:
                            self.estop_latched = True
                            if self.epoch is not None:
                                self._stop_locked("STM32 emergency stop")
                    if mode != "auto" and self.epoch is not None:
                        self._stop_locked("RC left auto mode")
                    self.mode = mode if self.link is not None else "unknown"
                    if self.epoch is not None:
                        if self.last_command is None or self.clock() - self.last_command >= self.timeout_s:
                            self._stop_locked("motion timeout")
                        elif self.effort == (0, 0):
                            self.link.stream_control(stm32.AUTO_STOP)
                        else:
                            self.link.stream_effort(*self.effort)
                except (OSError, RuntimeError, ValueError) as exc:
                    logger.error("chassis serial failure: %s", exc)
                    self._disconnect_locked()
            delay = 0.5 if self.link is None else 0.05
            time.sleep(max(0.0, delay - (self.clock() - started)))
