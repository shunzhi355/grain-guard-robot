"""Non-ROS, single-owner 3588 to STM32 chassis control.

All serial access uses one lock, including local mechanism requests. Loss of
fresh velocity commands revokes chassis motion locally.
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
from grain_sampling_devices.chassis_serial import ChassisSerial, DEFAULT_SERIAL_PORT

logger = logging.getLogger(__name__)
TELEMETRY_RECONNECT_TIMEOUT_S = 2.0


class ChassisController:
    def __init__(self, port: str | None = None, *, serial_factory=None,
                 clock=time.monotonic, timeout_s: float = 0.2):
        self.port = port or os.getenv("CHASSIS_SERIAL_PORT", DEFAULT_SERIAL_PORT)
        self.clock = clock
        self.timeout_s = timeout_s
        self.serial_factory = serial_factory or self._open_serial
        self.lock = threading.RLock()
        self.running = threading.Event()
        self.worker: threading.Thread | None = None
        self.link: ChassisSerial | None = None
        self.mode = "unknown"
        self.epoch: int | None = None
        self.goal_id: str | None = None
        self.faulted_goal_id: str | None = None
        self.last_command: float | None = None
        self.effort = (0, 0)
        self.estop_latched = False
        self._reset_requested_at: float | None = None
        self._reset_event = threading.Event()
        self.obstacle = False
        self.last_stop_reason = "startup"
        self._mcu_boot: int | None = None
        self._link_opened_at: float | None = None

    def _open_serial(self):
        import serial
        return serial.Serial(self.port, 115200, timeout=0.01,
                             write_timeout=0.05, exclusive=True)

    def start(self):
        if self.running.is_set():
            return
        self.running.set()
        self.worker = threading.Thread(target=self._loop, daemon=True, name="chassis-serial")
        self.worker.start()

    def close(self):
        self.running.clear()
        if self.worker is not None:
            self.worker.join(timeout=1)
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
        """Immediate, serialized send; no queued actions survive a reconnect."""
        mechanism.command_payload(command, device)
        with self.lock:
            if self.link is None:
                raise RuntimeError("STM32 mechanism serial link unavailable")
            # STOP on a bin is a powered closing move, not a neutral stop.
            moving = command == mechanism.START or (
                command == mechanism.STOP and device in mechanism.BIN_DEVICES.values()
            )
            if moving and (self.estop_latched or self.epoch is not None or self.mode != "auto"):
                raise RuntimeError("STM32 mechanism requires stopped chassis, auto mode and no estop")
            try:
                return self.link.mechanism_command(command, device)
            except (OSError, RuntimeError):
                self._stop_mechanisms_locked()
                self._disconnect_locked()
                raise

    def estop(self):
        with self.lock:
            self._stop_locked("estop")
            self.estop_latched = True
            self._reset_requested_at = None
            if self.link is not None:
                try:
                    self.link.stream_control(stm32.ESTOP)
                except (OSError, RuntimeError):
                    self._disconnect_locked()

    def clear_estop(self):
        with self.lock:
            if self.link is None or self.mode != "auto":
                raise RuntimeError("STM32 link and auto mode required for reset")
            self._reset_event.clear()
            self._reset_requested_at = self.clock()
            self.link.stream_control(stm32.CLEAR_ESTOP)
        if not self._reset_event.wait(1.0):
            with self.lock:
                self._reset_requested_at = None
            raise RuntimeError("STM32 did not confirm emergency stop reset")
        with self.lock:
            self._stop_locked("local estop reset")

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
            telemetry = getattr(self.link, "mode_telemetry", None)
            state = getattr(telemetry, "status", None)
            received_at = getattr(telemetry, "received_at", float("-inf"))
            if state is None or self.clock() - received_at >= 1.0:
                state = None
            return {"rc_mode": self.mode or "unknown", "rc_valid": self.mode in ("auto", "manual"),
                    "rc_age_ms": state["rc_age_ms"] if state else None,
                    "chassis_link": "online" if self.link is not None else "offline",
                    "chassis_serial_online": self.link is not None,
                    "chassis_armed": self.epoch is not None, "motion_armed": self.epoch is not None,
                    "motion_epoch": self.epoch or 0, "estop_latched": self.estop_latched,
                    "obstacle_stop": self.obstacle, "obstacle": self.obstacle,
                    "last_motion_age_ms": age, "last_stop_reason": self.last_stop_reason,
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
        self._link_opened_at = None
        if link is not None:
            try:
                link.serial.close()
            except OSError:
                pass

    def _loop(self):
        while self.running.is_set():
            started = self.clock()
            with self.lock:
                try:
                    if self.link is None:
                        self.link = ChassisSerial(self.serial_factory())
                        self._link_opened_at = self.clock()
                        self.link.stream_control(stm32.AUTO_STOP)
                        # Cancel MCU timers/pending starts from the previous
                        # connection. Never replay a mechanism START.
                        self.link.mechanism_command(mechanism.STOP_ALL, 0)
                    elif self._link_opened_at is None:
                        self._link_opened_at = self.clock()
                    mode = self.link.poll_mode()
                    telemetry = getattr(self.link, "mode_telemetry", None)
                    status = getattr(telemetry, "status", None)
                    received_at = getattr(telemetry, "received_at", float("-inf"))
                    # A USB serial adapter can disappear and re-enumerate
                    # without the old file descriptor immediately raising an
                    # I/O error. Never leave that stale descriptor "online".
                    last_fresh = max(received_at, self._link_opened_at)
                    if self.clock() - last_fresh >= TELEMETRY_RECONNECT_TIMEOUT_S:
                        self._stop_locked("STM32 telemetry stale")
                        raise RuntimeError("STM32 telemetry stale; reopening serial")
                    if status is not None and self.clock() - received_at < 1.0:
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
                        elif (self._reset_requested_at is not None
                              and received_at > self._reset_requested_at):
                            self.estop_latched = False
                            self._reset_requested_at = None
                            self._reset_event.set()
                    if mode != "auto" and self.epoch is not None:
                        self._stop_locked("RC left auto mode")
                    self.mode = mode
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
