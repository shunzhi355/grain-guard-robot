"""Fail-latched local bench integration using the existing chassis owner.

This does not bypass arm/epoch, RC, obstacle or velocity-timeout checks.
Only injected drivers are accepted here; the CLI supplies in-memory peers.
"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager

from grain_sampling_devices import mechanism_protocol as m
from .mechanisms import SerialMechanisms


class BenchSession:
    def __init__(self, chassis, motion):
        self.chassis = chassis
        self.motion = motion
        self.cancelled = threading.Event()
        self.finished = threading.Event()
        self.reason = None
        self.stop_results = []
        self.commands = []
        self.link = None
        self.boot = None
        self._armed = False
        self._trip_lock = threading.Lock()
        self._lift_lock = threading.Lock()
        self._stopped_link = None
        self.worker = None
        self.motion.cancel_check = self.check
        self.mechanisms = SerialMechanisms(self.send_mechanism, self.check)

    def start(self):
        self.chassis.start()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            with self.chassis.lock:
                if self.chassis.link is not None and self.chassis.mode == "auto":
                    self.link = self.chassis.link
                    status, _ = self.link.mode_telemetry.snapshot()
                    self.boot = status["boot"]
                    break
            time.sleep(0.01)
        else:
            self.trip("initial STM32 auto-mode telemetry unavailable")
            raise RuntimeError(self.reason)
        self.check()
        self.worker = threading.Thread(target=self._watch, name="bench-supervisor", daemon=True)
        self.worker.start()

    def check(self):
        if self.cancelled.is_set():
            raise RuntimeError(f"bench latched: {self.reason}; create a new supervised session")
        with self.chassis.lock:
            link = self.chassis.link
            state, _ = link.mode_telemetry.snapshot() if link else (None, None)
            reason = None
            if link is None or link is not self.link:
                reason = "STM32 link lost/replaced"
            elif state is None or link.mode_telemetry.mode() != "auto":
                reason = "RC manual/unknown/stale"
            elif state["boot"] != self.boot:
                reason = "STM32 rebooted"
            elif self.chassis.estop_latched or state["flags"] & 2:
                reason = "emergency stop"
            elif state["faults"] or self.chassis.obstacle:
                reason = "MCU fault/obstacle"
            elif self._armed and self.chassis.epoch is None:
                reason = "motion authorization revoked/velocity timeout"
        if reason:
            self.trip(reason)
            raise RuntimeError(reason)

    def _attempt(self, name, operation):
        try:
            operation()
        except Exception as exc:
            self.stop_results.append({"operation": name, "result": "unconfirmed", "error": str(exc)})
        else:
            # STM32 stop has no completion ACK; never label it confirmed.
            self.stop_results.append({"operation": name, "result": "verified_off" if name == "x2p_stop" else "submitted"})

    def _stop_stm32(self):
        with self.chassis.lock:
            link = self.chassis.link
            self._stopped_link = link
            if link:
                link.fail_pending("bench canceled")
                self._attempt("stm32_stop_all", lambda: link.mechanism_command(m.STOP_ALL, 0))
            else:
                self.stop_results.append({"operation": "stm32_stop_all", "result": "unconfirmed", "error": "link offline"})
            self.chassis.estop()

    def trip(self, reason):
        with self._trip_lock:
            if self.cancelled.is_set():
                return
            self.reason = reason
            self.cancelled.set()
        self._stop_stm32()
        # Never race a new trigger with a stop on another thread. A running
        # move observes cancel_check and performs its own finally: stop().
        if self._lift_lock.acquire(blocking=False):
            try:
                self._attempt("x2p_stop", self.motion.stop)
            finally:
                self._lift_lock.release()

    def _watch(self):
        while not self.finished.wait(0.02):
            if self.cancelled.is_set():
                # Reopened UART gets only STOP_ALL/ESTOP, never replayed motion.
                if self.chassis.link is not None and self.chassis.link is not self._stopped_link:
                    self._stop_stm32()
                continue
            try:
                self.check()
            except RuntimeError:
                pass

    def send_mechanism(self, command, device):
        m.command_payload(command, device)
        safe_stop = command == m.STOP_ALL or (command == m.STOP and device not in m.BIN_DEVICES.values())
        if not safe_stop:
            self.check()
        try:
            with self.chassis.lock:
                link = self.chassis.link
                if link is None:
                    raise RuntimeError("STM32 link unavailable")
                if not safe_stop and (self.cancelled.is_set() or self.chassis.epoch is not None):
                    raise RuntimeError("start mechanisms only before arming chassis")
                # Reserve/check under state lock; wait for ACK outside it, so
                # the RX worker and stop path cannot deadlock behind a reply.
                sequence = link.reserve_sequence()
            result = link.mechanism_command(command, device, confirm=not safe_stop,
                                            timeout=0.20, sequence=sequence,
                                            write_guard=self._write_guard(link, safe_stop))
            self.commands.append({"command": command, "device": device, "sequence": result,
                                  "accepted": not safe_stop, "execution_confirmed": False})
            if not safe_stop:
                self.check()
            return result
        except Exception as exc:
            self.trip(f"STM32 command failed (not retried): {exc}")
            raise

    @contextmanager
    def _write_guard(self, link, safe_stop):
        with self.chassis.lock:
            if link is not self.chassis.link:
                raise RuntimeError("STM32 handle replaced before write")
            if not safe_stop:
                self.check()
                if self.chassis.epoch is not None:
                    raise RuntimeError("mechanism/chassis start interlock")
            yield

    def arm(self, goal_id):
        self.check()
        with self.chassis.lock:
            epoch = self.chassis.arm(goal_id)
            self._armed = True
            return epoch

    def command(self, epoch, vx_mm_s, wz_mrad_s):
        self.check()
        self.chassis.command(epoch, vx_mm_s, wz_mrad_s)

    def stop_chassis(self):
        with self.chassis.lock:
            self._armed = False
            self.chassis.stop("bench leg complete")

    def move_to(self, target_pulses, *, rpm=30):
        if type(target_pulses) is not int:
            raise ValueError("absolute target must be integer encoder pulses")
        with self._lift_lock:
            try:
                self.check()
                current = self.motion.read_encoder_position()
                delta = target_pulses - current
                if abs(delta) <= 2:
                    return current
                sign = self.motion.config.forward_sign * self.motion.config.encoder_forward_sign
                self.motion.move_pulses("forward" if delta * sign > 0 else "reverse",
                                        abs(delta), rpm, tolerance_pulses=2)
                self.check()
                return self.motion.read_encoder_position()
            except Exception as exc:
                self.trip(f"X2P move failed (no automatic return/replay): {exc}")
                # Even a pre-trigger read can fail. Always attempt all stop
                # steps and keep stop failure distinct from the motion error.
                self._attempt("x2p_stop", self.motion.stop)
                raise

    def close(self):
        self.trip("session closed")
        self.finished.set()
        if self.worker:
            self.worker.join(timeout=2)
        # Drain any in-progress owner before closing its transport.
        with self._lift_lock:
            self.chassis.close()
            self.motion.drive.close()
