"""Local-only robot service. No TCP/UDP GRICP listener, TLS, SSH or remote host."""
from __future__ import annotations

import json
import logging
import math
import secrets
import threading
import time
import uuid
from contextlib import contextmanager

from grain_sampling_devices import mechanism_protocol as m
from grain_sampling_interhost.server import RobotServer
from .mechanism import SerialWorkflowRuntime


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class LocalRobotService:
    # Reuse only the permission-restricted AF_UNIX transport, not RobotServer
    # construction, serve(), TLS, peer session or any interhost request routing.
    _ipc_loop = RobotServer._ipc_loop

    def __init__(self, config, chassis, lift, *, simulation=False):
        self.config = config
        self.chassis = chassis
        self.ipc_path = str(config.socket_path)
        self.simulation = simulation
        self.lock = threading.RLock()
        self.running = threading.Event()
        self.latched = False
        self.last_error = ""
        self.events = []
        self._event_lock = threading.Lock()
        self._mechanism_gate = threading.Lock()
        self._baseline_link = None
        self._baseline_boot = None
        self._stop_link = None
        self._task_active = False
        self._ui_at = 0.0
        self._input_at = 0.0
        self._inputs = None
        self._nav_token = None
        self._nav_sequence = 0
        self._goal = None
        self.navigation = None
        self._zero_frames = 0
        self._epoch = None
        self.worker = None
        self.mechanism = SerialWorkflowRuntime(
            lift, rpm=config.lift_rpm, serial_command=self.send_mechanism,
            check=self.check_stationary, state_path=config.directory / "lift-recovery.json",
            suction_policy=config.suction_policy, event=self.event)
        self.latched = self.mechanism.estop_latched
        if self.latched:
            self.last_error = "incomplete lift operation from previous run; mechanical reset required"

    def event(self, kind, **values):
        record = {"time": time.time(), "monotonic": time.monotonic(), "kind": kind, **values}
        with self._event_lock:
            self.events.append(record)
            try:
                self.config.directory.mkdir(parents=True, exist_ok=True)
                with (self.config.directory / "events.jsonl").open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            except (OSError, ValueError) as exc:
                # Full disk / logging failure must never abort a safety stop.
                record["log_error"] = str(exc)
                logging.getLogger(__name__).error("Local event logging failed: %s", exc)

    def start(self):
        self.config.directory.mkdir(parents=True, exist_ok=True)
        self.chassis.start()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            with self.chassis.lock:
                link = self.chassis.link
                state, received = link.mode_telemetry.snapshot() if link else (None, 0)
                if state is not None and time.monotonic() - received < 1:
                    self._baseline_link, self._baseline_boot = link, state["boot"]
                    break
            time.sleep(0.02)
        else:
            raise RuntimeError("STM32 status not received; startup aborted")
        self.mechanism.start()
        self.running.set()
        self.worker = threading.Thread(target=self._supervise, daemon=True, name="local-hardware-safety")
        self.worker.start()
        self.event("service_started", simulation=self.simulation, navigation_mode=self.config.navigation_mode,
                   hardware_backend="stm32+x2p", pca9685=False)

    def check(self):
        if self.latched or self.mechanism.estop_latched:
            raise RuntimeError(self.last_error or "local hardware safety latch active")
        with self.chassis.lock:
            link = self.chassis.link
            state, _ = link.mode_telemetry.snapshot() if link else (None, 0)
            if link is not self._baseline_link or not link:
                raise RuntimeError("STM32 disconnected/re-enumerated; no automatic action replay")
            if not state or state["boot"] != self._baseline_boot:
                raise RuntimeError("STM32 rebooted")
            if link.mode_telemetry.mode() != "auto":
                raise RuntimeError("fresh RC automatic mode required")
            if self.chassis.estop_latched or state["flags"] & 2 or state["faults"]:
                raise RuntimeError("STM32 emergency stop/fault")

    def check_stationary(self):
        self.check()
        if self.chassis.epoch is not None:
            raise RuntimeError("chassis must be disarmed before mechanism operation")

    @contextmanager
    def _write_guard(self, link):
        with self.chassis.lock:
            self.check_stationary()
            if link is not self.chassis.link:
                raise RuntimeError("STM32 handle changed before write")
            yield

    def send_mechanism(self, command, device):
        with self.chassis.lock:
            link = self.chassis.link
        if command == m.STOP_ALL:
            if link is None:
                self.event("stop_unconfirmed", device="stm32", error="offline")
                return
            link.fail_pending("local STOP_ALL")
            try:
                link.mechanism_command(command, device)
                self.event("stop_submitted", device="stm32", execution_confirmed=False)
            except Exception as exc:
                self.event("stop_unconfirmed", device="stm32", error=str(exc))
            return
        self.check_stationary()
        # One attempt only. An ACK timeout is not permission to move twice.
        try:
            return link.mechanism_command(command, device, confirm=True,
                                          write_guard=self._write_guard(link))
        except Exception as exc:
            self.trip(f"STM32 action unconfirmed: {exc}")
            raise

    def trip(self, reason):
        with self.lock:
            first = not self.latched
            self.latched = True
            self.last_error = reason
            if self._goal:
                self.navigation = {"goal_id": self._goal["goal_id"], "result": "FAILED", "message": reason}
            self._goal = None
            self._epoch = None
            self._task_active = False
        if first:
            self.event("safety_latched", reason=reason)
        self.chassis.estop()
        self._stop_link = self.chassis.link
        try:
            self.mechanism.emergency_stop()
        except Exception as exc:
            self.event("stop_unconfirmed", device="x2p", error=str(exc))

    def _supervise(self):
        while self.running.is_set():
            try:
                if self.latched:
                    if self.chassis.link is not None and self.chassis.link is not self._stop_link:
                        self.chassis.estop()
                        self.send_mechanism(m.STOP_ALL, 0)
                        self._stop_link = self.chassis.link
                elif self._task_active or self._goal is not None:
                    self.check()
                    if time.monotonic() - self._ui_at > 1.5:
                        raise RuntimeError("workflow/UI heartbeat lost")
                    if self._epoch is not None:
                        self._check_navigation_inputs()
                        if self.chassis.epoch != self._epoch:
                            raise RuntimeError("local navigation velocity timeout/revoked authorization")
            except Exception as exc:
                self.trip(str(exc))
            time.sleep(0.03)

    def _check_navigation_inputs(self):
        value = self._inputs
        now = time.monotonic()
        if value is None or now - self._input_at > 0.3:
            raise RuntimeError("local navigation sensor feed stale")
        if not 0 <= now - value["odom_at"] <= 0.3:
            raise RuntimeError("odometry stale")
        if not 0 <= now - value["cloud_at"] <= 0.5 or value["blocked"]:
            raise RuntimeError("obstacle/cloud unavailable")
        if value["localization_valid"] is not True or not 0 <= now - value["localization_at"] <= 0.5:
            raise RuntimeError("localization quality not confirmed/fresh")
        if not self.config.map_id:
            raise RuntimeError("map_id must be configured for autonomous navigation")

    def _nav_request(self, request):
        if self.config.navigation_mode != "local":
            raise RuntimeError("autonomous chassis navigation is disabled in operator mode")
        kind = request["action"]
        if kind == "nav_register":
            if self._goal is not None:
                raise RuntimeError("cannot replace navigation worker during a goal")
            if self._nav_token and time.monotonic() - self._input_at < 0.5:
                raise RuntimeError("a local navigation worker is already active")
            self._nav_token = secrets.token_hex(24)
            self._nav_sequence = 0
            self._inputs = None
            return {"ok": True, "token": self._nav_token}
        if not self._nav_token or request.get("token") != self._nav_token:
            raise RuntimeError("invalid local navigation worker token")
        sequence = request.get("sequence")
        if type(sequence) is not int or sequence <= self._nav_sequence:
            raise ValueError("replayed local navigation update")
        self._nav_sequence = sequence
        if kind == "nav_inputs":
            value = request.get("inputs")
            if (not isinstance(value, dict)
                    or not all(finite(value.get(k)) for k in ("x_m", "y_m", "yaw_rad", "odom_at", "cloud_at", "localization_at"))
                    or type(value.get("blocked")) is not bool
                    or type(value.get("localization_valid")) is not bool
                    or value.get("frame_id") != self.config.map_frame):
                raise ValueError("invalid local sensor inputs/frame")
            now = time.monotonic()
            if any(value[k] > now + 0.02 for k in ("odom_at", "cloud_at", "localization_at")):
                raise ValueError("future sensor timestamp")
            self._inputs = value
            self._input_at = now
            return {"ok": True, "goal": self._goal, "latched": self.latched}
        if kind == "nav_velocity":
            if not self._goal or request.get("goal_id") != self._goal["goal_id"] or self._epoch is None:
                raise RuntimeError("no matching authorized local navigation goal")
            self.check()
            self._check_navigation_inputs()
            stamp = request.get("created_at")
            if not finite(stamp) or not 0 <= time.monotonic() - stamp <= 0.15:
                raise ValueError("expired velocity command")
            forward, turn = request.get("forward"), request.get("turn")
            if not all(finite(v) and -1 <= v <= 1 for v in (forward, turn)):
                raise ValueError("normalized effort must be finite and in [-1,1]")
            self.chassis.command(self._epoch, round(forward * 300), round(turn * 800))
            self._zero_frames = self._zero_frames + 1 if forward == turn == 0 else 0
            if request.get("arrived") is True:
                pose, goal = self._inputs, self._goal["pose"]
                error = math.hypot(pose["x_m"] - goal["x_m"], pose["y_m"] - goal["y_m"])
                yaw_error = abs(math.atan2(math.sin(pose["yaw_rad"] - goal["yaw_rad"]), math.cos(pose["yaw_rad"] - goal["yaw_rad"])))
                if self._zero_frames < 3 or error > 0.2 or yaw_error > 0.1745:
                    raise RuntimeError("arrival requires three zero frames and fresh in-tolerance pose")
                self.chassis.stop("local navigation arrival")
                self.navigation = {"goal_id": self._goal["goal_id"], "result": "SUCCEEDED"}
                self._goal, self._epoch = None, None
            return {"ok": True}
        raise ValueError("unknown local navigation request")

    def local_request(self, request):
        kind = request.get("action")
        if kind == "status":
            if request.get("client_role") == "ui":
                self._ui_at = time.monotonic()
            inputs = self._inputs if time.monotonic() - self._input_at < 0.3 else None
            return {"ok": True, "runtime": "lenovo-local", "simulation": self.simulation,
                    "chassis": self.chassis.status(), "navigation": self.navigation,
                    "lenovo_online": self.config.navigation_mode == "local" and inputs is not None,
                    "test_navigation_mode": self.config.navigation_mode == "operator",
                    "pose": inputs, "slam": {"map_id": self.config.map_id, "mode": "LOCALIZING" if inputs and inputs["localization_valid"] else "UNKNOWN"},
                    "safety_latched": self.latched, "last_error": self.last_error,
                    "external_suction": self.config.suction_policy == "external",
                    "requires_mechanical_reset": self.mechanism.requires_mechanical_reset()}
        if kind == "estop":
            self.trip("operator emergency stop")
            return {"ok": True}
        if kind == "clear_estop":
            if request.get("mechanical_reset_confirmed") is not True:
                raise RuntimeError("explicit operator mechanical inspection/reset confirmation required")
            if not self._mechanism_gate.acquire(blocking=False):
                raise RuntimeError("wait for active mechanism worker to stop before resetting")
            try:
                state = self.chassis.status()
                if state["rc_mode"] != "auto" or state["faults"] != 0 or state["motion_armed"]:
                    raise RuntimeError("reset requires online fresh auto RC and no faults/motion")
                self.mechanism.controller.lift_drive.stop()
                self.chassis.clear_estop()
                self.mechanism.reset(self.mechanism.grain, mechanical_reset_confirmed=True)
                with self.chassis.lock:
                    self._baseline_link = self.chassis.link
                    state, _ = self._baseline_link.mode_telemetry.snapshot()
                    self._baseline_boot = state["boot"]
                self._task_active = False
                self.latched, self.last_error = False, ""
                self.event("operator_reset", resumed=False)
                return {"ok": True}
            finally:
                self._mechanism_gate.release()
        if kind == "cancel":
            with self.lock:
                self.chassis.stop("local goal canceled")
                if self._goal:
                    self.navigation = {"goal_id": self._goal["goal_id"], "result": "CANCELED"}
                self._goal, self._epoch = None, None
            return {"ok": True}
        if kind == "mechanism":
            with self.lock:
                if not self._mechanism_gate.acquire(blocking=False):
                    raise RuntimeError("another mechanism action is active")
            try:
                self.check_stationary()
                if self._goal is not None:
                    raise RuntimeError("finish/cancel navigation before mechanisms")
                if request.get("name") == "set_grain":
                    if self.config.suction_policy != "external":
                        raise RuntimeError("full workflow needs confirmed external suction handling")
                    self.mechanism.set_grain(str(request.get("grain", "")))
                    self._task_active = True
                    self._ui_at = time.monotonic()
                else:
                    if not self._task_active or time.monotonic() - self._ui_at > 1.5:
                        raise RuntimeError("start a task and keep the local UI/workflow heartbeat running")
                    self.mechanism.execute(request.get("name"), **request.get("args", {}))
                return {"ok": True}
            except Exception as exc:
                if self._task_active:
                    self.trip(str(exc))
                raise
            finally:
                self._mechanism_gate.release()
        if kind in ("test_goal", "test_arrive", "goal"):
            with self.lock:
                self.check_stationary()
                if self._mechanism_gate.locked() or self.mechanism.requires_mechanical_reset():
                    raise RuntimeError("mechanism active/incomplete lift return; navigation forbidden")
                if kind == "test_arrive":
                    if self.config.navigation_mode != "operator" or not self._goal or request.get("goal_id") != self._goal["goal_id"]:
                        raise RuntimeError("no matching operator-confirmed goal")
                    self.navigation = {"goal_id": self._goal["goal_id"], "result": "SUCCEEDED", "test_only": True}
                    self._goal = None
                    return {"ok": True}
                if self._goal:
                    raise RuntimeError("cancel previous goal first")
                operator = self.config.navigation_mode == "operator"
                if (kind == "test_goal") != operator:
                    raise RuntimeError("navigation mode mismatch")
                pose = request.get("pose") if operator else (request.get("goal") or {}).get("pose")
                if not isinstance(pose, dict) or not all(finite(pose.get(k)) for k in ("x_m", "y_m")):
                    raise ValueError("finite goal coordinates required")
                if not operator:
                    goal = request["goal"]
                    if goal.get("map_id") != self.config.map_id or goal.get("frame_id") != "map" or not finite(pose.get("yaw_rad")):
                        raise ValueError("goal map/frame/yaw mismatch")
                    self._check_navigation_inputs()
                goal_id = str(uuid.uuid4())
                self._goal = {"goal_id": goal_id, "pose": pose}
                self.navigation = dict(self._goal, state="AWAITING_OPERATOR" if operator else "RUNNING", test_only=operator)
                self._ui_at = time.monotonic()
                if not operator:
                    self.chassis.set_obstacle(False)
                    try:
                        self._epoch = self.chassis.arm(goal_id)
                    except Exception:
                        self._goal = None
                        self.navigation = {"goal_id": goal_id, "result": "FAILED"}
                        raise
                    self._zero_frames = 0
                self.event("navigation_goal", operator_confirmed=operator, goal_id=goal_id, pose=pose)
                return {"ok": True, "goal_id": goal_id}
        if isinstance(kind, str) and kind.startswith("nav_"):
            with self.lock:
                try:
                    return self._nav_request(request)
                except Exception as exc:
                    if self._epoch is not None:
                        self.trip(f"local navigation rejected: {exc}")
                    raise
        if kind in ("slam", "map_list"):
            raise RuntimeError("use Lenovo local SLAM bridge; no remote map service")
        raise ValueError("unknown local action")

    def serve(self):
        try:
            self.start()
            self._ipc_loop()
        finally:
            self.close()

    def close(self):
        self.running.clear()
        self.trip("service shutdown")
        if self.worker:
            self.worker.join(timeout=2)
        try:
            self.mechanism.close()
        finally:
            self.chassis.close()
