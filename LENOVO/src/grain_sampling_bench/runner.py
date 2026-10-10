"""Executable integration scenarios. All transports are injected simulators."""
from __future__ import annotations

import hashlib
import platform
import threading
import time
from contextlib import contextmanager
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices import mechanism_protocol as m
from grain_sampling_interhost.chassis_controller import ChassisController
from x2p.drive import X2PDrive
from x2p.models import ControllerConfig, SafetyLimits
from x2p.motion import MotionController
from x2p.protocol import ModbusRTUClient

from .session import BenchSession
from .simulators import STM32Simulator, X2PSimulator


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("simulated condition did not become true before timeout")


def must_reject(operation):
    try:
        operation()
    except (RuntimeError, OSError, ValueError):
        return
    raise AssertionError("unsafe/failed operation unexpectedly succeeded")


@contextmanager
def simulated_stack(log_path, *, leg_time_s=0.12):
    stm32 = STM32Simulator()
    servo = X2PSimulator(leg_time_s=leg_time_s)
    client = ModbusRTUClient(serial_port=servo, min_request_interval_s=0)
    config = replace(ControllerConfig(), port="SIMULATED", log_path=str(log_path),
                     limits=SafetyLimits(position_timeout_s=0.45, stop_timeout_s=0.1))
    motion = MotionController(X2PDrive(client), config, monitor_interval_s=0.01, output=None)
    chassis = ChassisController(port="SIMULATED", serial_factory=stm32.open)
    session = BenchSession(chassis, motion)
    try:
        session.start()
        yield session, stm32, servo
    finally:
        session.close()


@contextmanager
def heartbeat(session):
    epoch = session.arm("simulated-local-navigation")
    session.command(epoch, 30, 0)
    done = threading.Event()
    def feed():
        while not done.wait(0.04):
            try:
                session.command(epoch, 30, 0)
            except RuntimeError:
                return
    worker = threading.Thread(target=feed, name="simulated-navigation", daemon=True)
    worker.start()
    try:
        yield
    finally:
        done.set()
        worker.join(timeout=1)
        session.stop_chassis()


def mechanisms_case(session, stm32, servo, cycles):
    actions = [(name, {}) for name in ("clamp", "unclamp", "tighten", "untighten")]
    actions.extend((name, {"depth": depth}) for depth in m.BIN_DEVICES
                   for name in ("open_bin", "hold_bin_open", "close_bin"))
    actions.extend((name, {}) for name in ("open_bin_default", "close_all_bins", "start_convey", "stop_convey"))
    actions.append(("convey", {"duration_s": 0.04}))
    for name, args in actions:
        session.mechanisms.execute(name, **args)
    require(m.CONVEY not in stm32.active, "timed convey did not stop")
    reliable = [f for f in stm32.frames if f["kind"] == m.RELIABLE_FRAME_TYPE]
    require(len({f["sequence"] for f in reliable}) == len(reliable), "action sequence was replayed")
    return {"actions": [dict(action=name, **args) for name, args in actions],
            "command_acceptance_only": True}


def reciprocation_case(session, stm32, servo, cycles, *, combined=False):
    origin = session.motion.read_encoder_position()
    target = origin - 131072  # simulated 5 mm at the existing calibration
    positions = []
    def legs():
        for _ in range(cycles):
            for absolute in (target, origin):
                actual = session.move_to(absolute)
                require(abs(actual - absolute) <= 2, "encoder position mismatch")
                positions.append(actual)
    if combined:
        # Preserve the existing interlock: START is sent while chassis stopped.
        # Then allow a running conveyor, base heartbeat and servo to coexist.
        session.mechanisms.execute("start_convey")
        with heartbeat(session):
            wait_for(lambda: stm32.effort != (0, 0))
            require(m.CONVEY in stm32.active, "conveyor not active in combined test")
            legs()
        session.mechanisms.execute("stop_convey")
    else:
        legs()
    require(servo.triggers == cycles * 2, "extra/missing servo motion trigger")
    return {"cycles": cycles, "positions": positions, "origin": origin,
            "combined_base_conveyor_servo": combined}


def timeout_case(session, stm32, servo, cycles):
    must_reject(lambda: session.chassis.command(123, 30, 0))
    epoch = session.arm("timeout-case")
    session.command(epoch, 30, 0)
    sent_at = time.monotonic()
    wait_for(lambda: stm32.effort != (0, 0))
    wait_for(session.cancelled.is_set)
    wait_for(lambda: stm32.effort == (0, 0))
    require(session.chassis.epoch is None, "timeout did not revoke epoch")
    must_reject(lambda: session.command(epoch, 30, 0))
    return {"observed_stop_ms": round((time.monotonic() - sent_at) * 1000, 1),
            "configured_watchdog_ms": 200, "hard_realtime_claim": False}


def stm32_fault_case(session, stm32, servo, cycles, *, fault):
    session.mechanisms.execute("start_convey")
    with heartbeat(session):
        wait_for(lambda: stm32.effort != (0, 0))
        if fault == "manual":
            stm32.mode = 1
        elif fault == "stale":
            stm32.reports = False
        elif fault == "rc_stale":
            stm32.rc_age_ms = 300
        elif fault == "estop":
            stm32.estop = True
        elif fault == "fault":
            stm32.faults = 1
        elif fault == "obstacle":
            session.chassis.set_obstacle(True)
        elif fault == "reboot":
            stm32.boot += 1
        elif fault == "disconnect":
            stm32.disconnect()
        wait_for(session.cancelled.is_set)
        if fault == "disconnect":
            # A fresh handle models re-enumeration, not actual OS/USB behaviour.
            time.sleep(0.08)
            stm32.reconnect()
            wait_for(lambda: session.chassis.link is not None)
            wait_for(lambda: not stm32.active)
        wait_for(lambda: stm32.effort == (0, 0))
    must_reject(lambda: session.arm("cannot-auto-resume"))
    must_reject(lambda: session.mechanisms.execute("clamp"))
    require(session.chassis.epoch is None, "fault left motion authorization active")
    return {"injected_fault": fault, "latched": session.reason, "automatic_resume": False}


def ack_fault_case(session, stm32, servo, cycles, *, fault):
    if fault == "lost_ack":
        stm32.drop_ack = True
    elif fault == "corrupt_ack":
        stm32.corrupt_ack = True
    else:
        stm32.reject = 1
    must_reject(lambda: session.mechanisms.execute("clamp"))
    requests = [f for f in stm32.frames if f["kind"] == m.RELIABLE_FRAME_TYPE]
    require(len(requests) == 1, "uncertain mechanism command was retried")
    require(session.cancelled.is_set(), "unconfirmed mechanism did not latch")
    require(not stm32.active, "STOP_ALL not submitted after failure")
    return {"injected_fault": fault, "request_attempts": len(requests)}


def servo_fault_case(session, stm32, servo, cycles, *, fault):
    servo.trigger_fault = fault
    session.mechanisms.execute("start_convey")
    with heartbeat(session):
        wait_for(lambda: stm32.effort != (0, 0))
        must_reject(lambda: session.move_to(-131072))
        wait_for(lambda: stm32.effort == (0, 0))
    require(session.cancelled.is_set(), "servo failure not latched")
    require(not stm32.active, "servo failure did not stop mechanisms")
    require(servo.triggers == 1, "one-shot trigger replayed after uncertain response")
    require(any(f["address"] == 0x0701 and f["value"] == 0 for f in servo.frames), "no stop attempt")
    if fault == "disconnect":
        require(any(s["result"] == "unconfirmed" for s in session.stop_results), "USB loss falsely declared stopped")
        servo.online = True
        servo.events.append({"event": "simulated_link_restored", "at": time.monotonic()})
    must_reject(lambda: session.move_to(0))
    require(servo.triggers == 1, "automatic return after failed leg")
    return {"injected_fault": fault, "triggers": servo.triggers, "automatic_return": False}


def run_suite(*, cycles, log_path, scenario="all"):
    if type(cycles) is not int or not 1 <= cycles <= 1000:
        raise ValueError("cycles must be in 1..1000")
    cases = {
        "stm32_actions": mechanisms_case,
        "x2p_reciprocation": reciprocation_case,
        "combined": lambda *args: reciprocation_case(*args, combined=True),
        "navigation_timeout": timeout_case,
    }
    for fault in ("manual", "stale", "rc_stale", "estop", "fault", "obstacle", "reboot", "disconnect"):
        cases[f"stm32_{fault}"] = lambda *args, fault=fault: stm32_fault_case(*args, fault=fault)
    for fault in ("lost_ack", "corrupt_ack", "rejected"):
        cases[f"stm32_{fault}"] = lambda *args, fault=fault: ack_fault_case(*args, fault=fault)
    for fault in ("drop_ack", "disconnect", "frozen_encoder"):
        cases[f"x2p_{fault}"] = lambda *args, fault=fault: servo_fault_case(*args, fault=fault)
    if scenario != "all" and scenario not in cases:
        raise ValueError(f"unknown scenario {scenario}; choose: all, {', '.join(cases)}")
    results = []
    for name, case in cases.items():
        if scenario not in ("all", name):
            continue
        record = {"name": name}
        started = time.monotonic()
        session = stm32 = servo = None
        try:
            with simulated_stack(log_path) as (session, stm32, servo):
                record["details"] = case(session, stm32, servo, cycles)
            record["passed"] = True
        except Exception as exc:
            record.update(passed=False, error=f"{type(exc).__name__}: {exc}")
        record["elapsed_s"] = round(time.monotonic() - started, 3)
        if session is not None:
            record.update(stm32_frames=stm32.frames, x2p_frames=servo.frames,
                          serial_events=stm32.events + servo.events, stop_results=session.stop_results,
                          mechanism_commands=session.commands)
        results.append(record)
    root = Path(__file__).resolve().parents[3]
    paths = ["3588/src/grain_sampling_devices/chassis_serial.py",
             "3588/src/grain_sampling_interhost/chassis_controller.py",
             "3588/src/grain_sampling_interhost/mechanism_controller.py",
             "3588/src/x2p/motion.py", "3588/src/x2p/protocol.py"]
    paths += [str(path.relative_to(root)).replace("\\", "/")
              for path in sorted(Path(__file__).parent.glob("*.py"))]
    return {
        "schema_version": 1, "mode": "simulation", "physical_hardware_verified": False,
        "decision": "PENDING_HARDWARE_VALIDATION", "lenovo_target": "192.168.1.200 (not contacted)",
        "host_platform": platform.platform(), "python": platform.python_version(),
        "created_utc": datetime.now(timezone.utc).isoformat(), "cycles": cycles,
        "passed": all(item["passed"] for item in results),
        "limitations": ["No USB/RS485 electrical reliability or OS re-enumeration validation",
                        "No firmware emulation or mechanical end-position verification",
                        "untighten preserves STOP device 1; reverse action unverified",
                        "start_suction/stop_suction have no STM32 frame in current flow",
                        "Timing accelerated; not a real-time/EMI/endurance measurement",
                        "No UI/Nav2 deployment or actual Lenovo-host test"],
        "source_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in paths},
        "scenarios": results,
    }
