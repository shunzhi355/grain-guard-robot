"""Drive the ORIGINAL FSM and orchestrator through a full simulated task.

Only time waits, operator confirmations and sensor/hardware peers are simulated.
No production flag makes the operator prompts automatically acknowledge.
"""
from __future__ import annotations

import threading
import time
from collections import deque

from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.robot_bridge import RobotBridge
from grain_sampling_workflow.state_machine import SamplingAction as A, SamplingState as S, SamplingStateMachine


class SimulationClient:
    def __init__(self, service):
        self.service = service

    def request(self, action, **values):
        if action == "status":
            values["client_role"] = "ui"
        reply = self.service.local_request({"action": action, **values})
        if action == "test_goal":
            self.service.local_request({"action": "test_arrive", "goal_id": reply["goal_id"]})
        return reply


class SimulationStopEvent(threading.Event):
    def wait(self, timeout=None):
        return super().wait(None if timeout is None else min(timeout, 0.001))


class OfflineCloud:
    base_url = "https://xxx.invalid"
    mac_address = "SIMULATED"
    def post(self, *args, **kwargs):
        raise AssertionError("simulation must never send cloud reports")


def run_workflow(config, factory):
    service, (stm32, servo) = factory(config, simulate=True)
    states, confirmations, queue = [], [], deque()
    error = None
    passed = False
    done = threading.Event()
    feeder = None
    try:
        service.start()
        def pulse():
            while not done.wait(0.1):
                service.local_request({"action": "status", "client_role": "ui"})
        feeder = threading.Thread(target=pulse, daemon=True)
        feeder.start()
        service.mechanism.controller._stop_flag = SimulationStopEvent()
        original_unclamp_wait = service.mechanism._wait_for_unclamp
        def accelerated_unclamp_wait():
            service.mechanism._unclamp_finish_at = time.monotonic()
            original_unclamp_wait()  # still enforces prior unclamp/cancel state
        service.mechanism._wait_for_unclamp = accelerated_unclamp_wait
        fsm = SamplingStateMachine(total_waypoints=2, max_depth=3)
        fsm.set_depth_targets([1.5, 2.5, 3.5])
        bridge = RobotBridge(client=SimulationClient(service))
        orchestrator = WorkflowOrchestrator(fsm, bridge, [(0.0, 0.0), (1.0, 0.0)], cloud_client=OfflineCloud())
        orchestrator.enable_mechanism()
        orchestrator._run_async = queue.append
        def wait(duration, poll_sec=0.25):
            service.check_stationary()
            service.event("simulated_wait", configured_seconds=duration)
            return fsm.is_running
        orchestrator._wait_interruptible = wait
        orchestrator._log_sampling_event = lambda action, detail: service.event("workflow", action=action, detail=detail)
        orchestrator.set_callback(lambda previous, current, action: states.append(current.name))
        if not orchestrator.set_grain("稻谷"):
            raise RuntimeError(bridge.last_error)
        prompt_actions = {S.INIT: A.CONFIRM_READY, S.ARRIVED_PROMPT: A.CONFIRM_READY,
                          S.ADD_PIPE_PROMPT: A.CONFIRM_PIPE_ADDED, S.DISCHARGE_WASTE: A.CONFIRM_WASTE_DISCHARGED,
                          S.CONVEY_DONE: A.CONFIRM_DONE, S.PIPE_SUPPORT_PROMPT: A.CONFIRM_PIPE_SUPPORTED,
                          S.REMOVE_PIPE_PROMPT: A.CONFIRM_PIPE_REMOVED, S.ALL_DONE_PROMPT: A.CONFIRM_RETURN}
        for _ in range(300):
            if fsm.current_state in (S.COMPLETED, S.STOPPED):
                break
            if queue:
                queue.popleft()()
            elif fsm.current_state in prompt_actions:
                action = prompt_actions[fsm.current_state]
                confirmations.append(action.name)
                fsm.transition(action)
            else:
                raise AssertionError(f"workflow stuck in {fsm.current_state.name}")
        if fsm.current_state != S.COMPLETED:
            raise AssertionError(f"workflow ended in {fsm.current_state.name}: {getattr(fsm, 'stop_reason', '')}")
        if service.mechanism.requires_mechanical_reset() or stm32.active:
            # The simple MCU model keeps timed gripper/tighten states active;
            # only conveyor cessation and origin handling are asserted here.
            if service.mechanism.requires_mechanical_reset() or 2 in stm32.active:
                raise AssertionError("workflow ended with unfinished lift/conveyor")
        passed = True
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        done.set()
        if feeder:
            feeder.join(timeout=1)
        service.close()
    return {"mode": "simulation", "passed": passed, "error": error,
            "physical_hardware_verified": False, "autonomous_navigation_verified": False,
            "full_original_workflow_used": True, "waypoints": 2, "depths_per_waypoint": 3,
            "states": states, "simulated_operator_confirmations": confirmations,
            "timing_accelerated": True, "external_suction_not_automated": True,
            "servo_triggers": servo.triggers, "events": service.events,
            "stm32_frames": stm32.frames, "x2p_frames": servo.frames}
