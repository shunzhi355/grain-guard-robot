"""Removal must finish, with operator confirmations, before leaving a point."""
from unittest.mock import MagicMock, call

import pytest

from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.state_machine import SamplingAction as A, SamplingState as S, SamplingStateMachine
from grain_sampling_devices.mechanism_driver import MechanismController
from grain_sampling_interhost.mechanism_controller import MechanismRuntime
from grain_sampling_workflow.robot_bridge import RobotBridge


def make_flow(pipes=2, points=2, depths=1, hardware=True):
    fsm = SamplingStateMachine(total_waypoints=points, max_depth=depths)
    fsm._state = S.NEXT_CHECK
    fsm.current_pipe_index = pipes
    bridge = MagicMock()
    bridge.last_error = "test failure"
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.set_mechanism_connected(hardware)
    orch._mechanism_retry_interval = 0
    orch._wait_interruptible = lambda seconds: fsm.is_running
    orch._log_sampling_event = lambda *args: None
    orch._run_async = lambda fn: fn() if fn.__name__ in (
        "_handle_next_check", "_handle_extract_pipe", "_handle_release_pipe"
    ) else None
    return orch, fsm, bridge


@pytest.mark.parametrize("points,hardware", [(1, True), (2, True), (1, False)])
def test_every_pipe_removed_before_next_point_or_return(points, hardware):
    orch, fsm, bridge = make_flow(points=points, hardware=hardware)
    orch._handle_next_check()
    for remaining in (2, 1):
        assert fsm.current_state == S.PIPE_SUPPORT_PROMPT
        assert fsm.current_pipe_index == remaining
        bridge.call_navigate.assert_not_called()
        # No release or unscrewing until the operator supports the raised pipe.
        fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
        assert fsm.current_state == S.REMOVE_PIPE_PROMPT
        assert fsm.current_pipe_index == remaining
        fsm.transition(A.CONFIRM_PIPE_REMOVED)
    assert fsm.current_pipe_index == 0
    assert fsm.current_state == (S.ALL_DONE_PROMPT if points == 1 else S.NAVIGATE_TO_POINT)
    if points == 2:
        assert fsm.current_waypoint_index == 1
        assert fsm.current_depth_index == 0
    if hardware:
        assert bridge.call_move_lift.call_args_list == [
            call("extract_prepare", 20), call("extract", 20),
            call("extract_prepare", 20), call("extract", 20),
        ]
        bridge.call_untighten.assert_called_once()  # Bottom pipe has no added joint.
        assert bridge.call_unclamp.call_count == 3  # Initial release + one per removed pipe.
    else:
        bridge.call_move_lift.assert_not_called()


def test_exact_removal_order_and_operator_waits():
    orch, fsm, bridge = make_flow()
    orch._handle_next_check()
    assert bridge.method_calls == [
        call.call_lift_health(), call.call_unclamp(),
        call.call_move_lift("extract_prepare", 20), call.call_clamp(),
        call.call_move_lift("extract", 20),
    ]
    assert orch._pipe_clamped
    with pytest.raises(ValueError):
        fsm.transition(A.CONFIRM_PIPE_REMOVED)
    fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
    assert bridge.method_calls[-2:] == [call.call_untighten(), call.call_unclamp()]
    assert not orch._pipe_clamped
    assert fsm.current_state == S.REMOVE_PIPE_PROMPT
    assert fsm.current_pipe_index == 2


@pytest.mark.parametrize("pipes", [2, 3, 6])
def test_confirmed_removal_lowers_next_pipe_without_repeating_release(pipes):
    orch, fsm, bridge = make_flow(pipes=pipes)
    orch._handle_next_check()
    for remaining in range(pipes, 0, -1):
        assert fsm.current_state == S.PIPE_SUPPORT_PROMPT
        fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
        assert fsm.current_state == S.REMOVE_PIPE_PROMPT
        assert orch._pipe_clamped is False
        bridge.reset_mock()
        fsm.transition(A.CONFIRM_PIPE_REMOVED)
        if remaining > 1:
            assert bridge.method_calls == [
                call.call_lift_health(), call.call_move_lift("extract_prepare", 20),
                call.call_clamp(), call.call_move_lift("extract", 20),
            ]
            bridge.call_unclamp.assert_not_called()
            assert fsm.current_state == S.PIPE_SUPPORT_PROMPT
        else:
            assert fsm.current_state == S.NAVIGATE_TO_POINT
            assert bridge.method_calls == []


@pytest.mark.parametrize("grip", [None, True, False])
def test_extraction_skips_initial_unclamp_only_when_release_is_known(grip):
    orch, fsm, bridge = make_flow()
    orch._pipe_clamped = grip
    orch._handle_next_check()
    assert bridge.call_unclamp.call_count == (0 if grip is False else 1)
    assert fsm.current_state == S.PIPE_SUPPORT_PROMPT
    assert orch._pipe_clamped is True


def test_interrupted_unclamp_does_not_mark_grip_released():
    orch, fsm, bridge = make_flow()
    orch._pipe_clamped = True
    def stop_while_releasing(seconds):
        assert orch._pipe_clamped is None
        fsm.stop()
        return False
    orch._wait_interruptible = stop_while_releasing
    orch._handle_next_check()
    assert orch._pipe_clamped is None
    assert fsm.current_state == S.STOPPED
    bridge.call_unclamp.assert_called_once()
    bridge.call_move_lift.assert_not_called()


def test_next_depth_does_not_start_removal():
    orch, fsm, bridge = make_flow(depths=2)
    orch._handle_next_check()
    assert fsm.current_depth_index == 1
    assert fsm.current_pipe_index == 2
    assert fsm.current_state == S.PRESS_AND_SUCTION
    assert bridge.method_calls == []


@pytest.mark.parametrize("action", [A.SYSTEM_NEXT_POINT, A.SYSTEM_ALL_DONE])
@pytest.mark.parametrize("state", [S.NEXT_CHECK, S.EXTRACT_PIPE, S.REMOVE_PIPE_PROMPT])
def test_cannot_bypass_pipe_removal(state, action):
    _, fsm, _ = make_flow()
    fsm._state = state
    with pytest.raises(ValueError):
        fsm.transition(action)
    assert fsm.current_state == state
    assert fsm.current_pipe_index == 2


@pytest.mark.parametrize("phase", ["extract_prepare", "extract"])
def test_failed_motion_not_replayed_and_blocks_navigation(phase):
    orch, fsm, bridge = make_flow()
    bridge.call_move_lift.side_effect = lambda direction, distance: direction != phase
    orch._handle_next_check()
    assert fsm.current_state == S.STOPPED
    assert fsm.current_pipe_index == 2
    assert bridge.call_move_lift.call_args_list.count(call(phase, 20)) == 1
    bridge.call_untighten.assert_not_called()
    bridge.call_navigate.assert_not_called()
    bridge.call_emergency_stop.assert_called_once()


@pytest.mark.parametrize("step", range(1, 5))
def test_stop_during_extraction_blocks_release(step):
    orch, fsm, bridge = make_flow()
    waits = []
    def interrupt(seconds):
        waits.append(seconds)
        if len(waits) == step:
            fsm.stop()
            return False
        return True
    orch._wait_interruptible = interrupt
    orch._handle_next_check()
    assert fsm.current_state == S.STOPPED
    assert fsm.current_pipe_index == 2
    assert not orch._pipe_clamped
    bridge.call_emergency_stop.assert_called_once()
    bridge.call_untighten.assert_not_called()
    bridge.call_navigate.assert_not_called()


@pytest.mark.parametrize("failed_action", ["untighten", "unclamp"])
def test_release_failure_does_not_count_pipe_as_removed(failed_action):
    orch, fsm, bridge = make_flow()
    orch._handle_next_check()
    getattr(bridge, "call_" + failed_action).return_value = False
    fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
    assert fsm.current_state == S.STOPPED
    assert fsm.current_pipe_index == 2
    bridge.call_emergency_stop.assert_called_once()
    bridge.call_navigate.assert_not_called()


def test_stop_while_waiting_for_manual_removal_prevents_further_motion():
    orch, fsm, bridge = make_flow()
    orch._handle_next_check()
    fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
    fsm.stop()
    bridge.reset_mock()
    with pytest.raises(ValueError):
        fsm.transition(A.CONFIRM_PIPE_REMOVED)
    assert fsm.current_pipe_index == 2
    assert bridge.method_calls == []


def test_production_bridge_runtime_and_lift_complete_insert_then_extract(monkeypatch):
    """Exercise the real bridge/daemon/controller chain with only the axis simulated."""
    monkeypatch.setattr("grain_sampling_devices.mechanism_driver.PRESS_PAUSE_S", 0)

    class EncoderLift:
        counts_per_mm = 100.0
        config = type("Config", (), {"encoder_forward_sign": 1})()
        position = 1234
        def read_position(self):
            return self.position
        def move_to_position(self, target, duration_s, tolerance_mm):
            self.position = target
        def stop(self):
            pass

    lift = EncoderLift()
    controller = MechanismController(mock_mode=True, lift_drive=lift)
    runtime = MechanismRuntime(controller)
    commands = []

    class LocalClient:
        def request(self, action, **values):
            assert action == "mechanism"
            commands.append((values["name"], values.get("args", {})))
            runtime.execute(values["name"], **values.get("args", {}))
            return {"ok": True}

    bridge = RobotBridge(client=LocalClient())
    fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
    fsm.set_depth_targets([2.0])
    fsm._state = S.ARRIVED_PROMPT
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._wait_interruptible = lambda seconds: fsm.is_running
    orch._run_async = lambda fn: fn()
    orch._log_sampling_event = lambda *args: None
    orch.sampling_duration_sec = 0
    orch.convey_duration_sec = 0
    try:
        fsm.transition(A.CONFIRM_READY)
        assert fsm.current_state == S.ADD_PIPE_PROMPT
        fsm.transition(A.CONFIRM_PIPE_ADDED)
        assert fsm.current_state == S.DISCHARGE_WASTE
        assert fsm.current_pipe_index == 2
        assert sum(name == "clamp" for name, _ in commands) == 4
        fsm.transition(A.CONFIRM_WASTE_DISCHARGED)
        assert fsm.current_state == S.CONVEY_DONE
        fsm.transition(A.CONFIRM_DONE)
        for remaining in (2, 1):
            assert fsm.current_state == S.PIPE_SUPPORT_PROMPT
            assert fsm.current_pipe_index == remaining
            assert controller._lift_cycle_origin is None
            assert lift.position == 1234
            fsm.transition(A.CONFIRM_PIPE_SUPPORTED)
            assert fsm.current_state == S.REMOVE_PIPE_PROMPT
            command_index = len(commands)
            fsm.transition(A.CONFIRM_PIPE_REMOVED)
            if remaining > 1:
                assert commands[command_index:] == [
                    ("lift_health", {}),
                    ("move_lift", {"direction": "extract_prepare", "distance_cm": 20}),
                    ("clamp", {}),
                    ("move_lift", {"direction": "extract", "distance_cm": 20}),
                ]
        assert fsm.current_state == S.ALL_DONE_PROMPT
        assert fsm.current_pipe_index == 0
        assert not runtime.requires_mechanical_reset()
        assert not runtime.estop_latched
    finally:
        runtime.close()
