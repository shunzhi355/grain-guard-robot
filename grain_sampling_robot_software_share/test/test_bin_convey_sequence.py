"""Verify production bin/conveyor ordering without energizing hardware."""
from unittest.mock import MagicMock

import pytest

from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.state_machine import SamplingState, SamplingStateMachine, SamplingAction
from grain_sampling_workflow.mechanism_node import MechanismNode
from grain_sampling_workflow.mechanism_config import get_grain_params
from grain_sampling_devices.mechanism_driver import CHANNELS


@pytest.mark.parametrize("depth", [0, 1, 2])
def test_waste_open_sample_convey_close_stop(depth):
    events = []
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.DISCHARGE_WASTE
    fsm.current_depth_index = depth
    bridge = MagicMock()
    for name in ("hold_bin_open", "start_suction", "stop_suction",
                 "start_convey", "close_bin", "stop_convey"):
        getattr(bridge, "call_" + name).side_effect = (
            lambda *args, name=name: events.append((name, args)) or True
        )
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._run_async = lambda fn: fn()
    orch._log_sampling_event = lambda *args: None
    orch._wait_interruptible = lambda secs: events.append(("wait", secs)) or True
    orch.sampling_duration_sec = 7
    orch.convey_duration_sec = 11
    fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
    params = get_grain_params("")
    assert events == [
        ("hold_bin_open", (depth,)), ("wait", params["open_duration"] + 0.5),
        ("start_suction", ()), ("wait", 7), ("stop_suction", ()),
        ("start_convey", ()), ("wait", 11),
        ("close_bin", (depth,)), ("wait", params["close_duration"] + 0.5),
        ("stop_convey", ()),
    ]
    assert fsm.current_state == SamplingState.CONVEY_DONE
    bridge.call_open_bin.assert_not_called()
    bridge.call_convey.assert_not_called()


@pytest.mark.parametrize("fail", ["start_convey", "close_bin", "stop_convey"])
def test_failure_stops_hardware_and_blocks_completion(fail):
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.CONVEY_1
    bridge = MagicMock()
    getattr(bridge, "call_" + fail).return_value = False
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._mechanism_retry_interval = 0
    orch._wait_interruptible = lambda secs: True
    orch._handle_convey()
    assert fsm.current_state == SamplingState.STOPPED
    bridge.call_emergency_stop.assert_called()


@pytest.mark.parametrize("interrupt_at", [1, 2])
def test_interruption_during_convey_or_closing_stops_outputs(interrupt_at):
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.CONVEY_1
    bridge = MagicMock()
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    waits = []
    def wait(secs):
        waits.append(secs)
        if len(waits) == interrupt_at:
            fsm.transition(SamplingAction.STOP)
            return False
        return True
    orch._wait_interruptible = wait
    orch._handle_convey()
    assert fsm.current_state == SamplingState.STOPPED
    bridge.call_emergency_stop.assert_called()


def test_continuous_services_no_timers_and_emergency_stop(mock_mechanism):
    node = MechanismNode(controller=mock_mechanism)
    assert node.run_action("hold_bin_open", depth="deep")[0]
    assert node.run_action("start_convey")[0]
    history = mock_mechanism.action_history
    assert all(kw["duration"] is None for name, kw in history
               if name in ("convey", "open_bin"))
    active = {CHANNELS["convey_1"], CHANNELS["convey_2"], CHANNELS["bin_deep"]}
    assert active <= mock_mechanism._running
    mock_mechanism.emergency_stop()
    stopped = next(kw["channels"] for name, kw in reversed(history)
                   if name == "emergency_stop")
    assert active <= set(stopped)
    assert not mock_mechanism._running


def test_stop_convey_releases_both_channels(mock_mechanism):
    node = MechanismNode(controller=mock_mechanism)
    assert node.run_action("start_convey")[0]
    assert node.run_action("stop_convey")[0]
    assert CHANNELS["convey_1"] not in mock_mechanism._running
    assert CHANNELS["convey_2"] not in mock_mechanism._running
