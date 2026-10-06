"""Verify production bin/conveyor ordering without energizing hardware."""
from unittest.mock import MagicMock

import pytest

from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.state_machine import SamplingState, SamplingStateMachine, SamplingAction
from grain_sampling_workflow.mechanism_node import MechanismNode
from grain_sampling_workflow.mechanism_config import GRAIN_MECHANISM_CONFIG, get_grain_params
from grain_sampling_devices.mechanism_driver import CHANNELS, BIN_OPEN_PULSE, BIN_CLOSE_PULSE
from grain_sampling_workflow.ros_bridge import SamplingBridge


def test_production_convey_duration_follows_grain_configuration():
    assert get_grain_params("")["convey_duration"] == 120.0
    assert {grain: params["convey_duration"]
            for grain, params in GRAIN_MECHANISM_CONFIG.items()} == {
                "稻谷": 10.0, "玉米": 10.0, "黄豆": 90.0,
            }


@pytest.mark.parametrize("grain,depth", [("稻谷", 0), ("黄豆", 1), ("", 2)])
def test_waste_select_sample_convey_close_all_then_stop(grain, depth):
    events = []
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.DISCHARGE_WASTE
    fsm.current_depth_index = depth
    bridge = MagicMock()
    for name in ("hold_bin_open", "start_suction", "stop_suction",
                 "start_convey", "close_all_bins", "stop_convey"):
        getattr(bridge, "call_" + name).side_effect = (
            lambda *args, name=name: events.append((name, args)) or True
        )
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._run_async = lambda fn: fn()
    orch._log_sampling_event = lambda *args: None
    orch._wait_interruptible = lambda secs: events.append(("wait", secs)) or True
    orch.sampling_duration_sec = 7
    orch._grain = grain
    params = get_grain_params(grain)
    orch.set_convey_duration(params["convey_duration"])
    fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
    assert events == [
        ("hold_bin_open", (depth,)),
        ("wait", 5.0),
        ("start_suction", ()), ("wait", 7), ("stop_suction", ()),
        ("start_convey", ()), ("wait", params["convey_duration"]),
        ("close_all_bins", ()), ("wait", params["close_duration"]),
        ("stop_convey", ()), ("wait", 0.5),
    ]
    assert fsm.current_state == SamplingState.CONVEY_DONE
    bridge.call_open_bin.assert_not_called()
    bridge.call_convey.assert_not_called()
    bridge.call_close_bin.assert_not_called()


def test_ui_stays_on_open_bin_for_five_seconds_then_enters_formal_sampling():
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.DISCHARGE_WASTE
    bridge = MagicMock()
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._run_async = lambda fn: fn() if fn == orch._handle_open_bin else None
    waits = []

    def wait(seconds):
        waits.append(seconds)
        assert fsm.current_state == SamplingState.OPEN_BIN
        bridge.call_start_convey.assert_not_called()
        return True

    orch._wait_interruptible = wait
    fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
    assert waits == [5.0]
    bridge.call_hold_bin_open.assert_called_once_with(0)
    bridge.call_start_convey.assert_not_called()
    assert fsm.current_state == SamplingState.FORMAL_SAMPLING


def test_stop_during_open_delay_never_starts_conveyor():
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.OPEN_BIN
    bridge = MagicMock()
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()

    def stop_during_wait(seconds):
        assert seconds == 5.0
        fsm.transition(SamplingAction.STOP)
        return False

    orch._wait_interruptible = stop_during_wait
    orch._handle_open_bin()
    assert fsm.current_state == SamplingState.STOPPED
    bridge.call_start_convey.assert_not_called()
    bridge.call_emergency_stop.assert_called_once()


def test_formal_sampling_failure_does_not_start_conveyor():
    fsm = SamplingStateMachine()
    fsm._state = SamplingState.FORMAL_SAMPLING
    bridge = MagicMock()
    bridge.call_start_suction.return_value = False
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._mechanism_retry_interval = 0
    orch._wait_interruptible = lambda seconds: True

    orch._handle_formal_sampling()
    assert fsm.current_state == SamplingState.STOPPED
    assert bridge.call_start_suction.call_count == 3
    bridge.call_start_convey.assert_not_called()
    bridge.call_emergency_stop.assert_called_once()


@pytest.mark.parametrize("fail", ["start_convey", "close_all_bins", "stop_convey"])
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
    if fail == "close_all_bins":
        bridge.call_stop_convey.assert_not_called()
    if fail == "stop_convey":
        bridge.call_close_all_bins.assert_called_once()


@pytest.mark.parametrize("interrupt_at", [1, 2, 3])
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
    if interrupt_at <= 2:
        bridge.call_stop_convey.assert_not_called()
    else:
        bridge.call_stop_convey.assert_called_once()


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


@pytest.fixture
def bin_timers(monkeypatch):
    """Deterministic timers: inspect simultaneous commands before completing any."""
    timers = []

    def make_timer(seconds, callback):
        timer = MagicMock()
        timer.seconds = seconds
        timer.callback = callback
        timers.append(timer)
        return timer

    monkeypatch.setattr("grain_sampling_devices.mechanism_driver.threading.Timer", make_timer)
    return timers


@pytest.mark.parametrize("depth", ["shallow", "mid", "deep"])
def test_select_bin_opens_only_target_closes_others_then_neutral(
    mock_mechanism, bin_timers, depth
):
    node = MechanismNode(controller=mock_mechanism)
    assert node.run_action("hold_bin_open", depth=depth)[0]
    params = get_grain_params("")
    assert len(bin_timers) == 3
    for name, timer in zip(("shallow", "mid", "deep"), bin_timers):
        channel = CHANNELS[f"bin_{name}"]
        pulse = BIN_OPEN_PULSE if name == depth else BIN_CLOSE_PULSE
        assert mock_mechanism.pca9685.register_history[channel] == [pulse]
        assert timer.seconds == params["open_duration" if name == depth else "close_duration"]
        timer.start.assert_called_once()
    # All three directions have been commanded before any movement timer expires.
    for timer in bin_timers:
        timer.callback()
    for name in ("shallow", "mid", "deep"):
        channel = CHANNELS[f"bin_{name}"]
        assert mock_mechanism.pca9685.register_history[channel][-1] == 1500
        assert channel not in mock_mechanism._running
    assert not mock_mechanism._bin_timers


def test_close_all_cancels_old_selection_timers(mock_mechanism, bin_timers):
    node = MechanismNode(controller=mock_mechanism)
    assert node.run_action("hold_bin_open", depth="deep")[0]
    old_timers = list(bin_timers)
    assert node.run_action("close_all_bins")[0]
    for timer in old_timers:
        timer.cancel.assert_called_once()
        timer.callback()  # Even an already-dispatched stale callback cannot stop new motion.
    for channel in (2, 3, 4):
        assert mock_mechanism.pca9685.register_history[channel][-1] == BIN_CLOSE_PULSE
        assert channel in mock_mechanism._running
    for timer in bin_timers[3:]:
        assert timer.seconds == get_grain_params("")["close_duration"]
        timer.callback()
    for channel in (2, 3, 4):
        assert mock_mechanism.pca9685.register_history[channel][-1] == 1500
        assert channel not in mock_mechanism._running


@pytest.mark.parametrize("action", ["hold_bin_open", "close_all_bins"])
def test_group_bin_emergency_stop_prevents_late_output(mock_mechanism, bin_timers, action):
    node = MechanismNode(controller=mock_mechanism)
    kwargs = {"depth": "mid"} if action == "hold_bin_open" else {}
    assert node.run_action(action, **kwargs)[0]
    mock_mechanism.emergency_stop()
    before = {ch: list(values) for ch, values in mock_mechanism.pca9685.register_history.items()}
    for timer in bin_timers:
        timer.cancel.assert_called_once()
        timer.callback()
    assert mock_mechanism.pca9685.register_history == before
    assert not mock_mechanism._running


@pytest.mark.parametrize("action", ["hold_bin_open", "close_all_bins"])
def test_partial_group_write_failure_stops_all(mock_mechanism, bin_timers, monkeypatch, action):
    original_write = mock_mechanism._write_hw

    def fail_second(channel, pulse):
        if channel == 3:
            raise OSError("injected bus failure")
        original_write(channel, pulse)

    monkeypatch.setattr(mock_mechanism, "_write_hw", fail_second)
    mock_mechanism.RETRY_INTERVAL = 0
    node = MechanismNode(controller=mock_mechanism, retry_interval=0)
    kwargs = {"depth": "deep"} if action == "hold_bin_open" else {}
    assert not node.run_action(action, **kwargs)[0]
    assert mock_mechanism._stop_flag.is_set()
    assert not mock_mechanism._running
    assert not mock_mechanism._bin_timers
    for channel in (2, 3, 4):
        assert mock_mechanism.pca9685.register_history[channel][-1] == 1500


def test_invalid_bin_selection_does_not_write(mock_mechanism, bin_timers):
    with pytest.raises(ValueError, match="unknown bin depth"):
        mock_mechanism.hold_bin_open("unknown")
    assert not mock_mechanism.pca9685.register_history
    assert not bin_timers


def test_open_stage_waits_for_slower_closing_doors(monkeypatch):
    monkeypatch.setattr(
        "grain_sampling_workflow.orchestrator.get_grain_params",
        lambda grain: {"open_duration": 2, "close_duration": 7},
    )
    bridge = MagicMock()
    fsm = MagicMock()
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.enable_mechanism()
    orch._wait_interruptible = MagicMock(return_value=True)
    orch._handle_open_bin()
    orch._wait_interruptible.assert_called_once_with(7.5)


def test_close_all_bridge_uses_registered_service():
    bridge = object.__new__(SamplingBridge)
    bridge._call_trigger = MagicMock(return_value=True)
    assert bridge.call_close_all_bins()
    bridge._call_trigger.assert_called_once_with("/mechanism/close_all_bins")
