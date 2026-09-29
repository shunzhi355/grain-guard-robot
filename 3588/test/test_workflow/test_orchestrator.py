"""Comprehensive unit tests for WorkflowOrchestrator with mocked dependencies.

All tests use ``unittest.mock`` — no real ROS, no real network, no hardware.
Target: 17 test scenarios covering core orchestration, cloud reporting,
mechanism placeholders, safety controls, and depth callbacks.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest

from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_cloud.protocol import DetectionResult, OrderInfo, ReportStatus


# ── Fixtures ───────────────────────────────────────────────────────────────


@pytest.fixture
def mock_bridge():
    bridge = MagicMock()
    bridge.record_start_position.return_value = (1.0, 2.0)
    bridge.call_navigate.return_value = True
    bridge.call_lift_health.return_value = True
    return bridge


@pytest.fixture
def mock_cloud():
    with patch("grain_sampling_workflow.orchestrator.CloudHttpClient") as m:
        client = m.return_value
        client.mac_address = "AA:BB:CC:DD:EE:FF"
        client.base_url = "http://cloud.example.com"
        client.post.return_value = {"success": True}
        # Orchestrator now builds the client via from_app_config(); make it
        # return the same mock so tests can assert on post calls.
        m.from_app_config.return_value = client
        yield client


@pytest.fixture
def fsm():
    return SamplingStateMachine(total_waypoints=2, max_depth=2)


@pytest.fixture
def orch(fsm, mock_bridge, mock_cloud):
    return WorkflowOrchestrator(
        fsm, mock_bridge, waypoints=[(1.0, 2.0), (3.0, 4.0)]
    )


# ══════════════════════════════════════════════════════════════════════════
# A. Core Orchestration (5 tests)
# ══════════════════════════════════════════════════════════════════════════


class TestCoreOrchestration:
    """Verify the orchestrator triggers the correct bridge operations."""

    def test_confirm_ready_triggers_record_start(self, orch, mock_bridge):
        """CONFIRM_READY triggers record_start_position via background handler."""
        orch._fsm.transition(SamplingAction.CONFIRM_READY)
        time.sleep(0.3)
        mock_bridge.record_start_position.assert_called_once()
        # State auto-advanced past INIT through the background chain
        assert orch._fsm.current_state != SamplingState.INIT

    def test_navigate_triggers_bridge_call(self, orch, mock_bridge):
        """After CONFIRM_READY, the auto-chain navigates to the first waypoint."""
        orch._fsm.transition(SamplingAction.CONFIRM_READY)
        time.sleep(0.3)
        mock_bridge.call_navigate.assert_called_with(1.0, 2.0)

    def test_return_to_start_navigates_to_recorded_position(
        self, orch, mock_bridge
    ):
        """_handle_return_to_start navigates to the recorded start position."""
        orch._start_position = (5.0, 6.0)
        orch._fsm._state = SamplingState.RETURN  # handler needs valid FSM context
        orch._handle_return_to_start()
        mock_bridge.call_navigate.assert_called_with(5.0, 6.0)

    def test_arrived_prompt_state(self, orch):
        """After the auto-chain completes, state is ARRIVED_PROMPT."""
        orch._fsm.transition(SamplingAction.CONFIRM_READY)
        time.sleep(0.3)
        assert orch._fsm.current_state == SamplingState.ARRIVED_PROMPT

    def test_completion_state(self, orch, mock_bridge):
        """Full synchronous orchestration walkthrough ends in COMPLETED."""
        orch._fsm.total_waypoints = 1
        orch._fsm.max_depth = 1
        orch._start_position = (0.0, 0.0)

        # Run all background handlers synchronously to avoid thread races
        with patch.object(orch, "_run_async", lambda f: f()):
            # Step 1→2→3→4: INIT → RECORD_START → NAVIGATE_TO_POINT → ARRIVED_PROMPT
            orch._fsm.transition(SamplingAction.CONFIRM_READY)
            # Step 4→5→8: ARRIVED_PROMPT → PRESS_AND_SUCTION → DISCHARGE_WASTE
            #   (depth reached with 1 pipe, CONFIRM_PIPE_ADDED / ADD_PIPE_PROMPT skipped)
            orch._fsm.transition(SamplingAction.CONFIRM_READY)
            # Step 8→9→10→11→12: DISCHARGE_WASTE → FORMAL_SAMPLING → CONVEY_1 → OPEN_BIN → CONVEY_DONE
            orch._fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
            # Step 12→13→14: CONVEY_DONE → NEXT_CHECK → ALL_DONE_PROMPT
            orch._fsm.transition(SamplingAction.CONFIRM_DONE)
            # Step 14→15→COMPLETED: ALL_DONE_PROMPT → RETURN → COMPLETED
            orch._fsm.transition(SamplingAction.CONFIRM_RETURN)

        assert orch._fsm.current_state == SamplingState.COMPLETED


# ══════════════════════════════════════════════════════════════════════════
# B. Cloud Status Reporting (5 tests)
# ══════════════════════════════════════════════════════════════════════════


class TestCloudReporting:
    """Verify the orchestrator reports status to the cloud via T7 API."""

    def test_set_task_order_stores_fields(self, orch):
        """set_task_order(OrderInfo) populates _order_id, _jiance, _depth_list."""
        order = OrderInfo(
            order_id="O-001",
            aojian_id=1,
            aojian="廒间1号",
            depth_list=[3.0, 2.0],
            jiance=[1, 5, 8],
            points=[{"x": 1.0, "y": 2.0}],
        )
        orch.set_task_order(order)
        assert orch._order_id == "O-001"
        assert orch._jiance == [1, 5, 8]
        assert orch._depth_list == [3.0, 2.0]

    def test_set_task_id_compat_alias(self, orch):
        """set_task_id(str) sets only _order_id, clears detection context."""
        orch.set_task_id("task-abc")
        assert orch._order_id == "task-abc"
        assert orch._jiance == []
        assert orch._depth_list == []

    def test_report_complete_calls_reader_and_posts(self, orch, mock_cloud):
        """report_complete() reads detection values and POSTs StatusReportRequest."""
        orch.set_task_order(OrderInfo(
            order_id="O-C", aojian_id=0, aojian="",
            depth_list=[1.5], jiance=[1, 2], points=[],
        ))

        with patch(
            "grain_sampling_devices.detection_reader.DetectionResultReader"
        ) as MockReader:
            reader = MockReader.return_value
            reader.read_all.return_value = [
                {"indicator_id": 1, "value": 0.85},
                {"indicator_id": 2, "value": 12.3, "depth": 1.5},
            ]
            result = orch.report_complete()

        assert result is True
        reader.read_all.assert_called_once_with([1, 2], [1.5])
        mock_cloud.post.assert_called_once()
        args, _ = mock_cloud.post.call_args
        assert "/report" in args[0]  # 相对路径，base_url 含 /admin-api 前缀
        payload = args[1]
        assert payload["order_id"] == "O-C"
        assert payload["status"] == ReportStatus.COMPLETE
        assert len(payload["jiance_results"]) == 2
        assert payload["jiance_results"][0].indicator_id == 1
        assert payload["jiance_results"][0].value == 0.85
        assert payload["jiance_results"][1].indicator_id == 2
        assert payload["jiance_results"][1].depth == 1.5

    def test_report_abandon_posts_without_results(self, orch, mock_cloud):
        """report_abandon() POSTs StatusReportRequest with no jiance_results."""
        orch.set_task_order(OrderInfo(
            order_id="O-A", aojian_id=0, aojian="",
            depth_list=[], jiance=[], points=[],
        ))
        result = orch.report_abandon()

        assert result is True
        mock_cloud.post.assert_called_once()
        args, _ = mock_cloud.post.call_args
        assert "/report" in args[0]  # 相对路径，base_url 含 /admin-api 前缀
        payload = args[1]
        assert payload["order_id"] == "O-A"
        assert payload["status"] == ReportStatus.ABANDON
        assert payload["jiance_results"] == []

    def test_report_handles_cloud_error(self, orch, mock_cloud):
        """report_complete returns False on CloudProtocolError."""
        from grain_sampling_cloud.http_client import CloudProtocolError

        orch.set_task_order(OrderInfo(
            order_id="O-E", aojian_id=0, aojian="",
            depth_list=[1.0], jiance=[1], points=[],
        ))
        mock_cloud.post.side_effect = CloudProtocolError("cloud error 1070700004: task not found")

        with patch(
            "grain_sampling_devices.detection_reader.DetectionResultReader"
        ) as MockReader:
            MockReader.return_value.read_all.return_value = [
                {"indicator_id": 1, "value": 0.5},
            ]
            result = orch.report_complete()

        assert result is False


# ══════════════════════════════════════════════════════════════════════════
# C. Mechanism Placeholders (3 tests)
# ══════════════════════════════════════════════════════════════════════════


class TestMechanismPlaceholders:
    """Verify placeholder mechanism handlers work correctly."""

    def test_press_and_suction_depth_reached(self, orch):
        """_handle_press_and_suction emits SYSTEM_DEPTH_REACHED when target reached."""
        # Default FSM: total_waypoints=2, max_depth=2 → target ~1.0m → 1 pipe needed
        # current_pipe_index starts at 0, incremented to 1 → depth reached
        orch._fsm.current_pipe_index = 0
        with patch.object(orch._fsm, "transition") as mock_transition:
            orch._handle_press_and_suction()
            mock_transition.assert_called_with(
                SamplingAction.SYSTEM_DEPTH_REACHED
            )

    def test_next_check_all_done(self, orch):
        """_handle_next_check emits SYSTEM_ALL_DONE when all points and depths done."""
        orch._fsm.total_waypoints = 1
        orch._fsm.max_depth = 1
        orch._fsm.current_waypoint_index = 0
        orch._fsm.current_depth_index = 0
        with patch.object(orch._fsm, "transition") as mock_transition:
            orch._handle_next_check()
            mock_transition.assert_called_with(
                SamplingAction.SYSTEM_ALL_DONE
            )

    def test_convey_duration_configurable(self, orch):
        """set_convey_duration updates the conveyor duration."""
        orch.set_convey_duration(180.0)
        assert orch.convey_duration_sec == 180.0


# ══════════════════════════════════════════════════════════════════════════
# C2. Mechanism Integration (real bridge calls when enabled)
# ══════════════════════════════════════════════════════════════════════════


class TestMechanismIntegration:
    """Verify handlers call real bridge methods once the mechanism is enabled.

    ``enable_mechanism()`` switches from the ``time.sleep`` placeholders to
    real ``SamplingBridge`` calls; failures are retried and then stop the FSM.
    """

    def test_enable_mechanism_toggles_flag(self, orch):
        """Mechanism is placeholder by default; enable_mechanism flips it on."""
        assert orch._mechanism_connected is False
        orch.enable_mechanism()
        assert orch._mechanism_connected is True
        orch.set_mechanism_connected(False)
        assert orch._mechanism_connected is False

    def test_press_and_suction_calls_bridge_chain(self, orch, mock_bridge):
        """Connected: full press cycle clamp -> press -> unclamp -> lift ->
        clamp -> start_suction (no real sleep), then transition."""
        orch.enable_mechanism()
        orch._fsm.current_pipe_index = 0
        # Task 9: each press-cycle step now waits its per-grain duration —
        # patch the interruptible wait so this unit test stays fast.
        with patch.object(orch, "_wait_interruptible", return_value=True):
            with patch.object(orch._fsm, "transition") as mock_transition:
                orch._handle_press_and_suction()
        mock_bridge.call_lift_health.assert_called_once()
        assert mock_bridge.call_clamp.call_count == 2  # 夹紧 + 回顶后再夹紧
        # press/lift 走 move_lift 精确距离控制（单次 PRESS_STEP_CM=20cm）
        mock_bridge.call_move_lift.assert_any_call("down_cycle", 20.0)
        mock_bridge.call_unclamp.assert_called_once()
        mock_bridge.call_move_lift.assert_any_call("return", 20.0)
        mock_bridge.call_start_suction.assert_called_once()
        mock_transition.assert_called_with(SamplingAction.SYSTEM_DEPTH_REACHED)
        # pipe counting still works with the real chain
        assert orch._fsm.current_pipe_index == 1

    def test_formal_sampling_calls_suction_chain(self, orch, mock_bridge):
        """Connected: start_suction -> wait -> stop_suction, then transition."""
        orch.enable_mechanism()
        orch.set_sampling_duration(0.1)
        with patch.object(orch._fsm, "transition") as mock_transition:
            orch._handle_formal_sampling()
        mock_bridge.call_start_suction.assert_called_once()
        mock_bridge.call_stop_suction.assert_called_once()
        mock_transition.assert_called_with(SamplingAction.SYSTEM_SUCTION_COMPLETE)

    def test_convey_calls_bridge(self, orch, mock_bridge):
        """Connected: call_convey + interruptible wait, then transition."""
        orch.enable_mechanism()
        orch.set_convey_duration(0.1)
        with patch.object(orch._fsm, "transition") as mock_transition:
            orch._handle_convey()
        mock_bridge.call_start_convey.assert_called_once()
        mock_bridge.call_stop_convey.assert_called_once()
        mock_transition.assert_called_with(SamplingAction.SYSTEM_CONVEY_COMPLETE)

    def test_open_bin_calls_bridge_with_depth(self, orch, mock_bridge):
        """Connected: call_open_bin(current_depth_index), then transition."""
        orch.enable_mechanism()
        orch._fsm.current_depth_index = 1
        with patch.object(orch._fsm, "transition") as mock_transition:
            orch._handle_open_bin()
        mock_bridge.call_hold_bin_open.assert_called_once_with(1)
        mock_transition.assert_called_with(SamplingAction.SYSTEM_BIN_OPENED)

    def test_close_bin_calls_bridge_with_depth(self, orch, mock_bridge):
        """Connected: call_close_bin(current_depth_index) — fire-and-forget, no transition."""
        orch.enable_mechanism()
        orch._fsm.current_depth_index = 2
        orch._handle_close_bin()
        mock_bridge.call_close_bin.assert_called_once_with(2)

    def test_set_task_order_calls_set_grain(self, orch, mock_bridge):
        """set_task_order pushes the order's grain variety to the mechanism."""
        order = OrderInfo(
            order_id="O-001", aojian_id=0, aojian="",
            depth_list=[1.0], jiance=[], points=[],
            pinzhong="稻谷",
        )
        orch.set_task_order(order)
        mock_bridge.call_set_grain.assert_called_once_with("稻谷")

    def test_set_grain_skipped_without_variety(self, orch, mock_bridge):
        """set_task_order with no grain variety does not call set_grain."""
        orch.set_task_order(OrderInfo(
            order_id="O-002", aojian_id=0, aojian="",
            depth_list=[], jiance=[], points=[],
        ))
        mock_bridge.call_set_grain.assert_not_called()

    def test_mechanism_failure_retries_then_stops_fsm(self, orch, mock_bridge):
        """A bridge method that always fails is retried, then the FSM stops."""
        orch.enable_mechanism()
        orch._mechanism_retry_interval = 0.01
        mock_bridge.call_clamp.return_value = False  # always fails
        orch._fsm._state = SamplingState.PRESS_AND_SUCTION
        orch._fsm.current_pipe_index = 0
        orch._handle_press_and_suction()
        # 1 initial attempt + 2 retries
        assert mock_bridge.call_clamp.call_count == 3
        assert orch._fsm.current_state == SamplingState.STOPPED
        # press/suction never started, FSM never auto-advanced past STOPPED
        mock_bridge.call_press.assert_not_called()

    def test_lift_health_failure_stops_before_clamp_and_shows_cause(
        self, orch, mock_bridge
    ):
        orch.enable_mechanism()
        orch._mechanism_retry_interval = 0.0
        orch._fsm._state = SamplingState.PRESS_AND_SUCTION
        mock_bridge.call_lift_health.return_value = False
        mock_bridge.last_error = (
            "/mechanism/lift_health: X2P communication health check failed: "
            "通信超时或应答过短"
        )

        orch._handle_press_and_suction()

        assert mock_bridge.call_lift_health.call_count == 3
        mock_bridge.call_clamp.assert_not_called()
        mock_bridge.call_move_lift.assert_not_called()
        assert orch._fsm.current_pipe_index == 0
        assert "通信超时或应答过短" in orch._fsm.stop_reason


# ══════════════════════════════════════════════════════════════════════════
# D. Safety & Control (3 tests)
# ══════════════════════════════════════════════════════════════════════════


class TestSafetyControl:
    """Verify safety and control operations."""

    def test_emergency_stop_calls_bridge(self, orch, mock_bridge):
        """emergency_stop calls bridge.call_emergency_stop and transitions to STOPPED."""
        # STOP is only valid from FORMAL_SAMPLING
        orch._fsm._state = SamplingState.FORMAL_SAMPLING
        orch.emergency_stop()
        mock_bridge.call_emergency_stop.assert_called_once()
        assert orch._fsm.current_state == SamplingState.STOPPED

    def test_sampling_duration_configurable(self, orch):
        """set_sampling_duration updates the formal sampling duration."""
        orch.set_sampling_duration(180.0)
        assert orch.sampling_duration_sec == 180.0

    def test_stop_transitions_to_stopped(self, orch):
        """STOP action from FORMAL_SAMPLING transitions to STOPPED."""
        orch._fsm._state = SamplingState.FORMAL_SAMPLING
        orch._fsm.transition(SamplingAction.STOP)
        assert orch._fsm.current_state == SamplingState.STOPPED


# ══════════════════════════════════════════════════════════════════════════
# E. Depth Callback (2 tests)
# ══════════════════════════════════════════════════════════════════════════


class TestDepthCallback:
    """Verify the optional depth display callback is invoked correctly."""

    def test_callback_invoked_on_transition(self, orch):
        """A registered depth_callback is called after state transitions."""
        callback_data = []

        def cb(depth: float) -> None:
            callback_data.append(depth)

        orch.depth_callback = cb
        orch._fsm.transition(SamplingAction.CONFIRM_READY)
        time.sleep(0.3)
        # At least one callback (for RECORD_START entry, possibly more via auto-chain)
        assert len(callback_data) > 0

    def test_none_callback_does_not_raise(self, orch):
        """A None depth_callback does not raise any exception."""
        orch.depth_callback = None
        orch._fsm.transition(SamplingAction.CONFIRM_READY)
        time.sleep(0.3)
        # Should complete without raising
