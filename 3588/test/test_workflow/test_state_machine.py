"""Tests for the 15-step grain sampling state machine."""

from __future__ import annotations

from typing import List, Optional, Tuple

import pytest

from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)


# ── Helpers ───────────────────────────────────────────────────


class TransitionRecorder:
    """Records state transition callbacks for test assertions."""

    def __init__(self) -> None:
        self.events: List[Tuple[SamplingState, SamplingState,
                                Optional[SamplingAction]]] = []

    def __call__(self, prev: SamplingState, current: SamplingState,
                 action: Optional[SamplingAction]) -> None:
        self.events.append((prev, current, action))


# ── Fixtures ──────────────────────────────────────────────────


@pytest.fixture
def fsm() -> SamplingStateMachine:
    """A basic state machine with default params."""
    return SamplingStateMachine(total_waypoints=2, max_depth=2)


@pytest.fixture
def recorder() -> TransitionRecorder:
    return TransitionRecorder()


# ── Happy-path: full 15-step walkthrough ──────────────────────


class TestFullWorkflow:
    """Walk through all 15 steps with a single-depth, single-point order."""

    def test_full_sequence_single_point(self) -> None:
        """Complete workflow with 1 waypoint, 1 depth — no loops."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        recorder = TransitionRecorder()
        fsm.on_state_change = recorder

        assert fsm.current_state == SamplingState.INIT

        # Step 1 → 2: CONFIRM_READY
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.RECORD_START

        # Step 2 → 3: system record complete
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        assert fsm.current_state == SamplingState.NAVIGATE_TO_POINT

        # Step 3 → 4: navigation complete
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        assert fsm.current_state == SamplingState.ARRIVED_PROMPT

        # Step 4 → 5: user confirms pipe connected
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        # pipe_index 由 orchestrator 在下压时递增（FSM 自身 0-based 起始）
        fsm.current_pipe_index += 1  # 模拟 orchestrator._handle_press_and_suction
        assert fsm.current_pipe_index == 1

        # Step 5 → 6: press complete (1m)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        assert fsm.current_state == SamplingState.ADD_PIPE_PROMPT

        # Step 6 → 7: user confirms pipe added
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        assert fsm.current_state == SamplingState.REPEAT_UNTIL_DEPTH

        # Step 7 → 8: depth reached (max_depth=1, we're done)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_state == SamplingState.DISCHARGE_WASTE

        # Step 8 → 9: waste discharged
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

        # Step 9 → 10: suction complete
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        assert fsm.current_state == SamplingState.CONVEY_1

        # Step 10 → 11: convey complete
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        assert fsm.current_state == SamplingState.CONVEY_DONE

        # Step 11 → 12: bin opened
        assert fsm.current_state == SamplingState.CONVEY_DONE

        # Step 12 → 13: user confirms done
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # Step 13 → 14: all done (single point, single depth)
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT

        # Step 14 → 15: user confirms return
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        assert fsm.current_state == SamplingState.RETURN

        # Step 15 → COMPLETED
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED

        # Verify all 15 transitions were recorded
        assert len(recorder.events) == 15

    def test_full_sequence_multi_depth(self) -> None:
        """Workflow with 1 waypoint, 2 depths — tests REPEAT_UNTIL_DEPTH loop.

        注意：pipe_index 由 orchestrator 在下压时递增（FSM 自身只跟踪状态）；
        depth_index 由 NEXT_CHECK + SYSTEM_NEXT_DEPTH 递增（跨深度 pipe 累计）。
        """
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=2)

        # Go through INIT → ... → PRESS_AND_SUCTION
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        assert fsm.current_pipe_index == 1

        # Press complete
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        assert fsm.current_state == SamplingState.REPEAT_UNTIL_DEPTH

        # Depth NOT reached yet (max_depth=2, we need more) → loop back
        fsm.transition(SamplingAction.SYSTEM_DEPTH_NOT_REACHED)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 2 节
        assert fsm.current_pipe_index == 2  # second pipe

        # Second press complete
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        assert fsm.current_state == SamplingState.REPEAT_UNTIL_DEPTH

        # Depth reached now
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_state == SamplingState.DISCHARGE_WASTE
        # DEPTH_REACHED 本身不递增 depth_index（由 NEXT_DEPTH 递增）
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 2  # pipe 跨深度累计，不 reset

        # Continue: DISCHARGE_WASTE → FORMAL_SAMPLING → CONVEY_1 → OPEN_BIN → CONVEY_DONE → NEXT_CHECK
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # NEXT_CHECK with more depth at same point
        fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_depth_index == 1  # NEXT_DEPTH 递增到 depth 1（显示 2/2）
        assert fsm.current_pipe_index == 2  # 跨深度累计，仍为 2

        # Complete second depth
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)  # depth reached
        assert fsm.current_depth_index == 1  # DEPTH_REACHED 不递增

        # Finish: DISCHARGE → FORMAL → CONVEY → BIN → DONE → NEXT_CHECK
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # All done
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT

        fsm.transition(SamplingAction.CONFIRM_RETURN)
        assert fsm.current_state == SamplingState.RETURN

        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED

    def test_full_sequence_multi_point(self) -> None:
        """Workflow with 2 waypoints, 1 depth — tests NEXT_CHECK → NAVIGATE."""
        fsm = SamplingStateMachine(total_waypoints=2, max_depth=1)

        # Fast-forward through first waypoint
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK
        assert fsm.current_waypoint_index == 0  # still first

        # NEXT_CHECK → next point
        fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        assert fsm.current_state == SamplingState.NAVIGATE_TO_POINT
        assert fsm.current_waypoint_index == 1  # now second
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 0

        # Complete second waypoint
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # All done
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED


# ── Safety controls during FORMAL_SAMPLING ────────────────────


class TestSafetyControls:
    """Pause / Resume / Stop are only valid during FORMAL_SAMPLING."""

    def _reach_formal_sampling(self, fsm: SamplingStateMachine) -> None:
        """Helper — fast-forward to FORMAL_SAMPLING state."""
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

    def test_pause_and_resume(self, fsm: SamplingStateMachine) -> None:
        self._reach_formal_sampling(fsm)
        assert not fsm.paused

        fsm.pause()
        assert fsm.paused
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

        fsm.resume()
        assert not fsm.paused
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

    def test_stop(self, fsm: SamplingStateMachine) -> None:
        self._reach_formal_sampling(fsm)

        fsm.stop()
        assert fsm.current_state == SamplingState.STOPPED
        assert not fsm.is_running

    def test_pause_outside_formal_raises(self, fsm: SamplingStateMachine) -> None:
        """PAUSE/RESUME should raise ValueError when not in FORMAL_SAMPLING."""
        with pytest.raises(ValueError, match="is only valid during FORMAL_SAMPLING"):
            fsm.pause()

        with pytest.raises(ValueError, match="is only valid during FORMAL_SAMPLING"):
            fsm.resume()

        # STOP 从 INIT 也非法（但文案不同：is not valid for state）
        with pytest.raises(ValueError, match="is not valid for state"):
            fsm.stop()

    def test_stop_from_paused(self, fsm: SamplingStateMachine) -> None:
        """STOP should work even when paused."""
        self._reach_formal_sampling(fsm)
        fsm.pause()
        assert fsm.paused
        fsm.stop()
        assert fsm.current_state == SamplingState.STOPPED

    def test_cannot_resume_when_not_paused(self, fsm: SamplingStateMachine) -> None:
        """Resume is allowed even if not paused (no-op that clears flag)."""
        self._reach_formal_sampling(fsm)
        # RESUME when not paused — should work fine (clears flag)
        fsm.resume()
        assert not fsm.paused


# ── Invalid transitions ──────────────────────────────────────


class TestInvalidTransitions:
    """Verify that invalid actions raise ValueError."""

    def test_confirm_ready_from_init(self, fsm: SamplingStateMachine) -> None:
        """CONFIRM_READY from INIT is valid — this is step 1→2."""

    def test_wrong_action_raises(self, fsm: SamplingStateMachine) -> None:
        """Applying an unrelated action raises ValueError."""
        fsm.transition(SamplingAction.CONFIRM_READY)  # step 1→2
        # Can't CONFIRM_PIPE_ADDED from RECORD_START
        with pytest.raises(ValueError, match="Invalid transition"):
            fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)

    def test_system_action_from_wrong_state(self, fsm: SamplingStateMachine) -> None:
        with pytest.raises(ValueError, match="Invalid transition"):
            fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)

    def test_repeated_system_complete(self, fsm: SamplingStateMachine) -> None:
        """Calling SYSTEM_NAV_COMPLETE twice in wrong state raises."""
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        # Now in NAVIGATE_TO_POINT — valid
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        assert fsm.current_state == SamplingState.ARRIVED_PROMPT
        # SYSTEM_NAV_COMPLETE again should fail
        with pytest.raises(ValueError, match="Invalid transition"):
            fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)


# ── Reset ─────────────────────────────────────────────────────


class TestReset:
    """Verify reset returns the machine to INIT."""

    def test_reset_from_initial(self, fsm: SamplingStateMachine) -> None:
        fsm.reset()
        assert fsm.current_state == SamplingState.INIT
        assert not fsm.paused

    def test_reset_clears_state(self, fsm: SamplingStateMachine) -> None:
        # Walk partway through
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        assert fsm.current_pipe_index == 1

        fsm.reset()
        assert fsm.current_state == SamplingState.INIT
        assert fsm.current_pipe_index == 0
        assert fsm.current_waypoint_index == 0
        assert fsm.current_depth_index == 0
        assert not fsm.paused

    def test_reset_triggers_callback(self, fsm: SamplingStateMachine,
                                     recorder: TransitionRecorder) -> None:
        fsm.on_state_change = recorder
        fsm.reset()
        assert len(recorder.events) == 1
        prev, current, action = recorder.events[0]
        # prev could be INIT (no change) — that's fine
        assert current == SamplingState.INIT
        assert action is None


# ── Waypoint / depth / pipe tracking ─────────────────────────


class TestTracking:
    """Index tracking during transitions."""

    def test_initial_indices(self, fsm: SamplingStateMachine) -> None:
        assert fsm.current_waypoint_index == 0
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 0

    def test_pipe_increments_on_loop(self, fsm: SamplingStateMachine) -> None:
        """Pipe index increases when DEPTH_NOT_REACHED loops back."""
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        assert fsm.current_pipe_index == 1

        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_NOT_REACHED)
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 2 节
        assert fsm.current_pipe_index == 2

    def test_depth_increment_on_reach(self, fsm: SamplingStateMachine) -> None:
        """Depth index increments via NEXT_DEPTH (not DEPTH_REACHED)."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=2)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        # DEPTH_REACHED 本身不递增 depth_index（由 NEXT_DEPTH 递增）
        assert fsm.current_depth_index == 0
        # pipe 跨深度累计，不 reset
        assert fsm.current_pipe_index == 1

    def test_waypoint_increment_on_next_point(self, fsm: SamplingStateMachine) -> None:
        """Waypoint index increments on SYSTEM_NEXT_POINT."""
        fsm = SamplingStateMachine(total_waypoints=3, max_depth=1)
        # Fast forward to NEXT_CHECK
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        assert fsm.current_waypoint_index == 1
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 0


# ── Progress string ──────────────────────────────────────────


class TestProgressString:
    """Verify the ``progress_str`` property format."""

    def test_default_progress(self, fsm: SamplingStateMachine) -> None:
        assert fsm.progress_str == "点位 1/2 | 深度 1/2"

    def test_with_pipe(self, fsm: SamplingStateMachine) -> None:
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        # progress_str 仅当 pipe_index>0 时追加管节（由 orchestrator 递增）
        assert "管节" not in fsm.progress_str
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        assert fsm.current_pipe_index == 1
        assert "管节 1" in fsm.progress_str

    def test_multi_pipe_display(self, fsm: SamplingStateMachine) -> None:
        fsm = SamplingStateMachine(total_waypoints=2, max_depth=3)
        # After first pipe
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压第 1 节
        assert "点位 1/2" in fsm.progress_str
        assert "深度 1/3" in fsm.progress_str
        assert "管节 1" in fsm.progress_str


# ── is_running property ──────────────────────────────────────


class TestIsRunning:
    def test_running_initial(self, fsm: SamplingStateMachine) -> None:
        assert fsm.is_running

    def test_not_running_when_stopped(self, fsm: SamplingStateMachine) -> None:
        """Stop during FORMAL_SAMPLING makes is_running False."""
        # Fast-forward to FORMAL_SAMPLING
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING
        assert fsm.is_running

        fsm.stop()
        assert fsm.current_state == SamplingState.STOPPED
        assert not fsm.is_running

    def test_not_running_when_completed(self, fsm: SamplingStateMachine) -> None:
        # Fast run through
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED
        assert not fsm.is_running


# ── Callback ──────────────────────────────────────────────────


class TestCallback:
    """Verify the on_state_change callback fires correctly."""

    def test_callback_fires_on_transition(self, fsm: SamplingStateMachine,
                                          recorder: TransitionRecorder) -> None:
        fsm.on_state_change = recorder
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert len(recorder.events) == 1
        prev, curr, act = recorder.events[0]
        assert prev == SamplingState.INIT
        assert curr == SamplingState.RECORD_START
        assert act == SamplingAction.CONFIRM_READY

    def test_callback_fires_on_pause(self, fsm: SamplingStateMachine,
                                     recorder: TransitionRecorder) -> None:
        # Fast-forward
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)

        fsm.on_state_change = recorder
        fsm.pause()
        assert len(recorder.events) == 1
        _, curr, act = recorder.events[0]
        assert curr == SamplingState.FORMAL_SAMPLING
        assert act == SamplingAction.PAUSE
        assert fsm.paused

    def test_callback_on_reset(self, fsm: SamplingStateMachine,
                               recorder: TransitionRecorder) -> None:
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.on_state_change = recorder
        fsm.reset()
        assert len(recorder.events) == 1
        _, curr, act = recorder.events[0]
        assert curr == SamplingState.INIT
        assert act is None

    def test_reattach_callback(self, fsm: SamplingStateMachine) -> None:
        """Setting on_state_change to None should stop callbacks."""
        calls: List[str] = []

        def cb(*args):  # type: ignore[no-untyped-def]
            calls.append("called")

        fsm.on_state_change = cb
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert len(calls) == 1

        # Detach
        fsm.on_state_change = None
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        assert len(calls) == 1  # no new call


# ── A. Illegal transition rejection ──────────────────────────


class TestIllegalTransitions:
    """Verify that invalid state transitions raise ValueError."""

    def test_cannot_stop_from_init(self) -> None:
        """STOP from INIT is illegal."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        with pytest.raises(ValueError, match="is not valid for state INIT"):
            fsm.transition(SamplingAction.STOP)

    def test_cannot_confirm_ready_during_navigation(self) -> None:
        """CONFIRM_READY from NAVIGATE_TO_POINT is illegal."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        assert fsm.current_state == SamplingState.NAVIGATE_TO_POINT
        with pytest.raises(ValueError, match="Invalid transition"):
            fsm.transition(SamplingAction.CONFIRM_READY)

    def test_stopped_rejects_all_transitions(self) -> None:
        """Once STOPPED (terminal), no further transitions are allowed."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        # Fast-forward to FORMAL_SAMPLING then STOP
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING
        fsm.transition(SamplingAction.STOP)
        assert fsm.current_state == SamplingState.STOPPED
        with pytest.raises(ValueError):
            fsm.transition(SamplingAction.CONFIRM_READY)


# ── B. PAUSE / RESUME / STOP safety controls (extended) ─────


class TestSafetyControlsExtended:
    """Additional edge-case coverage for safety controls."""

    @staticmethod
    def _reach_formal_sampling(fsm: SamplingStateMachine) -> None:
        """Fast-forward to FORMAL_SAMPLING state."""
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

    def test_stop_during_convey(self) -> None:
        """STOP is valid from any non-terminal state — CONVEY_1 stops too."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        self._reach_formal_sampling(fsm)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        assert fsm.current_state == SamplingState.CONVEY_1
        fsm.transition(SamplingAction.STOP)
        assert fsm.current_state == SamplingState.STOPPED

    def test_stopped_prevents_resume(self) -> None:
        """RESUME from STOPPED should raise ValueError."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        self._reach_formal_sampling(fsm)
        fsm.transition(SamplingAction.STOP)
        assert fsm.current_state == SamplingState.STOPPED
        with pytest.raises(ValueError, match="only valid during FORMAL_SAMPLING"):
            fsm.transition(SamplingAction.RESUME)

    def test_stopped_prevents_pause(self) -> None:
        """PAUSE from STOPPED should raise ValueError."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        self._reach_formal_sampling(fsm)
        fsm.transition(SamplingAction.STOP)
        with pytest.raises(ValueError, match="only valid during FORMAL_SAMPLING"):
            fsm.transition(SamplingAction.PAUSE)

    def test_stopped_prevents_stop(self) -> None:
        """STOP from STOPPED (double-stop) should raise ValueError."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        self._reach_formal_sampling(fsm)
        fsm.transition(SamplingAction.STOP)
        with pytest.raises(ValueError, match="is not valid for state STOPPED"):
            fsm.transition(SamplingAction.STOP)

    def test_pause_during_navigation(self) -> None:
        """PAUSE from NAVIGATE_TO_POINT should raise ValueError."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        assert fsm.current_state == SamplingState.NAVIGATE_TO_POINT
        with pytest.raises(ValueError, match="only valid during FORMAL_SAMPLING"):
            fsm.transition(SamplingAction.PAUSE)

    def test_resume_during_discharge(self) -> None:
        """RESUME from DISCHARGE_WASTE should raise ValueError."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        self._reach_formal_sampling(fsm)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK
        with pytest.raises(ValueError, match="only valid during FORMAL_SAMPLING"):
            fsm.transition(SamplingAction.RESUME)


# ── C. Pipe count / depth calculation ────────────────────────


class TestPipeCount:
    """Pipe count and depth calculation edge cases."""

    def test_total_pipes_needed_default(self) -> None:
        """When no depth targets are set, total_pipes_needed == 1."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        assert fsm.total_pipes_needed == 1

    @pytest.mark.parametrize("depth_m,expected_pipes", [
        (0.5, 1),   # < 1 m → 1 pipe (min 1)
        (1.0, 1),   # exactly 1 m → 1 pipe
        (1.5, 2),   # 1.5 m → 2 pipes
        (2.0, 2),   # exactly 2 m → 2 pipes
        (2.5, 3),   # 2.5 m → 3 pipes
    ])
    def test_total_pipes_needed_ceil_calculation(
        self, depth_m: float, expected_pipes: int,
    ) -> None:
        """total_pipes_needed uses math.ceil(depth_m / 1.0)."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=3)
        fsm.set_depth_targets([depth_m])
        assert fsm.total_pipes_needed == expected_pipes

    def test_pipe_index_starts_at_one_on_arrival(self) -> None:
        """current_pipe_index becomes 1 after the first press (orchestrator)."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=2)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_pipe_index == 0  # FSM 自身 0-based，未 press
        fsm.current_pipe_index += 1  # 模拟 orchestrator 下压
        assert fsm.current_pipe_index == 1

    def test_pipe_index_increments_on_not_reached(self) -> None:
        """Each press (orchestrator) increments the pipe index on loops."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=3)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 第 1 节
        assert fsm.current_pipe_index == 1

        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_NOT_REACHED)
        fsm.current_pipe_index += 1  # 第 2 节
        assert fsm.current_pipe_index == 2

        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_NOT_REACHED)
        fsm.current_pipe_index += 1  # 第 3 节
        assert fsm.current_pipe_index == 3

    def test_pipe_index_resets_after_depth_reached(self) -> None:
        """After DEPTH_REACHED, pipe index stays (cross-depth accumulation)."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=2)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 第 1 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        # DEPTH_REACHED 不递增 depth，也不 reset pipe（NEXT_DEPTH 才动 depth）
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 1


# ── D. Multi-point / multi-depth loops ───────────────────────


class TestMultiPointMultiDepth:
    """Verify that NEXT_CHECK branching works with multiple points and depths."""

    def test_two_waypoints_two_depths_full_sequence(self) -> None:
        """2 waypoints × 2 depths: (wp0,d0)→(wp0,d1)→(wp1,d0)→(wp1,d1)→ALL_DONE.

        语义：pipe_index 由 orchestrator press 时递增（FSM 测试模拟）；
        depth_index 仅由 NEXT_CHECK + SYSTEM_NEXT_DEPTH 递增；
        pipe 跨深度累计；换点位时 pipe/depth 归零。
        """
        fsm = SamplingStateMachine(total_waypoints=2, max_depth=2)

        # ── wp0, depth0 ─────────────────────────────────────
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_waypoint_index == 0 and fsm.current_depth_index == 0

        fsm.current_pipe_index += 1  # 下压第 1 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 0  # DEPTH_REACHED 不递增 depth

        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # ── wp0, depth1 (NEXT_DEPTH) ────────────────────────
        fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_depth_index == 1  # NEXT_DEPTH 递增到 depth 1
        assert fsm.current_pipe_index == 1  # pipe 跨深度累计

        fsm.current_pipe_index += 1  # 下压第 2 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 1  # DEPTH_REACHED 不递增

        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # ── wp1, depth0 (NEXT_POINT) ────────────────────────
        fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        assert fsm.current_state == SamplingState.NAVIGATE_TO_POINT
        assert fsm.current_waypoint_index == 1
        assert fsm.current_depth_index == 0
        assert fsm.current_pipe_index == 0  # 换点位归零

        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 下压第 1 节（新点位）
        assert fsm.current_pipe_index == 1

        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 0

        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # ── wp1, depth1 (NEXT_DEPTH) ────────────────────────
        fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        assert fsm.current_waypoint_index == 1
        assert fsm.current_depth_index == 1
        assert fsm.current_pipe_index == 1

        fsm.current_pipe_index += 1  # 下压第 2 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # ── ALL_DONE ────────────────────────────────────────
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED

    def test_three_waypoints_one_depth(self) -> None:
        """3 waypoints × 1 depth: each NEXT_CHECK → NEXT_POINT, then ALL_DONE."""
        fsm = SamplingStateMachine(total_waypoints=3, max_depth=1)

        # Waypoint 0 — full sequence from INIT
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK
        assert fsm.current_waypoint_index == 0

        # Waypoint 1 — via NEXT_POINT
        fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        assert fsm.current_waypoint_index == 1
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # Waypoint 2 — via NEXT_POINT
        fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        assert fsm.current_waypoint_index == 2
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # All done
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED

    def test_one_waypoint_three_depths(self) -> None:
        """1 waypoint × 3 depths: (d0)→(d1)→(d2)→ALL_DONE via NEXT_DEPTH.

        语义：depth_index 由 NEXT_DEPTH 递增（0→1→2→3 显示位）；
        DEPTH_REACHED 不递增；pipe 跨深度累计（1→2→3）。
        """
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=3)

        # Depth 0
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.current_pipe_index += 1  # 下压第 1 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 0
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # Depth 1
        fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_depth_index == 1
        assert fsm.current_pipe_index == 1  # 跨深度累计

        fsm.current_pipe_index += 1  # 下压第 2 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 1
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # Depth 2
        fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_depth_index == 2
        assert fsm.current_pipe_index == 2  # 跨深度累计

        fsm.current_pipe_index += 1  # 下压第 3 节
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        assert fsm.current_depth_index == 2
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        assert fsm.current_state == SamplingState.NEXT_CHECK

        # All done
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        assert fsm.current_state == SamplingState.ALL_DONE_PROMPT
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
        assert fsm.current_state == SamplingState.COMPLETED


# ── E. Callback / notification behavior (extended) ───────────


class TestCallbacksExtended:
    """Additional edge-case coverage for the on_state_change callback."""

    def test_on_state_change_called_on_every_transition(self) -> None:
        """Every normal transition in a full sequence fires the callback."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        recorder = TransitionRecorder()
        fsm.on_state_change = recorder

        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_PIPE_ADDED)
        fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        fsm.transition(SamplingAction.CONFIRM_WASTE_DISCHARGED)
        fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)
        fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_DONE)
        fsm.transition(SamplingAction.SYSTEM_ALL_DONE)
        fsm.transition(SamplingAction.CONFIRM_RETURN)
        fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)

        assert len(recorder.events) == 15

    def test_on_state_change_params_correct(self) -> None:
        """Verify (prev, current, action) tuple for first two transitions."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        recorder = TransitionRecorder()
        fsm.on_state_change = recorder

        fsm.transition(SamplingAction.CONFIRM_READY)
        prev, curr, act = recorder.events[0]
        assert prev == SamplingState.INIT
        assert curr == SamplingState.RECORD_START
        assert act == SamplingAction.CONFIRM_READY

        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        prev, curr, act = recorder.events[1]
        assert prev == SamplingState.RECORD_START
        assert curr == SamplingState.NAVIGATE_TO_POINT
        assert act == SamplingAction.SYSTEM_RECORD_COMPLETE

    def test_on_state_change_can_be_none(self) -> None:
        """Setting on_state_change to None should not cause errors."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        fsm.on_state_change = None
        # Walk through multiple transitions with no callback attached
        fsm.transition(SamplingAction.CONFIRM_READY)
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)
        fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
