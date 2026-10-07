"""Sampling workflow state machine, including removal before leaving a point.

Defines the full grain sampling guidance workflow as an enum-based finite
state machine with validation, callbacks, and waypoint/depth/pipe tracking.
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Callable, Optional


# ── State & Action Enums ──────────────────────────────────────


class SamplingState(Enum):
    """Sampling stages, pipe removal stages, and terminal states."""

    # Normal workflow (steps 1–15)
    INIT = auto()               # 1: 开机初始化，准备就绪
    RECORD_START = auto()       # 2: 记录当前所在位置为"任务起始点"
    NAVIGATE_TO_POINT = auto()  # 3: 自主导航到工单上的第一个点位(X,Y)
    ARRIVED_PROMPT = auto()     # 4: 到达后停车，屏幕弹出提示 (操作工连接取样管)
    PRESS_AND_SUCTION = auto()  # 5: 自动下压扦样管+启动负压吸粮
    ADD_PIPE_PROMPT = auto()    # 6: 第1节管压到约1m深度后暂停，提示加管
    REPEAT_UNTIL_DEPTH = auto() # 7: 重复步骤5-6直到累计深度达到工单要求
    DISCHARGE_WASTE = auto()    # 8: 排出废粮
    FORMAL_SAMPLING = auto()    # 9: 开始正式采样（吸粮2分钟）
    CONVEY_1 = auto()           # 正式采样结束 → 按品种时长输粮 → 停止输粮 → 关仓
    OPEN_BIN = auto()           # 废粮排完后开仓 → 等待 5s → 正式吸粮
    CONVEY_DONE = auto()        # 12: 输送完成，提示取粮完成
    NEXT_CHECK = auto()         # 13: 自动判断下一步
    ALL_DONE_PROMPT = auto()    # 14: 所有点位完成，提示确认返航
    RETURN = auto()             # 15: 自主导航回到起始点，完成

    EXTRACT_PIPE = auto()       # 松夹下降 → 夹紧 → 反向上提一节
    PIPE_SUPPORT_PROMPT = auto() # 上提完成，等待人工托住管节
    RELEASE_PIPE = auto()       # 拧松接头 → 松夹
    REMOVE_PIPE_PROMPT = auto() # 等待人工取出当前管节

    # Terminal states
    STOPPED = auto()
    COMPLETED = auto()


class SamplingAction(Enum):
    """Actions that trigger state transitions.

    ``CONFIRM_*`` actions are user-driven (button presses on the guidance
    page).  ``SYSTEM_*`` actions are emitted by the mechanism bridge when
    a hardware operation completes.
    """

    # ── User actions ────────────────────────────────────────
    CONFIRM_READY = auto()             # "已就绪"
    CONFIRM_PIPE_ADDED = auto()        # "已加管"
    CONFIRM_WASTE_DISCHARGED = auto()  # "废粮已排完"
    CONFIRM_DONE = auto()              # "确认"
    CONFIRM_RETURN = auto()            # "确认返航"
    CONFIRM_PIPE_SUPPORTED = auto()   # "已托住管子"
    CONFIRM_PIPE_REMOVED = auto()     # "已取出管子"

    # ── Safety controls (during FORMAL_SAMPLING only) ───────
    PAUSE = auto()
    RESUME = auto()
    STOP = auto()

    # ── System (internal) completion events ─────────────────
    SYSTEM_RECORD_COMPLETE = auto()
    SYSTEM_NAV_COMPLETE = auto()
    SYSTEM_PRESS_COMPLETE = auto()
    SYSTEM_DEPTH_NOT_REACHED = auto()
    SYSTEM_DEPTH_REACHED = auto()
    SYSTEM_SUCTION_COMPLETE = auto()
    SYSTEM_CONVEY_COMPLETE = auto()
    SYSTEM_BIN_OPENED = auto()
    SYSTEM_NEXT_DEPTH = auto()
    SYSTEM_NEXT_POINT = auto()
    SYSTEM_ALL_DONE = auto()
    SYSTEM_RETURN_COMPLETE = auto()
    SYSTEM_START_EXTRACTION = auto()
    SYSTEM_PIPE_EXTRACTED = auto()
    SYSTEM_PIPE_RELEASED = auto()


# Type alias for state-change callbacks
#   callback(previous_state, new_state, triggering_action)
StateCallback = Callable[
    [SamplingState, SamplingState, Optional[SamplingAction]], None
]


# ── State Machine ─────────────────────────────────────────────


class SamplingStateMachine:
    """Finite state machine for sampling and removing pipes at each point.

    Typical usage::

        fsm = SamplingStateMachine(total_waypoints=3, max_depth=3)
        fsm.on_state_change = my_callback

        # User clicks "已就绪"
        fsm.transition(SamplingAction.CONFIRM_READY)

        # System finishes recording start position
        fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)

        # … continue through all 15 steps …
    """

    def __init__(
        self,
        total_waypoints: int = 1,
        max_depth: int = 3,
    ) -> None:
        # ── State ───────────────────────────────────────────
        self._state: SamplingState = SamplingState.INIT
        self._paused: bool = False

        # ── Callback ────────────────────────────────────────
        self._on_state_change: Optional[StateCallback] = None

        # ── Waypoint / depth / pipe tracking ────────────────
        self.total_waypoints: int = total_waypoints
        self.current_waypoint_index: int = 0  # 0-based
        self.max_depth: int = max_depth
        self.current_depth_index: int = 0  # 0-based
        self.current_pipe_index: int = 0  # 0-based count
        self._depth_targets: list[float] = []  # e.g. [0.5, 1.5, 2.5]

        # ── Transition table ────────────────────────────────
        # Maps (current_state, action) -> next_state
        self._transitions: dict[
            tuple[SamplingState, SamplingAction], SamplingState
        ] = {
            # Step 1 -> 2: User confirms ready
            (SamplingState.INIT, SamplingAction.CONFIRM_READY):
                SamplingState.RECORD_START,
            # Step 2 -> 3: System records start position
            (SamplingState.RECORD_START, SamplingAction.SYSTEM_RECORD_COMPLETE):
                SamplingState.NAVIGATE_TO_POINT,
            # Step 3 -> 4: System finishes navigation
            (SamplingState.NAVIGATE_TO_POINT, SamplingAction.SYSTEM_NAV_COMPLETE):
                SamplingState.ARRIVED_PROMPT,
            # Step 4 -> 5: User confirms pipe connected
            (SamplingState.ARRIVED_PROMPT, SamplingAction.CONFIRM_READY):
                SamplingState.PRESS_AND_SUCTION,
            # Step 5 -> 6: System completes 1m press (more pipes needed)
            (SamplingState.PRESS_AND_SUCTION, SamplingAction.SYSTEM_PRESS_COMPLETE):
                SamplingState.ADD_PIPE_PROMPT,
            # Step 5 -> 8: Target depth reached after press (skip add-pipe prompt)
            (SamplingState.PRESS_AND_SUCTION, SamplingAction.SYSTEM_DEPTH_REACHED):
                SamplingState.DISCHARGE_WASTE,
            # Step 6 -> 7: User confirms pipe added
            (SamplingState.ADD_PIPE_PROMPT, SamplingAction.CONFIRM_PIPE_ADDED):
                SamplingState.REPEAT_UNTIL_DEPTH,
            # Step 7 -> 5 loop: Depth not yet reached
            (SamplingState.REPEAT_UNTIL_DEPTH, SamplingAction.SYSTEM_DEPTH_NOT_REACHED):
                SamplingState.PRESS_AND_SUCTION,
            # Step 7 -> 8: Target depth reached
            (SamplingState.REPEAT_UNTIL_DEPTH, SamplingAction.SYSTEM_DEPTH_REACHED):
                SamplingState.DISCHARGE_WASTE,
            # Waste discharged -> open the current depth's bin first
            (SamplingState.DISCHARGE_WASTE, SamplingAction.CONFIRM_WASTE_DISCHARGED):
                SamplingState.OPEN_BIN,
            # Step 9 -> 10: System completes formal sampling (2 min suction)
            (SamplingState.FORMAL_SAMPLING, SamplingAction.SYSTEM_SUCTION_COMPLETE):
                SamplingState.CONVEY_1,
            # Safety controls during FORMAL_SAMPLING (step 9)
            # PAUSE/RESUME handled specially (stay in FORMAL_SAMPLING);
            # STOP handled specially (→ STOPPED).
            # These are NOT in the table — handled before lookup.
            # Completion only after bin closing and conveyor shutdown
            (SamplingState.CONVEY_1, SamplingAction.SYSTEM_CONVEY_COMPLETE):
                SamplingState.CONVEY_DONE,
            # Bin opened -> retain the formal suction stage
            (SamplingState.OPEN_BIN, SamplingAction.SYSTEM_BIN_OPENED):
                SamplingState.FORMAL_SAMPLING,
            # Step 12 -> 13: User confirms grain collection done
            (SamplingState.CONVEY_DONE, SamplingAction.CONFIRM_DONE):
                SamplingState.NEXT_CHECK,
            # Step 13 -> 5: Same point, next depth level
            (SamplingState.NEXT_CHECK, SamplingAction.SYSTEM_NEXT_DEPTH):
                SamplingState.PRESS_AND_SUCTION,
            # Point complete: remove every pipe before navigation/return.
            (SamplingState.NEXT_CHECK, SamplingAction.SYSTEM_START_EXTRACTION):
                SamplingState.EXTRACT_PIPE,
            (SamplingState.EXTRACT_PIPE, SamplingAction.SYSTEM_PIPE_EXTRACTED):
                SamplingState.PIPE_SUPPORT_PROMPT,
            (SamplingState.PIPE_SUPPORT_PROMPT, SamplingAction.CONFIRM_PIPE_SUPPORTED):
                SamplingState.RELEASE_PIPE,
            (SamplingState.RELEASE_PIPE, SamplingAction.SYSTEM_PIPE_RELEASED):
                SamplingState.REMOVE_PIPE_PROMPT,
            (SamplingState.REMOVE_PIPE_PROMPT, SamplingAction.CONFIRM_PIPE_REMOVED):
                SamplingState.EXTRACT_PIPE,
            # These exits are valid only after the final removal confirmation.
            (SamplingState.EXTRACT_PIPE, SamplingAction.SYSTEM_NEXT_POINT):
                SamplingState.NAVIGATE_TO_POINT,
            (SamplingState.EXTRACT_PIPE, SamplingAction.SYSTEM_ALL_DONE):
                SamplingState.ALL_DONE_PROMPT,
            # Step 14 -> 15: User confirms return
            (SamplingState.ALL_DONE_PROMPT, SamplingAction.CONFIRM_RETURN):
                SamplingState.RETURN,
            # Step 15 -> COMPLETED: Robot returns to start
            (SamplingState.RETURN, SamplingAction.SYSTEM_RETURN_COMPLETE):
                SamplingState.COMPLETED,
        }

    # ── Properties ──────────────────────────────────────────

    @property
    def current_state(self) -> SamplingState:
        return self._state

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def is_running(self) -> bool:
        """``True`` while the machine is in a non-terminal state."""
        return self._state not in (
            SamplingState.STOPPED,
            SamplingState.COMPLETED,
        )

    @property
    def stop_reason(self) -> str:
        """Human-readable reason for the most recent forced stop."""
        return getattr(self, "_stop_reason", "")

    def set_stop_reason(self, reason: str) -> None:
        """Record a failure reason before transitioning to ``STOPPED``."""
        self._stop_reason = str(reason)

    @property
    def progress_str(self) -> str:
        """Human-readable progress summary.

        Examples::

            点位 1/3 | 深度 2/3
            点位 2/3 | 深度 2/3 | 管节 3/5
        """
        parts = [
            f"点位 {self.current_waypoint_index + 1}/{self.total_waypoints}",
            f"深度 {self.current_depth_index + 1}/{self.max_depth}",
        ]
        if self.current_pipe_index > 0:
            parts.append(f"管节 {self.current_pipe_index}/{self.total_pipes_needed}")
        return " | ".join(parts)

    @property
    def total_pipes_needed(self) -> int:
        """How many 1 m pipe sections are needed for the current depth target.

        Returns ``1`` when ``_depth_targets`` is empty or the current index
        is out of range.
        """
        if not self._depth_targets:
            return 1
        idx = max(0, min(self.current_depth_index, len(self._depth_targets) - 1))
        depth_m = self._depth_targets[idx]
        return max(1, int(math.ceil(depth_m / 1.0)))

    def set_depth_targets(self, targets: list[float]) -> None:
        """Configure the depth targets (in metres) for each depth level.

        Example::

            fsm.set_depth_targets([0.5, 1.5, 2.5])
        """
        self._depth_targets = list(targets)

    def get_target_depth(self) -> float:
        """Return the current depth target in metres for the active depth level.

        Returns ``0.0`` when ``_depth_targets`` is empty or the current index
        is out of range.
        """
        if not self._depth_targets:
            return 0.0
        idx = max(0, min(self.current_depth_index, len(self._depth_targets) - 1))
        return self._depth_targets[idx]

    # ── Callback ────────────────────────────────────────────

    @property
    def on_state_change(self) -> Optional[StateCallback]:
        return self._on_state_change

    @on_state_change.setter
    def on_state_change(self, callback: Optional[StateCallback]) -> None:
        self._on_state_change = callback

    # ── Core transition ─────────────────────────────────────

    def transition(self, action: SamplingAction) -> None:
        """Execute a state transition triggered by *action*.

        Args:
            action: The action triggering the transition.

        Raises:
            ValueError: If the transition is not valid from the current
                state, or if PAUSE/RESUME/STOP is used outside of
                ``FORMAL_SAMPLING``.
        """
        prev_state = self._state
        key = (self._state, action)

        # ── Special handling for safety controls (PAUSE / RESUME / STOP) ─
        if action in (SamplingAction.PAUSE, SamplingAction.RESUME, SamplingAction.STOP):
            if action == SamplingAction.STOP:
                # STOP is valid from any non-terminal state
                if self._state in (SamplingState.INIT, SamplingState.STOPPED):
                    raise ValueError(
                        f"Safety control '{action.name}' is not valid for "
                        f"state {self._state.name}"
                    )
                next_state = SamplingState.STOPPED
                self._state = next_state
                self._emit(prev_state, next_state, action)
                return
            # PAUSE / RESUME only valid during FORMAL_SAMPLING
            if self._state != SamplingState.FORMAL_SAMPLING:
                raise ValueError(
                    f"Safety control '{action.name}' is only valid during "
                    f"FORMAL_SAMPLING (current: {self._state.name})"
                )
                next_state = SamplingState.STOPPED
                self._state = next_state
                self._emit(prev_state, next_state, action)
                return
            # PAUSE / RESUME: toggle flag, stay in same state
            self._paused = action == SamplingAction.PAUSE
            self._emit(prev_state, self._state, action)
            return

        # ── Look up next state ──────────────────────────────
        next_state = self._transitions.get(key)
        if next_state is None:
            raise ValueError(
                f"Invalid transition: cannot apply '{action.name}' "
                f"while in state '{self._state.name}'"
            )

        if (action in (SamplingAction.SYSTEM_NEXT_POINT, SamplingAction.SYSTEM_ALL_DONE)
                and self.current_pipe_index != 0):
            raise ValueError("取样管尚未全部取出，禁止离开当前点位")
        if action == SamplingAction.CONFIRM_PIPE_REMOVED and self.current_pipe_index <= 0:
            raise ValueError("没有待取出的管节")

        # Update tracking metadata before changing state
        self._update_tracking(prev_state, action, next_state)

        self._state = next_state
        self._emit(prev_state, next_state, action)

    def _update_tracking(
        self,
        prev: SamplingState,
        action: SamplingAction,
        next_: SamplingState,
    ) -> None:
        """Update waypoint/depth/pipe indices based on the transition."""

        # First pipe press — pipe count incremented by orchestrator._handle_press_and_suction
        if (
            prev == SamplingState.ARRIVED_PROMPT
            and action == SamplingAction.CONFIRM_READY
        ):
            pass  # pipe_index incremented in _handle_press_and_suction

        # Repeated pipe press (depth target not yet met)
        elif (
            prev == SamplingState.REPEAT_UNTIL_DEPTH
            and action == SamplingAction.SYSTEM_DEPTH_NOT_REACHED
        ):
            pass  # pipe_index incremented in _handle_press_and_suction

        # Depth target reached at this press — just record, depth advance handled by NEXT_CHECK
        elif (
            prev in (SamplingState.REPEAT_UNTIL_DEPTH, SamplingState.PRESS_AND_SUCTION)
            and action == SamplingAction.SYSTEM_DEPTH_REACHED
        ):
            pass  # depth_index incremented by SYSTEM_NEXT_DEPTH in NEXT_CHECK

        # Advance to the next depth level at the same waypoint
        elif (
            prev == SamplingState.NEXT_CHECK
            and action == SamplingAction.SYSTEM_NEXT_DEPTH
        ):
            self.current_depth_index += 1
            # pipe_index NOT reset — pipes accumulate across depths

        # Advance to the next waypoint (different point in same warehouse)
        elif (
            prev == SamplingState.EXTRACT_PIPE
            and action == SamplingAction.SYSTEM_NEXT_POINT
        ):
            self.current_waypoint_index += 1
            self.current_depth_index = 0
            self.current_pipe_index = 0

        elif action == SamplingAction.CONFIRM_PIPE_REMOVED:
            self.current_pipe_index -= 1

    def _emit(
        self,
        prev: SamplingState,
        current: SamplingState,
        action: Optional[SamplingAction],
    ) -> None:
        """Fire the state-change callback if one is registered."""
        if self._on_state_change is not None:
            self._on_state_change(prev, current, action)

    # ── Safety convenience methods ──────────────────────────

    def pause(self) -> None:
        """Pause the formal sampling process (step 9 only)."""
        self.transition(SamplingAction.PAUSE)

    def resume(self) -> None:
        """Resume a paused formal sampling (step 9 only)."""
        self.transition(SamplingAction.RESUME)

    def stop(self) -> None:
        """Emergency-stop the formal sampling (step 9 only).

        Transitions to the ``STOPPED`` terminal state.
        """
        self.transition(SamplingAction.STOP)

    # ── Reset ───────────────────────────────────────────────

    def reset(self) -> None:
        """Return the state machine to ``INIT``, clearing all state."""
        prev = self._state
        self._state = SamplingState.INIT
        self._paused = False
        self.current_waypoint_index = 0
        self.current_depth_index = 0
        self.current_pipe_index = 0
        self._depth_targets = []
        self._emit(prev, self._state, None)
