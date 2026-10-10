"""CH5 夹紧联动时序测试（Task 9）。

覆盖 orchestrator 在 mechanism 已接线（``_mechanism_connected=True``）
时的完整时序：

1. 正常下压循环：clamp → press → unclamp → lift → clamp → start_suction
   （下压前先夹紧，下压到底松开，回顶后重新夹紧，再启动吸粮）
2. 加管分支（用户 CONFIRM_PIPE_ADDED 后进入 REPEAT_UNTIL_DEPTH）：
   tighten → unclamp → clamp，然后回到下压循环
3. 动作时长来自品种配置（set_grain("稻谷") 后按稻谷参数等待）

全程使用 mock bridge（记录调用顺序且恒成功），不触碰真实硬件，
也不做真实 sleep（``_wait_interruptible`` 替换为记录时长的即时返回）。
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

import grain_sampling_workflow.orchestrator as orch_module
from grain_sampling_workflow.mechanism_config import DEFAULT_GRAIN_PARAMS
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.state_machine import (
    SamplingState,
    SamplingStateMachine,
)


class RecordingBridge:
    """记录机构调用顺序的假 bridge，所有调用恒成功。"""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def _record(self, name: str) -> bool:
        self.calls.append(name)
        return True

    def call_clamp(self) -> bool:
        return self._record("clamp")

    def call_lift_health(self) -> bool:
        return self._record("lift_health")

    def call_unclamp(self) -> bool:
        return self._record("unclamp")

    def call_tighten(self) -> bool:
        return self._record("tighten")

    def call_untighten(self) -> bool:
        return self._record("untighten")

    def call_press(self) -> bool:
        return self._record("press")

    def call_lift(self) -> bool:
        return self._record("lift")

    def call_move_lift(self, direction: str, distance_cm: float) -> bool:
        return self._record(f"move_lift:{direction}:{distance_cm:.0f}")

    def call_start_suction(self) -> bool:
        return self._record("start_suction")

    def call_stop_suction(self) -> bool:
        return self._record("stop_suction")

    def call_emergency_stop(self) -> bool:
        return self._record("emergency_stop")

    def call_set_grain(self, grain: str) -> bool:
        return self._record("set_grain")

    def call_navigate(self, x: float = 0.0, y: float = 0.0) -> bool:
        return self._record("navigate")

    def call_convey(self) -> bool:
        return self._record("convey")

    def call_open_bin(self, depth_level: int = 0) -> bool:
        return self._record("open_bin")

    def call_close_bin(self, depth_level: int = 0) -> bool:
        return self._record("close_bin")

    def record_start_position(self):
        return (0.0, 0.0)


def _make_orch(
    bridge: RecordingBridge,
    depth_target: float = 2.0,
    state: SamplingState = SamplingState.PRESS_AND_SUCTION,
):
    """构造已接线 mechanism 的 orchestrator + 相关记录器。

    - ``_wait_interruptible``：记录每次等待时长并立即成功（不做真实 sleep）
    - ``_run_async``：no-op，避免后台线程破坏断言确定性
    - ``_log_sampling_event``：no-op，避免测试时写日志文件
    """
    fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
    fsm.set_depth_targets([depth_target])
    fsm._state = state
    orch = WorkflowOrchestrator(fsm, bridge, cloud_client=MagicMock())
    orch.set_mechanism_connected(True)
    orch._grain = "稻谷"
    durations: list[float] = []
    orch._wait_interruptible = (
        lambda d, poll_sec=0.25: durations.append(float(d)) or True
    )
    orch._run_async = lambda func: None
    orch._log_sampling_event = lambda action, detail: None
    return orch, fsm, durations


# ── 1. 正常下压循环 ────────────────────────────────────────────────────────


def test_press_cycle_full_call_order():
    """下压循环调用顺序：clamp → press → unclamp → lift → clamp → start_suction。"""
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(bridge, depth_target=2.0)

    orch._handle_press_and_suction()

    assert bridge.calls == [
        "lift_health", "clamp", "move_lift:down_cycle:20", "unclamp", "move_lift:return:20",
        "clamp", "start_suction",
    ]
    # 2.0m 需要 2 节管：第 1 节压完 → 等待加管（ADD_PIPE_PROMPT）
    assert fsm.current_state == SamplingState.ADD_PIPE_PROMPT


def test_press_cycle_reattaches_clamp_when_depth_reached():
    """单节管即达目标深度时，循环仍完整执行，再夹紧后再进 DISCHARGE_WASTE。"""
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(bridge, depth_target=1.0)

    orch._handle_press_and_suction()

    assert bridge.calls == [
        "lift_health", "clamp", "move_lift:down_cycle:20", "unclamp", "move_lift:return:20",
        "clamp", "start_suction",
    ]
    assert fsm.current_state == SamplingState.DISCHARGE_WASTE


# ── 2. 加管分支 ─────────────────────────────────────────────────────────────


def test_add_pipe_branch_tighten_unclamp_clamp():
    """CONFIRM_PIPE_ADDED 后（REPEAT_UNTIL_DEPTH）：tighten → unclamp → clamp。"""
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(
        bridge, depth_target=2.0, state=SamplingState.REPEAT_UNTIL_DEPTH
    )
    fsm.current_pipe_index = 1  # 已压 1 节，仍需 1 节

    # 用户点击"已完成加管" → ADD_PIPE_PROMPT → REPEAT_UNTIL_DEPTH，触发本 handler
    orch._handle_repeat_until_depth()

    assert bridge.calls == ["tighten", "unclamp", "clamp"]
    # 加管时序完成后回到下压循环
    assert fsm.current_state == SamplingState.PRESS_AND_SUCTION


def test_add_pipe_branch_skipped_when_depth_reached():
    """深度已到：REPEAT_UNTIL_DEPTH 不再执行加管时序，直接 DISCHARGE_WASTE。"""
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(
        bridge, depth_target=1.0, state=SamplingState.REPEAT_UNTIL_DEPTH
    )
    fsm.current_pipe_index = 1

    orch._handle_repeat_until_depth()

    assert bridge.calls == []
    assert fsm.current_state == SamplingState.DISCHARGE_WASTE


def test_add_pipe_then_press_does_not_clamp_twice():
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(
        bridge, depth_target=2.0, state=SamplingState.REPEAT_UNTIL_DEPTH
    )
    fsm.current_pipe_index = 1
    orch._handle_repeat_until_depth()
    orch._handle_press_and_suction()
    assert bridge.calls == [
        "tighten", "unclamp", "clamp", "lift_health",
        "move_lift:down_cycle:20", "unclamp", "move_lift:return:20",
        "clamp", "start_suction",
    ]
    assert fsm.current_pipe_index == 2


def test_continuing_press_uses_completed_grip():
    bridge = RecordingBridge()
    orch, fsm, _ = _make_orch(bridge, depth_target=3.0)
    orch._handle_press_and_suction()
    bridge.calls.clear()
    fsm._state = SamplingState.PRESS_AND_SUCTION
    orch._handle_press_and_suction()
    assert bridge.calls[:2] == ["lift_health", "move_lift:down_cycle:20"]
    assert bridge.calls.count("clamp") == 1  # Only after the return stroke.


# ── 3. 参数来自品种配置 ─────────────────────────────────────────────────────


def test_press_cycle_durations_from_grain_config(monkeypatch):
    """set_grain("稻谷") 后，下压循环的等待时长取自稻谷配置。"""
    bridge = RecordingBridge()
    orch, fsm, durations = _make_orch(bridge, depth_target=1.0)

    custom = dict(DEFAULT_GRAIN_PARAMS)
    custom.update({
        "clamp_duration": 2.5,
        "unclamp_duration": 1.5,
        "tighten_duration": 4.0,
        "untighten_duration": 0.5,
        "press_duration": 2.0,  # 伺服未接 → 固定延时，可独立配置
    })
    monkeypatch.setattr(
        orch_module,
        "get_grain_params",
        lambda name: custom if name == "稻谷" else DEFAULT_GRAIN_PARAMS,
    )
    orch.set_grain("稻谷")
    bridge.calls.clear()  # 清掉 set_grain 的调用记录

    orch._handle_press_and_suction()

    # 5 步等待 = [clamp, servo(press), unclamp, servo(lift), clamp(再夹紧)]
    # 每步 = 动作时长 + 停稳余量 0.5s
    assert durations == [3.5, 2.5, 2.0, 2.5, 3.5]
    assert bridge.calls == [
        "lift_health", "clamp", "move_lift:down_cycle:20", "unclamp", "move_lift:return:20",
        "clamp", "start_suction",
    ]


def test_add_pipe_durations_from_grain_config(monkeypatch):
    """加管分支的等待时长取自品种配置（tighten/unclamp/clamp）。"""
    bridge = RecordingBridge()
    orch, fsm, durations = _make_orch(
        bridge, depth_target=2.0, state=SamplingState.REPEAT_UNTIL_DEPTH
    )
    fsm.current_pipe_index = 1

    custom = dict(DEFAULT_GRAIN_PARAMS)
    custom.update({
        "clamp_duration": 2.5,
        "unclamp_duration": 1.5,
        "tighten_duration": 4.0,
    })
    monkeypatch.setattr(
        orch_module,
        "get_grain_params",
        lambda name: custom if name == "稻谷" else DEFAULT_GRAIN_PARAMS,
    )
    orch.set_grain("稻谷")
    bridge.calls.clear()

    orch._handle_repeat_until_depth()

    # 时长 = 品种配置 + 停稳余量 0.5s
    assert durations == [4.5, 2.0, 3.5]
    assert bridge.calls == ["tighten", "unclamp", "clamp"]


def test_unknown_grain_falls_back_to_defaults(monkeypatch):
    """未设置品种时回退默认时长（clamp=2s, unclamp=5s, press/lift 跟 clamp），时序仍完整。"""
    bridge = RecordingBridge()
    orch, fsm, durations = _make_orch(bridge, depth_target=1.0)
    orch._grain = ""  # 品种未知 → DEFAULT_GRAIN_PARAMS

    orch._handle_press_and_suction()

    # MCU clamp 3.0 + 停稳余量 0.5 = 3.5；servo(press/lift) 回退 clamp=2.0 → 2.5；
    # unclamp 5.0 + 0.5 = 5.5
    assert durations == [3.5, 2.5, 5.5, 2.5, 3.5]
    assert bridge.calls == [
        "lift_health", "clamp", "move_lift:down_cycle:20", "unclamp", "move_lift:return:20",
        "clamp", "start_suction",
    ]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
