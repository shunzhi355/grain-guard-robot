"""Exercise the desktop task handler and both navigation modes through removal.

Only widgets, daemon replies, cloud calls and elapsed time are replaced. The
task handler, FSM, orchestrator and RobotBridge are the production code.
"""
import ast
from collections import deque
import logging
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.robot_bridge import RobotBridge
from grain_sampling_workflow.state_machine import SamplingAction as A, SamplingState as S, SamplingStateMachine


@pytest.mark.parametrize("operator", [False, True], ids=["formal-navigation", "test-navigation"])
@pytest.mark.parametrize("grain,sampling,convey", [("稻谷", 120, 30), ("玉米", 180, 60), ("黄豆", 90, 90)])
@pytest.mark.parametrize("depths", [[2.0], [1.0, 2.0], [2.0, 2.0]],
                         ids=["single-depth", "two-depths", "duplicate-depths"])
def test_desktop_task_removes_all_pipes_before_next_navigation(monkeypatch, operator, grain, sampling, convey, depths):
    commands, goals, removed, supported = [], [], [], []
    active_goal = {}
    flow = {}

    class Client:
        def request(self, action, **values):
            if action == "status":
                return {"ok": True, "runtime": "lenovo-local", "lenovo_online": True,
                        "test_navigation_mode": operator, "navigation": active_goal,
                        "pose": {"x_m": -1, "y_m": -1, "localization_valid": True},
                        "slam": {"map_id": "test-map", "map_version": 1, "frame": "camera_init"}}
            if action in ("goal", "test_goal"):
                assert action == ("test_goal" if operator else "goal")
                fsm = flow["fsm"]
                assert fsm.current_pipe_index == 0
                # First point, next point, return: two explicit removals per point.
                assert len(removed) == len(goals) * 2
                goals.append(values)
                active_goal.update(goal_id=str(len(goals)), result="SUCCEEDED", test_only=operator)
                return {"ok": True, "goal_id": active_goal["goal_id"]}
            assert action == "mechanism"
            commands.append((values["name"], values.get("args", {})))
            return {"ok": True}

    client = Client()
    monkeypatch.setattr("grain_sampling_workflow.robot_bridge.RobotClient", lambda: client)
    monkeypatch.setenv("GRAIN_LOCAL_RUNTIME", "1")
    monkeypatch.setenv("GRAIN_LOCAL_NAV_MODE", "operator" if operator else "autonomous")
    monkeypatch.setenv("GRAIN_SAMPLING_UI_ENABLE_MECHANISM", "1")
    monkeypatch.setenv("GRAIN_SAMPLING_UI_FAKE_NAVIGATION", "0")
    monkeypatch.setenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", "0")

    # Execute the actual task-selected method without constructing Qt windows.
    path = Path(__file__).resolve().parents[1] / "src/grain_sampling_ui/main.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MainWindow")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_on_task_selected")
    namespace = {"os": os, "logger": logging.getLogger(__name__),
                 "_env_flag": lambda name: os.getenv(name) == "1",
                 "SamplingStateMachine": SamplingStateMachine,
                 "SamplingBridge": lambda: RobotBridge(client=client),
                 "WorkflowOrchestrator": WorkflowOrchestrator,
                 "CloudHttpClient": MagicMock(), "AppConfig": MagicMock(),
                 "save_current_task": MagicMock()}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    window = SimpleNamespace(_guidance_page=MagicMock(), _slam_bridge=None,
                             _alarm_bar=MagicMock(), page_manager=MagicMock())
    namespace["_on_task_selected"](window, {
        "id": "extraction-regression", "source": "local", "grain_type": grain,
        "depth_list": depths, "waypoints": [{"x": 1, "y": 0}, {"x": 2, "y": 0}],
    })
    window._alarm_bar.set_alarm.assert_not_called()
    fsm, orch = window._guidance_page.attach_orchestrator.call_args.args
    flow["fsm"] = fsm
    assert fsm.max_depth == len(set(depths))
    assert orch.sampling_duration_sec == sampling
    assert orch.convey_duration_sec == convey
    queue = deque()
    orch._run_async = queue.append
    orch._wait_interruptible = lambda *_: fsm.is_running
    orch._log_sampling_event = lambda *_: None
    orch._handle_task_completed = lambda: None
    prompts = {S.INIT: A.CONFIRM_READY, S.ARRIVED_PROMPT: A.CONFIRM_READY,
               S.ADD_PIPE_PROMPT: A.CONFIRM_PIPE_ADDED, S.DISCHARGE_WASTE: A.CONFIRM_WASTE_DISCHARGED,
               S.CONVEY_DONE: A.CONFIRM_DONE, S.PIPE_SUPPORT_PROMPT: A.CONFIRM_PIPE_SUPPORTED,
               S.REMOVE_PIPE_PROMPT: A.CONFIRM_PIPE_REMOVED, S.ALL_DONE_PROMPT: A.CONFIRM_RETURN}
    for _ in range(150):
        if fsm.current_state == S.COMPLETED:
            break
        if queue:
            queue.popleft()()
            continue
        if fsm.current_state == S.PIPE_SUPPORT_PROMPT:
            supported.append(fsm.current_pipe_index)
            # At the bottom: clamp must precede ascent, in either navigation mode.
            assert commands[-3:] == [
                ("move_lift", {"direction": "extract_prepare", "distance_cm": 20}),
                ("clamp", {}), ("move_lift", {"direction": "extract", "distance_cm": 20}),
            ]
        if fsm.current_state == S.REMOVE_PIPE_PROMPT:
            removed.append(fsm.current_pipe_index)
            assert commands[-1] == ("unclamp", {})
            if fsm.current_pipe_index > 1:
                assert commands[-2] == ("untighten", {})
        assert fsm.current_state in prompts, fsm.stop_reason
        fsm.transition(prompts[fsm.current_state])
    assert fsm.current_state == S.COMPLETED
    assert supported == removed == [2, 1, 2, 1]
    assert len(goals) == 3  # two points, then return only after all removal confirmations
    assert sum(name == "hold_bin_open" for name, _ in commands) == 2 * len(set(depths))
