"""The 100 ms local safety heartbeat must not repaint the UI at 10 Hz."""

import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock


def test_local_status_heartbeat_coalesces_unchanged_qt_signals():
    source = Path(__file__).resolve().parents[1] / "src/grain_sampling_ui/robot_event_thread.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == "RobotEventThread")
    run = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name == "run")
    clock = [0.0]
    namespace = {
        "os": SimpleNamespace(getenv=lambda _: "1"),
        "time": SimpleNamespace(monotonic=lambda: clock[0]),
        "math": __import__("math"),
        "chassis_safety_message": lambda _: None,
    }
    exec(compile(ast.Module(body=[run], type_ignores=[]), str(source), "exec"), namespace)
    status = {"runtime": "lenovo-local", "simulation": False,
              "test_navigation_mode": True, "lenovo_online": False,
              "chassis": {"rc_mode": "auto"}, "pose": None}
    thread = SimpleNamespace(
        _running=True, _connected=False, _last_error="", _last_safety_message=None,
        _last_local_view=None, _last_rc_mode=None, _last_rc_emit_at=float("-inf"),
        client=SimpleNamespace(request=MagicMock(return_value=status)),
        connection_changed=SimpleNamespace(emit=MagicMock()),
        local_runtime_updated=SimpleNamespace(emit=MagicMock()),
        rc_mode_updated=SimpleNamespace(emit=MagicMock()),
        safety_status_updated=SimpleNamespace(emit=MagicMock()),
        odometry_updated=SimpleNamespace(emit=MagicMock()),
        error=SimpleNamespace(emit=MagicMock()),
        _local_preview=lambda: None,
    )
    calls = [0]

    def advance(_):
        calls[0] += 1
        clock[0] += 0.1
        if calls[0] == 12:
            thread._running = False

    thread.msleep = advance
    namespace["run"](thread)

    assert thread.client.request.call_count == 12  # safety heartbeat unchanged
    assert thread.local_runtime_updated.emit.call_count == 1
    assert thread.rc_mode_updated.emit.call_count <= 3
    thread.connection_changed.emit.assert_called_once_with(True)
