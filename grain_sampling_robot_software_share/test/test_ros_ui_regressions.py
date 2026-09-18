"""Regression tests for ROS/UI startup and service-call diagnostics."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _class_method(path: Path, class_name: str, method_name: str) -> ast.FunctionDef:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )


def _method_source(path: Path, class_name: str, method_name: str) -> str:
    method = _class_method(path, class_name, method_name)
    lines = path.read_text(encoding="utf-8").splitlines()
    return "\n".join(lines[method.lineno - 1:method.end_lineno])


def test_ros_connected_callback_is_bound_before_thread_start():
    source = _method_source(
        ROOT / "src/grain_sampling_ui/main.py", "MainWindow", "start_ros"
    )
    start_index = source.index("self._ros_thread.start()")
    connect_index = source.index("connection_changed.connect(self._on_ros_connected)")
    assert connect_index < start_index


def test_status_update_covers_ros_label_and_dot():
    source = _method_source(
        ROOT / "src/grain_sampling_ui/main.py", "MainWindow", "set_connected"
    )
    assert "self._ros_dot" in source
    assert 'self._ros_value.setText("\\u8fd0\\u884c\\u4e2d")' in source
    assert 'self._ros_value.setText("\\u672a\\u8fd0\\u884c")' in source


def test_trigger_catches_non_ros_exception_and_logs_traceback(monkeypatch, caplog):
    from grain_sampling_workflow import ros_bridge

    class FakeRospy:
        @staticmethod
        def get_name():
            return "/grain_sampling_ui"

        @staticmethod
        def wait_for_service(_name, timeout):
            assert timeout > 0

        @staticmethod
        def ServiceProxy(_name, _type):
            def fail():
                raise RuntimeError("transport closed")
            return fail

    monkeypatch.setattr(ros_bridge, "HAS_ROS", True)
    monkeypatch.setattr(ros_bridge, "rospy", FakeRospy, raising=False)
    bridge = ros_bridge.SamplingBridge.__new__(ros_bridge.SamplingBridge)

    with caplog.at_level("INFO"):
        assert bridge._call_trigger("/mechanism/clamp") is False

    assert "SERVICE_CALL_BEGIN service=/mechanism/clamp" in caplog.text
    assert "SERVICE_WAIT_OK service=/mechanism/clamp" in caplog.text
    assert "SERVICE_CALL_EXCEPTION service=/mechanism/clamp" in caplog.text


def test_ros_thread_emits_explicit_init_result():
    source = _method_source(
        ROOT / "src/grain_sampling_ui/ros_thread.py", "ROSNodeThread", "run"
    )
    assert "connected = self._worker.init_node()" in source
    assert "self.connection_changed.emit(connected)" in source
