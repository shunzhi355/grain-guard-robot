"""Exercise the isolated descent preview with a real installed Qt binding."""
import importlib.util
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("robot_ui_launcher", ROOT / "scripts/start_ui.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


@pytest.fixture(scope="module")
def qt_app():
    try:
        binding = launcher.load_qt()
    except RuntimeError as exc:
        pytest.skip(str(exc))
    from PySide2.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    return app, binding


def test_uses_installed_qt_instead_of_circular_project_shims(qt_app):
    _, binding = qt_app
    assert ROOT / "src" not in Path(binding.__file__).resolve().parents
    assert sys.modules["PySide2.QtWidgets"] is sys.modules["PySide6.QtWidgets"]


def test_adds_generated_ros_service_path_from_configured_workspace(tmp_path, monkeypatch):
    generated = tmp_path / "devel" / "lib" / "python3" / "dist-packages"
    generated.mkdir(parents=True)
    monkeypatch.setenv("MECHANISM_WS_SETUP", str(tmp_path / "devel" / "setup.bash"))
    monkeypatch.setattr(Path, "home", staticmethod(lambda: tmp_path / "unused-home"))
    monkeypatch.setattr(sys, "path", list(sys.path))

    added = launcher.add_ros_workspace_pythonpath()

    assert added == [generated.resolve()]
    assert sys.path[0] == str(generated.resolve())


def test_descent_preview_never_constructs_bridge_or_orchestrator(qt_app, monkeypatch):
    app, _ = qt_app
    from grain_sampling_ui.pages import guidance_page
    from grain_sampling_workflow.state_machine import SamplingState

    def forbidden(*args, **kwargs):
        pytest.fail("UI-only preview attempted to construct a ROS bridge or hardware workflow")

    monkeypatch.setattr(guidance_page, "SamplingBridge", forbidden)
    monkeypatch.setattr(guidance_page, "WorkflowOrchestrator", forbidden)
    window = launcher.build_descent_preview()
    try:
        window.show()
        app.processEvents()
        assert window.preview_fsm.current_state == SamplingState.ARRIVED_PROMPT
        assert window.preview_page._orchestrator is None
        window.preview_page._action_buttons["CONFIRM_READY"].click()
        app.processEvents()
        assert window.preview_fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert "未发送" in window.preview_page._detail.text()
        assert window.preview_page._orchestrator is None
    finally:
        window.close()
        app.processEvents()
