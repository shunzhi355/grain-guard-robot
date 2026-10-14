"""The local work-order dialog must use the task043 map store.

It previously scanned a legacy host path (``~/fastlio2_ws/.../PCD``) that does
not exist on the industrial PC, so the warehouse list stayed empty and the
"暂无地图，请先建图" hint blocked every local work order.
"""
import importlib.util
import os
from pathlib import Path
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]


def _load_qt():
    """Load the real Qt binding before the PySide2 compatibility shim."""
    spec = importlib.util.spec_from_file_location("work_order_ui_launcher",
                                                  ROOT / "3588/scripts/start_ui.py")
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    launcher.load_qt()


@pytest.fixture
def dialog(monkeypatch):
    _load_qt()
    from PySide2.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    import grain_sampling_ui.pages.task_list_page as module
    monkeypatch.setenv("GRAIN_LOCAL_RUNTIME", "1")
    monkeypatch.delenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", raising=False)
    monkeypatch.setattr(module, "_local_saved_maps",
                        lambda: [{"name": "new_map_a", "map_id": "map-a"},
                                 {"name": "new_map_b", "map_id": "map-b"}])
    monkeypatch.setattr(module, "_active_runtime_map_id", lambda: "map-b")
    widget = module._CreateTaskDialog()
    yield module, widget, app
    widget.close()
    app.processEvents()


def test_saved_maps_are_listed_and_the_loaded_one_is_preselected(dialog):
    module, widget, app = dialog
    assert widget._warehouse_input.count() == 2
    assert widget._warehouse_input.currentText() == "new_map_b（当前定位图）"
    assert widget._create_btn.isEnabled()
    assert not widget._error_label.isVisible()


def test_creating_an_order_on_a_map_that_is_not_loaded_is_blocked(dialog):
    module, widget, app = dialog
    signals = []
    widget.task_created.connect(signals.append)
    widget._warehouse_input.setCurrentIndex(0)          # new_map_a, not loaded
    widget._on_create()
    assert signals == []
    assert "不是当前定位地图" in widget._error_label.text()


def test_creating_an_order_on_the_loaded_map_emits_clean_name(dialog):
    module, widget, app = dialog
    signals = []
    widget.task_created.connect(signals.append)
    widget._waypoint_list.clear()
    widget._waypoint_list.addItem("点位1: X=2.5  Y=0.5")
    widget._on_create()
    assert len(signals) == 1
    data = signals[0]
    assert data["warehouse"] == "new_map_b"              # no UI decoration
    assert data["waypoints"] == [{"x": 2.5, "y": 0.5}]
    assert data["skip_mapping"] is False


def test_without_a_loaded_map_creation_is_refused(dialog, monkeypatch):
    module, widget, app = dialog
    monkeypatch.setattr(module, "_active_runtime_map_id", lambda: "")
    signals = []
    widget.task_created.connect(signals.append)
    widget._warehouse_input.setCurrentIndex(0)
    widget._on_create()
    assert signals == []
    assert "没有加载中的定位地图" in widget._error_label.text()


def test_two_points_with_repeated_depth_sample_once_at_each_point(dialog):
    module, widget, app = dialog
    signals = []
    widget.task_created.connect(signals.append)
    widget._waypoint_list.clear()
    widget._waypoint_list.addItem("点位1: X=7.5  Y=0.7")
    widget._waypoint_list.addItem("点位2: X=10.0  Y=1.5")
    widget._depth_inputs[0].setCurrentText("2.0 米")
    widget._depth_inputs[1].setCurrentText("2.0 米")
    widget._depth_inputs[2].setCurrentText("无")
    widget._on_create()
    assert len(signals) == 1
    assert signals[0]["depth_list"] == [2.0]
    assert signals[0]["waypoints"] == [{"x": 7.5, "y": 0.7}, {"x": 10.0, "y": 1.5}]


def test_current_pose_helper_fills_xy_from_live_pose(dialog, monkeypatch):
    module, widget, app = dialog
    import grain_sampling_workflow.robot_bridge as bridge

    class Client:
        def request(self, action, **values):
            return {"pose": {"x_m": 2.96, "y_m": 0.49}}

    monkeypatch.setattr(bridge, "RobotClient", lambda *a, **k: Client())
    widget._use_current_pose()
    assert widget._x_input.value() == pytest.approx(2.96)
    assert widget._y_input.value() == pytest.approx(0.49)
