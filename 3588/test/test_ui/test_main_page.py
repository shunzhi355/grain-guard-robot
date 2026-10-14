"""Unit tests for the main operation page and its widgets."""

from __future__ import annotations

import pytest
from unittest.mock import MagicMock, patch

from PySide6.QtCore import Qt, Signal, QPointF
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtGui import QPainter


# ── Module-level app (shared across tests) ─────────────────
@pytest.fixture(scope="module")
def qapp() -> QApplication:
    """Provide a QApplication instance for the test module."""
    import sys
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app


# ═══════════════════════════════════════════════════════════════
# ControlPanel tests
# ═══════════════════════════════════════════════════════════════


class TestControlPanel:
    """Tests for the ControlPanel widget."""

    def test_instantiation(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        assert panel is not None
        assert panel.objectName() == "control_panel"
        assert panel.width() == 170

    def test_signals_exist(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        # Verify all five signals are present
        assert hasattr(panel, "start_sampling")
        assert hasattr(panel, "pause_sampling")
        assert hasattr(panel, "emergency_stop")
        assert hasattr(panel, "return_to_charge")
        assert hasattr(panel, "manual_control")

    def test_start_sampling_signal_emitted(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        received: list = []

        panel.start_sampling.connect(lambda: received.append(True))
        panel.start_sampling.emit()

        assert len(received) == 1
        assert received[0] is True

    def test_pause_sampling_signal_emitted(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        received: list = []

        panel.pause_sampling.connect(lambda: received.append(True))
        panel.pause_sampling.emit()

        assert len(received) == 1

    def test_emergency_stop_signal_emitted(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        received: list = []

        panel.emergency_stop.connect(lambda: received.append(True))
        panel.emergency_stop.emit()

        assert received == [True]

    def test_return_to_charge_signal_emitted(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        received: list = []

        panel.return_to_charge.connect(lambda: received.append(True))
        panel.return_to_charge.emit()

        assert len(received) == 1

    def test_manual_control_signal_emitted(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel

        panel = ControlPanel()
        received: list = []

        panel.manual_control.connect(lambda: received.append(True))
        panel.manual_control.emit()

        assert len(received) == 1

    def test_button_texts_correct(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.control_panel import ControlPanel
        from PySide6.QtWidgets import QPushButton

        panel = ControlPanel()
        buttons = panel.findChildren(QPushButton)
        texts = {b.text() for b in buttons}

        assert "开始采样" in texts
        assert "暂停" in texts
        assert "急停" in texts
        assert "返回充电" in texts
        assert "手动遥控" in texts


# ═══════════════════════════════════════════════════════════════
# MapWidget tests
# ═══════════════════════════════════════════════════════════════


class TestMapWidget:
    """Tests for the MapWidget."""

    def test_instantiation(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        assert widget is not None
        assert widget.objectName() == "map_widget"
        assert widget.minimumWidth() >= 400
        assert widget.minimumHeight() >= 300

    def test_coord_label_exists(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        # The coord label should be a child
        labels = widget.findChildren(type(widget._coord_label))
        assert len(labels) >= 1

    def test_update_odometry_no_crash(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        # Should not raise
        widget.update_odometry(1.0, 2.0, 0.785)
        widget.update_odometry(-0.5, 3.1, -1.57)

    def test_update_map_valid_data(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        valid_map = {
            "width": 50,
            "height": 50,
            "resolution": 0.05,
            "origin_x": -1.25,
            "origin_y": -1.25,
            "cells": [0] * 2500,  # all free
        }
        # Should not raise
        widget.update_map(valid_map)

    def test_update_map_malformed_data(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        # Malformed data should not crash
        widget.update_map({"bad": "data"})

    def test_update_waypoints(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        waypoints = [
            (0.0, 0.0, "1"),
            (1.0, 2.0, "2"),
            (-1.0, 3.0, "3"),
        ]
        widget.update_waypoints(waypoints)

    def test_update_nav_path(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        path = [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]
        widget.update_nav_path(path)

    def test_reset_view(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)

        widget._zoom = 3.0
        widget._offset_x = 5.0
        widget.reset_view()

        assert widget._zoom == 1.0
        assert widget._offset_x == 0.0
        assert widget._offset_y == 0.0

    def test_world_widget_roundtrip(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)
        widget.reset_view()

        # Origin should map to centre
        pt = widget._world_to_widget(0.0, 0.0)
        wx, wy = widget._widget_to_world(pt.x(), pt.y())
        assert abs(wx) < 1e-6
        assert abs(wy) < 1e-6

    def test_paint_event_no_crash(self, qapp: QApplication) -> None:
        from grain_sampling_ui.widgets.map_widget import MapWidget

        widget = MapWidget()
        widget.resize(500, 400)
        widget.show()

        # Force a repaint — should not crash even with default data
        widget.update()
        qapp.processEvents()


# ═══════════════════════════════════════════════════════════════
# MainPage tests
# ═══════════════════════════════════════════════════════════════


class TestMainPage:
    """Tests for the MainPage."""

    def test_instantiation(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        assert page is not None
        assert page.objectName() == "main_page"

    def test_map_widget_created(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        assert page.map_widget is not None

    def test_status_labels_exist(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage
        from PySide6.QtWidgets import QLabel

        page = MainPage()
        labels = page.findChildren(QLabel)

        # Should find multiple labels for telemetry
        assert len(labels) > 5

    def test_progress_bar_exists(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage
        from PySide6.QtWidgets import QProgressBar

        page = MainPage()
        bars = page.findChildren(QProgressBar)
        assert len(bars) == 1
        assert bars[0].minimum() == 0
        assert bars[0].maximum() == 100

    def test_update_task_name(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page.update_task_name("扦样任务-001")

        assert page._lbl_task_name is not None
        assert "扦样任务-001" in page._lbl_task_name.text()

    def test_set_waypoints(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        waypoints = [(1.0, 2.0, "A"), (3.0, 4.0, "B")]
        page.set_waypoints(waypoints)

        # Should not raise
        assert page.map_widget is not None

    def test_set_nav_path(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        path = [(0.0, 0.0), (1.0, 1.0)]
        page.set_nav_path(path)

        assert page.map_widget is not None

    def test_on_odometry_updates_map(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._map_widget = MagicMock()
        page._on_odometry(1.5, 2.5, 0.3)

        page._map_widget.update_odometry.assert_called_once_with(1.5, 2.5, 0.3)

    def test_on_mechanism_status_updates_ui(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        data = {
            "sampling_depth_mm": 350,
            "negative_pressure_pa": -1200,
            "bin_weights_kg": {
                "bin1": 12.5,
                "bin2": 8.3,
                "bin3": 15.0,
            },
            "completion_pct": 75,
        }
        page._on_mechanism_status(data)

        assert page._lbl_depth is not None
        assert "350" in page._lbl_depth.text()

        assert page._lbl_pressure is not None
        assert "-1200" in page._lbl_pressure.text()

        assert page._lbl_bin1 is not None
        assert "12.5" in page._lbl_bin1.text()

        assert page._lbl_bin2 is not None
        assert "8.3" in page._lbl_bin2.text()

        assert page._lbl_bin3 is not None
        assert "15.0" in page._lbl_bin3.text()

        assert page._progress_bar is not None
        assert page._progress_bar.value() == 75

    def test_on_map_data_updates_map_widget(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._map_widget = MagicMock()

        map_data = {"width": 10, "height": 10, "cells": []}
        page._on_map_data(map_data)

        page._map_widget.update_map.assert_called_once_with(map_data)

    def test_emergency_stop_updates_display(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._on_emergency_stop()

        assert page._lbl_task_name is not None
        assert "急停" in page._lbl_task_name.text()

        assert page._progress_bar is not None
        assert page._progress_bar.value() == 0

    def test_start_sampling_updates_display(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._on_start_sampling()

        assert page._lbl_task_name is not None
        assert "采样中" in page._lbl_task_name.text()

    def test_pause_sampling_updates_display(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._on_pause_sampling()

        assert page._lbl_task_name is not None
        assert "已暂停" in page._lbl_task_name.text()

    def test_return_to_charge_updates_display(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._on_return_to_charge()

        assert page._lbl_task_name is not None
        assert "返回充电" in page._lbl_task_name.text()

    def test_manual_control_updates_display(self, qapp: QApplication) -> None:
        from grain_sampling_ui.pages.main_page import MainPage

        page = MainPage()
        page._on_manual_control()

        assert page._lbl_task_name is not None
        assert "手动遥控" in page._lbl_task_name.text()


# ═══════════════════════════════════════════════════════════════
# TaskSelection tests (T10 — _on_task_selected accepts dict)
# ═══════════════════════════════════════════════════════════════


class TestTaskSelection:
    """Verify that ``_on_task_selected`` handles dict payload (T10)."""

    def test_signature_accepts_dict_not_str(self) -> None:
        """_on_task_selected parameter is named task_data and typed dict."""
        import inspect
        import typing
        from grain_sampling_ui.main import MainWindow

        hints = typing.get_type_hints(MainWindow._on_task_selected)
        assert "task_data" in hints, f"expected 'task_data' param, got {list(hints.keys())}"
        assert hints["task_data"] == dict, (
            f"expected dict annotation, got {hints['task_data']}"
        )

        sig = inspect.signature(MainWindow._on_task_selected)
        params = list(sig.parameters.values())
        assert len(params) >= 2, "expected self + task_data param"
        assert params[1].name == "task_data", (
            f"expected 'task_data', got '{params[1].name}'"
        )

    def test_handles_dict_payload_without_error(self, qapp: QApplication) -> None:
        """Calling _on_task_selected with a dict payload does not raise."""
        from unittest.mock import MagicMock, patch
        from grain_sampling_ui.main import MainWindow

        # ── Mock all page/widget imports that MainWindow.__init__ needs ──
        with patch("grain_sampling_ui.main.StatusBar"), \
             patch("grain_sampling_ui.main.ControlPanel"), \
             patch("grain_sampling_ui.main.MainPage"), \
             patch("grain_sampling_ui.main.GuidancePage"), \
             patch("grain_sampling_ui.main.MapPage"), \
             patch("grain_sampling_ui.main.TaskListPage"), \
             patch("grain_sampling_ui.main.MappingPage"), \
             patch("grain_sampling_ui.main.SettingsPage"), \
             patch("grain_sampling_ui.main.AlarmBar"), \
             patch("grain_sampling_ui.main.PageManager"), \
             patch("grain_sampling_ui.main.WorkflowOrchestrator"), \
             patch("grain_sampling_ui.main.SamplingBridge"), \
             patch("grain_sampling_ui.main.SamplingStateMachine"), \
             patch("grain_sampling_ui.main.OrderInfo"):

            win = MainWindow()

            # ── Replace task_list / guidance / page_manager with simple mocks ──
            win._task_list_page = MagicMock()
            win._guidance_page = MagicMock()
            win.page_manager = MagicMock()
            win._slam_bridge = None

            task_data: dict = {
                "order_id": "test-001",
                "warehouse": "WH-A",
                "waypoints": [{"x": 1.0, "y": 2.0}],
                "depth_list": [3.0],
                "grain_type": "稻谷",
                "source": "local",
            }

            # T10 changed signature from str → dict — this must not raise
            win._on_task_selected(task_data)

            # ── Verify orchestrator was created with waypoints from dict ──
            # (the test above exercises the full dict processing path)
            assert win._guidance_page.attach_orchestrator.called
            assert win.page_manager.push.called

    @pytest.mark.parametrize(
        ("env_value", "expected"),
        [(None, False), ("0", False), ("false", False), ("1", True), ("YES", True)],
    )
    def test_real_mechanism_requires_explicit_env_flag(
        self, qapp: QApplication, monkeypatch, env_value, expected
    ) -> None:
        from grain_sampling_ui.main import MainWindow

        if env_value is None:
            monkeypatch.delenv("GRAIN_SAMPLING_UI_ENABLE_MECHANISM", raising=False)
        else:
            monkeypatch.setenv("GRAIN_SAMPLING_UI_ENABLE_MECHANISM", env_value)

        with patch("grain_sampling_ui.main.StatusBar"), \
             patch("grain_sampling_ui.main.ControlPanel"), \
             patch("grain_sampling_ui.main.MainPage"), \
             patch("grain_sampling_ui.main.GuidancePage"), \
             patch("grain_sampling_ui.main.MapPage"), \
             patch("grain_sampling_ui.main.TaskListPage"), \
             patch("grain_sampling_ui.main.MappingPage"), \
             patch("grain_sampling_ui.main.SettingsPage"), \
             patch("grain_sampling_ui.main.AlarmBar"), \
             patch("grain_sampling_ui.main.PageManager"), \
             patch("grain_sampling_ui.main.SamplingBridge"), \
             patch("grain_sampling_ui.main.SamplingStateMachine"), \
             patch("grain_sampling_ui.main.OrderInfo"), \
             patch("grain_sampling_ui.main.WorkflowOrchestrator") as orchestrator_cls:
            win = MainWindow()
            win._guidance_page = MagicMock()
            win.page_manager = MagicMock()
            win._slam_bridge = None
            win._on_task_selected({
                "order_id": "local-test",
                "waypoints": [{"x": 1, "y": 2}],
                "depth_list": [1.0],
                "grain_type": "稻谷",
                "source": "local",
            })

            orchestrator = orchestrator_cls.return_value
            assert orchestrator.enable_mechanism.called is expected
            orchestrator.set_grain.assert_called_once_with("稻谷")

    @pytest.mark.parametrize(
        ("env_value", "task_skip", "expected_map_lookup"),
        [(None, True, True), ("0", True, True), ("1", False, True), ("1", True, False)],
    )
    def test_mapping_skip_requires_env_and_task_marker(
        self, qapp: QApplication, monkeypatch, env_value, task_skip, expected_map_lookup
    ) -> None:
        from grain_sampling_ui.main import MainWindow

        if env_value is None:
            monkeypatch.delenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", raising=False)
        else:
            monkeypatch.setenv("GRAIN_SAMPLING_UI_SKIP_MAPPING", env_value)

        with patch("grain_sampling_ui.main.StatusBar"), \
             patch("grain_sampling_ui.main.ControlPanel"), \
             patch("grain_sampling_ui.main.MainPage"), \
             patch("grain_sampling_ui.main.GuidancePage"), \
             patch("grain_sampling_ui.main.MapPage"), \
             patch("grain_sampling_ui.main.TaskListPage"), \
             patch("grain_sampling_ui.main.MappingPage"), \
             patch("grain_sampling_ui.main.SettingsPage"), \
             patch("grain_sampling_ui.main.AlarmBar"), \
             patch("grain_sampling_ui.main.PageManager"), \
             patch("grain_sampling_ui.main.SamplingBridge"), \
             patch("grain_sampling_ui.main.SamplingStateMachine"), \
             patch("grain_sampling_ui.main.OrderInfo"), \
             patch("grain_sampling_ui.main.WorkflowOrchestrator"):
            win = MainWindow()
            win._guidance_page = MagicMock()
            win.page_manager = MagicMock()
            win._slam_bridge = MagicMock()
            win._slam_bridge.find_map_by_warehouse.return_value = None
            win._on_task_selected({
                "order_id": "commissioning-test",
                "warehouse": "联调模式（跳过地图）",
                "skip_mapping": task_skip,
                "waypoints": [{"x": 0, "y": 0}],
                "depth_list": [2.0],
                "grain_type": "稻谷",
                "source": "local",
            })

            assert win._slam_bridge.find_map_by_warehouse.called is expected_map_lookup
