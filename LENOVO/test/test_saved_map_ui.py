"""Real Qt tests; all navigation outputs and hardware access are replaced."""
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import pytest

ROOT = Path(__file__).resolve().parents[2]
os.environ["QT_QPA_PLATFORM"] = "offscreen"
spec = importlib.util.spec_from_file_location("map_ui_test_launcher", ROOT / "3588/scripts/start_ui.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

@pytest.fixture(scope="module")
def app():
    launcher.load_qt()
    from PySide2.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


def test_distance_updates_from_live_xy_and_invalidates_on_stale(app):
    from grain_sampling_ui.widgets.navigation_telemetry import NavigationTelemetry
    widget = NavigationTelemetry()
    widget.set_map_ready(True)
    widget.set_goal(3, 4)
    widget.update_status({"pose": {"x_m": 0., "y_m": 0., "odom_at": time.monotonic(),
                                   "localization_valid": True}})
    assert "5.000 m" in widget.distance_label.text()
    assert "3.000, 4.000" in widget.goal_label.text()
    assert widget.ready
    widget.update_status({"pose": {"x_m": 3., "y_m": 0., "odom_at": time.monotonic(),
                                   "localization_valid": True}})
    assert "4.000 m" in widget.distance_label.text()
    widget._pose["odom_at"] -= 1
    widget.refresh()
    assert not widget.ready
    assert "STALE" in widget.health_label.text()
    assert "定位失效" in widget.distance_label.text()
    widget.close()


def test_raster_pose_and_click_share_world_transform_after_pan_zoom_resize(app):
    from PySide2.QtGui import QImage, QColor
    from grain_sampling_ui.widgets.map_widget import MapWidget
    widget = MapWidget()
    widget.resize(800, 500)
    image = QImage(80, 40, QImage.Format.Format_RGB32)
    image.fill(QColor("#EE0000"))
    widget.set_saved_map(image, [-2, 6, -1, 3])
    widget.show()
    app.processEvents()
    assert widget._saved_map_image is not None
    def check():
        for x, y in ((-2, 3), (6, -1), (1.2, 2.3)):
            point = widget._world_to_widget(x, y)
            assert widget._widget_to_world(point.x(), point.y()) == pytest.approx((x, y))
        point = widget._world_to_widget(1.2, 2.3)
        pixel = widget.grab().toImage().pixelColor(round(point.x()), round(point.y()))
        assert pixel.red() > 180 and pixel.green() < 100
    check()
    widget._zoom *= 1.2
    widget._offset_x += 0.3
    check()
    widget.resize(1000, 600)
    app.processEvents()
    check()
    widget.close()


def test_preview_page_select_only_and_explicit_start_are_separate(app):
    from grain_sampling_ui.pages.map_page import MapPage
    page = MapPage()
    selected, started = [], []
    page.goal_selected.connect(lambda x, y: selected.append((x, y)))
    page.navigation_requested.connect(lambda x, y: started.append((x, y)))
    page.map_widget.goal_selected.emit(.3, .4)
    assert selected == [(.3, .4)] and not started
    assert not page._btn_navigate.isEnabled()
    page.telemetry.set_map_ready(True)
    page.telemetry.update_status({"pose": {"x_m": 0., "y_m": 0., "odom_at": time.monotonic(),
                                          "localization_valid": True}})
    assert page._btn_navigate.isEnabled()
    assert "0.500 m" in page.telemetry.distance_label.text()
    page._btn_navigate.click()
    assert started == [(.3, .4)]
    assert not page._btn_navigate.isEnabled()
    page.close()


def test_both_pages_share_selected_goal_and_saved_map(app, monkeypatch):
    monkeypatch.delenv("GRAIN_LOCAL_RUNTIME", raising=False)
    from grain_sampling_ui.pages.main_page import CameraWidget
    monkeypatch.setattr(CameraWidget, "_start", lambda *args: None)
    from grain_sampling_ui.main import MainWindow
    from PySide2.QtGui import QImage, QColor
    window = MainWindow()
    window.show()
    app.processEvents()
    image = QImage(80, 40, QImage.Format.Format_RGB32)
    image.fill(QColor("white"))
    window._on_saved_map_preview({"image": image, "world_bounds": [-2,6,-1,3],
                                "map_id": "test-map", "frame": "camera_init", "z_slice": [.4,2.]})
    window._map_page.map_widget.goal_selected.emit(1., 2.)
    assert window._main_page._selected_goal == window._map_page._selected_goal == (1., 2.)
    assert window._main_page.map_widget._saved_map_bounds == window._map_page.map_widget._saved_map_bounds
    window._on_saved_map_preview({"error": "map identity mismatch"})
    assert window._main_page.map_widget._saved_map_image is None
    assert not window._main_page._btn_navigate.isEnabled()
    assert not window._map_page._btn_navigate.isEnabled()
    window.close()
    app.processEvents()


@pytest.mark.parametrize("angle,pixel,direction", [(120,0,1), (-120,0,-1), (0,40,1), (0,-40,-1)])
def test_real_wheel_event_zooms_around_cursor_without_selecting_goal(app, angle, pixel, direction):
    from PySide2.QtCore import QPoint, QPointF, Qt
    from PySide2.QtGui import QWheelEvent
    from grain_sampling_ui.widgets.map_widget import MapWidget
    widget = MapWidget()
    widget.resize(800, 500)
    widget.show()
    app.processEvents()
    selected = []
    widget.goal_selected.connect(lambda x, y: selected.append((x,y)))
    cursor = QPointF(570, 175)
    before = widget._widget_to_world(cursor.x(), cursor.y())
    zoom_before = widget._zoom
    event = QWheelEvent(cursor, cursor, QPoint(0,pixel), QPoint(0,angle),
                        Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                        Qt.ScrollPhase.NoScrollPhase, False)
    app.sendEvent(widget, event)
    assert event.isAccepted()
    assert (widget._zoom - zoom_before) * direction > 0
    assert widget._widget_to_world(cursor.x(),cursor.y()) == pytest.approx(before)
    assert selected == []
    widget.close()


def test_wheel_zero_delta_and_zoom_limits(app):
    from PySide2.QtCore import QPoint, QPointF, Qt
    from PySide2.QtGui import QWheelEvent
    from grain_sampling_ui.widgets.map_widget import MapWidget, MIN_ZOOM, MAX_ZOOM
    widget = MapWidget()
    def wheel(delta):
        event = QWheelEvent(QPointF(100,100), QPointF(100,100), QPoint(), QPoint(0,delta),
                            Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                            Qt.ScrollPhase.NoScrollPhase, False)
        app.sendEvent(widget,event)
        return event
    before = widget._zoom
    assert not wheel(0).isAccepted()
    assert widget._zoom == before
    widget._zoom = MAX_ZOOM
    wheel(120)
    assert widget._zoom == MAX_ZOOM
    widget._zoom = MIN_ZOOM
    wheel(-120)
    assert widget._zoom == MIN_ZOOM
    widget.close()



def test_reset_required_blocks_start_but_keeps_live_distance(app):
    from grain_sampling_ui.widgets.navigation_telemetry import NavigationTelemetry
    widget=NavigationTelemetry()
    widget.set_map_ready(True);widget.set_goal(.4,0.)
    status={"pose":{"x_m":0.,"y_m":0.,"odom_at":time.monotonic(),"localization_valid":True},
            "recovery":{"reset_required":True,"reset_detected":True,"reset_ready":True}}
    widget.update_status(status)
    assert "0.400 m" in widget.distance_label.text()
    assert not widget.ready
    assert "单片机已复位" in widget.recovery_label.text()
    status["recovery"]={"reset_required":False,"reset_detected":False,"reset_ready":True}
    widget.update_status(status)
    assert widget.ready
    widget.close()
