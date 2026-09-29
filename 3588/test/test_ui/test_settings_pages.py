"""Tests for mapping_page.py, settings_page.py, and stream_integration.py.

No hardware dependency — all hardware APIs are mocked or faked.
When PySide6 is unavailable the UI tests are skipped gracefully.
"""

from __future__ import annotations

import logging
from unittest.mock import MagicMock

import pytest

# ── Imports that may need PySide6 ────────────────────────────────────────────
_pyside6_available = False
try:
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QLabel

    _pyside6_available = True
except ModuleNotFoundError:
    pass

from grain_sampling_camera.rtsp_server import RTSPServer
from grain_sampling_camera.stream_integration import StreamManager

# Conditionally import UI pages
if _pyside6_available:
    from grain_sampling_ui.pages.mapping_page import MappingPage
    from grain_sampling_ui.pages.settings_page import SettingsPage


# ────────────────────────────────────────────────────────────────────────────
# PySide6-dependent fixtures
# ────────────────────────────────────────────────────────────────────────────


@pytest.fixture(scope="session")
def qapp() -> QApplication:
    """Create a session-scoped QApplication for all UI tests."""
    if not _pyside6_available:
        pytest.skip("PySide6 not available")
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    yield app


@pytest.fixture
def mapping_page(qapp: QApplication) -> QWidget:
    """Return a fresh MappingPage instance."""
    page = MappingPage()
    yield page


@pytest.fixture
def settings_page(qapp: QApplication) -> QWidget:
    """Return a fresh SettingsPage instance."""
    page = SettingsPage()
    yield page
    page.remove_log_handler()


# ────────────────────────────────────────────────────────────────────────────
# MappingPage tests
# ────────────────────────────────────────────────────────────────────────────


@pytest.mark.skipif(not _pyside6_available, reason="PySide6 not available")
class TestMappingPage:
    """Tests for the 建图设置 page."""

    def test_page_creation(self, mapping_page) -> None:
        """Verify the page is created with the correct object name."""
        assert mapping_page.objectName() == "mapping_page"
        assert mapping_page.isVisible() is False

    def test_initial_state_is_idle(self, mapping_page) -> None:
        """The page should start in the 'idle' state."""
        assert mapping_page.mapping_state == "idle"

    def test_start_mapping_emits_signal(self, mapping_page) -> None:
        """Clicking 开始建图 should emit start_mapping_requested."""
        spy: list[bool] = []

        def _handler() -> None:
            spy.append(True)

        mapping_page.start_mapping_requested.connect(_handler)
        btn = mapping_page._btn_start
        QTest.mouseClick(btn, Qt.MouseButton.LeftButton)
        assert len(spy) == 1
        assert mapping_page.mapping_state == "mapping"

    def test_stop_mapping_emits_signal(self, mapping_page) -> None:
        """Clicking 停止建图 should emit stop_mapping_requested and return to idle."""
        mapping_page.set_mapping_status("mapping")
        spy: list[bool] = []

        def _handler() -> None:
            spy.append(True)

        mapping_page.stop_mapping_requested.connect(_handler)
        QTest.mouseClick(mapping_page._btn_stop, Qt.MouseButton.LeftButton)
        assert len(spy) == 1
        assert mapping_page.mapping_state == "idle"

    def test_save_map_emits_signal(self, mapping_page) -> None:
        """Clicking 保存地图 should emit save_map_requested and transition to saved."""
        mapping_page.set_mapping_status("mapping")
        spy: list[bool] = []

        def _handler() -> None:
            spy.append(True)

        mapping_page.save_map_requested.connect(_handler)
        QTest.mouseClick(mapping_page._btn_save, Qt.MouseButton.LeftButton)
        assert len(spy) == 1
        assert mapping_page.mapping_state == "saved"

    def test_button_states_by_status(self, mapping_page) -> None:
        """Verify button enable states change with mapping status."""
        mapping_page.set_mapping_status("idle")
        assert mapping_page._btn_start.isEnabled()
        assert not mapping_page._btn_stop.isEnabled()
        assert not mapping_page._btn_save.isEnabled()

        mapping_page.set_mapping_status("mapping")
        assert not mapping_page._btn_start.isEnabled()
        assert mapping_page._btn_stop.isEnabled()
        assert mapping_page._btn_save.isEnabled()

        mapping_page.set_mapping_status("saved")
        assert mapping_page._btn_start.isEnabled()
        assert not mapping_page._btn_stop.isEnabled()
        assert mapping_page._btn_save.isEnabled()

        mapping_page.set_mapping_status("error")
        assert mapping_page._btn_start.isEnabled()
        assert not mapping_page._btn_stop.isEnabled()
        assert not mapping_page._btn_save.isEnabled()

    def test_get_map_list_returns_stub_data(self, mapping_page) -> None:
        """Verifies the two stub maps are present."""
        maps = mapping_page.get_map_list()
        assert len(maps) == 2
        names = [m[0] for m in maps]
        assert "warehouse_a_20250701_1430" in names
        assert "silo_3_scan_20250702_0915" in names

    def test_save_map_adds_to_list(self, mapping_page) -> None:
        """After save_map, a new entry should appear at the top of the list."""
        initial_count = mapping_page._map_list.count()
        mapping_page.set_mapping_status("mapping")
        mapping_page._on_save_clicked()
        assert mapping_page._map_list.count() == initial_count + 1

    def test_invalid_state_raises(self, mapping_page) -> None:
        """Passing an unknown state should raise ValueError."""
        with pytest.raises(ValueError, match="Unknown mapping state"):
            mapping_page.set_mapping_status("bogus")


# ────────────────────────────────────────────────────────────────────────────
# SettingsPage tests
# ────────────────────────────────────────────────────────────────────────────


@pytest.mark.skipif(not _pyside6_available, reason="PySide6 not available")
class TestSettingsPage:
    """Tests for the 系统设置 page."""

    def test_page_creation(self, settings_page) -> None:
        """Verify the page is created with correct object name."""
        assert settings_page.objectName() == "settings_page"

    def test_tabs_exist(self, settings_page) -> None:
        """Verify all four tabs are present."""
        tabs = settings_page._tabs
        assert tabs.count() == 4
        assert tabs.tabText(0) == "网络设置"
        assert tabs.tabText(1) == "速度限制"
        assert tabs.tabText(2) == "日志查看"
        assert tabs.tabText(3) == "版本信息"

    def test_speed_sliders_default_values(self, settings_page) -> None:
        """The sliders should start at their default values."""
        assert settings_page.linear_value == pytest.approx(0.5, abs=0.01)
        assert settings_page.angular_value == pytest.approx(1.5, abs=0.01)

    def test_speed_slider_clamp_min(self, settings_page) -> None:
        """Setting a value below the minimum should clamp to the minimum."""
        settings_page.linear_value = 0.0
        assert settings_page.linear_value == pytest.approx(0.1, abs=0.01)

        settings_page.angular_value = 0.0
        assert settings_page.angular_value == pytest.approx(0.1, abs=0.01)

    def test_speed_slider_clamp_max(self, settings_page) -> None:
        """Setting a value above the maximum should clamp to the maximum."""
        settings_page.linear_value = 99.0
        assert settings_page.linear_value == pytest.approx(1.0, abs=0.01)

        settings_page.angular_value = 99.0
        assert settings_page.angular_value == pytest.approx(3.0, abs=0.01)

    def test_speed_slider_precise_values(self, settings_page) -> None:
        """Verify slider round-trip for in-range values."""
        settings_page.linear_value = 0.35
        assert settings_page.linear_value == 0.35

        settings_page.linear_value = 0.75
        assert settings_page.linear_value == 0.75

        settings_page.angular_value = 2.00
        assert settings_page.angular_value == 2.00

    def test_speed_apply_emits_signal(self, settings_page) -> None:
        """Clicking 应用 should emit speed_apply_requested with current values."""
        settings_page.linear_value = 0.8
        settings_page.angular_value = 2.0

        results: list[tuple[float, float]] = []

        def _handler(lin: float, ang: float) -> None:
            results.append((lin, ang))

        settings_page.speed_apply_requested.connect(_handler)
        QTest.mouseClick(settings_page._btn_apply, Qt.MouseButton.LeftButton)
        assert len(results) == 1
        assert results[0] == (0.8, 2.0)

    def test_wifi_connect_emits_signal(self, settings_page) -> None:
        """Clicking 连接 should emit wifi_connect_requested with ssid+pwd."""
        settings_page._wifi_combo.setCurrentIndex(0)
        settings_page._wifi_password.setText("test-pass")

        results: list[tuple[str, str]] = []

        def _handler(ssid: str, pwd: str) -> None:
            results.append((ssid, pwd))

        settings_page.wifi_connect_requested.connect(_handler)
        QTest.mouseClick(settings_page._btn_connect, Qt.MouseButton.LeftButton)
        assert len(results) == 1
        assert results[0][0] == "WiFi-Net-5G"
        assert results[0][1] == "test-pass"

    def test_set_5g_status(self, settings_page) -> None:
        """5G status display should update correctly."""
        settings_page.set_5g_status(True, 85)
        assert "已连接" in settings_page._5g_status_label.text()
        assert "85%" in settings_page._5g_signal_label.text()

        settings_page.set_5g_status(False)
        assert "未连接" in settings_page._5g_status_label.text()
        assert "0%" in settings_page._5g_signal_label.text()

    def test_set_ip_address(self, settings_page) -> None:
        """IP address label should update."""
        settings_page.set_ip_address("10.0.0.55")
        assert settings_page._ip_label.text() == "10.0.0.55"

    def test_set_wifi_networks(self, settings_page) -> None:
        """WiFi combo box should reflect the supplied networks."""
        settings_page.set_wifi_networks(["Net-A", "Net-B", "Net-C"])
        assert settings_page._wifi_combo.count() == 3
        assert settings_page._wifi_combo.itemText(0) == "Net-A"

    def test_log_clear(self, settings_page) -> None:
        """Clear log should empty the text area."""
        logging.getLogger().info("test-message-cleared")
        assert len(settings_page.log_text) > 0
        QTest.mouseClick(settings_page._btn_clear, Qt.MouseButton.LeftButton)
        assert settings_page.log_text == ""

    def test_log_viewer_is_read_only(self, settings_page) -> None:
        """The log text area should be read-only."""
        assert settings_page._log_text.isReadOnly()

    def test_version_labels_exist(self, settings_page) -> None:
        """Tab 3 (版本信息) should have the expected content labels."""
        tab = settings_page._tabs.widget(3)
        labels = tab.findChildren(QLabel)
        texts = {lbl.text() for lbl in labels if lbl.text()}
        assert "v0.1.0" in texts
        assert "Noetic" in texts
        assert "RK3588" in texts
        assert "Livox Mid-360" in texts

    def test_page_name_label(self, settings_page) -> None:
        """The page title should be 系统设置."""
        page_name_labels = settings_page.findChildren(QLabel, "page_name")
        titles = [lbl.text() for lbl in page_name_labels]
        assert "系统设置" in titles


# ────────────────────────────────────────────────────────────────────────────
# StreamManager tests (no PySide6 required)
# ────────────────────────────────────────────────────────────────────────────


class TestStreamIntegration:
    """Tests for the StreamManager orchestration class.

    Cloud connectivity is handled separately via CloudHttpClient;
    StreamManager only manages local RTSP streaming.
    """

    @pytest.fixture
    def mock_rtsp(self) -> MagicMock:
        """Return a mock RTSPServer."""
        srv = MagicMock(spec=RTSPServer)
        srv.start.return_value = True
        srv.stop.return_value = None
        return srv

    def test_default_url(self) -> None:
        """Default RTSP URL uses detected IP and configured port."""
        mgr = StreamManager(
            device_id="test-bot",
            config={"host": "192.168.1.50", "port": 8554},
            rtsp_server=MagicMock(spec=RTSPServer),
        )
        assert mgr.rtsp_url == "rtsp://192.168.1.50:8554/camera"

    def test_start_streaming_success(
        self, mock_rtsp: MagicMock
    ) -> None:
        """start_streaming should start the RTSP server."""
        mock_rtsp.start.return_value = True
        mgr = StreamManager(
            device_id="test-bot",
            rtsp_server=mock_rtsp,
        )
        result = mgr.start_streaming()
        assert result is True
        mock_rtsp.start.assert_called_once()

    def test_stop_streaming(
        self, mock_rtsp: MagicMock
    ) -> None:
        """stop_streaming should stop the server."""
        mock_rtsp.start.return_value = True
        mgr = StreamManager(
            device_id="bot-x",
            rtsp_server=mock_rtsp,
        )
        mgr.start_streaming()
        mock_rtsp.stop.reset_mock()

        mgr.stop_streaming()
        mock_rtsp.stop.assert_called_once()

    def test_get_stream_status(self) -> None:
        """get_stream_status should return a dict with correct fields."""
        mgr = StreamManager(
            device_id="b1",
            config={"host": "10.0.0.1"},
            rtsp_server=MagicMock(spec=RTSPServer),
        )
        status = mgr.get_stream_status()
        assert status["streaming"] is False
        assert status["rtsp_url"] == "rtsp://10.0.0.1:8554/camera"
        assert "config" in status
        assert status["duration_s"] == 0.0

    def test_update_config(self) -> None:
        """update_config should merge new values into internal config."""
        mgr = StreamManager(
            device_id="c1",
            rtsp_server=MagicMock(spec=RTSPServer),
        )
        mgr.update_config({"port": 9999, "extra": "val"})
        assert mgr._config["port"] == 9999
        assert mgr._config["extra"] == "val"
        assert "host" in mgr._config

    def test_start_streaming_with_url_override(self) -> None:
        """Passing an RTSP URL should override host/port/mount_point."""
        mgr = StreamManager(
            device_id="d1",
            rtsp_server=MagicMock(spec=RTSPServer),
        )
        mgr.start_streaming("rtsp://192.168.2.1:5555/stream")
        assert mgr._config["host"] == "192.168.2.1"
        assert mgr._config["port"] == 5555
        assert mgr._config["mount_point"] == "/stream"

    def test_start_stop_roundtrip(self) -> None:
        """Start then stop should work without errors."""
        mgr = StreamManager(
            device_id="g1",
            rtsp_server=MagicMock(spec=RTSPServer),
        )
        assert mgr.start_streaming() is True
        mgr.stop_streaming()


# ────────────────────────────────────────────────────────────────────────────
# Import sanity checks (no PySide6 needed)
# ────────────────────────────────────────────────────────────────────────────


def test_stream_integration_import() -> None:
    """Verify StreamManager can be imported without hardware deps."""
    from grain_sampling_camera.stream_integration import StreamManager as SM
    assert SM is not None
