"""Settings page — cloud, camera, WiFi, and debug configuration.

Matches grain-sampling-console.html #screen-settings prototype layout:
  4 cards in 2×2 grid: 云端服务 / 摄像头 / WiFi网络 / 调试

Preserves all existing public API: signals, properties, log handler integration.
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Optional

try:
    from PySide2.QtCore import Qt, Signal, Slot
    from PySide2.QtGui import QTextCursor
    from PySide2.QtWidgets import (
        QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
        QPushButton, QSlider, QTextEdit, QVBoxLayout, QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, Signal, Slot  # type: ignore
    from PySide6.QtGui import QTextCursor  # type: ignore
    from PySide6.QtWidgets import (  # type: ignore
        QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
        QPushButton, QSlider, QTextEdit, QVBoxLayout, QWidget,
    )

from grain_sampling_ui.theme import THEME_COLORS
from utils.config import AppConfig

_logger = logging.getLogger("settings_page")

SETTINGS_JSON_PATH = os.path.expanduser("~/.grain_robot/settings.json")


def _read_device_mac() -> str:
    """Auto-read the device MAC address from the network interface."""
    try:
        import uuid as _uuid
        node = _uuid.getnode()
        if node == 0:
            return "—"
        return ":".join(f"{(node >> (8 * i)) & 0xFF:02X}" for i in range(5, -1, -1))
    except Exception:
        return "—"


# ── Log handler (preserved from original) ────────────────────────────────

class _LogHandler(logging.Handler):
    """Handler that appends log records to a QTextEdit widget."""

    def __init__(self, widget: QTextEdit) -> None:
        super().__init__()
        self._widget = widget
        self.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        )

    def emit(self, record: logging.LogRecord) -> None:
        msg = self.format(record)
        self._widget.append(msg)


class SettingsPage(QWidget):
    """Settings page with card-based configuration panels.

    Cards (2×2 grid):
        1. 云端服务 — server address, device MAC (auto-read), 5G/IP status
        2. 摄像头 — resolution and FPS dropdowns
        3. WiFi 网络 — SSID, password, connect button
        4. 调试 — log toggle, speed limits, save button (→ AppConfig)
    """

    #: Emitted when the user presses the 连接 button for WiFi.
    wifi_connect_requested = Signal(str, str)  # ssid, password

    #: Emitted when the user presses the 应用 button for speed limits.
    speed_apply_requested = Signal(float, float)  # linear, angular

    # ── Constants ──────────────────────────────────────────────────────

    LINEAR_MIN: float = 0.1  # m/s
    LINEAR_MAX: float = 1.0
    LINEAR_DEFAULT: float = 0.5
    LINEAR_STEP: float = 0.05

    ANGULAR_MIN: float = 0.1  # rad/s
    ANGULAR_MAX: float = 3.0
    ANGULAR_DEFAULT: float = 1.5
    ANGULAR_STEP: float = 0.05

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("settings_page")
        self._log_handler: Optional[_LogHandler] = None
        self._app_config: AppConfig = AppConfig()
        self._load_saved_config()
        self._setup_ui()
        self._install_log_handler()

    # ── Config persistence ─────────────────────────────────────────────

    def _load_saved_config(self) -> None:
        """Load previously saved settings from JSON if available."""
        try:
            path = Path(SETTINGS_JSON_PATH).expanduser().resolve()
            if path.exists():
                self._app_config = AppConfig.from_json_file(str(path))
        except Exception:
            _logger.warning("无法加载已有设置，使用默认值")

    def _save_config(self) -> None:
        """Persist current settings to JSON via AppConfig."""
        try:
            # Update AppConfig from current UI values
            self._app_config.cloud_base_url = self._server_addr.text().strip()
            self._app_config.device_mac = self._device_mac_label.text()

            data = {
                "cloud_base_url": self._app_config.cloud_base_url,
                "device_mac": self._app_config.device_mac,
            }

            path = Path(SETTINGS_JSON_PATH).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            self._save_note.setText("✓ 设置已保存")
            self._save_note.setStyleSheet(
                f"color: {THEME_COLORS['success']}; font-size: 11pt; background: transparent;"
            )
            _logger.info("设置已保存到 %s", path)
        except Exception as e:
            self._save_note.setText(f"✗ 保存失败: {e}")
            self._save_note.setStyleSheet(
                f"color: {THEME_COLORS['danger']}; font-size: 11pt; background: transparent;"
            )
            _logger.error("保存设置失败: %s", e)

    # ── UI setup ──────────────────────────────────────────────────────

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(16)

        # Title row
        title_row = QHBoxLayout()
        page_title = QLabel("系统设置")
        page_title.setObjectName("page_title")
        page_sub = QLabel("云端、摄像头与网络配置")
        page_sub.setObjectName("page_sub")
        title_row.addWidget(page_title)
        title_row.addWidget(page_sub)
        title_row.addStretch()
        root.addLayout(title_row)

        # ── 2×2 card grid ──
        grid = QGridLayout()
        grid.setSpacing(16)

        grid.addWidget(self._create_cloud_card(), 0, 0)
        grid.addWidget(self._create_camera_card(), 0, 1)
        grid.addWidget(self._create_wifi_card(), 1, 0)
        grid.addWidget(self._create_debug_card(), 1, 1)

        root.addLayout(grid, 1)

        # ── Log viewer (collapsible, below cards) ──
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumHeight(160)
        self._log_text.setStyleSheet(
            "QTextEdit {"
            f"  background-color: {THEME_COLORS['bg_dark']};"
            f"  border: 1px solid {THEME_COLORS['border']};"
            f"  color: {THEME_COLORS['text_primary']};"
            "   font-family: 'Consolas', 'Courier New', monospace;"
            "   font-size: 11pt;"
            "   border-radius: 6px;"
            "   padding: 8px;"
            "}"
        )
        self._log_text.hide()  # hidden by default, shown when log toggle is on
        root.addWidget(self._log_text)

    # ── Card 1: 云端服务 ──────────────────────────────────────────────

    def _create_cloud_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(14)

        h3 = QLabel("云端服务")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # Server address
        addr_label = QLabel("服务器地址")
        addr_label.setObjectName("field_label")

        self._server_addr = QLineEdit()
        self._server_addr.setPlaceholderText("http://<服务器IP>:<端口>")
        self._server_addr.setText(self._app_config.cloud_base_url)

        layout.addWidget(addr_label)
        layout.addWidget(self._server_addr)

        # Device MAC (readonly)
        mac_label = QLabel("设备 MAC")
        mac_label.setObjectName("field_label")

        self._device_mac_label = QLineEdit()
        self._device_mac_label.setText(self._app_config.device_mac)
        self._device_mac_label.setReadOnly(True)

        mac_hint = QLabel("板端联网后自动读取，无需填写。")
        mac_hint.setObjectName("field_hint")

        layout.addWidget(mac_label)
        layout.addWidget(self._device_mac_label)
        layout.addWidget(mac_hint)

        # 5G status + IP (compact inline)
        status_row = QHBoxLayout()
        status_row.setSpacing(16)

        self._5g_status_label = QLabel("5G: —")
        self._5g_status_label.setStyleSheet(
            f"color: {THEME_COLORS['text_secondary']}; font-size: 12pt; background: transparent;"
        )
        self._5g_signal_label = QLabel("信号: —")
        self._5g_signal_label.setStyleSheet(
            f"color: {THEME_COLORS['text_secondary']}; font-size: 12pt; background: transparent;"
        )
        self._ip_label = QLabel("IP: —")
        self._ip_label.setStyleSheet(
            f"color: {THEME_COLORS['info']}; font-size: 12pt; font-weight: bold; background: transparent;"
        )

        status_row.addWidget(self._5g_status_label)
        status_row.addWidget(self._5g_signal_label)
        status_row.addWidget(self._ip_label)
        status_row.addStretch()
        layout.addLayout(status_row)

        layout.addStretch()
        return card

    # ── Card 2: 摄像头 ────────────────────────────────────────────────

    def _create_camera_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(14)

        h3 = QLabel("摄像头")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # Resolution dropdown
        res_label = QLabel("分辨率")
        res_label.setObjectName("field_label")
        layout.addWidget(res_label)

        self._cam_res = QComboBox()
        self._cam_res.addItems(["1920 × 1080", "1280 × 720"])
        self._cam_res.setCurrentIndex(0)
        layout.addWidget(self._cam_res)

        # FPS dropdown
        fps_label = QLabel("帧率")
        fps_label.setObjectName("field_label")
        layout.addWidget(fps_label)

        self._cam_fps = QComboBox()
        self._cam_fps.addItems(["30 fps", "15 fps"])
        self._cam_fps.setCurrentIndex(0)
        layout.addWidget(self._cam_fps)

        layout.addStretch()
        return card

    # ── Card 3: WiFi 网络 ─────────────────────────────────────────────

    def _create_wifi_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(14)

        h3 = QLabel("WiFi 网络")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # SSID
        ssid_label = QLabel("SSID")
        ssid_label.setObjectName("field_label")
        layout.addWidget(ssid_label)

        self._wifi_combo = QComboBox()
        self._wifi_combo.setEditable(True)
        self._wifi_combo.addItems(["WiFi-Net-5G", "Office-WiFi", "Robot-Hotspot"])
        layout.addWidget(self._wifi_combo)

        # Password
        pass_label = QLabel("密码")
        pass_label.setObjectName("field_label")
        layout.addWidget(pass_label)

        self._wifi_password = QLineEdit()
        self._wifi_password.setPlaceholderText("网络密码")
        self._wifi_password.setEchoMode(QLineEdit.EchoMode.Password)
        layout.addWidget(self._wifi_password)

        # Connect button
        self._btn_connect = QPushButton("连接")
        self._btn_connect.setObjectName("btn_primary")
        self._btn_connect.clicked.connect(self._on_wifi_connect)
        layout.addWidget(self._btn_connect)

        layout.addStretch()
        return card

    # ── Card 4: 调试 ──────────────────────────────────────────────────

    def _create_debug_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(14)

        h3 = QLabel("调试")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # Log switch row
        switch_row = QHBoxLayout()
        switch_row.setSpacing(10)

        self._log_switch_btn = QPushButton("○")
        self._log_switch_btn.setObjectName("btn_success")
        self._log_switch_btn.setFixedSize(52, 52)
        self._log_switch_btn.setCheckable(True)
        self._log_switch_btn.clicked.connect(self._on_log_toggle)
        self._log_on = False

        log_label = QLabel("调试日志")
        log_label.setStyleSheet(
            f"font-size: 16px; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )

        log_hint = QLabel("开启后记录详细运行日志")
        log_hint.setObjectName("field_hint")

        switch_row.addWidget(self._log_switch_btn)
        switch_row.addWidget(log_label)
        switch_row.addWidget(log_hint)
        switch_row.addStretch()
        layout.addLayout(switch_row)

        # ── Speed limits (preserved from original) ──
        speed_section = QLabel("速度限制")
        speed_section.setStyleSheet(
            f"font-size: 13pt; font-weight: bold; color: {THEME_COLORS['text_secondary']}; background: transparent; margin-top: 8px;"
        )
        layout.addWidget(speed_section)

        # Linear velocity
        lin_row = QHBoxLayout()
        lin_row.setSpacing(8)

        lin_label = QLabel(f"线速度: {self.LINEAR_DEFAULT:.2f} m/s")
        lin_label.setStyleSheet(
            f"color: {THEME_COLORS['text_primary']}; font-size: 12pt; background: transparent; min-width: 100px;"
        )

        self._linear_slider = QSlider(Qt.Orientation.Horizontal)
        self._linear_slider.setMinimum(0)
        self._linear_slider.setMaximum(
            int((self.LINEAR_MAX - self.LINEAR_MIN) / self.LINEAR_STEP)
        )
        self._linear_slider.setValue(
            int((self.LINEAR_DEFAULT - self.LINEAR_MIN) / self.LINEAR_STEP)
        )
        self._linear_slider.valueChanged.connect(
            lambda v, lbl=lin_label: self._update_slider_label(v, lbl, self.LINEAR_MIN, self.LINEAR_STEP, "m/s")
        )

        lin_row.addWidget(lin_label)
        lin_row.addWidget(self._linear_slider, 1)
        layout.addLayout(lin_row)

        # Angular velocity
        ang_row = QHBoxLayout()
        ang_row.setSpacing(8)

        ang_label = QLabel(f"角速度: {self.ANGULAR_DEFAULT:.2f} rad/s")
        ang_label.setStyleSheet(
            f"color: {THEME_COLORS['text_primary']}; font-size: 12pt; background: transparent; min-width: 100px;"
        )

        self._angular_slider = QSlider(Qt.Orientation.Horizontal)
        self._angular_slider.setMinimum(0)
        self._angular_slider.setMaximum(
            int((self.ANGULAR_MAX - self.ANGULAR_MIN) / self.ANGULAR_STEP)
        )
        self._angular_slider.setValue(
            int((self.ANGULAR_DEFAULT - self.ANGULAR_MIN) / self.ANGULAR_STEP)
        )
        self._angular_slider.valueChanged.connect(
            lambda v, lbl=ang_label: self._update_slider_label(v, lbl, self.ANGULAR_MIN, self.ANGULAR_STEP, "rad/s")
        )

        ang_row.addWidget(ang_label)
        ang_row.addWidget(self._angular_slider, 1)
        layout.addLayout(ang_row)

        # Apply speed button
        self._btn_speed_apply = QPushButton("应用速度限制")
        self._btn_speed_apply.clicked.connect(self._on_speed_apply)
        layout.addWidget(self._btn_speed_apply)

        # ── Save row ──
        save_row = QHBoxLayout()
        save_row.setSpacing(10)
        save_row.addStretch()

        self._save_note = QLabel("")
        self._save_note.setObjectName("save_note")

        self._btn_save = QPushButton("保存设置")
        self._btn_save.setObjectName("btn_primary")
        self._btn_save.clicked.connect(self._save_config)

        save_row.addWidget(self._save_note)
        save_row.addWidget(self._btn_save)
        layout.addLayout(save_row)

        layout.addStretch()
        return card

    @staticmethod
    def _update_slider_label(value: int, label: QLabel, offset: float, step: float, unit: str) -> None:
        actual = round(offset + value * step, 2)
        label.setText(f"{label.text().split(':')[0]}: {actual:.2f} {unit}")

    # ── Logging integration ───────────────────────────────────────────

    def _install_log_handler(self) -> None:
        """Attach a custom handler that mirrors root log output to the log tab."""
        self._log_handler = _LogHandler(self._log_text)
        self._log_handler.setLevel(logging.INFO)
        root_logger = logging.getLogger()
        root_logger.addHandler(self._log_handler)
        _logger.info("日志系统已初始化")

    def remove_log_handler(self) -> None:
        """Remove the custom log handler (call before widget destruction)."""
        if self._log_handler is not None:
            root_logger = logging.getLogger()
            root_logger.removeHandler(self._log_handler)
            self._log_handler = None

    # ── Slot handlers ─────────────────────────────────────────────────

    @Slot()
    def _on_log_toggle(self) -> None:
        """Toggle the log viewer visibility."""
        self._log_on = not self._log_on
        if self._log_on:
            self._log_switch_btn.setText("●")
            self._log_switch_btn.setObjectName("btn_danger")
            self._log_text.show()
        else:
            self._log_switch_btn.setText("○")
            self._log_switch_btn.setObjectName("btn_success")
            self._log_text.hide()
        self._log_switch_btn.style().unpolish(self._log_switch_btn)
        self._log_switch_btn.style().polish(self._log_switch_btn)

    @Slot()
    def _on_wifi_connect(self) -> None:
        ssid = self._wifi_combo.currentText()
        password = self._wifi_password.text()
        _logger.info("WiFi 连接请求: SSID=%s", ssid)
        self.wifi_connect_requested.emit(ssid, password)

    @Slot()
    def _on_speed_apply(self) -> None:
        _logger.info(
            "速度限制已应用: 线速度=%.2f m/s, 角速度=%.2f rad/s",
            self.linear_value,
            self.angular_value,
        )
        self.speed_apply_requested.emit(self.linear_value, self.angular_value)

    # ── Public API (preserved from original) ───────────────────────────

    @property
    def linear_value(self) -> float:
        """Current linear velocity slider value in m/s."""
        raw = self._linear_slider.value()
        return round(self.LINEAR_MIN + raw * self.LINEAR_STEP, 2)

    @linear_value.setter
    def linear_value(self, value: float) -> None:
        clamped = max(self.LINEAR_MIN, min(self.LINEAR_MAX, value))
        tick = int(round((clamped - self.LINEAR_MIN) / self.LINEAR_STEP))
        self._linear_slider.setValue(tick)

    @property
    def angular_value(self) -> float:
        """Current angular velocity slider value in rad/s."""
        raw = self._angular_slider.value()
        return round(self.ANGULAR_MIN + raw * self.ANGULAR_STEP, 2)

    @angular_value.setter
    def angular_value(self, value: float) -> None:
        clamped = max(self.ANGULAR_MIN, min(self.ANGULAR_MAX, value))
        tick = int(round((clamped - self.ANGULAR_MIN) / self.ANGULAR_STEP))
        self._angular_slider.setValue(tick)

    def set_5g_status(self, connected: bool, signal_percent: int = 0) -> None:
        """Update the 5G status display."""
        if connected:
            self._5g_status_label.setText(f"5G: 已连接")
            self._5g_status_label.setStyleSheet(
                f"color: {THEME_COLORS['success']}; font-size: 12pt; background: transparent;"
            )
        else:
            self._5g_status_label.setText(f"5G: 未连接")
            self._5g_status_label.setStyleSheet(
                f"color: {THEME_COLORS['danger']}; font-size: 12pt; background: transparent;"
            )
        self._5g_signal_label.setText(f"信号: {signal_percent}%")

    def set_ip_address(self, ip: str) -> None:
        """Update the displayed IP address."""
        self._ip_label.setText(f"IP: {ip}")

    def set_wifi_networks(self, networks: list[str]) -> None:
        """Replace the WiFi network list."""
        self._wifi_combo.clear()
        self._wifi_combo.addItems(networks)

    @property
    def log_text(self) -> str:
        """Return the current content of the log viewer."""
        return self._log_text.toPlainText()
