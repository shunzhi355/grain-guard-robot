"""Status bar widget — top bar with connection and speed info."""

from __future__ import annotations

from PySide2.QtWidgets import QWidget, QHBoxLayout, QLabel
from PySide2.QtCore import Qt, Signal, Slot
from PySide2.QtGui import QPixmap, QPainter, QColor, QPen, QBrush


class StatusBar(QWidget):
    """Top status bar with connection indicator, network type, robot speed."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("status_bar")
        self.setFixedHeight(44)
        self._connected = False
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 4, 12, 4)
        layout.setSpacing(16)

        # Connection indicator (green/red dot)
        self._conn_dot = QLabel()
        self._conn_dot.setFixedSize(14, 14)
        self._update_conn_dot(False)
        layout.addWidget(self._conn_dot)

        # Network type label
        self._network_label = QLabel("离线")
        self._network_label.setObjectName("subtitle")
        self._network_label.setStyleSheet("font-size: 12pt;")
        layout.addWidget(self._network_label)

        layout.addStretch()

        # Robot speed display
        speed_container = QWidget()
        speed_layout = QHBoxLayout(speed_container)
        speed_layout.setContentsMargins(0, 0, 0, 0)
        speed_layout.setSpacing(6)

        speed_icon = QLabel("⚡")
        speed_layout.addWidget(speed_icon)

        self._speed_label = QLabel("0.0 m/s")
        self._speed_label.setStyleSheet("font-size: 12pt; color: #58A6FF;")
        speed_layout.addWidget(self._speed_label)

        layout.addWidget(speed_container)

    def _update_conn_dot(self, connected: bool) -> None:
        """Draw a coloured dot using a QPixmap."""
        size = 14
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor("#2EA043") if connected else QColor("#DA3633")
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(1, 1, size - 2, size - 2)
        painter.end()
        self._conn_dot.setPixmap(pixmap)

    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self._update_conn_dot(connected)
        self._network_label.setText("已连接" if connected else "离线")

    def set_speed(self, speed_ms: float) -> None:
        self._speed_label.setText(f"{speed_ms:.1f} m/s")

    def set_network_type(self, net_type: str) -> None:
        self._network_label.setText(net_type)
