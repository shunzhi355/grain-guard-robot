"""Control panel widget — left panel with touch-friendly action buttons."""

from __future__ import annotations

from PySide2.QtCore import Qt, Signal
from PySide2.QtWidgets import QPushButton, QSizePolicy, QVBoxLayout, QWidget


class ControlPanel(QWidget):
    """Left-side control panel with large, touch-friendly primary action buttons.

    Signals are emitted on click and wired by the parent page to trigger
    ROS services or internal state changes.
    """

    # ── Signals ─────────────────────────────────────────────
    start_sampling = Signal()
    pause_sampling = Signal()
    emergency_stop = Signal()
    manual_control = Signal()
    show_map_page = Signal()
    show_mapping_page = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("control_panel")
        self.setFixedWidth(130)
        self._setup_ui()

    # ── UI construction ─────────────────────────────────────

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 16, 8, 16)
        layout.setSpacing(12)

        button_configs: list[dict] = [
            {
                "object_name": "btn_success",
                "text": "开始采样",
                "min_height": 80,
                "signal": self.start_sampling,
            },
            {
                "object_name": "btn_warning",
                "text": "暂停",
                "min_height": 80,
                "signal": self.pause_sampling,
            },
            {
                "object_name": "btn_emergency",
                "text": "急停",
                "min_height": 100,
                "signal": self.emergency_stop,
            },
        ]

        for cfg in button_configs:
            btn = self._make_button(
                text=cfg["text"],
                object_name=cfg["object_name"],
                min_height=cfg["min_height"],
            )
            btn.clicked.connect(cfg["signal"].emit)
            layout.addWidget(btn)

        layout.addStretch()

        bottom_configs: list[dict] = [
            {
                "object_name": "btn_manual",
                "text": "手动遥控",
                "min_height": 72,
                "signal": self.manual_control,
            },
            {
                "object_name": "btn_info",
                "text": "廒间地图",
                "min_height": 54,
                "signal": self.show_map_page,
            },
            {
                "object_name": "btn_info",
                "text": "建图设置",
                "min_height": 54,
                "signal": self.show_mapping_page,
            },
        ]

        for cfg in bottom_configs:
            btn = self._make_button(
                text=cfg["text"],
                object_name=cfg["object_name"],
                min_height=cfg["min_height"],
            )
            btn.clicked.connect(cfg["signal"].emit)
            layout.addWidget(btn)

    # ── Helpers ─────────────────────────────────────────────

    @staticmethod
    def _make_button(text: str, object_name: str, min_height: int) -> QPushButton:
        btn = QPushButton(text)
        btn.setObjectName(object_name)
        btn.setMinimumHeight(min_height)
        btn.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        btn.setCursor(Qt.CursorShape.PointingHandCursor)
        return btn
