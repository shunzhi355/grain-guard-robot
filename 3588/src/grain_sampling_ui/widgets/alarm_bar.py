"""Alarm bar widget — bottom bar for warnings and alarms."""

from __future__ import annotations

from PySide2.QtWidgets import QWidget, QHBoxLayout, QLabel, QScrollArea
from PySide2.QtCore import Qt, Signal, Slot, QTimer


class AlarmBar(QWidget):
    """Bottom alarm bar showing warning and error messages."""

    alarm_cleared = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("alarm_bar")
        self.setFixedHeight(36)
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 2, 12, 2)
        layout.setSpacing(8)

        # Alarm icon
        icon = QLabel("⚠")
        icon.setStyleSheet("font-size: 14pt; color: #D4A72C; background: transparent;")
        layout.addWidget(icon)

        # Alarm text (scrollable for long messages)
        self._alarm_label = QLabel("系统正常")
        self._alarm_label.setStyleSheet("font-size: 12pt; color: #8B949E; background: transparent;")
        layout.addWidget(self._alarm_label)

        layout.addStretch()

        # Clear button
        self._clear_btn = QLabel("✕")
        self._clear_btn.setStyleSheet(
            "font-size: 14pt; color: #484F58; background: transparent; padding: 4px;"
        )
        self._clear_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._clear_btn.mousePressEvent = self._on_clear  # type: ignore[assignment]
        layout.addWidget(self._clear_btn)

    def _on_clear(self, event) -> None:  # type: ignore[no-untyped-def]
        self.clear_alarm()

    def set_alarm(self, message: str, level: str = "warning") -> None:
        """Display an alarm message.

        Args:
            message: Alarm text to display.
            level: 'info', 'warning', or 'danger'.
        """
        colors = {
            "info": "#58A6FF",
            "warning": "#D4A72C",
            "danger": "#DA3633",
        }
        color = colors.get(level, "#D4A72C")
        self._alarm_label.setStyleSheet(f"font-size: 12pt; color: {color}; background: transparent; font-weight: bold;")
        self._alarm_label.setText(message)
        self.setStyleSheet(f"""
            QWidget#alarm_bar {{
                background-color: #161B22;
                border-top: 2px solid {color};
            }}
        """)

    def clear_alarm(self) -> None:
        """Clear the alarm display."""
        self._alarm_label.setText("系统正常")
        self._alarm_label.setStyleSheet("font-size: 12pt; color: #8B949E; background: transparent;")
        self.setStyleSheet("""
            QWidget#alarm_bar {
                background-color: #161B22;
                border-top: 1px solid #30363D;
            }
        """)
        self.alarm_cleared.emit()
