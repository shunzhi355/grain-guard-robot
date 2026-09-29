"""Warehouse map page for the grain sampling robot UI.

Layout: HBox(left panel 320px | right canvas stretch).
Matches the HTML prototype grain-sampling-console.html (screen-map).
"""
from __future__ import annotations

from typing import Optional

try:
    from PySide2.QtCore import Qt, Signal
    from PySide2.QtWidgets import (
        QDialog, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel,
        QLineEdit, QProgressBar, QPushButton, QVBoxLayout, QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, Signal  # type: ignore
    from PySide6.QtWidgets import (  # type: ignore
        QDialog, QDoubleSpinBox, QFrame, QHBoxLayout, QLabel,
        QLineEdit, QProgressBar, QPushButton, QVBoxLayout, QWidget,
    )

from grain_sampling_ui.theme import THEME_COLORS
from grain_sampling_ui.widgets.map_widget import MapWidget

C = THEME_COLORS


# --- Upload Dialog (kept for future use) ---
class _UploadDialog(QDialog):
    request_preview = Signal(float)
    upload_map = Signal(float)

    def __init__(self, parent: QWidget | None = None, laser_map_getter=None, room_id: str = "") -> None:
        super().__init__(parent)
        self.setWindowTitle("Upload Map")
        self.setMinimumWidth(420)
        self._preview_shown = False
        self._laser_map_getter = laser_map_getter
        self._contour_points = None
        self._room_id = room_id
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(16)

        title = QLabel("Upload Map")
        title.setStyleSheet(f"font-size: 14pt; font-weight: bold; color: {C['text_primary']};")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        room_layout = QHBoxLayout()
        room_layout.addWidget(QLabel("廒间 ID:"))
        self._room_input = QLineEdit(self._room_id)
        self._room_input.setPlaceholderText("输入廒间编号 (如 WH-A01)")
        room_layout.addWidget(self._room_input)
        room_layout.addStretch()
        layout.addLayout(room_layout)

        h_layout = QHBoxLayout()
        h_layout.addWidget(QLabel("Height (m):"))
        self._height_spin = QDoubleSpinBox()
        self._height_spin.setRange(0.0, 5.0)
        self._height_spin.setValue(1.5)
        h_layout.addWidget(self._height_spin)
        h_layout.addStretch()
        layout.addLayout(h_layout)

        self._preview_area = QLabel("Preview")
        self._preview_area.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._preview_area.setMinimumHeight(120)
        self._preview_area.setStyleSheet(
            f"background-color: {C['bg_medium']}; border: 1px dashed {C['border']}; "
            f"border-radius: 6px; color: {C['text_disabled']};"
        )
        self._preview_area.setVisible(False)
        layout.addWidget(self._preview_area)

        self._progress_bar = QProgressBar()
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setVisible(False)
        layout.addWidget(self._progress_bar)

        self._status_label = QLabel("")
        self._status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_label.setVisible(False)
        layout.addWidget(self._status_label)

        btn_layout = QHBoxLayout()
        btn_layout.addStretch()
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(cancel_btn)
        preview_btn = QPushButton("Preview")
        preview_btn.clicked.connect(self._on_preview)
        btn_layout.addWidget(preview_btn)
        self._upload_btn = QPushButton("Upload")
        self._upload_btn.clicked.connect(self._on_upload)
        self._upload_btn.setVisible(False)
        btn_layout.addWidget(self._upload_btn)
        layout.addLayout(btn_layout)

    def _on_preview(self) -> None:
        h = self._height_spin.value()
        self._preview_shown = True
        self._preview_area.setVisible(True)
        self._upload_btn.setVisible(False)
        self.request_preview.emit(h)
        self._preview_area.setText("切面预览由联想地图服务生成，当前尚未接入")

    def _on_upload(self) -> None:
        self._progress_bar.setVisible(True)
        self._progress_bar.setValue(0)
        self._status_label.setVisible(False)
        self.upload_map.emit(self._height_spin.value())

        self.show_upload_failure("地图保存在联想主机；点云切片上传需由联想地图服务执行")

    def show_upload_success(self) -> None:
        if self._status_label:
            self._status_label.setText("Upload OK")
            self._status_label.setStyleSheet(f"color: {C['success']}; font-weight: bold;")
            self._status_label.setVisible(True)
        if self._progress_bar:
            self._progress_bar.setValue(100)

    def show_upload_failure(self, message: str) -> None:
        if self._status_label:
            self._status_label.setText(f"Upload failed: {message}")
            self._status_label.setStyleSheet(f"color: {C['danger']}; font-weight: bold;")
            self._status_label.setVisible(True)


# --- Map Page ---
class MapPage(QWidget):
    """Warehouse map page with left slice panel (320px) and right canvas.

    Layout matches the HTML prototype:
      page-head (title + subtitle)
      HBox:
        left: card (slice-panel) 320px — height input, gen/upload buttons, note
        right: canvas + map tag overlay
    """
    request_preview = Signal(float)
    upload_map = Signal(float)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("map_page")
        self._upload_dialog: Optional[_UploadDialog] = None
        self._map_widget: Optional[MapWidget] = None
        self._ros_thread = None
        self._current_room_id: str = ""
        self._slice_height_value: float = 1.5
        self._setup_ui()

    def set_ros_thread(self, ros_thread) -> None:
        self._ros_thread = ros_thread

    def set_room_id(self, room_id: str) -> None:
        self._current_room_id = room_id

    # ── UI setup ─────────────────────────────────────────────
    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Page header ──────────────────────────────────────
        header = QHBoxLayout()
        header.setContentsMargins(20, 16, 20, 12)

        title = QLabel("地图预览")
        title.setObjectName("page_title")
        header.addWidget(title)

        subtitle = QLabel("点云切片与廒间边界")
        subtitle.setObjectName("page_sub")
        header.addWidget(subtitle)

        header.addStretch()
        root.addLayout(header)

        # ── Body: left panel (320px) + right canvas ─────────
        body = QHBoxLayout()
        body.setContentsMargins(12, 0, 12, 12)
        body.setSpacing(12)

        # --- Left: slice-panel card ---
        self._slice_panel = self._build_slice_panel()
        body.addWidget(self._slice_panel, alignment=Qt.AlignmentFlag.AlignTop)

        # --- Right: canvas area ---
        self._canvas_area = self._build_canvas_area()
        body.addWidget(self._canvas_area, stretch=1)

        root.addLayout(body, stretch=1)

    def _build_slice_panel(self) -> QFrame:
        """Build the left panel: height input, buttons, hint, note."""
        card = QFrame()
        card.setObjectName("card")
        card.setFixedWidth(240)

        layout = QVBoxLayout(card)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)

        # Field label
        label = QLabel("切片高度（m）")
        label.setObjectName("field_label")
        layout.addWidget(label)

        # Height input
        self._height_spin = QDoubleSpinBox()
        self._height_spin.setRange(0.0, 5.0)
        self._height_spin.setSingleStep(0.1)
        self._height_spin.setValue(self._slice_height_value)
        self._height_spin.valueChanged.connect(self._on_height_changed)
        layout.addWidget(self._height_spin)

        # Hint
        hint = QLabel("有效范围 0.0 – 5.0 m")
        hint.setObjectName("field_hint")
        layout.addWidget(hint)

        # Error (hidden by default)
        self._slice_error = QLabel("高度需在 0.0 – 5.0 m 之间")
        self._slice_error.setObjectName("field_error")
        self._slice_error.setVisible(False)
        layout.addWidget(self._slice_error)

        layout.addSpacing(4)

        # Generate slice button
        gen_btn = QPushButton("生成切面")
        gen_btn.setObjectName("btn_primary")
        gen_btn.clicked.connect(self._on_generate_slice)
        layout.addWidget(gen_btn)

        # Upload slice button
        upload_btn = QPushButton("上传切面")
        upload_btn.clicked.connect(self._show_upload)
        layout.addWidget(upload_btn)

        layout.addSpacing(8)

        # Panel note
        self._slice_note = QLabel(
            f"当前显示 {self._slice_height_value:.1f} m 切面（示例数据）。"
            "修改高度后点击「生成切面」刷新。"
        )
        self._slice_note.setObjectName("field_hint")
        self._slice_note.setWordWrap(True)
        layout.addWidget(self._slice_note)

        layout.addStretch()
        return card

    def _build_canvas_area(self) -> QWidget:
        """Build the right canvas area with map widget and tag overlay."""
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Map widget
        self._map_widget = MapWidget()
        layout.addWidget(self._map_widget, stretch=1)

        # Map tag overlay (bottom-left)
        self._map_tag = QLabel(f"切片高度 <strong>{self._slice_height_value:.1f}</strong> m · 廒间 —")
        self._map_tag.setObjectName("map_tag")
        layout.addWidget(self._map_tag, alignment=Qt.AlignmentFlag.AlignLeft)

        return container

    # ── Event handlers ──────────────────────────────────────
    def _on_height_changed(self, value: float) -> None:
        self._slice_height_value = value
        self._slice_error.setVisible(False)

    def _on_generate_slice(self) -> None:
        """Extract slice at current height and update map display."""
        h = self._slice_height_value

        if h < 0.0 or h > 5.0:
            self._slice_error.setVisible(True)
            return
        self._slice_error.setVisible(False)

        self._slice_note.setText(f"当前显示 {h:.1f} m 切面。修改高度后点击「生成切面」刷新。")
        self._map_tag.setText(f"切片高度 <strong>{h:.1f}</strong> m · 廒间 —")

        self.request_preview.emit(h)

        self._slice_note.setText("联想地图服务尚未提供该高度的切面预览")

    def _show_upload(self) -> None:
        if self._upload_dialog is None:
            getter = self._ros_thread.get_latest_laser_map if self._ros_thread else None
            self._upload_dialog = _UploadDialog(self, laser_map_getter=getter, room_id=self._current_room_id)
        # Sync current height into the dialog
        self._upload_dialog._height_spin.setValue(self._slice_height_value)
        self._upload_dialog.show()

    # ── Properties ──────────────────────────────────────────
    @property
    def map_widget(self) -> MapWidget:
        return self._map_widget
