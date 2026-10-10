"""Main operation page for the grain sampling robot UI.

Displays the warehouse map with robot position overlay, waypoint markers,
navigation path, and a right-side status panel with live telemetry.

Integrates with the MainWindow's ROS thread and control panel via
the parent window reference.
"""

from __future__ import annotations

import logging


try:
    from PySide2.QtCore import Qt, QTimer, Signal, Slot
    from PySide2.QtGui import QImage, QPixmap
    from PySide2.QtWidgets import (
        QFrame,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QProgressBar,
        QPushButton,
        QSizePolicy,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, QTimer, Signal, Slot  # type: ignore[no-redef]
    from PySide6.QtGui import QImage, QPixmap  # type: ignore[no-redef]
    from PySide6.QtWidgets import (  # type: ignore[no-redef]
        QFrame,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QProgressBar,
        QPushButton,
        QSizePolicy,
        QVBoxLayout,
        QWidget,
    )

from grain_sampling_ui.widgets.map_widget import MapWidget
from grain_sampling_ui.theme import THEME_COLORS
from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)
from grain_sampling_workflow.robot_bridge import RobotBridge as SamplingBridge
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator

logger = logging.getLogger(__name__)

# ── Status panel styles ────────────────────────────────────
_STATUS_LABEL_STYLE = (
    "font-size: 13pt; color: #E6EDF3; background: transparent;"
)
_STATUS_VALUE_STYLE = (
    "font-size: 12pt; font-weight: bold; color: #58A6FF; background: transparent;"
    " font-family: 'Consolas', 'Courier New', monospace;"
)
_STATUS_TITLE_STYLE = (
    "font-size: 12pt; font-weight: bold; color: #8B949E;"
    " background: transparent; padding-bottom: 4px;"
)


def _status_group(title: str) -> QGroupBox:
    """Create a styled QGroupBox for the status panel."""
    group = QGroupBox(title)
    group.setStyleSheet(
        "QGroupBox {"
        "  background-color: #161B22;"
        "  border: 1px solid #484F58;"
        "  border-radius: 8px;"
        "  margin-top: 10pt;"
        "  padding: 12pt 10pt 10pt 10pt;"
        "  font-size: 13pt;"
        "  font-weight: bold;"
        "  color: #8B949E;"
        "}"
        "QGroupBox::title {"
        "  subcontrol-origin: margin;"
        "  left: 10px;"
        "  padding: 2px 6px;"
        "  color: #58A6FF;"
        "}"
    )
    return group


def _value_label() -> QLabel:
    """Create a label styled for telemetry values."""
    lbl = QLabel("--")
    lbl.setStyleSheet(_STATUS_VALUE_STYLE)
    lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    return lbl


def _row(label_text: str, value_widget: QWidget) -> QWidget:
    """Build a horizontal row with label + value."""
    row = QWidget()
    row.setStyleSheet("background: transparent;")
    layout = QHBoxLayout(row)
    layout.setContentsMargins(0, 2, 0, 2)
    layout.setSpacing(8)

    lbl = QLabel(label_text)
    lbl.setStyleSheet(_STATUS_LABEL_STYLE)
    layout.addWidget(lbl)
    layout.addStretch()
    layout.addWidget(value_widget)
    return row


# ═══════════════════════════════════════════════════════════════
# CameraWidget — live camera preview overlay
# ═══════════════════════════════════════════════════════════════


class CameraWidget(QWidget):
    """Live camera preview widget using OpenCV on /dev/video0."""

    def __init__(self, device: str = "/dev/video0", parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(200, 140)
        self.setStyleSheet(
            "background-color: #0D1117;"
            " border: 2px solid #30363D;"
            " border-radius: 8px;"
        )

        self._label = QLabel("Camera\nOffline")
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setStyleSheet(
            "color: #484F58; font-size: 12pt; border: none;"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

        self._cap = None
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_frame)
        self._start(device)

    def _start(self, device: str) -> None:
        """Open the camera device and start the frame timer."""
        try:
            import cv2  # noqa: PLC0415

            # On RK3588 the GStreamer backend can block indefinitely while
            # probing /dev/video0 and prevent the main window from appearing.
            # Use the requested device and the direct V4L2 backend instead.
            backend = cv2.CAP_V4L2 if str(device).startswith("/dev/video") else cv2.CAP_ANY
            self._cap = cv2.VideoCapture(device, backend)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                self._timer.start(66)  # ~15 fps
        except Exception:
            logger.warning("CameraWidget: OpenCV unavailable", exc_info=False)

    def _update_frame(self) -> None:
        """Read a frame from the camera and display it."""
        if self._cap is None or not self._cap.isOpened():
            return
        ok, frame = self._cap.read()
        if not ok:
            return

        import cv2  # noqa: PLC0415

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format.Format_RGB888)
        self._label.setPixmap(
            QPixmap.fromImage(img).scaled(
                self.width(),
                self.height(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def stop(self) -> None:
        """Stop the frame timer and release the camera."""
        self._timer.stop()
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def closeEvent(self, event) -> None:
        self.stop()
        super().closeEvent(event)


# ═══════════════════════════════════════════════════════════════
# MainPage
# ═══════════════════════════════════════════════════════════════


class MainPage(QWidget):
    """Main operation page — map + status panel."""

    # ── Signals ───────────────────────────────────────────
    start_task_requested = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("main_page")

        # ── Widgets ──────────────────────────────────────────
        self._map_widget: MapWidget | None = None
        self._camera: CameraWidget | None = None
        self._lbl_task_name: QLabel | None = None
        self._progress_bar: QProgressBar | None = None
        self._lbl_depth: QLabel | None = None

        # ── Workflow state ─────────────────────────────────
        self._orchestrator: WorkflowOrchestrator | None = None
        self._fsm: SamplingStateMachine | None = None
        self._bridge: SamplingBridge | None = None

        self._setup_ui()
        self._connect_signals()

    # ── UI construction ─────────────────────────────────────

    def _setup_ui(self) -> None:
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # ── Centre: Map widget ───────────────────────────────
        self._map_widget = MapWidget()
        self._map_widget.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        layout.addWidget(self._map_widget, stretch=1)

        # ── Right: Status panel ──────────────────────────────
        status_panel = self._build_status_panel()
        status_panel.setFixedWidth(180)
        layout.addWidget(status_panel)

        # ── Camera preview overlay (bottom-left of map area) ──
        self._camera = CameraWidget(parent=self)
        self._camera.move(12, self.height() - 160)

    def _build_status_panel(self) -> QWidget:
        """Create the right-side telemetry panel."""
        panel = QWidget()
        panel.setObjectName("status_panel")
        panel.setStyleSheet(
            "QWidget#status_panel { background-color: #0D1117; }"
        )

        outer = QVBoxLayout(panel)
        outer.setContentsMargins(4, 8, 4, 8)
        outer.setSpacing(8)

        # ── Task info ────────────────────────────────────────
        task_group = _status_group("当前任务")
        task_layout = QVBoxLayout(task_group)
        task_layout.setSpacing(6)

        self._lbl_task_name = QLabel("等待任务…")
        self._lbl_task_name.setStyleSheet(
            "font-size: 12pt; font-weight: bold; color: #E6EDF3;"
            " background: transparent; padding: 4px 0;"
        )
        self._lbl_task_name.setWordWrap(True)
        task_layout.addWidget(self._lbl_task_name)

        # Progress bar
        self._progress_bar = QProgressBar()
        self._progress_bar.setObjectName("task_progress")
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(True)
        self._progress_bar.setFormat("%p%")
        self._progress_bar.setFixedHeight(14)
        self._progress_bar.setStyleSheet(
            "QProgressBar {"
            "  background-color: #21262D;"
            "  border: 1px solid #30363D;"
            "  border-radius: 4px;"
            "  text-align: center;"
            "  color: #E6EDF3;"
            "  font-size: 9pt;"
            "}"
            "QProgressBar::chunk {"
            "  background-color: #2EA043;"
            "  border-radius: 3px;"
            "}"
        )
        task_layout.addWidget(self._progress_bar)

        outer.addWidget(task_group)

        # ── Mechanism status ─────────────────────────────────
        mech_group = _status_group("机构状态")
        mech_layout = QVBoxLayout(mech_group)
        mech_layout.setSpacing(6)

        self._lbl_depth = _value_label()
        mech_layout.addWidget(_row("扦样深度:", self._lbl_depth))

        outer.addWidget(mech_group)

        outer.addStretch()

        # ── Separator line ───────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(
            "QFrame { color: #30363D; background-color: #30363D;"
            " max-height: 1px; }"
        )
        outer.addWidget(sep)

        # ── Coordinate info (compact) ────────────────────────
        coord_lbl = QLabel("坐标信息")
        coord_lbl.setStyleSheet(
            "font-size: 12pt; color: #8B949E; background: transparent;"
            " padding-top: 4px;"
        )
        outer.addWidget(coord_lbl)

        return panel

    # ── Signal wiring ───────────────────────────────────────

    def _connect_signals(self) -> None:
        """Connect to MainWindow's ROS thread and control panel signals."""
        main_win = self.window()
        if main_win is None:
            logger.warning("MainPage has no parent window; ROS signals not wired")
            return

        # ── ROS data signals (use getattr for robustness) ───
        ros = getattr(main_win, "_ros_thread", None)
        if ros is not None:
            ros.odometry_updated.connect(self._on_odometry)
            ros.mechanism_status_updated.connect(self._on_mechanism_status)
            ros.map_updated.connect(self._on_map_data)
            logger.info("MainPage connected to ROS thread signals")
        else:
            logger.info("ROS thread not yet started; signals deferred")

        # ── Control panel button signals ─────────────────────
        control = getattr(main_win, "_control_panel", None)
        if control is not None:
            control.start_sampling.connect(self._on_start_sampling)
            control.pause_sampling.connect(self._on_pause_sampling)
            control.emergency_stop.connect(self._on_emergency_stop)
            control.return_to_charge.connect(self._on_return_to_charge)
            control.manual_control.connect(self._on_manual_control)
        else:
            logger.info("Control panel not yet available; signals deferred")

    # ── ROS data slots ─────────────────────────────────────

    def _on_odometry(self, x: float, y: float, yaw: float) -> None:
        """Handle incoming odometry data."""
        if self._map_widget:
            self._map_widget.update_odometry(x, y, yaw)

    def _on_mechanism_status(self, data: dict) -> None:
        """Handle /mechanism/status JSON updates."""
        if isinstance(data, dict):
            depth = data.get("sampling_depth_mm")
            if depth is not None and self._lbl_depth:
                self._lbl_depth.setText(f"{depth} mm")

            # Progress
            progress = data.get("completion_pct")
            if progress is not None and self._progress_bar:
                self._progress_bar.setValue(int(progress))

    def _on_map_data(self, data: dict) -> None:
        """Handle /map JSON updates."""
        if self._map_widget:
            self._map_widget.update_map(data)

    # ── Control button slots ────────────────────────────────

    def _on_start_sampling(self) -> None:
        """Start a new sampling operation."""
        logger.info("Start sampling requested")
        if self._lbl_task_name:
            self._lbl_task_name.setText("采样中…")
        if self._progress_bar:
            self._progress_bar.setValue(0)

    def _on_pause_sampling(self) -> None:
        """Pause the current sampling operation."""
        logger.info("Pause sampling requested")
        if self._lbl_task_name:
            self._lbl_task_name.setText("已暂停")

    def _on_emergency_stop(self) -> None:
        """Emergency stop — emit immediately, no confirmation dialog."""
        logger.warning("EMERGENCY STOP triggered")
        if self._lbl_task_name:
            self._lbl_task_name.setText("⚠ 急停")
        if self._progress_bar:
            self._progress_bar.setValue(0)

    def _on_return_to_charge(self) -> None:
        """Navigate robot back to charging dock."""
        logger.info("Return to charge requested")
        if self._lbl_task_name:
            self._lbl_task_name.setText("返回充电…")

    def _on_manual_control(self) -> None:
        """Enter manual remote-control mode."""
        logger.info("Manual control requested")
        if self._lbl_task_name:
            self._lbl_task_name.setText("手动遥控")

    # ── Public API ──────────────────────────────────────────

    def update_task_name(self, name: str) -> None:
        """Update the displayed task name (e.g. from work order)."""
        if self._lbl_task_name:
            self._lbl_task_name.setText(name)

    # ── Live telemetry update methods (task D) ──────────────

    def update_depth(self, depth_m: float) -> None:
        """Update the depth label with a live depth value in metres.

        Called by the orchestrator's ``depth_callback`` on every FSM
        transition that may change the current pipe index.
        """
        if self._lbl_depth:
            self._lbl_depth.setText("{:.1f} m".format(depth_m))

    def set_waypoints(
        self, waypoints: list[tuple[float, float, str]]
    ) -> None:
        """Push waypoint data to the map widget."""
        if self._map_widget:
            self._map_widget.update_waypoints(waypoints)

    def set_nav_path(self, path: list[tuple[float, float]]) -> None:
        """Push navigation path to the map widget."""
        if self._map_widget:
            self._map_widget.update_nav_path(path)

    @property
    def map_widget(self) -> MapWidget | None:
        """Expose the map widget for external access (e.g. tests)."""
        return self._map_widget
