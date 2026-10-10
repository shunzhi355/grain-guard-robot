"""Mapping page — SLAM mapping controls, status, and saved maps list.

Matches grain-sampling-console.html #screen-mapping prototype layout:
  card grid: 建图控制 (top-left) + 保存地图 (top-right) + 已存地图列表 (bottom, full-width)
"""
from __future__ import annotations

from dataclasses import dataclass
import logging
import os
import subprocess
import threading
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from grain_sampling_workflow.remote_slam_bridge import RemoteSlamBridge as SlamBridge

try:
    from PySide2.QtCore import Qt, Signal, Slot
    from PySide2.QtWidgets import (
        QApplication, QFrame, QHBoxLayout, QLabel, QLineEdit,
        QListWidget, QListWidgetItem, QMessageBox, QPushButton,
        QVBoxLayout, QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, Signal, Slot  # type: ignore
    from PySide6.QtWidgets import (  # type: ignore
        QApplication, QFrame, QHBoxLayout, QLabel, QLineEdit,
        QListWidget, QListWidgetItem, QMessageBox, QPushButton,
        QVBoxLayout, QWidget,
    )

from grain_sampling_ui.theme import THEME_COLORS
from grain_sampling_workflow.map_save import (
    MapFileIdentity,
    copy_fresh_map,
    snapshot_map,
)

PCD_DIR = os.path.expanduser(os.getenv("SFAST_PCD_DIR", "~/fastlio2_ws/src/S-FAST_LIO/PCD"))
_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class _MapSaveOutcome:
    success: bool
    destination: str | None = None
    error: str | None = None


class MappingPage(QWidget):
    start_mapping_requested = Signal()
    stop_mapping_requested = Signal()
    save_map_requested = Signal()
    _save_finished = Signal(object)

    MAPPING_STATES = {
        "idle": ("空闲", THEME_COLORS["text_secondary"]),
        "mapping": ("建图中", THEME_COLORS["success"]),
        "saved": ("已保存", THEME_COLORS["info"]),
        "error": ("错误", THEME_COLORS["danger"]),
        "relocalizing": ("重定位中", THEME_COLORS["info"]),
    }

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("mapping_page")
        self._current_state: str = "idle"
        self._slam_bridge: Optional[SlamBridge] = None  # type: ignore[name-defined]
        self._map_saved_in_session: bool = False
        self._saved_map_path: str | None = None
        self._save_in_progress = False
        self._save_thread: threading.Thread | None = None
        self._setup_ui()
        self._save_finished.connect(self._on_save_finished)
        self._refresh_map_list()

    # ── UI setup ──────────────────────────────────────────────────────

    def _setup_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 24, 24, 24)
        root.setSpacing(20)

        # ── Top row: two cards side-by-side ──
        top_row = QHBoxLayout()
        top_row.setSpacing(16)
        top_row.addWidget(self._create_control_card(), 1)
        top_row.addWidget(self._create_save_card(), 1)
        root.addLayout(top_row)

        # ── Bottom: full-width map list card ──
        root.addWidget(self._create_map_list_card(), 1)
        root.addStretch()

    # ── Card 1: 建图控制 ─────────────────────────────────────────────

    def _create_control_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        h3 = QLabel("建图控制")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # Status row: dot + state text
        status_row = QHBoxLayout()
        status_row.setSpacing(10)

        self._mapping_dot = QLabel("●")
        self._mapping_dot.setStyleSheet(
            f"color: {THEME_COLORS['text_secondary']}; font-size: 10pt; background: transparent;"
        )
        self._mapping_dot.setFixedWidth(18)

        self._mapping_state_label = QLabel("建图未启动")
        self._mapping_state_label.setStyleSheet(
            f"font-size: 16px; color: {THEME_COLORS['text_secondary']}; background: transparent;"
        )

        status_row.addWidget(self._mapping_dot)
        status_row.addWidget(self._mapping_state_label)
        status_row.addStretch()
        layout.addLayout(status_row)

        # Toggle button
        self._btn_toggle = QPushButton("启动建图")
        self._btn_toggle.setObjectName("btn_success")
        self._btn_toggle.clicked.connect(self._on_toggle_mapping)
        layout.addWidget(self._btn_toggle)

        layout.addStretch()
        return card

    # ── Card 2: 保存当前地图 ─────────────────────────────────────────

    def _create_save_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        h3 = QLabel("保存当前地图")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        layout.addWidget(h3)

        # Input + save button row
        save_row = QHBoxLayout()
        save_row.setSpacing(10)

        self._map_name_input = QLineEdit()
        self._map_name_input.setPlaceholderText("地图名称，例如：三号廒间-复测")

        self._btn_save = QPushButton("保存")
        self._btn_save.setObjectName("btn_primary")
        self._btn_save.clicked.connect(self._on_save_clicked)

        save_row.addWidget(self._map_name_input, 1)
        save_row.addWidget(self._btn_save)
        layout.addLayout(save_row)

        # Inline feedback keeps the save workflow non-blocking.
        self._save_hint = QLabel("保存后可在下方列表中加载。")
        self._save_hint.setObjectName("field_hint")
        layout.addWidget(self._save_hint)

        layout.addStretch()
        return card

    # ── Card 3: 已保存地图 ──────────────────────────────────────────

    def _create_map_list_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        # Header row: title + refresh
        header_row = QHBoxLayout()
        h3 = QLabel("已保存地图")
        h3.setStyleSheet(
            f"font-size: 12pt; font-weight: bold; color: {THEME_COLORS['text_primary']}; background: transparent;"
        )
        header_row.addWidget(h3)
        header_row.addStretch()

        refresh_btn = QPushButton("刷新列表")
        refresh_btn.setFixedHeight(28)
        refresh_btn.clicked.connect(self._refresh_map_list)
        header_row.addWidget(refresh_btn)
        layout.addLayout(header_row)

        # Map list
        self._map_list = QListWidget()
        self._map_list.setMinimumHeight(120)
        self._map_list.setStyleSheet(
            "QListWidget {"
            f"  background-color: {THEME_COLORS['bg_dark']};"
            f"  border: 1px solid {THEME_COLORS['border']};"
            "   border-radius: 6px; padding: 4px;"
            "}"
            "QListWidget::item {"
            f"  color: {THEME_COLORS['text_primary']}; padding: 8px;"
            "}"
            "QListWidget::item:selected {"
            f"  background-color: {THEME_COLORS['accent']};"
            "}"
        )
        layout.addWidget(self._map_list, 1)

        return card

    # ── Map list refresh ──────────────────────────────────────────────

    def _refresh_map_list(self) -> None:
        self._map_list.clear()
        if self._slam_bridge is not None and getattr(self._slam_bridge, "is_remote", False):
            maps = self._slam_bridge.list_maps()
            for entry in maps:
                item = QListWidgetItem(f"{entry.get('name', entry.get('map_id', '未命名'))}    {entry.get('map_id', '')}")
                item.setToolTip(entry.get("map_id", ""))
                self._map_list.addItem(item)
            if not maps:
                self._map_list.addItem("(联想主机暂无地图或未连接)")
            return
        if not os.path.isdir(PCD_DIR):
            item = QListWidgetItem("(PCD 目录不存在)")
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            self._map_list.addItem(item)
            return
        files = sorted([f for f in os.listdir(PCD_DIR) if f.endswith(".pcd")])
        if not files:
            item = QListWidgetItem("(暂无地图文件)")
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsSelectable)
            self._map_list.addItem(item)
            return
        from datetime import datetime
        for f in files:
            path = os.path.join(PCD_DIR, f)
            size_mb = os.path.getsize(path) / (1024 * 1024)
            mod_time = os.path.getmtime(path)
            ts = datetime.fromtimestamp(mod_time).strftime("%Y-%m-%d %H:%M")
            item = QListWidgetItem(f"{f}    {size_mb:.1f}MB    {ts}")
            item.setToolTip(path)
            self._map_list.addItem(item)

    # ── Odometry check ────────────────────────────────────────────────

    def _check_odometry(self) -> bool:
        if self._slam_bridge is not None and getattr(self._slam_bridge, "is_remote", False):
            try:
                pose = self._slam_bridge.client.request("status").get("pose") or {}
                return pose.get("localization_valid") is True and pose.get("odom_age_ms", 9999) < 200
            except (OSError, RuntimeError, ValueError):
                return False
        try:
            result = subprocess.run(
                ["timeout", "2", "rostopic", "hz", "/Odometry"],
                capture_output=True, text=True, timeout=5,
                env={**os.environ, "ROS_MASTER_URI": "http://localhost:11311"}
            )
            return "average rate" in result.stdout
        except Exception:
            return False

    # ── SlamBridge injection ──────────────────────────────────────────

    def set_slam_bridge(self, bridge: SlamBridge) -> None:  # type: ignore[name-defined]
        """Receive a SlamBridge instance for SLAM service calls."""
        self._slam_bridge = bridge
        self._refresh_map_list()

    # ── Toggle mapping (start / stop) ─────────────────────────────────

    @Slot()
    def _on_toggle_mapping(self) -> None:
        """Toggle between starting and stopping mapping."""
        if self._current_state in ("mapping", "relocalizing"):
            self._on_stop_clicked()
        else:
            self._on_start_clicked()

    @Slot()
    def _on_start_clicked(self) -> None:
        if self._slam_bridge is None:
            self._transition_state("error")
            return
        # start_mapping() also stops relocalization first (mutually exclusive)
        if self._slam_bridge.start_mapping():
            self._map_saved_in_session = False
            self._saved_map_path = None
            self._transition_state("mapping")
            self.start_mapping_requested.emit()
        else:
            self._transition_state("error")

    @Slot()
    def _on_stop_clicked(self) -> None:
        if not self._map_saved_in_session:
            reply = QMessageBox.question(
                self, "确认停止", "还未保存地图，确定要停止吗？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        if self._slam_bridge is not None:
            self._slam_bridge.stop_mapping()
        # Back to idle — relocalization is triggered only by task selection
        # (MainWindow._on_task_selected), NOT by stopping mapping.
        self._transition_state("idle")
        self.stop_mapping_requested.emit()

    # ── Save map ──────────────────────────────────────────────────────

    def _set_save_feedback(self, message: str, color_key: str = "text_secondary") -> None:
        color = THEME_COLORS.get(color_key, THEME_COLORS["text_secondary"])
        self._save_hint.setText(message)
        self._save_hint.setStyleSheet(
            f"font-size: 7pt; color: {color}; background: transparent;"
        )

    @Slot()
    def _on_save_clicked(self) -> None:
        from datetime import datetime

        if self._save_in_progress:
            return

        # Always use the inline name input.  A modal fallback here can leave
        # the UI blocked after the map has already been saved, because the
        # dialog is not always dismissed by the window manager.
        name = self._map_name_input.text().strip()
        if not name:
            self._set_save_feedback("请先输入地图名称。", "danger")
            self._map_name_input.setFocus()
            return

        if self._slam_bridge is None:
            self._show_save_failure("地图保存失败：建图服务不可用")
            return

        if getattr(self._slam_bridge, "is_remote", False):
            self._save_in_progress = True
            self._btn_save.setEnabled(False)
            self._btn_toggle.setEnabled(False)
            self._save_thread = threading.Thread(
                target=self._save_remote_map_worker, args=(name,), daemon=True,
                name="remote-map-save")
            self._save_thread.start()
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_filename = f"{name}_{ts}.pcd"

        if not os.path.isdir(PCD_DIR):
            os.makedirs(PCD_DIR, exist_ok=True)

        existing = [f for f in os.listdir(PCD_DIR) if f.startswith(f"{name}_") and f.endswith(".pcd")]
        if existing:
            reply = QMessageBox.warning(
                self,
                "名称重复",
                f"已存在名为「{name}」的地图文件，是否覆盖？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return

        source = os.path.join(PCD_DIR, "GlobalMap.pcd")
        before = snapshot_map(source)
        destination = os.path.join(PCD_DIR, new_filename)
        old_paths = [os.path.join(PCD_DIR, filename) for filename in existing]

        self._save_in_progress = True
        self._mapping_state_label.setText("正在保存地图...")
        self._btn_save.setEnabled(False)
        self._btn_toggle.setEnabled(False)
        self._save_thread = threading.Thread(
            target=self._save_map_worker,
            args=(before, source, destination, old_paths),
            name="mapping-save",
            daemon=True,
        )
        self._save_thread.start()

    def _save_remote_map_worker(self, name: str) -> None:
        assert self._slam_bridge is not None
        if self._slam_bridge.save_current_map(name=name):
            self._save_finished.emit(_MapSaveOutcome(True, destination=name))
        else:
            self._save_finished.emit(_MapSaveOutcome(False, error=self._slam_bridge.last_error))

    def _save_map_worker(
        self,
        before: MapFileIdentity,
        source: str,
        destination: str,
        old_paths: list[str],
    ) -> None:
        """Finalize and copy a map without blocking the Qt GUI thread."""
        try:
            assert self._slam_bridge is not None
            if not self._slam_bridge.save_current_map(before_identity=before):
                raise RuntimeError("未检测到本轮生成的新 GlobalMap.pcd")
            copy_fresh_map(source, destination, before)
            for old_path in old_paths:
                if old_path != destination:
                    try:
                        os.remove(old_path)
                    except OSError:
                        _logger.warning("Could not remove replaced map: %s", old_path)
            outcome = _MapSaveOutcome(True, destination=destination)
        except Exception as exc:
            _logger.exception("Map save failed")
            outcome = _MapSaveOutcome(False, error=str(exc))
        self._save_finished.emit(outcome)

    @Slot(object)
    def _on_save_finished(self, outcome: _MapSaveOutcome) -> None:
        self._save_in_progress = False
        self._save_thread = None
        if not outcome.success or not outcome.destination:
            detail = outcome.error or "未检测到本轮生成的新 GlobalMap.pcd"
            self._show_save_failure(f"地图保存失败：{detail}")
            return

        self._map_saved_in_session = True
        self._saved_map_path = outcome.destination
        saved_name = os.path.basename(outcome.destination)
        self._map_name_input.clear()
        self._set_save_feedback(f"地图已保存：{saved_name}", "success")
        self._refresh_map_list()
        self._transition_state("saved")
        self.save_map_requested.emit()

    def _show_save_failure(self, message: str) -> None:
        self._save_in_progress = False
        self._transition_state("error")
        self._mapping_state_label.setText("地图保存失败")
        self._set_save_feedback(message, "danger")

    # ── State transitions ─────────────────────────────────────────────

    def _transition_state(self, state: str) -> None:
        self._current_state = state
        label, color = self.MAPPING_STATES.get(state, ("未知", THEME_COLORS["text_disabled"]))

        # Update dot and state label (matching HTML prototype pattern)
        self._mapping_dot.setStyleSheet(
            f"color: {color}; font-size: 10pt; background: transparent;"
        )
        self._mapping_state_label.setText(f"状态: {label}")
        self._mapping_state_label.setStyleSheet(
            f"font-size: 16px; color: {color}; background: transparent;"
        )

        is_mapping = state == "mapping"
        is_relocalizing = state == "relocalizing"

        # Update toggle button text and style
        if is_mapping or is_relocalizing:
            self._btn_toggle.setText("停止建图")
            self._btn_toggle.setObjectName("btn_danger")
        else:
            self._btn_toggle.setText("启动建图")
            self._btn_toggle.setObjectName("btn_success")

        # Refresh the button style (PySide needs re-polish)
        self._btn_toggle.style().unpolish(self._btn_toggle)
        self._btn_toggle.style().polish(self._btn_toggle)

        self._btn_toggle.setEnabled(not self._save_in_progress)
        self._btn_save.setEnabled(is_mapping and not self._save_in_progress)
