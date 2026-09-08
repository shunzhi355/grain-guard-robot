"""Task list page (任务列表) for the grain sampling robot UI.

Rewritten to match grain-sampling-console.html card layout with chip indicators.
"""

from __future__ import annotations

import os
import threading
from collections import Counter
from datetime import datetime
from PySide2.QtCore import Qt, Signal, QTimer
from PySide2.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from grain_sampling_ui.theme import THEME_COLORS
from grain_sampling_cloud.protocol import TaskListRequest, TaskListResponse, OrderInfo, ApiPath
from grain_sampling_cloud.http_client import CloudHttpClient, CloudConnectionError, CloudProtocolError
from grain_sampling_workflow.mechanism_config import GRAIN_MECHANISM_CONFIG


C = THEME_COLORS

PCD_DIR = os.path.expanduser("~/fastlio2_ws/src/S-FAST_LIO/PCD")

# ── Stub task data ───────────────────────────────────────────────────────
SAMPLE_TASKS: list[dict[str, object]] = [
    {"id": "T-20250701-001", "warehouse": "粮仓A区-01号", "status": "未完成",
     "created": "2025-07-01 08:30",
     "waypoints": [{"x": 1.5, "y": 2.0}, {"x": 3.0, "y": 4.5}],
     "depth_list": [2.0], "depth": 2.0,
     "grain_type": "稻谷"},
    {"id": "T-20250702-002", "warehouse": "粮仓B区-03号", "status": "进行中",
     "created": "2025-07-02 09:00",
     "waypoints": [{"x": 5.0, "y": 1.0}, {"x": 7.5, "y": 3.0}, {"x": 10.0, "y": 6.0}],
     "depth_list": [3.0], "depth": 3.0,
     "grain_type": "稻谷"},
    {"id": "T-20250702-003", "warehouse": "粮仓A区-02号", "status": "已完成",
     "created": "2025-07-02 10:15",
     "waypoints": [{"x": 2.0, "y": 2.0}],
     "depth_list": [1.5], "depth": 1.5,
     "grain_type": "稻谷"},
    {"id": "T-20250703-004", "warehouse": "粮仓C区-01号", "status": "未完成",
     "created": "2025-07-03 07:45",
     "waypoints": [{"x": 8.0, "y": 0.0}, {"x": 12.0, "y": 4.0}],
     "depth_list": [2.5], "depth": 2.5,
     "grain_type": "稻谷"},
    {"id": "T-20250703-005", "warehouse": "粮仓C区-02号", "status": "已完成",
     "created": "2025-07-03 14:20",
     "waypoints": [{"x": 3.0, "y": 3.0}, {"x": 6.0, "y": 1.0}, {"x": 9.0, "y": 5.0}],
     "depth_list": [2.0], "depth": 2.0,
     "grain_type": "稻谷"},
]

# ── Status colour mapping ────────────────────────────────────────────────
STATUS_COLORS: dict[str, str] = {
    "未完成": "#D4A72C",
    "进行中": "#58A6FF",
    "已完成": "#2EA043",
}

STATUS_BADGES: dict[str, str] = {
    "未完成": "○ 未完成",
    "进行中": "● 进行中",
    "已完成": "✓ 已完成",
}

# ── Grain mechanism durations now live in mechanism_config ─────────────
# Grain type → full mechanism parameter dict (sampling/convey durations
# and actuator timings), defined in grain_sampling_workflow.mechanism_config
# as GRAIN_MECHANISM_CONFIG. This page derives the grain dropdown from it.

# ── 检测指标ID → 类别名称 ─────────────────────────────────────────────
JIANCE_CATEGORY_MAP: dict[int, str] = {
    1: "生化",
    2: "理化",
}


def format_jiance(jiance: list[int]) -> str:
    """Format jiance ID list as readable category labels, e.g. "生化(1), 理化(2)"."""
    return ", ".join(
        f"{JIANCE_CATEGORY_MAP.get(j, str(j))}({j})" for j in jiance
    )


def _build_jiance_chips(jiance: list[int]) -> list[tuple[str, str]]:
    """Return list of (label, objectName) for chip display.

    Example: [1, 2, 2] → [("生化 ×1", "chip_bio"), ("理化 ×2", "chip_phy")]
    """
    counts: Counter[str] = Counter()
    for j in jiance:
        cat = JIANCE_CATEGORY_MAP.get(j, str(j))
        counts[cat] += 1
    result: list[tuple[str, str]] = []
    for cat in ["生化", "理化"]:
        if cat in counts:
            obj_name = "chip_bio" if cat == "生化" else "chip_phy"
            result.append((f"{cat} ×{counts[cat]}", obj_name))
    # Any unrecognized categories
    for cat, cnt in counts.items():
        if cat not in ("生化", "理化"):
            result.append((f"{cat} ×{cnt}", "chip_phy"))
    return result


# ── Animated status label (for 进行中) ───────────────────────────────────
class _AnimatedStatusLabel(QFrame):
    """A status badge with animated dots for '进行中'."""

    def __init__(self, text: str, color: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._dot_count = 0
        self._base_text = text
        self._color = color

        self._label = QLabel(text)
        self._label.setStyleSheet(
            f"""
            font-size: 10pt; font-weight: bold; color: {color};
            background-color: {C["bg_medium"]}; border-radius: 6px; padding: 4px 12px;
            """
        )
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._animate)
        self._timer.start(600)

    def _animate(self) -> None:
        self._dot_count = (self._dot_count + 1) % 4
        dots = "." * self._dot_count
        self._label.setText(f"{self._base_text}{dots}")

    def stop_animation(self) -> None:
        self._timer.stop()


# ── Task card widget (unified for cloud + local) ──────────────────────────
class _TaskCardWidget(QFrame):
    """Single task card matching HTML .task-card layout.

    Grid: [checkbox 32px] [info 1fr] [aojian badge auto]
    Rows:  line1 (task-id) / line2 (chips) / line3 (detail)
    """

    task_selected = Signal(str)

    # Card variant
    VARIANT_CLOUD = "cloud"
    VARIANT_LOCAL = "local"

    def __init__(
        self,
        variant: str,
        task_id: str,
        chips: list[tuple[str, str]] | None = None,
        detail: str = "",
        aojian: str = "",
        task_data: dict[str, object] | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._variant = variant
        self._task_id = task_id
        self._task_data = task_data or {}
        self._selected = False

        self.setObjectName("task_card")
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._apply_style(selected=False)

        self._build_layout(chips or [], detail, aojian)

    def _apply_style(self, selected: bool) -> None:
        """Apply selected or unselected styles."""
        if selected:
            self.setStyleSheet(
                f"""
                #task_card_selected {{
                    background-color: {C["bg_dark"]};
                    border: 1px solid {C["accent"]};
                    border-radius: 8px;
                    padding: 16px 20px;
                }}
                """
            )
            self.setObjectName("task_card_selected")
        else:
            self.setObjectName("task_card")
            self.setStyleSheet("")  # Use global QSS from theme

    def _build_layout(
        self, chips: list[tuple[str, str]], detail: str, aojian: str
    ) -> None:
        """Build the 3-column grid layout."""
        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(14)

        # ── Column 0: Checkbox ──
        self._checkbox = QLabel()
        self._checkbox.setFixedSize(24, 24)
        self._checkbox.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._checkbox.setStyleSheet(
            f"""
            QLabel {{
                border-radius: 6px;
                border: 1px solid {C["border"]};
                background: transparent;
                color: transparent;
                font-size: 10pt;
            }}
            """
        )
        outer.addWidget(self._checkbox, alignment=Qt.AlignmentFlag.AlignTop)

        # ── Column 1: Info (task_id / chips / detail) ──
        info_layout = QVBoxLayout()
        info_layout.setSpacing(4)
        info_layout.setContentsMargins(0, 0, 0, 0)

        # Line 1: order_id / task_id (mono bold)
        id_label = QLabel(self._task_id)
        id_label.setObjectName("task_id")
        info_layout.addWidget(id_label)

        # Line 2: chips
        if chips:
            chip_layout = QHBoxLayout()
            chip_layout.setSpacing(6)
            chip_layout.setContentsMargins(0, 0, 0, 0)
            for text, obj_name in chips:
                chip = QLabel(text)
                chip.setObjectName(obj_name)
                chip_layout.addWidget(chip)
            chip_layout.addStretch()
            info_layout.addLayout(chip_layout)

        # Line 3: detail row (gray sub-text)
        if detail:
            detail_label = QLabel(detail)
            detail_label.setObjectName("task_line3")
            info_layout.addWidget(detail_label)

        outer.addLayout(info_layout, stretch=1)

        # ── Column 2: Aojian badge (top-right) ──
        if aojian:
            ao_badge = QLabel(aojian)
            ao_badge.setObjectName("task_ao")
            outer.addWidget(ao_badge, alignment=Qt.AlignmentFlag.AlignTop)

    def set_selected(self, selected: bool) -> None:
        """Update visual selection state."""
        self._selected = selected
        self._apply_style(selected)
        if selected:
            self._checkbox.setStyleSheet(
                f"""
                QLabel {{
                    background-color: {C["accent"]};
                    border: 1px solid {C["accent"]};
                    color: #FFFFFF;
                    border-radius: 6px;
                    font-size: 10pt;
                }}
                """
            )
            self._checkbox.setText("✓")
        else:
            self._checkbox.setStyleSheet(
                f"""
                QLabel {{
                    border-radius: 6px;
                    border: 1px solid {C["border"]};
                    background: transparent;
                    color: transparent;
                    font-size: 10pt;
                }}
                """
            )
            self._checkbox.setText("")

    def mousePressEvent(self, event) -> None:
        """Click anywhere on the card to select it."""
        self.task_selected.emit(self._task_id)
        super().mousePressEvent(event)

    @property
    def task_id(self) -> str:
        return self._task_id

    @property
    def task_data(self) -> dict[str, object]:
        return self._task_data


# ── Local task creation dialog ────────────────────────────────────────────
class _CreateTaskDialog(QDialog):
    """Dialog to create a local task."""

    task_created = Signal(dict)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("本地创建工单")
        self.setMinimumWidth(400)
        self.setStyleSheet(
            f"""
            QDialog {{
                background-color: {C["bg_darkest"]};
                border: 2px solid {C["border"]};
                border-radius: 10px;
            }}
            QLabel {{
                font-size: 9pt; color: {C["text_secondary"]};
            }}
            """
        )
        self._depth_inputs: list[QComboBox] = []
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(16)

        title = QLabel("创建本地工单")
        title.setStyleSheet(f"font-size: 12pt; font-weight: bold; color: {C['text_primary']};")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        form = QFormLayout()
        form.setSpacing(10)

        self._warehouse_input = QComboBox()
        self._warehouse_input.setEditable(False)
        self._warehouse_input.currentIndexChanged.connect(self._on_warehouse_changed)
        form.addRow("仓号:", self._warehouse_input)
        self._refresh_warehouse_list()

        self._grain_input = QComboBox()
        self._grain_input.addItems(list(GRAIN_MECHANISM_CONFIG.keys()))
        form.addRow("粮食品种:", self._grain_input)

        # Depth selectors (up to 3)
        depth_options = ["无"] + [f"{d:.1f} 米" for d in
                          [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]]
        for i in range(3):
            lbl = QLabel(f"深度{i+1}:")
            cmb = QComboBox()
            cmb.addItems(depth_options)
            if i == 0:
                cmb.setCurrentIndex(4)
            else:
                cmb.setCurrentIndex(0)
            self._depth_inputs.append(cmb)
            form.addRow(lbl, cmb)

        layout.addLayout(form)

        # Waypoint list
        wp_label = QLabel("采样点位:")
        wp_label.setStyleSheet(f"font-size: 10pt; color: {C['text_secondary']};")
        layout.addWidget(wp_label)

        self._waypoint_list = QListWidget()
        self._waypoint_list.setMinimumHeight(80)
        self._waypoint_list.setStyleSheet(
            f"""
            QListWidget {{
                background-color: {C["bg_dark"]};
                border: 1px solid {C["border"]};
                border-radius: 6px;
                color: {C["text_primary"]};
                font-size: 10pt;
            }}
            QListWidget::item {{
                padding: 4px 8px;
            }}
            QListWidget::item:selected {{
                background-color: {C["accent"]};
            }}
            """
        )
        layout.addWidget(self._waypoint_list)

        self._waypoint_list.currentRowChanged.connect(self._on_waypoint_selected)

        # X/Y inputs
        coord_layout = QHBoxLayout()
        coord_layout.setSpacing(8)
        coord_layout.addWidget(QLabel("X:"))
        self._x_input = QDoubleSpinBox()
        self._x_input.setRange(-50.0, 50.0)
        self._x_input.setDecimals(2)
        self._x_input.setSingleStep(0.1)
        self._x_input.setValue(0.0)
        coord_layout.addWidget(self._x_input)
        coord_layout.addWidget(QLabel("Y:"))
        self._y_input = QDoubleSpinBox()
        self._y_input.setRange(-50.0, 50.0)
        self._y_input.setDecimals(2)
        self._y_input.setSingleStep(0.1)
        self._y_input.setValue(0.0)
        coord_layout.addWidget(self._y_input)
        layout.addLayout(coord_layout)

        wp_btn_layout = QHBoxLayout()
        wp_btn_layout.setSpacing(8)

        add_wp_btn = QPushButton("添加点位")
        add_wp_btn.setObjectName("btn_info")
        add_wp_btn.clicked.connect(self._add_waypoint)
        wp_btn_layout.addWidget(add_wp_btn)

        del_wp_btn = QPushButton("删除选中点位")
        del_wp_btn.setObjectName("btn_danger")
        del_wp_btn.clicked.connect(self._remove_waypoint)
        wp_btn_layout.addWidget(del_wp_btn)

        wp_btn_layout.addStretch()
        layout.addLayout(wp_btn_layout)

        # Default waypoint
        self._waypoint_list.addItem("点位1: X=0.0  Y=0.0")

        # Error label
        self._error_label = QLabel("")
        self._error_label.setStyleSheet(
            f"font-size: 9pt; font-weight: bold; color: {C['danger']}; padding: 4px 0px;"
        )
        self._error_label.setVisible(False)
        layout.addWidget(self._error_label)

        # Buttons
        btn_layout = QHBoxLayout()
        btn_layout.addStretch()

        cancel_btn = QPushButton("取消")
        cancel_btn.setObjectName("btn_danger")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(cancel_btn)

        self._create_btn = QPushButton("创建")
        self._create_btn.setObjectName("btn_primary")
        self._create_btn.clicked.connect(self._on_create)
        btn_layout.addWidget(self._create_btn)

        layout.addLayout(btn_layout)

        if self._warehouse_input.count() == 0:
            self._create_btn.setEnabled(False)
            self._error_label.setText("暂无地图，请先建图")
            self._error_label.setVisible(True)

    def _on_warehouse_changed(self, _index: int) -> None:
        if hasattr(self, "_error_label") and self._error_label.isVisible():
            self._error_label.setVisible(False)

    def _refresh_warehouse_list(self) -> None:
        self._warehouse_input.clear()
        if not os.path.isdir(PCD_DIR):
            return
        warehouses: dict[str, str] = {}
        for f in os.listdir(PCD_DIR):
            if not f.endswith(".pcd"):
                continue
            base = f[:-4]
            parts = base.split("_", 1)
            if len(parts) < 2:
                continue
            name = parts[0]
            ts_str = parts[1] if len(parts) > 1 else ""
            if name not in warehouses or ts_str > warehouses[name]:
                warehouses[name] = ts_str
        for name in sorted(warehouses.keys()):
            self._warehouse_input.addItem(name)

    def _add_waypoint(self) -> None:
        x = self._x_input.value()
        y = self._y_input.value()
        count = self._waypoint_list.count() + 1
        self._waypoint_list.addItem(f"点位{count}: X={x:.1f}  Y={y:.1f}")
        self._waypoint_list.setCurrentRow(self._waypoint_list.count() - 1)

    def _on_waypoint_selected(self, row: int) -> None:
        if row < 0:
            return
        item = self._waypoint_list.item(row)
        if item:
            wp = self._parse_waypoint_text(item.text())
            if wp:
                self._x_input.setValue(wp["x"])
                self._y_input.setValue(wp["y"])

    def _remove_waypoint(self) -> None:
        if self._waypoint_list.count() <= 1:
            return
        current_row = self._waypoint_list.currentRow()
        if current_row >= 0:
            self._waypoint_list.takeItem(current_row)
        import re
        for i in range(self._waypoint_list.count()):
            item = self._waypoint_list.item(i)
            if item:
                text = item.text()
                m = re.search(r"X=([\d.-]+)\s+Y=([\d.-]+)", text)
                if m:
                    x_val = m.group(1)
                    y_val = m.group(2)
                    item.setText(f"点位{i + 1}: X={x_val}  Y={y_val}")

    @staticmethod
    def _parse_waypoint_text(text: str) -> dict[str, float] | None:
        import re
        m = re.search(r"X=([\d.-]+)\s+Y=([\d.-]+)", text)
        if m:
            try:
                return {"x": float(m.group(1)), "y": float(m.group(2))}
            except ValueError:
                pass
        return None

    def _on_create(self) -> None:
        warehouse = self._warehouse_input.currentText()
        if not warehouse:
            self._error_label.setText("暂无地图，请先建图")
            self._error_label.setVisible(True)
            return

        waypoints: list[dict[str, float]] = []
        for i in range(self._waypoint_list.count()):
            item = self._waypoint_list.item(i)
            if item:
                wp = self._parse_waypoint_text(item.text())
                if wp:
                    waypoints.append(wp)

        if not waypoints:
            self._error_label.setText("至少需要一个有效点位！")
            self._error_label.setVisible(True)
            return

        depth_list: list[float] = []
        for cmb in self._depth_inputs:
            text = cmb.currentText()
            if text != "无":
                depth_list.append(float(text.replace(" 米", "")))
        if not depth_list:
            depth_list = [2.0]

        grain_type = self._grain_input.currentText()
        data = {
            "warehouse": warehouse,
            "grain_type": grain_type,
            "depth_list": depth_list,
            "waypoints": waypoints,
            "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.task_created.emit(data)
        self.accept()


# ── Task List Page ────────────────────────────────────────────────────────
class TaskListPage(QWidget):
    """Task list page (任务列表) — cloud order pulling, sampling task queue, and local creation.

    Features HTML-matching layout:
      - Tabs: 云端任务 / 本地调试
      - Cloud pane: fetch button + card list + accept button
      - Local pane: filter bar + card list + create button
      - Cards: checkbox + order_id(mono bold) + chips + detail + aojian badge
    """

    task_selected = Signal(dict)
    task_created = Signal(dict)

    _cloud_tasks_ready = Signal(list)
    _cloud_pull_error = Signal(str)
    _cloud_accept_success = Signal(OrderInfo)
    _cloud_accept_error = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("task_list_page")

        # Cloud client
        from utils.config import AppConfig
        try:
            self._cloud_client = CloudHttpClient.from_app_config(AppConfig())
        except Exception:
            self._cloud_client = CloudHttpClient()

        # State
        self._all_tasks = SAMPLE_TASKS.copy()
        self._active_filter = "全部"
        self._filter_buttons: dict[str, QPushButton] = {}
        self._cloud_orders: list[OrderInfo] = []
        self._selected_cloud_index: int = -1
        self._selected_local_index: int = -1
        self._cloud_cards: list[_TaskCardWidget] = []
        self._local_cards: list[_TaskCardWidget] = []

        # Wire cross-thread signals
        self._cloud_tasks_ready.connect(self._on_cloud_tasks_received)
        self._cloud_pull_error.connect(self._on_cloud_pull_error)
        self._cloud_accept_success.connect(self._on_accept_success)
        self._cloud_accept_error.connect(self._on_accept_error)

        self._setup_ui()
        self._refresh_local_list()

    # ── UI Setup ───────────────────────────────────────────────────
    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)

        # Page header
        page_head = QHBoxLayout()
        title = QLabel("任务列表")
        title.setObjectName("page_title")
        page_head.addWidget(title)
        page_head.addStretch()
        sub = QLabel("选择一项任务后承接执行")
        sub.setObjectName("page_sub")
        page_head.addWidget(sub)
        layout.addLayout(page_head)

        # ── Tabs ──
        tab_layout = QHBoxLayout()
        tab_layout.setSpacing(4)
        tab_layout.setContentsMargins(0, 0, 0, 0)

        self._tab_cloud = QPushButton("云端任务")
        self._tab_cloud.setCheckable(True)
        self._tab_cloud.setChecked(True)
        self._tab_cloud.setMinimumHeight(28)
        self._tab_cloud.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tab_cloud.clicked.connect(lambda: self._switch_tab(0))
        tab_layout.addWidget(self._tab_cloud)

        self._tab_local = QPushButton("本地调试")
        self._tab_local.setCheckable(True)
        self._tab_local.setChecked(False)
        self._tab_local.setMinimumHeight(28)
        self._tab_local.setCursor(Qt.CursorShape.PointingHandCursor)
        self._tab_local.clicked.connect(lambda: self._switch_tab(1))
        tab_layout.addWidget(self._tab_local)

        tab_layout.addStretch()
        layout.addLayout(tab_layout)
        self._apply_tab_styles()

        # ── Stacked panes ──
        self._stack = QStackedWidget()
        self._stack.setStyleSheet("QStackedWidget { background: transparent; }")

        # Page 0: Cloud
        self._cloud_page = self._build_cloud_page()
        self._stack.addWidget(self._cloud_page)

        # Page 1: Local
        self._local_page = self._build_local_page()
        self._stack.addWidget(self._local_page)

        layout.addWidget(self._stack, stretch=1)

    def _apply_tab_styles(self) -> None:
        """Style tabs to match HTML .tabs / .tab.active."""
        active_style = (
            "QPushButton {"
            f"  background-color: {C['accent']};"
            "  color: #FFFFFF;"
            "  border: none;"
            "  border-radius: 6px;"
            "  padding: 0px 14px;"
            "  font-size: 10pt;"
            "  font-weight: bold;"
            "}"
        )
        inactive_style = (
            "QPushButton {"
            "  background: transparent;"
            f"  color: {C['text_secondary']};"
            "  border: none;"
            "  border-radius: 6px;"
            "  padding: 0px 14px;"
            "  font-size: 10pt;"
            "  font-weight: normal;"
            "}"
        )

        self._tab_cloud.setStyleSheet(active_style if self._tab_cloud.isChecked() else inactive_style)
        self._tab_local.setStyleSheet(active_style if self._tab_local.isChecked() else inactive_style)

    def _switch_tab(self, index: int) -> None:
        """Switch between cloud (0) and local (1) tabs."""
        self._tab_cloud.setChecked(index == 0)
        self._tab_local.setChecked(index == 1)
        self._apply_tab_styles()
        self._stack.setCurrentIndex(index)

    # ── Cloud page ────────────────────────────────────────────────
    def _build_cloud_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("pane_cloud")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(14)

        self._pull_cloud_btn = QPushButton("获取云端任务")
        self._pull_cloud_btn.setObjectName("btn_primary")
        self._pull_cloud_btn.setMinimumWidth(160)
        self._pull_cloud_btn.clicked.connect(self._pull_cloud_tasks)
        if "xxx" in self._cloud_client.base_url:
            self._pull_cloud_btn.setEnabled(False)
            self._pull_cloud_btn.setToolTip("云端未配置，请配置后重试")
        toolbar.addWidget(self._pull_cloud_btn)

        toolbar.addStretch()

        self._accept_btn = QPushButton("承接")
        self._accept_btn.setObjectName("btn_success")
        self._accept_btn.setMinimumWidth(120)
        self._accept_btn.setEnabled(False)
        self._accept_btn.clicked.connect(self._accept_cloud_task)
        toolbar.addWidget(self._accept_btn)

        layout.addLayout(toolbar)

        # Cloud task list
        self._cloud_list_widget = QListWidget()
        self._cloud_list_widget.setSpacing(6)
        self._cloud_list_widget.setStyleSheet(
            f"""
            QListWidget {{
                background-color: transparent;
                border: none;
            }}
            QListWidget::item {{
                background-color: transparent;
                border: none;
                padding: 0px;
            }}
            QListWidget::item:selected {{
                background-color: transparent;
            }}
            """
        )
        self._cloud_list_widget.currentRowChanged.connect(self._on_cloud_selection_changed)
        layout.addWidget(self._cloud_list_widget, stretch=1)

        return page

    # ── Local page ────────────────────────────────────────────────
    def _build_local_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("pane_local")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        # Toolbar
        toolbar = QHBoxLayout()
        toolbar.setSpacing(14)

        toolbar.addStretch()

        create_btn = QPushButton("本地创建工单")
        create_btn.setObjectName("btn_primary")
        create_btn.setMinimumWidth(160)
        create_btn.clicked.connect(self._open_create_task_dialog)
        toolbar.addWidget(create_btn)

        layout.addLayout(toolbar)

        # Filter bar
        filter_layout = QHBoxLayout()
        filter_layout.setSpacing(8)

        for idx, label in enumerate(["全部", "未完成", "进行中", "已完成"]):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setMinimumHeight(28)
            btn.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

            if idx == 0:
                btn.setStyleSheet(
                    f"""
                    QPushButton {{
                        background-color: {C["accent"]};
                        border-color: {C["accent"]};
                        color: {C["text_primary"]};
                        font-weight: bold;
                        font-size: 10pt;
                        border-radius: 6px;
                        padding: 4px 12px;
                        min-height: 20pt;
                    }}
                    """
                )
                btn.setChecked(True)

            btn.clicked.connect(lambda checked, f=label: self._on_filter_changed(f))
            self._filter_buttons[label] = btn
            filter_layout.addWidget(btn)

        filter_layout.addStretch()
        layout.addLayout(filter_layout)

        # Local task list
        self._local_list_widget = QListWidget()
        self._local_list_widget.setSpacing(6)
        self._local_list_widget.setStyleSheet(
            f"""
            QListWidget {{
                background-color: transparent;
                border: none;
            }}
            QListWidget::item {{
                background-color: transparent;
                border: none;
                padding: 0px;
            }}
            QListWidget::item:selected {{
                background-color: transparent;
            }}
            """
        )
        self._local_list_widget.currentRowChanged.connect(self._on_local_selection_changed)
        layout.addWidget(self._local_list_widget, stretch=1)

        return page

    # ── Cloud: pull tasks ──────────────────────────────────────────
    def _pull_cloud_tasks(self) -> None:
        """Pull cloud task list in a background thread."""
        self._pull_cloud_btn.setEnabled(False)
        self._pull_cloud_btn.setText("拉取中...")

        def _do_pull() -> None:
            try:
                resp = self._cloud_client.get(
                    ApiPath.TASK_LIST,
                    {"mac": self._cloud_client.mac_address},
                )
                orders_raw = resp.get("orders", []) if isinstance(resp, dict) else []
                orders: list[OrderInfo] = []
                for o in orders_raw:
                    orders.append(OrderInfo(
                        order_id=str(o.get("order_id", "")),
                        aojian_id=int(o.get("aojian_id", 0)),
                        aojian=str(o.get("aojian", "")),
                        depth_list=[float(d) for d in o.get("depth_list", [])],
                        jiance=[int(j) for j in o.get("jiance", [])],
                        points=[{"x": float(p["x"]), "y": float(p["y"])} for p in o.get("points", [])],
                        pinzhong_code=str(o.get("pinzhong_code", "")),
                        pinzhong=str(o.get("pinzhong", "")),
                    ))
                self._cloud_tasks_ready.emit(orders)
            except (CloudConnectionError, CloudProtocolError) as e:
                self._cloud_pull_error.emit(str(e))

        threading.Thread(target=_do_pull, daemon=True).start()

    def _on_cloud_tasks_received(self, orders: list[OrderInfo]) -> None:
        """Called on main thread after successful cloud pull."""
        self._pull_cloud_btn.setEnabled(True)
        self._pull_cloud_btn.setText("获取云端任务")
        self._cloud_orders = orders
        self._selected_cloud_index = -1
        self._accept_btn.setEnabled(False)
        self._refresh_cloud_list()

    def _on_cloud_pull_error(self, error_msg: str) -> None:
        """Called on main thread after cloud pull error."""
        self._pull_cloud_btn.setEnabled(True)
        self._pull_cloud_btn.setText("获取云端任务")
        QMessageBox.warning(self, "拉取失败", f"无法获取云端任务：\n{error_msg}")

    # ── Cloud: order list display ──────────────────────────────────
    def _refresh_cloud_list(self) -> None:
        self._cloud_list_widget.clear()
        self._cloud_cards.clear()
        for idx, order in enumerate(self._cloud_orders):
            chips = _build_jiance_chips(order.jiance)
            depth_strs = [f"{d:.0f}" for d in order.depth_list]
            depth_detail = " / ".join(depth_strs)
            first_pt = order.points[0] if order.points else {"x": 0, "y": 0}
            pinzhong_part = f"品种 {order.pinzhong} · " if order.pinzhong else ""
            detail = (
                f"{pinzhong_part}点位 ({first_pt['x']:.0f}, {first_pt['y']:.0f})"
                f" · 采样深度 {len(order.depth_list)} 层：{depth_detail} m"
            )
            aojian_text = order.aojian if order.aojian else f"廒间 {order.aojian_id}"

            card = _TaskCardWidget(
                variant=_TaskCardWidget.VARIANT_CLOUD,
                task_id=order.order_id,
                chips=chips,
                detail=detail,
                aojian=aojian_text,
            )
            card.task_selected.connect(self._on_cloud_card_clicked)
            self._cloud_cards.append(card)

            item = QListWidgetItem()
            item.setSizeHint(card.sizeHint())
            self._cloud_list_widget.addItem(item)
            self._cloud_list_widget.setItemWidget(item, card)

    def _on_cloud_card_clicked(self, task_id: str) -> None:
        """Handle card click — find matching row and select it."""
        for i, card in enumerate(self._cloud_cards):
            if card.task_id == task_id:
                self._cloud_list_widget.setCurrentRow(i)
                return

    def _on_cloud_selection_changed(self, row: int) -> None:
        """Enable accept button when a cloud order is selected."""
        # Deselect previous
        if 0 <= self._selected_cloud_index < len(self._cloud_cards):
            self._cloud_cards[self._selected_cloud_index].set_selected(False)
        # Select new
        self._selected_cloud_index = row
        if 0 <= row < len(self._cloud_cards):
            self._cloud_cards[row].set_selected(True)
            self._accept_btn.setEnabled(True)
        else:
            self._accept_btn.setEnabled(False)

    # ── Cloud: accept task ─────────────────────────────────────────
    def _accept_cloud_task(self) -> None:
        """Accept the selected cloud order in a background thread."""
        if self._selected_cloud_index < 0:
            return
        order = self._cloud_orders[self._selected_cloud_index]
        self._accept_btn.setEnabled(False)
        self._accept_btn.setText("承接中...")

        def _do_accept() -> None:
            try:
                self._cloud_client.post(
                    ApiPath.ACCEPT_TASK,
                    {"order_id": order.order_id},
                )
                self._cloud_accept_success.emit(order)
            except (CloudConnectionError, CloudProtocolError) as e:
                self._cloud_accept_error.emit(str(e))

        threading.Thread(target=_do_accept, daemon=True).start()

    def _on_accept_success(self, order: OrderInfo) -> None:
        """Called on main thread after successful accept."""
        self._accept_btn.setEnabled(True)
        self._accept_btn.setText("承接")

        # 品种直接取云端 pinzhong，空则兜底"稻谷"（不再弹手动选择框）
        grain_type = order.pinzhong or "稻谷"

        waypoints = [{"x": p["x"], "y": p["y"]} for p in order.points]
        payload: dict = {
            "order_id": order.order_id,
            "aojian": order.aojian,
            "waypoints": waypoints,
            "depth_list": order.depth_list,
            "grain_type": grain_type,
            "pinzhong": order.pinzhong,
            "pinzhong_code": order.pinzhong_code,
            "jiance": order.jiance,
            "source": "cloud",
        }
        self.task_selected.emit(payload)

    def _on_accept_error(self, error_msg: str) -> None:
        """Called on main thread after accept error."""
        self._accept_btn.setEnabled(True)
        self._accept_btn.setText("承接")
        QMessageBox.warning(self, "承接失败", f"无法承接任务：\n{error_msg}")

    # ── Local task list display ────────────────────────────────────
    def _refresh_local_list(self) -> None:
        self._local_list_widget.clear()
        self._local_cards.clear()
        filtered = (
            self._all_tasks
            if self._active_filter == "全部"
            else [t for t in self._all_tasks if t["status"] == self._active_filter]
        )
        for task in filtered:
            task_id = str(task["id"])
            warehouse = str(task.get("warehouse", ""))
            status = str(task.get("status", ""))
            waypoints = task.get("waypoints", [])
            depth_list = task.get("depth_list", [2.0])

            # No chips for local tasks; show warehouse + status as detail
            wp_count = len(waypoints) if isinstance(waypoints, list) else 0
            if isinstance(depth_list, list) and depth_list:
                depth_strs = [f"{d:.0f}" for d in depth_list]
                depth_detail = " / ".join(depth_strs) + " m"
            else:
                depth_detail = "—"

            detail = f"{warehouse} · 点位 {wp_count}个 · 深度 {depth_detail} · {status}"

            card = _TaskCardWidget(
                variant=_TaskCardWidget.VARIANT_LOCAL,
                task_id=task_id,
                chips=[],
                detail=detail,
                aojian="",
                task_data=dict(task),
            )
            card.task_selected.connect(self._on_local_card_clicked)
            self._local_cards.append(card)

            item = QListWidgetItem()
            item.setSizeHint(card.sizeHint())
            self._local_list_widget.addItem(item)
            self._local_list_widget.setItemWidget(item, card)

    def _on_local_card_clicked(self, task_id: str) -> None:
        """Handle card click — find matching row and select it."""
        for i, card in enumerate(self._local_cards):
            if card.task_id == task_id:
                self._local_list_widget.setCurrentRow(i)
                return

    def _on_local_selection_changed(self, row: int) -> None:
        """Update visual selection for local task cards."""
        if 0 <= self._selected_local_index < len(self._local_cards):
            self._local_cards[self._selected_local_index].set_selected(False)
        self._selected_local_index = row
        if 0 <= row < len(self._local_cards):
            self._local_cards[row].set_selected(True)
            # Emit task_selected with payload
            self._on_local_task_selected(self._local_cards[row].task_id)
        else:
            self._selected_local_index = -1

    # ── Local task selection → dict payload ────────────────────────
    def _on_local_task_selected(self, task_id: str) -> None:
        """Map local task_id to dict payload and emit task_selected."""
        task = self.get_task_by_id(task_id)
        if task is None:
            return
        payload: dict = {
            "order_id": task["id"],
            "warehouse": task.get("warehouse", ""),
            "waypoints": task.get("waypoints", []),
            "depth_list": task.get("depth_list", [2.0]),
            "grain_type": task.get("grain_type", "稻谷"),
            "source": "local",
        }
        self.task_selected.emit(payload)

    # ── Filter logic ───────────────────────────────────────────────
    def _on_filter_changed(self, status: str) -> None:
        self._active_filter = status
        for label, btn in self._filter_buttons.items():
            if label == status:
                btn.setStyleSheet(
                    f"""
                    QPushButton {{
                        background-color: {C["accent"]};
                        border-color: {C["accent"]};
                        color: {C["text_primary"]};
                        font-weight: bold;
                        font-size: 10pt;
                        border-radius: 6px;
                        padding: 4px 12px;
                        min-height: 20pt;
                    }}
                    """
                )
                btn.setChecked(True)
            else:
                btn.setStyleSheet("")
                btn.setChecked(False)
        self._refresh_local_list()

    # ── Create task dialog ─────────────────────────────────────────
    def _open_create_task_dialog(self) -> None:
        dlg = _CreateTaskDialog(self)
        dlg.task_created.connect(self._on_task_created)
        dlg.task_created.connect(self.task_created.emit)
        dlg.exec()

    def _on_task_created(self, data: dict) -> None:
        """Add the newly created task to the local task list."""
        new_id = f"T-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        new_task = {
            "id": new_id,
            "warehouse": data["warehouse"],
            "grain_type": data.get("grain_type", "稻谷"),
            "status": "未完成",
            "created": data["created"],
            "waypoints": data.get("waypoints", []),
            "depth_list": data.get("depth_list", [2.0]),
            "depth": data.get("depth_list", [2.0])[0],
        }
        self._all_tasks.append(new_task)
        self._refresh_local_list()

    # ── Public helpers ─────────────────────────────────────────────
    def add_task(self, task: dict[str, object]) -> None:
        """Programmatically add a task (e.g., from cloud sync)."""
        self._all_tasks.append(task)
        self._refresh_local_list()

    def update_task_status(self, task_id: str, new_status: str) -> None:
        """Update the status of an existing task."""
        for t in self._all_tasks:
            if t["id"] == task_id:
                t["status"] = new_status
                break
        self._refresh_local_list()

    def get_filtered_tasks(self) -> list[dict[str, object]]:
        """Return tasks matching the active filter."""
        if self._active_filter == "全部":
            return list(self._all_tasks)
        return [t for t in self._all_tasks if t["status"] == self._active_filter]

    def get_task_by_id(self, task_id: str) -> dict[str, object] | None:
        """Return a copy of the task dict for the given task ID, or None."""
        for t in self._all_tasks:
            if t["id"] == task_id:
                return dict(t)
        return None

    @property
    def active_filter(self) -> str:
        return self._active_filter
