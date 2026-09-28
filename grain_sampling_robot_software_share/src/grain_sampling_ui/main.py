"""Application entry point for the grain sampling robot UI.

Run with: ``grain-sampling-ui`` (console_scripts entry point)
or: ``python -m grain_sampling_ui.main``

Layout (matching grain-sampling-console.html prototype):
  StatusBar (60px) — brand + breadcrumb | spacer | stat indicators | clock
  HBox: NavPanel (120px) + QStackedWidget (content)
  AlarmBar (48px)
"""

from __future__ import annotations

import logging
import os
import sys
import time
from datetime import datetime
from typing import Optional

from PySide2.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QStackedWidget,
    QPushButton,
    QLabel,
    QSizePolicy,
    QMessageBox,
)
from PySide2.QtCore import Qt, QTimer, Slot
from PySide2.QtGui import QFont, QPixmap, QPainter, QColor, QBrush

# ── Internal modules ──────────────────────────────────────
from grain_sampling_ui.theme import THEME_QSS
from grain_sampling_ui.page_manager import PageManager
from grain_sampling_ui.ros_thread import ROSNodeThread, HAS_ROS

# Page stubs
from grain_sampling_ui.pages.main_page import MainPage
from grain_sampling_ui.pages.guidance_page import GuidancePage
from grain_sampling_ui.pages.map_page import MapPage
from grain_sampling_ui.pages.task_list_page import TaskListPage
from grain_sampling_ui.pages.mapping_page import MappingPage
from grain_sampling_ui.pages.settings_page import SettingsPage

# Widget stubs (alarm bar kept, StatusBar/ControlPanel replaced inline)
from grain_sampling_ui.widgets.map_widget import MapWidget  # referenced via _main_page
from grain_sampling_ui.widgets.alarm_bar import AlarmBar

# Workflow components (used for task selection flow)
from grain_sampling_workflow.state_machine import SamplingStateMachine
from grain_sampling_workflow.ros_bridge import SamplingBridge
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator
from grain_sampling_workflow.slam_bridge import SlamBridge

# RC manual control (remote CH5 switch owns the mode; UI mirrors it)
from grain_sampling_workflow.rc_control import RCControl
from grain_sampling_devices.rc_receiver import create_rc_receiver

# Cloud protocol models (OrderInfo for cloud task context)
from grain_sampling_cloud.protocol import OrderInfo, ApiPath, ReportStatus
from grain_sampling_cloud.http_client import CloudHttpClient
from grain_sampling_cloud.task_state import (
    clear_current_task,
    load_current_task,
    save_current_task,
)
from utils.config import AppConfig

logger = logging.getLogger(__name__)


def _env_flag(name: str) -> bool:
    """Return True only for an explicit, conventional truthy value."""
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


# ── Main Window ────────────────────────────────────────────

class MainWindow(QMainWindow):
    """Top-level window for the grain sampling robot UI."""

    # Page name → Chinese breadcrumb used in the status bar
    PAGE_NAMES: dict[str, str] = {
        "main": "主页面",
        "guidance": "作业引导",
        "map": "地图预览",
        "task_list": "任务列表",
        "mapping": "建图管理",
        "settings": "系统设置",
    }

    def __init__(self) -> None:
        super().__init__()
        self._page_manager: Optional[PageManager] = None
        self._ros_thread: Optional[ROSNodeThread] = None
        self._slam_bridge: Optional[SlamBridge] = None

        # Nav button active-state tracking: QPushButton → set of page names
        self._nav_mappings: list[tuple[QPushButton, set[str]]] = []

        # Clock timer
        self._clock_timer: Optional[QTimer] = None

        # Status bar labels (updated by ROS thread / clock)
        self._net_dot: Optional[QLabel] = None
        self._net_value: Optional[QLabel] = None
        self._ros_dot: Optional[QLabel] = None
        self._ros_value: Optional[QLabel] = None
        self._speed_value: Optional[QLabel] = None
        self._clock_label: Optional[QLabel] = None
        self._crumb_label: Optional[QLabel] = None

        # RC manual mode (方案 A: CH5 switch owns the mode, UI mirrors it)
        self._rc_control: Optional[RCControl] = None
        self._rc_timer: Optional[QTimer] = None
        self._rc_mode_mirror: Optional[str] = None
        self._rc_mode_updated_at = float("-inf")
        self._rc_mirror_timer: Optional[QTimer] = None
        self._rc_mirror_started = False
        self._manual_btn: Optional[QPushButton] = None
        self._mode_dot: Optional[QLabel] = None
        self._mode_value: Optional[QLabel] = None

        self._setup_window()
        self._build_ui()
        self._start_clock()

    # ── Window setup ────────────────────────────────────────

    def _setup_window(self) -> None:
        """Configure window properties."""
        self.setWindowTitle("粮食扦样机器人")
        self.setFixedSize(1024, 600)

    # ── UI construction ─────────────────────────────────────

    def _build_ui(self) -> None:
        """Assemble the UI layout matching the HTML prototype structure."""
        central = QWidget()
        central.setObjectName("central_widget")
        self.setCentralWidget(central)

        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ── Top: Status bar (60px) ─────────────────────────
        self._build_status_bar(root_layout)

        # ── Middle: NavPanel (120px) + StackedWidget ───────
        middle = QWidget()
        middle_layout = QHBoxLayout(middle)
        middle_layout.setContentsMargins(0, 0, 0, 0)
        middle_layout.setSpacing(0)

        self._build_nav_panel(middle_layout)

        self._stacked_widget = QStackedWidget()
        middle_layout.addWidget(self._stacked_widget, stretch=1)

        root_layout.addWidget(middle, stretch=1)

        # ── Bottom: Alarm bar (48px) ───────────────────────
        self._alarm_bar = AlarmBar()
        root_layout.addWidget(self._alarm_bar)

        # ── Page registration ──────────────────────────────
        self._register_pages()

    # ── Status bar (matching .statusbar in HTML prototype) ──

    def _build_status_bar(self, root_layout: QVBoxLayout) -> None:
        """Build the 60px top status bar with brand, stats, and clock."""
        bar = QWidget()
        bar.setObjectName("status_bar")
        bar.setFixedHeight(60)

        layout = QHBoxLayout(bar)
        layout.setContentsMargins(24, 0, 24, 0)
        layout.setSpacing(12)
        layout.setAlignment(Qt.AlignVCenter)

        # Brand label (non-clickable text, matching QLabel#brand in theme)
        brand_label = QLabel("粮食扦样机器人控制台")
        brand_label.setObjectName("brand")
        layout.addWidget(brand_label)

        # Breadcrumb (page name, updated on page change)
        self._crumb_label = QLabel("/ 主页面")
        self._crumb_label.setObjectName("crumb")
        layout.addWidget(self._crumb_label)

        layout.addStretch()

        # ── Network stat ──
        self._net_dot = self._make_dot(QColor("#DA3633"))  # red = disconnected
        layout.addWidget(self._net_dot)
        net_label = QLabel("网络")
        net_label.setObjectName("stat_label")
        layout.addWidget(net_label)
        self._net_value = QLabel("离线")
        self._net_value.setObjectName("stat_value")
        layout.addWidget(self._net_value)

        layout.addSpacing(16)

        # ── ROS stat ──
        self._ros_dot = self._make_dot(QColor("#DA3633"))
        layout.addWidget(self._ros_dot)
        ros_label = QLabel("ROS")
        ros_label.setObjectName("stat_label")
        layout.addWidget(ros_label)
        self._ros_value = QLabel("未运行")
        self._ros_value.setObjectName("stat_value")
        layout.addWidget(self._ros_value)

        layout.addSpacing(16)

        # ── Speed stat ──
        speed_dot = self._make_dot(QColor("#2EA043"))
        layout.addWidget(speed_dot)
        speed_label = QLabel("速度")
        speed_label.setObjectName("stat_label")
        layout.addWidget(speed_label)
        self._speed_value = QLabel("0.0 m/s")
        self._speed_value.setObjectName("stat_value")
        layout.addWidget(self._speed_value)

        layout.addSpacing(16)

        # ── Mode stat (RC manual/auto) ──
        self._mode_dot = self._make_dot(QColor("#8B949E"))  # grey = no RC signal
        layout.addWidget(self._mode_dot)
        mode_label = QLabel("模式")
        mode_label.setObjectName("stat_label")
        layout.addWidget(mode_label)
        self._mode_value = QLabel("未知")
        self._mode_value.setObjectName("stat_value")
        layout.addWidget(self._mode_value)

        layout.addSpacing(16)

        # ── Clock ──
        self._clock_label = QLabel("--:--:--")
        self._clock_label.setObjectName("clock")
        layout.addWidget(self._clock_label)

        root_layout.addWidget(bar)

    # ── Navigation panel (matching .navpanel in HTML prototype) ──

    def _build_nav_panel(self, parent_layout: QHBoxLayout) -> None:
        """Build the 120px wide left navigation panel with 4 nav buttons + estop."""
        panel = QWidget()
        panel.setObjectName("nav_panel")
        panel.setFixedWidth(120)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 12, 8, 12)
        layout.setSpacing(10)

        # Nav buttons — config: (text, handler, active_page_names)
        nav_configs: list[tuple[str, object, set[str]]] = [
            ("开始采样", self._on_start_sampling,  {"main", "task_list", "guidance"}),
            ("地图预览", lambda: self.page_manager.push("map"),    {"map"}),
            ("建图管理", lambda: self.page_manager.push("mapping"), {"mapping"}),
            ("系统设置", lambda: self.page_manager.push("settings"), {"settings"}),
        ]

        for text, handler, active_pages in nav_configs:
            btn = QPushButton(text)
            btn.setObjectName("nav_btn")
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            btn.clicked.connect(handler)
            layout.addWidget(btn)
            self._nav_mappings.append((btn, active_pages))

        layout.addStretch()

        # Manual mode switch button (RC manual mode — CH5 switch owns the
        # actual mode; this button starts/stops RCControl and shows state)
        self._manual_btn = QPushButton("手动模式")
        self._manual_btn.setObjectName("manual_btn")
        self._manual_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        self._manual_btn.clicked.connect(self._on_manual_mode_clicked)
        layout.addWidget(self._manual_btn)

        # Emergency stop button (at panel bottom via stretch above)
        estop_btn = QPushButton("急停")
        estop_btn.setObjectName("estop")
        estop_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        estop_btn.clicked.connect(self._on_emergency_stop)
        layout.addWidget(estop_btn)

        parent_layout.addWidget(panel)

    # ── Page registration ───────────────────────────────────

    def _register_pages(self) -> None:
        """Create page manager and register all pages."""
        self._page_manager = PageManager(self._stacked_widget, self)

        pages = [
            ("main", MainPage()),
            ("guidance", GuidancePage()),
            ("map", MapPage()),
            ("task_list", TaskListPage()),
            ("mapping", MappingPage()),
            ("settings", SettingsPage()),
        ]

        for name, widget in pages:
            self._page_manager.register_page(name, widget)

        self._page_manager.set_home("main")
        self._page_manager.page_changed.connect(self._on_page_changed)

        # Show default page
        self._page_manager.go_home()

        # Store page references for task selection flow
        self._guidance_page = pages[1][1]   # guidance page
        self._task_list_page = pages[3][1]  # task list page

        # Wire task selection signal
        self._task_list_page.task_selected.connect(self._on_task_selected)

        # Store main/map/mapping pages for ROS thread access
        self._main_page = pages[0][1]
        self._map_page = pages[2][1]
        self._mapping_page = pages[4][1]

    # ── Page changed handler (breadcrumb + nav active state) ──

    def _on_page_changed(self, name: str) -> None:
        """Handle page switch: update breadcrumb and nav button active state."""
        logger.debug("Page changed to: %s", name)

        # Update breadcrumb label
        crumb_text = self.PAGE_NAMES.get(name, name)
        if self._crumb_label is not None:
            self._crumb_label.setText(f"/ {crumb_text}")

        # Update active state on nav buttons
        for btn, active_pages in self._nav_mappings:
            is_active = name in active_pages
            btn.setProperty("active", "true" if is_active else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
            btn.update()

    @property
    def page_manager(self) -> PageManager:
        if self._page_manager is None:
            raise RuntimeError("PageManager not initialized")
        return self._page_manager

    # ── Control panel button handlers ──────────────────────

    def _send_udp(self, cmd: str) -> None:
        """Send UDP command to motor_driver."""
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.sendto(cmd.encode(), ("127.0.0.1", 8765))
            s.close()
        except Exception:
            pass

    def _on_emergency_stop(self) -> None:
        """Emergency stop: halt motors + cancel navigation goal."""
        self._send_udp("lr 0 0")
        try:
            import rospy
            from std_msgs.msg import Empty
            cancel_pub = rospy.Publisher("/cancel_goal", Empty, queue_size=1)
            rospy.sleep(0.1)
            cancel_pub.publish(Empty())
        except Exception:
            pass

    def _on_pause(self) -> None:
        """Pause: stop motors without canceling navigation."""
        self._send_udp("lr 0 0")

    # ── RC manual mode (方案 A: CH5 switch owns mode, UI mirrors it) ──

    def _on_manual_mode_clicked(self) -> None:
        """Toggle the RCControl tick loop (start/stop manual override).

        方案 A: the remote CH5 switch is the authority for manual/auto —
        this button only enables/disables RCControl so the operator can
        hand the chassis back to navigation.  The status-bar indicator
        always mirrors the live debounced mode while the loop runs.
        """
        existed = self._rc_control is not None
        if not existed:
            self._setup_rc_control()
        if self._rc_control is None:
            logger.warning("RCControl unavailable - cannot enable manual mode")
            return

        if existed:
            # Control already exists: toggle the tick loop.
            if self._rc_timer is not None and self._rc_timer.isActive():
                self._rc_timer.stop()
                logger.info("RC manual control stopped")
            else:
                if self._rc_timer is None:
                    self._rc_timer = QTimer(self)
                    self._rc_timer.timeout.connect(self._on_rc_tick)
                self._rc_timer.start(100)  # 10 Hz tick
                logger.info("RC manual control started")
        else:
            # Freshly created and already auto-started by _setup_rc_control.
            logger.info("RC manual control enabled")
        self._update_manual_mode_ui()

    def _setup_rc_control(self) -> None:
        """Create RCControl (receiver + ROS bridge) and start the tick loop.

        Falls back gracefully: on a dev box ``create_rc_receiver()``
        returns a MockRCReceiver and no ROS bridge is touched, so the UI
        stays usable even without hardware.

        独立 rc_node 部署(启动脚本 GRAIN_SAMPLING_UI_RC_PUBLISH=0 或检测到外部
        rc_control_node)时, UI 不再自建接收机/控制器 —— 其忙等采样线程会锁死
        本进程 GIL 令 Qt 界面几乎冻结; 改为仅镜像 /rc_mode(2026-09-03 实测)。
        """
        if self._rc_control is not None:
            return

        single_controller = os.getenv(
            "GRAIN_SAMPLING_UI_RC_PUBLISH", "1"
        ).strip().lower() in ("0", "false", "no", "off")
        external_rc = self._has_external_rc_node()
        if single_controller or external_rc or os.getenv("CHASSIS_BACKEND") == "serial":
            logger.info(
                "独立 rc_node 模式 (env=%s external_rc=%s): UI 不建本地 RC 控制器, 仅镜像 /rc_mode",
                single_controller,
                external_rc,
            )
            self._start_rc_mode_mirror()
            return

        # 传统模式: UI 自行采样/控制(无独立 rc_node 时)。
        try:
            receiver = create_rc_receiver()
            bridge = SamplingBridge()
            self._rc_control = RCControl(receiver=receiver, bridge=bridge)
            logger.info("RCControl created with %s", type(receiver).__name__)
        except Exception:  # noqa: BLE001 - UI must survive any RC failure
            logger.exception("Failed to create RCControl - manual mode disabled")
            self._rc_control = None
            return

        if self._rc_timer is None:
            self._rc_timer = QTimer(self)
            self._rc_timer.timeout.connect(self._on_rc_tick)
        self._rc_timer.start(100)  # 10 Hz tick
        logger.info("RC manual control loop started")
        self._update_manual_mode_ui()

    def _start_rc_mode_mirror(self) -> None:
        """镜像独立 rc_node 的 /rc_mode 到状态栏/按钮(不自建接收机, 不占 CPU)。"""
        if self._rc_mirror_started:
            return
        self._rc_mirror_started = True
        if self._ros_thread is not None:
            self._ros_thread.rc_mode_updated.connect(self._on_rc_mode_mirror)
        self._rc_mirror_timer = QTimer(self)
        self._rc_mirror_timer.timeout.connect(self._expire_rc_mode_mirror)
        self._rc_mirror_timer.start(250)
        self._update_manual_mode_ui()

    def _on_rc_mode_mirror(self, mode: str) -> None:
        """Handle mirrored /rc_mode updates from the standalone rc_node."""
        self._rc_mode_mirror = mode if mode in ("manual", "auto") else None
        self._rc_mode_updated_at = time.monotonic()
        self._update_manual_mode_ui()

    def _expire_rc_mode_mirror(self) -> None:
        """Also clear a stale display if the ROS publisher itself disappears."""
        if self._rc_mode_mirror is not None and time.monotonic() - self._rc_mode_updated_at >= 1.5:
            self._rc_mode_mirror = None
            self._update_manual_mode_ui()

    def _has_external_rc_node(self) -> bool:
        """True when a standalone ``rc_control_node`` (rc_node.py) is running.

        Only called once ROS is confirmed up (see ``_on_ros_connected``).
        """
        try:
            import rospy  # noqa: PLC0415 - deferred until ROS is guaranteed up

            names = rospy.get_node_names()
            me = rospy.get_name()
            return any(
                n != me and n.startswith("/rc_control_node") for n in names
            )
        except Exception:  # noqa: BLE001 - fail open to legacy UI behavior
            return False

    def _on_rc_tick(self) -> None:
        """Sample the RC receiver once and refresh the mode UI."""
        if self._rc_control is None:
            return
        try:
            self._rc_control.tick()
        except Exception:  # noqa: BLE001 - control loop must never die
            logger.exception("RC tick failed")
        self._update_manual_mode_ui()

    def _update_manual_mode_ui(self) -> None:
        """Refresh the manual-mode button + status bar indicator."""
        if self._rc_control is not None:
            manual = bool(self._rc_control.manual_active)
            running = self._rc_timer is not None and self._rc_timer.isActive()
        else:
            # 镜像模式: 由独立 rc_node 的 /rc_mode 话题驱动
            manual = self._rc_mode_mirror == "manual"
            running = self._rc_mode_mirror in ("manual", "auto")

        # Nav panel button reflects the debounced CH5 mode
        if self._manual_btn is not None:
            if manual:
                self._manual_btn.setText("手动模式")
                self._manual_btn.setProperty("active", "true")
            else:
                self._manual_btn.setText("手动模式")
                self._manual_btn.setProperty("active", "false")
            self._manual_btn.style().unpolish(self._manual_btn)
            self._manual_btn.style().polish(self._manual_btn)
            self._manual_btn.update()

        # Status bar "模式" indicator: amber = manual, green = auto,
        # grey = RC control stopped/unavailable.
        if self._mode_dot is not None and self._mode_value is not None:
            if not running:
                self._update_dot(self._mode_dot, QColor("#8B949E"))
                self._mode_value.setText("未知")
            elif manual:
                self._update_dot(self._mode_dot, QColor("#D4A72C"))
                self._mode_value.setText("手动")
            else:
                self._update_dot(self._mode_dot, QColor("#2EA043"))
                self._mode_value.setText("自动")

    # ── Task selection flow ───────────────────────────────

    def _on_start_sampling(self) -> None:
        """Handle "开始采样" button: go to active task or task list."""
        if self._guidance_page.has_active_task():
            self.page_manager.push("guidance")
        else:
            self.page_manager.push("task_list")

    def _on_task_selected(self, task_data: dict) -> None:
        """Handle task selection: create FSM/orchestrator and navigate to guidance.

        ``task_data`` is the dict payload emitted by ``task_selected`` (T9):
        - cloud: {order_id, aojian, waypoints, depth_list, grain_type, jiance, source}
        - local: {order_id, warehouse, waypoints, depth_list, grain_type, source}
        """
        # Local tasks keep a legacy 'id' field; prefer it over 'order_id'
        order_id = task_data.get("id") or task_data.get("order_id")

        # ── Warehouse-based map switching ──
        warehouse = task_data.get("warehouse", "")
        skip_mapping = bool(task_data.get("skip_mapping")) and _env_flag(
            "GRAIN_SAMPLING_UI_SKIP_MAPPING"
        )
        if skip_mapping:
            logger.warning("Skipping map relocalization for commissioning task %s", order_id)
        elif warehouse and self._slam_bridge is not None:
            map_path = self._slam_bridge.find_map_by_warehouse(warehouse)
            if map_path:
                logger.info("Switching to map %s for warehouse %s", map_path, warehouse)
                self._slam_bridge.stop_relocalization()
                # start_relocalization() also stops mapping first (mutually
                # exclusive) — no time.sleep(), which would freeze the UI.
                self._slam_bridge.start_relocalization(map_path)
            else:
                logger.warning("No map found for warehouse: %s", warehouse)

        # Extract waypoints and depth
        waypoints_raw = task_data.get("waypoints", [])
        depth_list_raw = task_data.get("depth_list", task_data.get("depth", 2.0))
        if isinstance(depth_list_raw, list) and depth_list_raw:
            depth_list = [float(d) for d in depth_list_raw]
        else:
            fallback = float(depth_list_raw) if isinstance(depth_list_raw, (int, float)) else 2.0
            depth_list = [fallback]

        waypoints = []
        for wp in waypoints_raw:
            if isinstance(wp, dict):
                x = float(wp.get("x", 0.0))
                y = float(wp.get("y", 0.0))
                waypoints.append((x, y))

        if not waypoints:
            waypoints = [(0.0, 0.0)]

        grain_type = task_data.get("pinzhong") or task_data.get("grain_type", "稻谷")

        logger.info("Starting task %s: %d waypoints, depths %s",
                    order_id, len(waypoints), depth_list)

        # Create workflow components
        fsm = SamplingStateMachine(total_waypoints=len(waypoints), max_depth=len(depth_list))
        fsm.set_depth_targets(depth_list)
        bridge = SamplingBridge()
        cloud_client = CloudHttpClient.from_app_config(AppConfig())
        orchestrator = WorkflowOrchestrator(
            fsm=fsm, bridge=bridge, waypoints=waypoints, cloud_client=cloud_client,
        )

        # Real mechanism operation is opt-in so opening the UI cannot move
        # hardware unexpectedly.  The deployment terminal must set this flag.
        if _env_flag("GRAIN_SAMPLING_UI_ENABLE_MECHANISM"):
            orchestrator.enable_mechanism()
            logger.warning("REAL MECHANISM MODE enabled for task %s", order_id)
        else:
            logger.warning(
                "Mechanism placeholder mode; set "
                "GRAIN_SAMPLING_UI_ENABLE_MECHANISM=1 for real hardware"
            )

        # Cloud tasks carry full order context; local tasks use the compat alias
        if task_data.get("source") == "cloud":
            order = OrderInfo(
                order_id=order_id,
                aojian_id=0,  # no aojian_id in payload; default to 0
                aojian=task_data.get("aojian", ""),
                depth_list=depth_list,
                jiance=task_data.get("jiance", []),
                points=[{"x": wp[0], "y": wp[1]} for wp in waypoints],
                pinzhong_code=str(task_data.get("pinzhong_code", "")),
                pinzhong=str(task_data.get("pinzhong", "")),
            )
            orchestrator.set_task_order(order)
        else:
            orchestrator.set_task_id(order_id)
            orchestrator.set_grain(grain_type)

        # Set grain-specific durations
        from grain_sampling_workflow.mechanism_config import GRAIN_MECHANISM_CONFIG
        params = GRAIN_MECHANISM_CONFIG.get(grain_type)
        if params is not None:
            orchestrator.set_sampling_duration(params["sampling_duration"])
            orchestrator.set_convey_duration(params["convey_duration"])

        # Attach to guidance page
        self._guidance_page.attach_orchestrator(fsm, orchestrator)

        # Persist the running task so a reboot can detect & abandon it.
        save_current_task(
            str(order_id),
            source=str(task_data.get("source", "cloud")),
            aojian=str(task_data.get("aojian", "")),
        )

        # Navigate to guidance page
        self.page_manager.push("guidance")

    # ── Reboot recovery: unfinished task detection ─────────

    def check_unfinished_task(self) -> None:
        """On startup, detect a task that was running before shutdown.

        If a task was accepted but never completed/abandoned, the cloud
        keeps it in a running state and locks the device (BUSY).  Prompt
        the operator to abandon it, which POSTs ``/report status=2`` to
        terminate the task on the platform.
        """
        saved = load_current_task()
        if not saved:
            return
        task_id = str(saved.get("task_id", ""))
        if not task_id:
            return
        aojian = str(saved.get("aojian", ""))

        detail = f"检测到未完成的任务：{task_id}"
        if aojian:
            detail += f"（{aojian}）"
        detail += "\n该任务可能仍占用设备。是否通知云端放弃该任务？"

        # Modal prompt: auto-dismiss after 8s so a stale task marker never
        # leaves the UI waiting indefinitely for operator input.
        box = QMessageBox(
            QMessageBox.Icon.Question,
            "检测到未完成任务",
            detail,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            self,
        )
        box.setDefaultButton(QMessageBox.StandardButton.Yes)
        QTimer.singleShot(8000, box.reject)  # 8s timeout → No
        reply = box.exec_()
        if reply != QMessageBox.StandardButton.Yes:
            # Keep the record so the operator can decide later.
            return

        # Attempt to abandon the stale task on the cloud.
        ok = False
        try:
            client = CloudHttpClient.from_app_config(AppConfig())
            client.post(
                ApiPath.STATUS_REPORT,
                {
                    "mac": client.mac_address,
                    "order_id": task_id,
                    "status": ReportStatus.ABANDON,
                },
            )
            ok = True
        except Exception as exc:  # network / protocol errors
            logger.warning("Abandon stale task %s failed: %s", task_id, exc)

        if ok:
            # Cloud accepted the abandon — remove the recovery marker.
            clear_current_task()
            QMessageBox.information(
                self,
                "任务已放弃",
                f"已通知云端终止任务 {task_id}。",
            )
        else:
            # Keep the record so the next startup offers to abandon again.
            QMessageBox.warning(
                self,
                "上报失败",
                f"任务 {task_id} 上报失败，请检查网络。\n"
                "下次开机时将再次询问是否放弃。",
            )

    # ── Status bar update helpers ──────────────────────────

    @staticmethod
    def _make_dot(color: QColor, size: int = 10) -> QLabel:
        """Create a QLabel displaying a solid-colour circle."""
        dot = QLabel()
        dot.setFixedSize(size, size)
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(0, 0, size, size)
        painter.end()
        dot.setPixmap(pixmap)
        return dot

    @staticmethod
    def _update_dot(dot: QLabel, color: QColor) -> None:
        """Repaint an existing dot label with a new colour."""
        size = dot.width()
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(0, 0, size, size)
        painter.end()
        dot.setPixmap(pixmap)

    def set_connected(self, connected: bool) -> None:
        """Update network and ROS indicators from the ROS thread state."""
        if connected:
            if self._net_dot is not None:
                self._update_dot(self._net_dot, QColor("#2EA043"))
            if self._net_value is not None:
                self._net_value.setText("已连接")
            if self._ros_dot is not None:
                self._update_dot(self._ros_dot, QColor("#2EA043"))
            if self._ros_value is not None:
                self._ros_value.setText("\u8fd0\u884c\u4e2d")
        else:
            if self._net_dot is not None:
                self._update_dot(self._net_dot, QColor("#DA3633"))
            if self._net_value is not None:
                self._net_value.setText("离线")
            if self._ros_dot is not None:
                self._update_dot(self._ros_dot, QColor("#DA3633"))
            if self._ros_value is not None:
                self._ros_value.setText("\u672a\u8fd0\u884c")

    def set_speed(self, speed_ms: float) -> None:
        """Update robot speed display in the status bar."""
        if self._speed_value is not None:
            self._speed_value.setText(f"{speed_ms:.1f} m/s")

    # ── Clock ───────────────────────────────────────────────

    def _start_clock(self) -> None:
        """Start a 1-second timer to update the status bar clock."""
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)
        self._update_clock()

    @Slot()
    def _update_clock(self) -> None:
        """Refresh the clock label with current time."""
        if self._clock_label is not None:
            now = datetime.now().strftime("%H:%M:%S")
            self._clock_label.setText(now)

    # ── ROS thread management ─────────────────────────────

    def start_ros(self) -> None:
        """Start the ROS background thread."""
        if self._ros_thread is not None:
            return
        self._ros_thread = ROSNodeThread()
        self._ros_thread.connection_changed.connect(self.set_connected)
        self._ros_thread.ROS_error.connect(self._alarm_bar.set_alarm)
        if HAS_ROS:
            # Connect before start(): ROS initialization can complete before
            # the main thread reaches the next statement on fast systems.
            self._ros_thread.connection_changed.connect(self._on_ros_connected)
        self._ros_thread.cloud_registered_updated.connect(
            self._main_page._map_widget.set_pointcloud_image
        )
        # Give map page access to point cloud data
        self._map_page.set_ros_thread(self._ros_thread)

        # Delay SlamBridge creation until after UI is shown (rospy.init_node can block)
        if self._mapping_page is not None:
            slam_bridge = SlamBridge()
            self._slam_bridge = slam_bridge
            self._mapping_page.set_slam_bridge(slam_bridge)
            # NOTE: do NOT auto-relocalize/auto-map on startup.  SLAM mode is
            # driven explicitly by the operator: select a task -> relocalize,
            # press "start mapping" -> map.  The UI opens in idle state.

        self._ros_thread.start()
        logger.info("ROS thread started")

        # RC manual control loop (方案 A: CH5 switch owns the mode, UI
        # mirrors it).  SamplingBridge() calls rospy.init_node(), which is
        # single-call per process, so defer RC setup until the ROS node
        # exists (or skip the bridge entirely when rospy is unavailable).
        if not HAS_ROS:
            # No ROS (dev box): SamplingBridge() is a harmless no-op.
            self._setup_rc_control()

    def _on_ros_connected(self, connected: bool) -> None:
        """Start the RC loop once the ROS node is confirmed up."""
        if not connected:
            return
        self._ros_thread.connection_changed.disconnect(self._on_ros_connected)
        self._setup_rc_control()

    def stop_ros(self) -> None:
        """Stop the ROS background thread."""
        if self._rc_timer is not None:
            self._rc_timer.stop()
            self._rc_timer = None
        if self._ros_thread is not None:
            self._ros_thread.stop()
            self._ros_thread = None

    def closeEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        """Clean up on window close."""
        self.stop_ros()
        super().closeEvent(event)


# ── Entry point ────────────────────────────────────────────

def main() -> int:
    """Application entry point for ``console_scripts``."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    app = QApplication(sys.argv)
    # Some lightweight X11 window managers briefly unmap a window created by
    # a process launched outside the desktop session.  Keep the event loop
    # alive and present the window again once X11 event handling has started.
    app.setQuitOnLastWindowClosed(False)
    app.setApplicationName("粮食扦样机器人")
    app.setApplicationVersion("0.1.0")
    app.setStyleSheet(THEME_QSS)
    app.setFont(QFont("Noto Sans CJK SC", 9))

    window = MainWindow()
    window.show()
    QTimer.singleShot(250, window.show)
    QTimer.singleShot(300, window.raise_)
    QTimer.singleShot(350, window.activateWindow)

    # Start ROS in background FIRST so the connection indicator lights up
    # immediately (graceful degradation if unavailable).  Deferred work such
    # as the unfinished-task prompt then runs without delaying ROS startup.
    window.start_ros()

    # Reboot recovery: defer the modal prompt until after the Qt event loop
    # starts, otherwise the main window can remain unmapped and appear frozen.
    QTimer.singleShot(1500, window.check_unfinished_task)

    exit_code = app.exec_()

    # Cleanup
    window.stop_ros()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
