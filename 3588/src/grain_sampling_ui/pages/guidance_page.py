"""Guidance page — 15-step sampling workflow controller.

Displays step-by-step instructions, context-sensitive action buttons,
safety controls (pause / resume / stop), and progress tracking for the
grain sampling workflow.

Centred layout matches grain-sampling-console.html prototype:
    kicker → title (40pt) → status pill → detail → progress bar → buttons
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

try:
    from PySide2.QtCore import Qt, Slot, Signal
    from PySide2.QtWidgets import (
        QHBoxLayout, QLabel, QMessageBox, QProgressBar, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget,
    )
except ImportError:
    from PySide6.QtCore import Qt, Slot, Signal  # type: ignore[no-redef]
    from PySide6.QtWidgets import (  # type: ignore[no-redef]
        QHBoxLayout, QLabel, QMessageBox, QProgressBar, QPushButton,
        QSizePolicy, QVBoxLayout, QWidget,
    )

from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)
from grain_sampling_workflow.robot_bridge import RobotBridge as SamplingBridge
from grain_sampling_workflow.orchestrator import WorkflowOrchestrator


# ── Per-state UI metadata ────────────────────────────────────
# (title, detail_text, user_action_button_names)

_STATE_META: Dict[SamplingState, Tuple[str, str, Tuple[str, ...]]] = {
    SamplingState.INIT: (
        "系统初始化中",
        "请等待控制系统完成初始化…",
        ("CONFIRM_READY",),
    ),
    SamplingState.RECORD_START: (
        "正在记录起始位置",
        "系统正在记录当前位置为任务起始点…",
        (),
    ),
    SamplingState.NAVIGATE_TO_POINT: (
        "正在导航到目标点位",
        "正在自主导航到指定位置，请确保周围安全…",
        (),
    ),
    SamplingState.ARRIVED_PROMPT: (
        "已到达指定点位",
        "连接取样管并插入负压风机电源，完成后点击下方按钮",
        ("CONFIRM_READY",),
    ),
    SamplingState.PRESS_AND_SUCTION: (
        "正在下压取样",
        "正在自动下压扦样管并启动负压吸粮…",
        (),
    ),
    SamplingState.ADD_PIPE_PROMPT: (
        "请添加取样管",
        "当前取样管已压到约 1m 深度，请添加下一节管，完成后点击下方按钮",
        ("CONFIRM_PIPE_ADDED",),
    ),
    SamplingState.REPEAT_UNTIL_DEPTH: (
        "深度检查中",
        "系统正在检查累计深度是否达到工单要求…",
        (),
    ),
    SamplingState.DISCHARGE_WASTE: (
        "排出废粮",
        "正在排出废粮，完成后点击下方按钮",
        ("CONFIRM_WASTE_DISCHARGED",),
    ),
    SamplingState.FORMAL_SAMPLING: (
        "正式采样中",
        "仓门已打开、输粮装置已启动，正在正式吸粮采样，可通过下方按钮控制",
        (),
    ),
    SamplingState.CONVEY_1: (
        "输送粮食中",
        "正式吸粮已结束，输粮装置继续运行；计时结束后停止输粮并关闭仓门…",
        (),
    ),
    SamplingState.OPEN_BIN: (
        "开启对应仓口",
        "废粮已排完，正在打开对应仓门；等待 5 秒后启动输粮装置并继续正式采样…",
        (),
    ),
    SamplingState.CONVEY_DONE: (
        "取粮完成",
        "关仓时序已完成，输粮已停止；取粮完成后点击下方按钮",
        ("CONFIRM_DONE",),
    ),
    SamplingState.NEXT_CHECK: (
        "任务判断中",
        "系统正在判断下一步操作…",
        (),
    ),
    SamplingState.ALL_DONE_PROMPT: (
        "所有点位已完成",
        "所有点位采样任务已全部完成，请确认返航",
        ("CONFIRM_RETURN",),
    ),
    SamplingState.RETURN: (
        "正在返航",
        "正在自主导航回到起始点…",
        (),
    ),
    SamplingState.COMPLETED: (
        "任务完成",
        "本次扦样任务已全部完成",
        (),
    ),
    SamplingState.STOPPED: (
        "任务已停止",
        "任务已停止。可通过下方按钮放弃任务（通知云端终止），或返回主菜单",
        (),
    ),
}

# Map button name → SamplingAction
_BUTTON_ACTION_MAP: Dict[str, SamplingAction] = {
    "CONFIRM_READY": SamplingAction.CONFIRM_READY,
    "CONFIRM_PIPE_ADDED": SamplingAction.CONFIRM_PIPE_ADDED,
    "CONFIRM_WASTE_DISCHARGED": SamplingAction.CONFIRM_WASTE_DISCHARGED,
    "CONFIRM_DONE": SamplingAction.CONFIRM_DONE,
    "CONFIRM_RETURN": SamplingAction.CONFIRM_RETURN,
}

_BUTTON_LABELS: Dict[str, str] = {
    "CONFIRM_READY": "已就绪",
    "CONFIRM_PIPE_ADDED": "已加管",
    "CONFIRM_WASTE_DISCHARGED": "废粮已排完",
    "CONFIRM_DONE": "确认",
    "CONFIRM_RETURN": "确认返航",
}

_BUTTON_STYLES: Dict[str, str] = {
    "CONFIRM_READY": "btn_primary",
    "CONFIRM_PIPE_ADDED": "btn_primary",
    "CONFIRM_WASTE_DISCHARGED": "btn_primary",
    "CONFIRM_DONE": "btn_success",
    "CONFIRM_RETURN": "btn_primary",
}


# ── Guidance Page ─────────────────────────────────────────────


class GuidancePage(QWidget):
    _state_changed_signal = Signal(object, object)  # state, action
    home_requested = Signal()
    """Guidance page — drives the 15-step sampling workflow and reacts to
    :class:`SamplingStateMachine` state changes.

    Centred layout: kicker → title (40pt) → status pill → detail →
    progress bar → action/safety buttons → finish button.
    """

    # ── Progress bar constants ──────────────────────────────
    _PROGRESS_BAR_WIDTH: int = 480
    _PROGRESS_BAR_HEIGHT: int = 12

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("guidance_page")

        # Backend state machine (attached externally)
        self._fsm: Optional[SamplingStateMachine] = None
        self._orchestrator: Optional[WorkflowOrchestrator] = None

        # Dynamically created user-action buttons, keyed by name
        self._action_buttons: Dict[str, QPushButton] = {}

        self._setup_ui()
        self._state_changed_signal.connect(self._render)

    # ── UI construction ─────────────────────────────────────

    def _setup_ui(self) -> None:
        """Build the static widget tree — centred layout matching
        grain-sampling-console.html prototype."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(32, 24, 32, 24)
        layout.setSpacing(10)

        # ── Top stretch ─────────────────────────────────────
        layout.addStretch(1)

        # ── Kicker: "当前步骤" ──────────────────────────────
        self._kicker = QLabel("当前步骤")
        self._kicker.setObjectName("guide_kicker")
        self._kicker.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._kicker.setWordWrap(True)
        layout.addWidget(self._kicker, alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Title: state-dependent, 40pt via theme ─────────
        self._title = QLabel("等待任务开始")
        self._title.setObjectName("guide_title")
        self._title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._title.setWordWrap(True)
        layout.addWidget(self._title, alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Status pill: "⏸ 已暂停" / "采样中" ────────────
        self._status_pill = QLabel("")
        self._status_pill.setObjectName("guide_status")
        self._status_pill.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_pill.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed,
        )
        self._status_pill.setVisible(False)
        layout.addWidget(self._status_pill,
                         alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Detail: state-dependent, theme-styled ──────────
        self._detail = QLabel("请从主菜单选择任务开始")
        self._detail.setObjectName("guide_detail")
        self._detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._detail.setWordWrap(True)
        self._detail.setMinimumWidth(480)
        layout.addWidget(self._detail, alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Progress bar: 720px × 20px, blue fill ──────────
        self._progress_bar = QProgressBar()
        self._progress_bar.setObjectName("guide_progress")
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(0)
        self._progress_bar.setTextVisible(True)
        self._progress_bar.setFormat("%p%")
        self._progress_bar.setFixedWidth(self._PROGRESS_BAR_WIDTH)
        self._progress_bar.setFixedHeight(self._PROGRESS_BAR_HEIGHT)
        self._progress_bar.setStyleSheet(
            "QProgressBar {"
            "  background-color: #21262D;"
            "  border: 1px solid #484F58;"
            "  border-radius: 4px;"
            "  text-align: center;"
            "  color: #E6EDF3;"
            "  font-size: 10pt;"
            "}"
            "QProgressBar::chunk {"
            "  background-color: #2B579A;"
            "  border-radius: 3px;"
            "}"
        )
        self._progress_bar.setToolTip("")
        layout.addWidget(self._progress_bar,
                         alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Mid stretch ─────────────────────────────────────
        layout.addStretch(1)

        # ── Safety buttons: 暂停 / 继续 / 急停 (same row) ──
        safety_layout = QHBoxLayout()
        safety_layout.setSpacing(16)

        self._btn_pause = QPushButton("暂停")
        self._btn_pause.setObjectName("btn_warning")
        self._btn_pause.clicked.connect(self._on_pause)
        safety_layout.addWidget(self._btn_pause)

        self._btn_resume = QPushButton("继续")
        self._btn_resume.setObjectName("btn_success")
        self._btn_resume.clicked.connect(self._on_resume)
        safety_layout.addWidget(self._btn_resume)

        self._btn_stop = QPushButton("急停")
        self._btn_stop.setObjectName("btn_danger")
        self._btn_stop.clicked.connect(self._on_stop)
        safety_layout.addWidget(self._btn_stop)

        self._safety_widget = QWidget()
        self._safety_widget.setLayout(safety_layout)
        self._safety_widget.setVisible(False)
        layout.addWidget(self._safety_widget,
                         alignment=Qt.AlignmentFlag.AlignCenter)

        # ── User-action buttons (context-dependent) ────────
        self._action_bar = QWidget()
        self._action_bar_layout = QHBoxLayout(self._action_bar)
        self._action_bar_layout.setContentsMargins(0, 0, 0, 0)
        self._action_bar_layout.setSpacing(16)
        layout.addWidget(self._action_bar,
                         alignment=Qt.AlignmentFlag.AlignCenter)

        # Pre-create all user-action buttons
        for name, label in _BUTTON_LABELS.items():
            btn = QPushButton(label)
            btn.setObjectName(_BUTTON_STYLES.get(name, "btn_primary"))
            btn.setMinimumWidth(120)
            btn.setMinimumHeight(32)
            btn.setVisible(False)
            btn.clicked.connect(lambda _=False, n=name: self._on_user_action(n))
            self._action_buttons[name] = btn
            self._action_bar_layout.addWidget(btn)

        # ── Abandon button: standalone row below ────────────
        self._btn_abandon = QPushButton("放弃任务")
        self._btn_abandon.setObjectName("btn_warning")
        self._btn_abandon.clicked.connect(self._on_abandon)
        self._finish_widget = QWidget()
        finish_layout = QHBoxLayout(self._finish_widget)
        finish_layout.setContentsMargins(0, 0, 0, 0)
        finish_layout.addStretch()
        finish_layout.addWidget(self._btn_abandon)
        self._btn_home = QPushButton("返回主菜单")
        self._btn_home.setObjectName("btn_primary")
        self._btn_home.clicked.connect(self._on_return_home)
        finish_layout.addWidget(self._btn_home)
        finish_layout.addStretch()
        self._finish_widget.setVisible(False)
        layout.addWidget(self._finish_widget,
                         alignment=Qt.AlignmentFlag.AlignCenter)

        # ── Bottom stretch ──────────────────────────────────
        layout.addStretch(1)

    # ── State machine attachment ────────────────────────────

    def showEvent(self, event):
        """Auto-initialise workflow when page becomes visible."""
        super().showEvent(event)
        if self._fsm is None:
            self._init_workflow()

    def _init_workflow(self) -> None:
        """Create FSM + bridge + orchestrator and attach to this page.

        Uses stub/no-op mode when rospy is not available (e.g. dev machine).
        """
        try:
            bridge = SamplingBridge(node_name="guidance_bridge")
            bridge.NAV_TIMEOUT_SEC = 60.0
        except Exception:
            bridge = None

        fsm = SamplingStateMachine(total_waypoints=2, max_depth=1)
        fsm.set_depth_targets([2.0])

        if bridge is not None:
            self._orchestrator = WorkflowOrchestrator(
                fsm, bridge,
                waypoints=[(0.5, 0.3), (0.8, -0.3)],  # placeholder waypoints
            )
            # Chain UI callback via orchestrator — do NOT overwrite the FSM
            # callback, otherwise mechanism auto-advance handlers never fire.
            self._orchestrator.set_callback(self._on_state_changed)
        else:
            # No bridge (dev machine): attach UI callback directly
            fsm.on_state_change = self._on_state_changed

        # Detach previous FSM if any
        if self._fsm is not None:
            self._fsm.on_state_change = None

        self._fsm = fsm
        # Initial render
        self._render(fsm.current_state, None)

    def attach_state_machine(self, fsm: SamplingStateMachine) -> None:
        """Connect this page to a :class:`SamplingStateMachine`.

        Disconnects any previous state machine first.
        """
        # Detach previous
        if self._fsm is not None:
            self._fsm.on_state_change = None

        self._fsm = fsm
        fsm.on_state_change = self._on_state_changed
        # Initial render
        self._render(fsm.current_state, None)

    def attach_orchestrator(
        self,
        fsm: SamplingStateMachine,
        orchestrator: WorkflowOrchestrator,
    ) -> None:
        """Attach pre-configured FSM and orchestrator to this page.

        Unlike :meth:`attach_state_machine`, this method preserves the
        orchestrator's hook on ``fsm.on_state_change`` and chains the
        UI callback via ``orchestrator.set_callback()``.
        """
        # Detach previous FSM if any
        if self._fsm is not None:
            self._fsm.on_state_change = None

        self._fsm = fsm
        self._orchestrator = orchestrator
        # Chain UI callback through orchestrator (preserves bridge hooks)
        orchestrator.set_callback(self._on_state_changed)
        self._safety_widget.setVisible(False)
        self._render(fsm.current_state, None)

    def set_fake_navigation_mode(self, enabled: bool) -> None:
        """Label operator-confirmed bench navigation clearly in the live UI."""
        self._fake_navigation_mode = bool(enabled)

    def has_active_task(self) -> bool:
        """Return True if there is a task currently in progress.

        A task is considered active when the FSM has been created and is
        in a state other than ``INIT``, ``STOPPED`` or ``COMPLETED``.
        """
        if self._fsm is None:
            return False
        return self._fsm.current_state not in (
            SamplingState.INIT, SamplingState.STOPPED, SamplingState.COMPLETED,
        )

    # ── Slots ───────────────────────────────────────────────

    @Slot()
    def _on_return_home(self) -> None:
        """Leave an idle or terminal task without resetting its workflow."""
        if not self.has_active_task():
            self.home_requested.emit()

    @Slot()
    def _on_user_action(self, button_name: str) -> None:
        """Handle a user action button click."""
        if self._fsm is None:
            return
        action = _BUTTON_ACTION_MAP.get(button_name)
        if action is not None:
            try:
                self._fsm.transition(action)
            except ValueError as exc:
                self._detail.setText(f"操作无效: {exc}")

    @Slot()
    def _on_pause(self) -> None:
        if self._fsm is not None:
            try:
                self._fsm.pause()
            except ValueError as exc:
                self._detail.setText(f"操作无效: {exc}")

    @Slot()
    def _on_resume(self) -> None:
        if self._fsm is not None:
            try:
                self._fsm.resume()
            except ValueError as exc:
                self._detail.setText(f"操作无效: {exc}")

    @Slot()
    def _on_stop(self) -> None:
        if self._fsm is not None:
            try:
                self._fsm.stop()
            except ValueError as exc:
                self._detail.setText(f"操作无效: {exc}")

    @Slot()
    def _on_abandon(self) -> None:
        """Confirm with the user before abandoning the current task.

        On confirmation, notifies the cloud that the task is being
        abandoned (``POST /report status=2``) so the platform can terminate
        the task and release the device lock, then stops the local FSM.
        """
        if self._fsm is None:
            return
        reply = QMessageBox.question(
            self,
            "放弃任务",
            "确认放弃当前任务？\n将通知云端终止该任务，此操作不可恢复。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        # 1) Notify cloud to terminate the task (best-effort; never blocks stop).
        report_ok = False
        if self._orchestrator is not None:
            report_ok = self._orchestrator.report_abandon()
        # 2) Stop the local FSM regardless of report outcome.
        try:
            self._fsm.stop()
        except ValueError as exc:
            self._detail.setText(f"操作无效: {exc}")
            return
        # 3) Clear the persisted record only when the cloud report succeeded;
        #    otherwise keep it so the reboot prompt can retry the abandon.
        if report_ok:
            try:
                from grain_sampling_cloud.task_state import clear_current_task
                clear_current_task()
            except Exception:
                pass
            self._detail.setText("任务已放弃，云端已终止该任务")
        else:
            self._detail.setText(
                "任务已本地停止，但云端上报失败。可再次点击放弃任务重试。"
            )

    # ── Progress computation ────────────────────────────────

    def _compute_progress_percent(self) -> int:
        """Compute overall progress percentage (0–100) from FSM state.

        Maps waypoint / depth progression into a 0–100 range that
        reflects actual task advancement.
        """
        if self._fsm is None:
            return 0
        state = self._fsm.current_state

        # Terminal states
        if state == SamplingState.COMPLETED:
            return 100
        if state in (SamplingState.INIT, SamplingState.STOPPED):
            return 0

        # ── Waypoint + depth composite (mapped to 5–90%) ────
        wp = self._fsm.current_waypoint_index
        tw = max(1, self._fsm.total_waypoints)
        di = self._fsm.current_depth_index
        md = max(1, self._fsm.max_depth)

        wp_frac: float = wp / tw
        depth_frac: float = di / md
        base: float = 5.0 + 85.0 * (wp_frac * 0.7 + depth_frac * 0.3)

        # ── Post-depth states (OPEN_BIN → COMPLETED) ──
        # Map into 90–99% range
        if state.value >= SamplingState.FORMAL_SAMPLING.value:
            post_depth_states = [
                SamplingState.OPEN_BIN,
                SamplingState.FORMAL_SAMPLING,
                SamplingState.CONVEY_1,
                SamplingState.CONVEY_DONE,
                SamplingState.NEXT_CHECK,
                SamplingState.ALL_DONE_PROMPT,
                SamplingState.RETURN,
            ]
            try:
                idx = post_depth_states.index(state)
                total = max(1, len(post_depth_states) - 1)
                base = 90.0 + 9.0 * (idx / total)
            except ValueError:
                pass

        return max(0, min(100, int(round(base))))

    # ── State rendering ─────────────────────────────────────

    def _on_state_changed(
        self,
        prev: SamplingState,
        current: SamplingState,
        action: Optional[SamplingAction],
    ) -> None:
        """Thread-safe callback - emits signal to GUI thread."""
        self._state_changed_signal.emit(current, action)

    def _render(
        self,
        state: SamplingState,
        action: Optional[SamplingAction],
    ) -> None:
        """Update the entire UI to reflect *state*.

        Order matches the centred column layout:
        kicker → title (40pt) → status pill → detail →
        progress bar → safety buttons → action buttons → finish button.
        """
        meta = _STATE_META.get(state)
        if meta is None:
            return

        title_text, detail_text, button_names = meta
        if getattr(self, "_fake_navigation_mode", False) and state in (
            SamplingState.NAVIGATE_TO_POINT, SamplingState.RETURN,
        ):
            title_text = "假导航联调：等待人工确认"
            detail_text = "底盘不会移动；请在假导航终端确认现场安全并按 Enter。"

        # ── Title ───────────────────────────────────────────
        self._title.setText(title_text)

        # ── Detail ──────────────────────────────────────────
        self._detail.setText(detail_text)

        # Override detail for ADD_PIPE_PROMPT with dynamic pipe/depth info
        if state == SamplingState.ADD_PIPE_PROMPT and self._fsm is not None:
            import math
            target_m = self._fsm.get_target_depth()
            total = max(1, int(math.ceil(target_m)))
            current = self._fsm.current_pipe_index
            depth_m = float(current)
            if current < total:
                self._detail.setText(
                    f"当前深度约 {depth_m:.0f}m，请插入第 {current + 1} 节管"
                    f"（共需 {total} 节），完成后点击下方按钮"
                )
            else:
                self._detail.setText(
                    f"当前深度已达到目标 {target_m:.1f}m（{total} 节管），"
                    f"点击下方按钮继续"
                )

        # ── Status pill ─────────────────────────────────────
        in_formal = state == SamplingState.FORMAL_SAMPLING
        if in_formal and self._fsm is not None:
            paused = self._fsm.paused
            if paused:
                self._status_pill.setText("⏸ 已暂停")
                self._status_pill.setObjectName("guide_status")
            else:
                self._status_pill.setText("采样中")
                self._status_pill.setObjectName("guide_status_running")
            self._status_pill.setVisible(True)
            # Force style refresh after objectName change
            self._status_pill.style().unpolish(self._status_pill)
            self._status_pill.style().polish(self._status_pill)
        else:
            self._status_pill.setVisible(False)

        # ── Progress bar ────────────────────────────────────
        percentage = self._compute_progress_percent()
        self._progress_bar.setValue(percentage)
        if self._fsm is not None:
            self._progress_bar.setToolTip(self._fsm.progress_str)

        # ── User-action buttons ─────────────────────────────
        for name, btn in self._action_buttons.items():
            btn.setVisible(name in button_names)

        # ── Safety controls (during FORMAL_SAMPLING) ────────
        self._safety_widget.setVisible(in_formal)
        if in_formal and self._fsm is not None:
            paused = self._fsm.paused
            self._btn_pause.setVisible(not paused)
            self._btn_resume.setVisible(paused)
            self._btn_stop.setVisible(True)

        # ── Abandon button ───────────────────────────────────
        # Visible during any active task AND while stopped (so a failed
        # cloud report can be retried). Hidden on INIT (no task) and
        # COMPLETED (task already reported done).
        show_abandon = state not in (
            SamplingState.INIT,
            SamplingState.COMPLETED,
        )
        self._btn_abandon.setVisible(show_abandon)
        self._btn_home.setVisible(not self.has_active_task())
        self._finish_widget.setVisible(True)

        # ── Terminal state hint ─────────────────────────────
        if state == SamplingState.COMPLETED:
            self._detail.setText("您可返回主菜单或继续新的任务")
        elif state == SamplingState.STOPPED:
            reason = getattr(self._fsm, "stop_reason", "") if self._fsm else ""
            if reason:
                self._detail.setText(f"停止原因：{reason}\n请检查设备状态后返回主菜单")
            else:
                self._detail.setText("请检查设备状态后返回主菜单")
