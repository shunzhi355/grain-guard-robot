"""Workflow orchestrator — bridges the state machine and ROS bridge.

Runs bridge operations in background threads so the UI stays responsive.
When a bridge operation completes, it automatically transitions the FSM.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from datetime import date, datetime
from typing import Callable, Optional
from utils.sampling_params import PRESS_SEGMENT_CM

from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)
from grain_sampling_workflow.mechanism_config import get_grain_params
from grain_sampling_workflow.ros_bridge import SamplingBridge
from grain_sampling_cloud.http_client import CloudConnectionError, CloudHttpClient, CloudProtocolError
from grain_sampling_cloud.protocol import ApiPath, DetectionResult, OrderInfo, ReportStatus, StatusReportRequest

logger = logging.getLogger(__name__)

#: 机构动作完成后的额外停稳余量（秒）。mechanism_node 的 Trigger 服务是
#: "立即返回"的（duration 走后台 auto_stop 线程），而电调 full-off 后电机
#: 仍有惯性。实机确认：夹紧/松开后需 +0.5s，拧紧（5s 动作）后需 +0.5s，
#: 否则下一步会与仍在转的机构冲突。
MECHANISM_SETTLE_MARGIN: float = 0.5

#: 单次下压/上升距离（厘米）。X2P 伺服单次 move_lift ≤ 30cm
#: （max_distance_mm=300）。扦样时每次 press 下压一节管的一部分，
#: 由状态机 REPEAT_UNTIL_DEPTH 循环累加直到目标深度。
#: 实机联调：单次下压/上升 20cm（用户 2026-09 标定：20cm 长行程编码器
#: 闭环精度好，实测误差 ~1.2mm；< 30cm max_distance，且 < 25cm 限位）。
PRESS_STEP_CM: float = PRESS_SEGMENT_CM


class WorkflowOrchestrator:
    """Connects the 15-step state machine to the ROS bridge.

    Listens for state changes and automatically triggers the appropriate
    bridge action (navigate, suction, convey, etc.) in a background thread.

    Usage::

        fsm = SamplingStateMachine(total_waypoints=2, max_depth=2)
        bridge = SamplingBridge()
        orch = WorkflowOrchestrator(fsm, bridge)
        # User actions still drive the FSM:
        fsm.transition(SamplingAction.CONFIRM_READY)
        # → orchestrator auto-runs record_start → SYSTEM_RECORD_COMPLETE
    """

    def __init__(
        self,
        fsm: SamplingStateMachine,
        bridge: SamplingBridge,
        waypoints: Optional[list[tuple[float, float]]] = None,
        cloud_client: Optional[CloudHttpClient] = None,
    ) -> None:
        self._fsm = fsm
        self._bridge = bridge
        self._waypoints = waypoints or [(0.0, 0.0)]
        self._start_position: Optional[tuple[float, float]] = None  # recorded by RECORD_START
        self._thread: Optional[threading.Thread] = None
        self._pending = 0  # count of queued bridge operations
        self._running = False

        # Cloud reporting — use the real AppConfig cloud endpoint so that
        # status reports (complete/abandon) actually reach the platform.
        # A placeholder client would silently no-op (http_client returns {}
        # for "xxx" base URLs) and leave the device BUSY on the cloud.
        if cloud_client is not None:
            self._cloud_client = cloud_client
        else:
            try:
                from utils.config import AppConfig
                self._cloud_client = CloudHttpClient.from_app_config(AppConfig())
            except Exception:
                self._cloud_client = CloudHttpClient()
        self._order_id: Optional[str] = None
        self._jiance: list = []
        self._depth_list: list = []

        # ── Configurable durations (task C) ─────────────────
        self.sampling_duration_sec: float = 120.0  # default 2 min
        self.convey_duration_sec: float = 120.0  # default 2 min
        self._mechanism_connected: bool = False  # placeholder: no hardware
        self._grain: str = ""  # grain variety reported to the mechanism at task start
        self._mechanism_retry_interval: float = 0.5  # backoff between mechanism retries

        # ── Depth live display callback (task D) ────────────
        self.depth_callback: Optional[Callable[[float], None]] = None

        # Save any existing user callback, then hook our own
        self._user_callback = fsm.on_state_change
        self._fsm.on_state_change = self._on_state_change

    def set_callback(self, callback) -> None:
        """Set a user callback without breaking the orchestrator's own hook."""
        self._user_callback = callback

    def set_task_order(self, order) -> None:
        """Store complete order info for cloud status reporting.

        Parameters
        ----------
        order : OrderInfo
            Order from cloud containing order_id, jiance, depth_list, etc.
        """
        self._order_id = order.order_id
        self._jiance = order.jiance
        self._depth_list = order.depth_list
        # Notify the mechanism of the grain variety at task start (best-effort,
        # retried with FSM stop on final failure). OrderInfo v2 carries the
        # variety in ``pinzhong`` / ``pinzhong_code``.
        grain = getattr(order, "pinzhong", "") or getattr(order, "pinzhong_code", "")
        if grain:
            self.set_grain(grain)

    def set_task_id(self, task_id: str) -> None:
        """Compatibility alias — sets only order_id, no detection context."""
        self._order_id = task_id
        self._jiance = []
        self._depth_list = []

    def set_grain(self, grain: str) -> None:
        """Record the grain variety and push it to the mechanism (best-effort).

        Called at task start so the mechanism uses the per-grain actuation
        parameters (and re-enables itself after an emergency stop).
        """
        self._grain = str(grain or "")
        if not self._grain:
            logger.info("set_grain skipped — no grain variety provided")
            return
        logger.info("set_grain(%s) at task start", self._grain)
        self._call_mechanism(
            "set_grain", lambda: self._bridge.call_set_grain(self._grain)
        )

    # ── Mechanism connection switch (real hardware vs placeholder) ────────

    def enable_mechanism(self) -> None:
        """Switch to real mechanism operation (bridge method calls).

        Deployment hook: call once the mechanism team's services are up so
        the handlers drive the hardware instead of the ``time.sleep``
        placeholders.
        """
        self._mechanism_connected = True
        logger.info("Mechanism enabled — bridge method calls active")

    def set_mechanism_connected(self, connected: bool = True) -> None:
        """Enable/disable real mechanism operation.

        Parameters
        ----------
        connected : bool
            ``True`` (default) to drive real hardware, ``False`` to fall
            back to the placeholder (``time.sleep``) mode.
        """
        if connected:
            self.enable_mechanism()
        else:
            self._mechanism_connected = False
            logger.info("Mechanism disabled — placeholder mode")

    # ── Configurable durations (task C) ──────────────────────

    def set_sampling_duration(self, sec: float) -> None:
        """Configure the formal sampling duration in seconds."""
        self.sampling_duration_sec = float(sec)

    def set_convey_duration(self, sec: float) -> None:
        """Configure the conveyor run duration in seconds."""
        self.convey_duration_sec = float(sec)

    # ── Operation logging (task A) ───────────────────────────

    def _log_sampling_event(self, action: str, detail: str) -> None:
        """Append a JSON-line log entry for an FSM transition event.

        Log file: ``~/grain_sampling_logs/sampling_{date}.log``
        """
        try:
            log_dir = os.path.join(
                os.path.expanduser("~"), "grain_sampling_logs"
            )
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir, f"sampling_{date.today().isoformat()}.log"
            )
            entry = {
                "ts": datetime.now().isoformat(timespec="seconds"),
                "waypoint": self._fsm.current_waypoint_index + 1,
                "depth": self._fsm.current_depth_index + 1,
                "action": action,
                "detail": detail,
            }
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception:
            logger.exception("Failed to write sampling log entry")

    # ----------------------------------------------------------------
    # FSM callback — decides what bridge action to run for each state
    # ----------------------------------------------------------------

    def _on_state_change(
        self,
        prev: SamplingState,
        current: SamplingState,
        action: Optional[SamplingAction],
    ) -> None:
        """Auto-trigger bridge actions when the FSM enters certain states."""
        # Chain user callback first (e.g. UI or test logger)
        if self._user_callback:
            self._user_callback(prev, current, action)

        logger.info("FSM: %s -> %s  (action: %s)", prev, current, action)

        # ── Operation logging (task A) ─────────────────────
        self._log_on_transition(prev, current, action)

        # ── Depth live display (task D) ────────────────────
        if self.depth_callback is not None:
            depth_m = float(self._fsm.current_pipe_index) * 1.0
            try:
                self.depth_callback(depth_m)
            except Exception:
                logger.exception("Depth callback failed")

        if current == SamplingState.RECORD_START:
            self._run_async(self._handle_record_start)

        elif current == SamplingState.NAVIGATE_TO_POINT:
            self._run_async(self._handle_navigate)

        elif current == SamplingState.RETURN:
            self._run_async(self._handle_return_to_start)

        elif current == SamplingState.PRESS_AND_SUCTION:
            self._run_async(self._handle_press_and_suction)

        elif current == SamplingState.FORMAL_SAMPLING:
            self._run_async(self._handle_formal_sampling)

        elif current == SamplingState.CONVEY_1:
            self._run_async(self._handle_convey)

        elif current == SamplingState.OPEN_BIN:
            self._run_async(self._handle_open_bin)

        elif current == SamplingState.REPEAT_UNTIL_DEPTH:
            self._run_async(self._handle_repeat_until_depth)

        if current == SamplingState.NEXT_CHECK:
            self._run_async(self._handle_next_check)

        # ── Task finished: report COMPLETED to the cloud ──
        elif current == SamplingState.COMPLETED:
            self._run_async(self._handle_task_completed)

    # ----------------------------------------------------------------
    # Cloud status reporting
    # ----------------------------------------------------------------

    def report_complete(self) -> bool:
        """Report task complete status to cloud with detection results.

        Reads detection values from the biochemical/physicochemical
        instruments via DetectionResultReader, then POSTs a
        ``StatusReportRequest(status=COMPLETE)`` to the cloud.

        Returns
        -------
        bool
            ``True`` when the report was delivered successfully,
            ``False`` on network or protocol errors.
        """
        from grain_sampling_devices.detection_reader import DetectionResultReader

        reader = DetectionResultReader()
        results = reader.read_all(self._jiance, self._depth_list)
        if not results:
            logger.warning(
                "No detection results available for task %s", self._order_id
            )

        req = StatusReportRequest(
            mac=self._cloud_client.mac_address,
            order_id=self._order_id,
            status=ReportStatus.COMPLETE,
            jiance_results=[
                DetectionResult(r["indicator_id"], r["value"], r.get("depth"))
                for r in results
            ],
        )
        return self._post_report(req)

    def report_abandon(self) -> bool:
        """Report task abandoned status to cloud.

        POSTs a ``StatusReportRequest(status=ABANDON)`` with no detection
        results.

        Returns
        -------
        bool
            ``True`` when the report was delivered successfully,
            ``False`` on network or protocol errors.
        """
        req = StatusReportRequest(
            mac=self._cloud_client.mac_address,
            order_id=self._order_id,
            status=ReportStatus.ABANDON,
        )
        return self._post_report(req)

    def _post_report(self, req: StatusReportRequest) -> bool:
        """Send a status report to the cloud.

        Returns ``True`` on success, ``False`` on failure.  Never raises.
        """
        # Guard against a placeholder/unconfigured cloud client, which would
        # silently no-op and falsely report success (leaving the device BUSY).
        if "xxx" in self._cloud_client.base_url:
            logger.error("Cloud client is unconfigured (placeholder URL); report not sent")
            return False
        try:
            self._cloud_client.post(ApiPath.STATUS_REPORT, req.__dict__)
            return True
        except (CloudProtocolError, CloudConnectionError) as e:
            logger.error("Report failed: %s", e)
            return False

    def _log_on_transition(
        self,
        prev: SamplingState,
        current: SamplingState,
        action: Optional[SamplingAction],
    ) -> None:
        """Log key FSM transitions to the sampling event log (task A)."""
        # FORMAL_SAMPLING start
        if current == SamplingState.FORMAL_SAMPLING:
            self._log_sampling_event(
                "formal_sampling_start",
                "吸粮{:.0f}分钟".format(self.sampling_duration_sec / 60.0),
            )
        # Depth reached (DISCHARGE_WASTE = entering after REPEAT_UNTIL_DEPTH)
        elif current == SamplingState.DISCHARGE_WASTE:
            self._log_sampling_event("depth_reached", "到达目标深度")
        # Nave arrive (ARRIVED_PROMPT)
        elif current == SamplingState.ARRIVED_PROMPT:
            self._log_sampling_event("nave_arrive", "到达扦样点位")
        # Waypoint complete (upon reaching ALL_DONE_PROMPT)
        elif current == SamplingState.ALL_DONE_PROMPT:
            self._log_sampling_event("waypoint_complete", "所有点位完成")

    # ----------------------------------------------------------------
    # Bridge action handlers (each runs in a background thread)
    # ----------------------------------------------------------------

    def _handle_record_start(self) -> None:
        """Step 2: record start position, then auto-advance."""
        pos = self._bridge.record_start_position()
        if pos is not None:
            self._start_position = pos
        self._fsm.transition(SamplingAction.SYSTEM_RECORD_COMPLETE)

    def _handle_navigate(self) -> None:
        """Step 3: navigate to current waypoint, then auto-advance."""
        idx = self._fsm.current_waypoint_index
        if idx < len(self._waypoints):
            x, y = self._waypoints[idx]
        else:
            x, y = 0.0, 0.0
        logger.info("Navigating to waypoint %d: (%.2f, %.2f)", idx, x, y)
        ok = self._bridge.call_navigate(x, y)
        if ok:
            self._fsm.transition(SamplingAction.SYSTEM_NAV_COMPLETE)

    def _handle_return_to_start(self) -> None:
        """Step 15: navigate back to start, then complete."""
        target = self._start_position if self._start_position is not None else (0.0, 0.0)
        logger.info("Returning to start position (%.2f, %.2f)", target[0], target[1])
        ok = self._bridge.call_navigate(target[0], target[1])
        if ok:
            self._fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)

    def _handle_press_and_suction(self) -> None:
        """Step 5: press pipe + suction, auto-advance with depth check.

        With the mechanism connected the full down-press cycle for one pipe
        section is: clamp -> press -> unclamp -> lift -> clamp, then start
        suction.  Every step waits its per-grain duration (interruptible, so
        STOP/emergency aborts mid-cycle); press/lift use a fixed delay since
        the servo is not yet wired.  The placeholder path keeps the
        ``time.sleep`` mock behaviour.
        """
        import math
        target_m = self._fsm.get_target_depth()
        pipes_needed = max(1, int(math.ceil(target_m)))
        if self._mechanism_connected:
            d = self._get_mechanism_durations()
            step_cm = PRESS_STEP_CM
            # The clamp must not operate until the complete X2P communication
            # path has answered a read-only encoder request.  This makes a dead
            # RS485 link fail safely before any mechanism starts moving.
            if not self._call_mechanism(
                "lift health", self._bridge.call_lift_health
            ):
                return
            # 正常下压循环：夹紧 → 下压(精确距离) → 松开 → 上升 → 再夹紧。
            # press/lift 走 move_lift 编码器闭环精确距离控制（单次 step_cm cm，
            # 状态机 REPEAT_UNTIL_DEPTH 循环累加直到目标深度）。
            press_cycle = (
                ("clamp", self._bridge.call_clamp, d["clamp"]),
                ("press", lambda: self._bridge.call_move_lift("down_cycle", step_cm), d["servo"]),
                ("unclamp", self._bridge.call_unclamp, d["unclamp"]),
                ("lift", lambda: self._bridge.call_move_lift("return", step_cm), d["servo"]),
                ("clamp", self._bridge.call_clamp, d["clamp"]),  # 再夹紧，准备下一次下压
            )
            if not self._run_mechanism_sequence(press_cycle):
                return  # failure/interruption already stopped the FSM
            if not self._call_mechanism(
                "start_suction", self._bridge.call_start_suction
            ):
                return  # final failure already stopped the FSM
        else:
            time.sleep(0.5)  # mechanism placeholder
        # Only account for a pipe after the complete cycle succeeded.
        self._fsm.current_pipe_index += 1
        current = self._fsm.current_pipe_index
        logger.info(
            "Press: pipe %d/%d pressed for %.1fm target",
            current, pipes_needed, target_m,
        )
        if current >= pipes_needed:
            # Target depth reached → skip add-pipe prompt, go to waste discharge
            self._fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)
        else:
            self._fsm.transition(SamplingAction.SYSTEM_PRESS_COMPLETE)

    def _handle_formal_sampling(self) -> None:
        """Step 9: formal sampling, auto-advance after configurable delay.

        With the mechanism connected: start suction -> wait (interruptible,
        so an emergency STOP aborts the wait) -> stop suction.  The
        placeholder path keeps the short ``time.sleep`` mock behaviour.
        """
        if self._mechanism_connected:
            if not self._call_mechanism(
                "start_suction", self._bridge.call_start_suction
            ):
                return  # final failure already stopped the FSM
            if not self._wait_interruptible(self.sampling_duration_sec):
                logger.warning("Formal sampling interrupted — stopping suction")
                self._bridge.call_stop_suction()  # best-effort hardware cleanup
                return
            if not self._call_mechanism(
                "stop_suction", self._bridge.call_stop_suction
            ):
                return  # final failure already stopped the FSM
            logger.info("Formal sampling complete: suction ran %.1f sec",
                        self.sampling_duration_sec)
            self._fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)
        else:
            duration = 0.5  # placeholder when no mechanism
            logger.info(
                "Formal sampling: sleeping %.1f sec (mechanism=placeholder)",
                duration,
            )
            time.sleep(duration)
            self._fsm.transition(SamplingAction.SYSTEM_SUCTION_COMPLETE)

    def _handle_convey(self) -> None:
        """Convey into the selected bin, stop conveyors, then close all bins."""
        if self._mechanism_connected:
            try:
                if not self._call_mechanism("start_convey", self._bridge.call_start_convey):
                    return
                if not self._wait_interruptible(self.convey_duration_sec):
                    return
                if not self._call_mechanism("stop_convey", self._bridge.call_stop_convey):
                    return
                if not self._call_mechanism(
                    "close_all_bins", self._bridge.call_close_all_bins
                ):
                    return
                close_sec = float(get_grain_params(self._grain)["close_duration"])
                if not self._wait_interruptible(close_sec + MECHANISM_SETTLE_MARGIN):
                    return
            except Exception:
                self._stop_fsm("bin/conveyor sequence failed")
                logger.exception("Bin/conveyor sequence failed")
                return
            finally:
                if not self._fsm.is_running:
                    self._bridge.call_emergency_stop()
            logger.info("Convey complete: ran %.1f sec", self.convey_duration_sec)
            self._fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)
        else:
            duration = 0.5  # placeholder when no mechanism
            logger.info(
                "Convey: sleeping %.1f sec (mechanism=placeholder)",
                duration,
            )
            time.sleep(duration)
            self._fsm.transition(SamplingAction.SYSTEM_CONVEY_COMPLETE)

    def _handle_open_bin(self) -> None:
        """After waste discharge, open the selected bin and close the others."""
        depth = self._fsm.current_depth_index
        if self._mechanism_connected:
            if not self._call_mechanism(
                "hold_bin_open", lambda: self._bridge.call_hold_bin_open(depth)
            ):
                return  # final failure already stopped the FSM
            params = get_grain_params(self._grain)
            doors_sec = max(float(params["open_duration"]), float(params["close_duration"]))
            if not self._wait_interruptible(doors_sec + MECHANISM_SETTLE_MARGIN):
                self._bridge.call_emergency_stop()
                return
            logger.info("Open bin depth=%d (mechanism connected)", depth)
        else:
            logger.info("Open bin depth=%d (mechanism placeholder)", depth)
        self._fsm.transition(SamplingAction.SYSTEM_BIN_OPENED)

    def _handle_close_bin(self) -> None:
        """Manual helper; production closing is serialized in _handle_convey."""
        depth = self._fsm.current_depth_index
        if self._mechanism_connected:
            self._bridge.call_close_bin(depth)
            logger.info("Close bin depth=%d (mechanism connected)", depth)
        else:
            logger.info("Close bin depth=%d (mechanism placeholder)", depth)

    def _handle_repeat_until_depth(self) -> None:
        target_m = self._fsm.get_target_depth()  # current depth level in meters
        pipes_needed = max(1, int(math.ceil(target_m)))  # 1 pipe per ~1m
        current_pipe = self._fsm.current_pipe_index  # 1-based after first press
        logger.info(
            "Repeat until depth: pipe %d/%d needed for %.1fm target",
            current_pipe, pipes_needed, target_m,
        )
        if current_pipe < pipes_needed:
            # User has confirmed the new pipe section (CONFIRM_PIPE_ADDED) →
            # REPEAT_UNTIL_DEPTH.  CH6 tightens the new joint, then the pipe
            # is unclamped and re-clamped before looping back to the next
            # press cycle.
            if self._mechanism_connected:
                d = self._get_mechanism_durations()
                # 加管分支：拧紧 → 松开 → 夹紧
                add_pipe_cycle = (
                    ("tighten", self._bridge.call_tighten, d["tighten"]),
                    ("unclamp", self._bridge.call_unclamp, d["unclamp"]),
                    ("clamp", self._bridge.call_clamp, d["clamp"]),
                )
                if not self._run_mechanism_sequence(add_pipe_cycle):
                    return  # failure/interruption already stopped the FSM
            # Need more pipe sections → loop back to step 5 (PRESS_AND_SUCTION)
            self._fsm.transition(SamplingAction.SYSTEM_DEPTH_NOT_REACHED)
        else:
            # Target depth reached → continue to step 8 (DISCHARGE_WASTE)
            self._fsm.transition(SamplingAction.SYSTEM_DEPTH_REACHED)

    def _handle_next_check(self) -> None:
        logger.info("Next check: depth=%d/%d waypoint=%d/%d",
                    self._fsm.current_depth_index + 1, self._fsm.max_depth,
                    self._fsm.current_waypoint_index + 1, self._fsm.total_waypoints)
        if self._fsm.current_depth_index + 1 < self._fsm.max_depth:
            self._fsm.transition(SamplingAction.SYSTEM_NEXT_DEPTH)
        elif self._fsm.current_waypoint_index + 1 < self._fsm.total_waypoints:
            self._fsm.transition(SamplingAction.SYSTEM_NEXT_POINT)
        else:
            self._fsm.transition(SamplingAction.SYSTEM_ALL_DONE)

    def _handle_task_completed(self) -> None:
        """Step final: report task COMPLETED to the cloud.

        Best-effort — runs in a background thread so a network failure
        never blocks the UI.  The local task state is cleared afterwards
        so the next reboot does not treat it as unfinished.
        """
        if not self._order_id:
            logger.info("Task completed with no order_id — nothing to report")
        else:
            ok = self.report_complete()
            logger.info("Cloud COMPLETE report for %s: %s", self._order_id, ok)
        # Clear persisted current-task state (reboot-recovery marker).
        try:
            from grain_sampling_cloud.task_state import clear_current_task
            clear_current_task()
        except Exception:
            logger.exception("Failed to clear current task state")

    # ----------------------------------------------------------------
    # Mechanism call helpers (retry + FSM stop on final failure)
    # ----------------------------------------------------------------

    def _get_mechanism_durations(self) -> dict[str, float]:
        """Per-grain actuation durations from ``mechanism_config``.

        Returns a dict keyed by ``clamp``/``unclamp``/``tighten``/
        ``untighten`` (read from the grain config) plus ``servo`` — a fixed
        delay simulating the not-yet-wired press/lift servo.  The servo
        delay falls back to the clamp duration so it always tracks the
        grain config.

        Every duration includes the ``MECHANISM_SETTLE_MARGIN`` so the
        ``_wait_interruptible`` wait covers both the actuation time AND the
        motor-settle time (mechanism_node Trigger services return immediately
        while the real actuation runs in a background auto-stop thread).
        """
        params = get_grain_params(self._grain)
        settle = MECHANISM_SETTLE_MARGIN
        return {
            "clamp": float(params.get("clamp_duration", 3.0)) + settle,
            "unclamp": float(params.get("unclamp_duration", 3.0)) + settle,
            "tighten": float(params.get("tighten_duration", 3.0)) + settle,
            "untighten": float(params.get("untighten_duration", 3.0)) + settle,
            "servo": float(
                params.get("press_duration", params.get("clamp_duration", 3.0))
            ) + settle,
        }

    def _run_mechanism_sequence(
        self, steps: list[tuple[str, Callable[[], bool], float]]
    ) -> bool:
        """Run a timed mechanism sequence; abort on failure or interruption.

        Each step is ``(name, call, duration)``: the bridge call runs with
        retry (``_call_mechanism``) and the controller then waits
        ``duration`` seconds (interruptible, so STOP/emergency aborts the
        whole sequence).  Returns ``False`` when the sequence was aborted —
        the FSM has already been stopped in that case.
        """
        for name, call, duration in steps:
            if not self._call_mechanism(name, call):
                return False  # final failure already stopped the FSM
            if not self._wait_interruptible(float(duration)):
                logger.warning(
                    "Mechanism sequence interrupted at step '%s'", name
                )
                return False
        return True

    def _call_mechanism(
        self, action: str, call: Callable[[], bool], retries: int = 2
    ) -> bool:
        """Call a mechanism bridge method with retry; stop the FSM on failure.

        Retries *retries* times (default 2, i.e. 3 attempts total) after a
        ``False`` return or an exception.  If every attempt fails, the FSM
        is transitioned to STOPPED and the hardware is emergency-stopped —
        never silently continue.

        Returns ``True`` on success, ``False`` when the FSM was stopped.
        """
        # Relative movement may already have completed before reporting failure.
        # Replaying it would add a second full stroke.
        if action in ("press", "lift", "move_lift"):
            retries = 0
        for attempt in range(retries + 1):
            if not self._fsm.is_running:
                logger.warning("Mechanism %s skipped — FSM not running", action)
                return False
            try:
                logger.info("Mechanism %s begin attempt=%d/%d", action, attempt + 1, retries + 1)
                ok = call()
            except Exception:
                logger.exception(
                    "Mechanism %s raised on attempt %d/%d",
                    action, attempt + 1, retries + 1,
                )
                ok = False
            if ok:
                logger.info("Mechanism %s success attempt=%d", action, attempt + 1)
                return True
            logger.warning(
                "Mechanism %s failed (attempt %d/%d)",
                action, attempt + 1, retries + 1,
            )
            if attempt < retries:
                time.sleep(self._mechanism_retry_interval)

        logger.error(
            "Mechanism %s failed after %d attempts — stopping FSM",
            action, retries + 1,
        )
        detail = getattr(getattr(self, "_bridge", None), "last_error", "")
        if not isinstance(detail, str):
            detail = ""
        reason = f"mechanism {action} failed"
        if detail.strip():
            reason = f"{reason}: {detail.strip()}"
        self._stop_fsm(reason)
        return False

    def _stop_fsm(self, reason: str) -> None:
        """Transition the FSM to STOPPED after a mechanism failure."""
        logger.error("Stopping FSM: %s", reason)
        self._fsm.set_stop_reason(reason)
        # Best-effort hardware halt so nothing keeps running after a failure
        try:
            self._bridge.call_emergency_stop()
        except Exception:
            logger.exception("Emergency stop call failed during mechanism failure")
        try:
            self._fsm.transition(SamplingAction.STOP)
        except ValueError:
            logger.warning(
                "Could not transition to STOPPED from %s",
                self._fsm.current_state,
            )

    def _wait_interruptible(self, duration: float, poll_sec: float = 0.25) -> bool:
        """Sleep for *duration* in small slices so STOP/emergency can abort.

        Returns ``True`` when the full duration elapsed, ``False`` when the
        FSM left the running states (STOPPED/COMPLETED) before that.  A
        PAUSE holds the countdown without aborting.
        """
        remaining = float(duration)
        while remaining > 0.0:
            if not self._fsm.is_running:
                return False
            slice_ = min(poll_sec, remaining)
            time.sleep(slice_)
            if self._fsm.paused:
                continue  # hold the countdown while paused
            remaining -= slice_
        return True

    # ----------------------------------------------------------------
    # Thread management
    # ----------------------------------------------------------------

    def _run_async(self, func: callable) -> None:
        """Run a handler function in a background daemon thread."""
        self._pending += 1
        self._thread = threading.Thread(target=self._wrap_handler(func), daemon=True)
        self._thread.start()

    def _wrap_handler(self, func: callable) -> callable:
        """Wrap a handler so _pending is decremented before nested transitions."""
        def wrapper():
            self._pending -= 1
            try:
                func()
            except Exception:
                logger.exception("Handler %s failed", getattr(func, "__name__", func))
        return wrapper

    def emergency_stop(self) -> None:
        """Stop all operations immediately."""
        self._bridge.call_emergency_stop()
        self._fsm.transition(SamplingAction.STOP)
        self._pending -= 1

    @property
    def is_running(self) -> bool:
        return self._running
