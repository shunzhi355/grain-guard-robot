#!/usr/bin/env python3
"""Load installed Qt before project compatibility shims, then launch the UI.

--preview-descent opens only an isolated state-machine preview: no ROS node,
navigation, serial device, PCA9685 controller or workflow orchestrator is started.
"""
from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"


def load_qt():
    """Avoid src/PySide2 and src/PySide6 recursively importing each other."""
    original_path = list(sys.path)
    try:
        sys.path[:] = [p for p in original_path if Path(p or ".").resolve() != SRC_DIR]
        for name in ("PySide2", "PySide6"):
            try:
                binding = importlib.import_module(name)
            except ModuleNotFoundError as exc:
                if exc.name == name:
                    continue
                raise
            modules = {
                part: importlib.import_module(f"{name}.{part}")
                for part in ("QtCore", "QtGui", "QtWidgets", "QtNetwork")
            }
            # The current application mixes PySide2 and PySide6 imports.
            # Both names must refer to the same installed binding and types.
            for alias in ("PySide2", "PySide6"):
                sys.modules[alias] = binding
                for part, module in modules.items():
                    sys.modules[f"{alias}.{part}"] = module
            return binding
        raise RuntimeError("No installed Qt binding. Install PySide2 QtCore/QtGui/QtWidgets/QtNetwork or PySide6.")
    finally:
        sys.path[:] = original_path


def build_descent_preview():
    from PySide2.QtWidgets import QLabel, QVBoxLayout, QWidget
    from grain_sampling_ui.pages.guidance_page import GuidancePage
    from grain_sampling_workflow.state_machine import SamplingAction, SamplingState, SamplingStateMachine

    window = QWidget()
    window.setWindowTitle("下潜界面预览 — 不连接 ROS 或硬件")
    window.resize(1000, 720)
    layout = QVBoxLayout(window)
    banner = QLabel("仅界面模拟 · 保持电机主电源断开 · 不代表导航到点或伺服已执行")
    banner.setWordWrap(True)
    banner.setStyleSheet("color: #ffcc66; font-size: 18px; padding: 12px;")
    layout.addWidget(banner)

    page = GuidancePage()
    fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
    fsm.set_depth_targets([1.0])
    # Deliberately advance only the isolated state machine; no ROS messages.
    for action in (SamplingAction.CONFIRM_READY,
                   SamplingAction.SYSTEM_RECORD_COMPLETE,
                   SamplingAction.SYSTEM_NAV_COMPLETE):
        fsm.transition(action)
    page.attach_state_machine(fsm)  # Prevent showEvent from creating an orchestrator.

    def describe_preview(state, action=None):
        if state == SamplingState.ARRIVED_PROMPT:
            page._detail.setText("模拟已到点。保持电机断电，点击“已就绪”预览下潜页面。")
        elif state == SamplingState.PRESS_AND_SUCTION:
            page._detail.setText("仅预览下潜状态：未发送夹紧、伺服或吸粮指令。关闭窗口结束。")

    page._state_changed_signal.connect(describe_preview)
    describe_preview(fsm.current_state)
    layout.addWidget(page)
    # Keep references for the Qt lifetime and for non-hardware regression tests.
    window.preview_page = page
    window.preview_fsm = fsm
    return window


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-qt", action="store_true", help="Check installed Qt only, then exit")
    parser.add_argument("--preview-descent", action="store_true", help="Isolated UI-only descent preview")
    args = parser.parse_args(argv)
    try:
        binding = load_qt()
    except (ImportError, RuntimeError) as exc:
        print(f"Qt initialization failed: {exc}", file=sys.stderr)
        return 1
    print(f"Qt: {binding.__name__} {getattr(binding, '__version__', '')} ({binding.__file__})", flush=True)
    if args.check_qt:
        return 0
    sys.path[:0] = [str(SRC_DIR), str(PROJECT_ROOT)]
    if not args.preview_descent:
        from grain_sampling_ui.main import main as run_main_ui
        return run_main_ui()

    from PySide2.QtWidgets import QApplication
    from grain_sampling_ui.theme import THEME_QSS
    app = QApplication([sys.argv[0]])
    app.setStyleSheet(THEME_QSS)
    window = build_descent_preview()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
