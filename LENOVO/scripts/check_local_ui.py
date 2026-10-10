#!/usr/bin/env python3
"""Offscreen UI render check. No cloud, IPC, ROS or hardware startup."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("ui_launcher_check", ROOT / "3588/scripts/start_ui.py")
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)
launcher.load_qt()
sys.path[:0] = [str(ROOT / "3588/src"), str(ROOT / "3588"), str(ROOT / "LENOVO/src")]
from PySide2.QtWidgets import QApplication
from grain_sampling_ui.pages.guidance_page import GuidancePage
from grain_sampling_workflow.state_machine import SamplingStateMachine, SamplingAction as A

app = QApplication([])
page = GuidancePage()
page._local_operator_navigation = True
fsm = SamplingStateMachine()
fsm.transition(A.CONFIRM_READY)
fsm.transition(A.SYSTEM_RECORD_COMPLETE)
page.attach_state_machine(fsm)
page.resize(1100, 850)
page.show()
app.processEvents()
assert page._action_buttons["CONFIRM_LOCAL_ARRIVAL"].isVisible()
assert "不驱动底盘" in page._detail.text()
output = Path(sys.argv[1])
output.parent.mkdir(parents=True, exist_ok=True)
assert page.grab().save(str(output))
page.close()
print("PASS local operator-arrival UI rendered; no hardware/network connected")
