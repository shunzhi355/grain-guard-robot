"""Offscreen checks for manual support/removal controls in the production page."""
import os
from pathlib import Path
import subprocess
import sys
import pytest


def test_removal_controls_visible_only_when_expected():
    # Import installed Qt before the board compatibility shims under src/.
    # Isolate Qt from the older ROS tests' module stubs and GUI state.
    script = r'''
import sys
from pathlib import Path
root = Path.cwd()
sys.path = [p for p in sys.path if Path(p or '.').resolve() != root / 'src']
try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    try:
        from PySide2.QtWidgets import QApplication
    except ImportError:
        sys.exit(77)
sys.path.insert(0, str(root / 'src'))
from grain_sampling_ui.pages.guidance_page import GuidancePage
from grain_sampling_workflow.state_machine import SamplingState as S, SamplingStateMachine
qapp = QApplication.instance() or QApplication([])
for state, button in (
    (S.EXTRACT_PIPE, None), (S.PIPE_SUPPORT_PROMPT, 'CONFIRM_PIPE_SUPPORTED'),
    (S.RELEASE_PIPE, None), (S.REMOVE_PIPE_PROMPT, 'CONFIRM_PIPE_REMOVED'),
):
    fsm = SamplingStateMachine()
    fsm._state = state
    fsm.current_pipe_index = 3
    page = GuidancePage()
    page.attach_state_machine(fsm)
    page.show()
    qapp.processEvents()
    try:
        visible = {name for name, btn in page._action_buttons.items() if btn.isVisible()}
        assert visible == ({button} if button else set())
        assert not page._btn_home.isVisible()
        if button:
            assert "剩余 3 节" in page._detail.text()
            page._action_buttons[button].click()
            qapp.processEvents()
            assert fsm.current_state == (S.RELEASE_PIPE if state == S.PIPE_SUPPORT_PROMPT else S.EXTRACT_PIPE)
            assert fsm.current_pipe_index == (3 if state == S.PIPE_SUPPORT_PROMPT else 2)
    finally:
        page.close()
'''
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run(
        [sys.executable, "-c", script], cwd=Path(__file__).resolve().parents[1],
        env=env, capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=30,
    )
    if result.returncode == 77:
        pytest.skip("Qt bindings unavailable")
    assert result.returncode == 0, result.stdout + result.stderr
