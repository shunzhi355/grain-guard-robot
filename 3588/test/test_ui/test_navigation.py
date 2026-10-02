"""Regression coverage for leaving the sampling workflow's terminal pages."""

import pytest

from PySide2.QtWidgets import QApplication, QPushButton, QStackedWidget, QWidget

from grain_sampling_ui.main import MainWindow
from grain_sampling_ui.page_manager import PageManager
from grain_sampling_ui.pages.main_page import CameraWidget
from grain_sampling_workflow.state_machine import SamplingAction, SamplingState, SamplingStateMachine


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def window(qapp, monkeypatch):
    # Navigation tests must never open the camera or start the robot daemon.
    monkeypatch.setattr(CameraWidget, "_start", lambda *args: None)
    win = MainWindow()
    win.show()
    qapp.processEvents()
    yield win
    win.close()


def attach_task(window, state):
    fsm = SamplingStateMachine()
    fsm._state = state
    window._guidance_page.attach_state_machine(fsm)
    window.page_manager.push("guidance")
    return fsm


def start_button(window):
    return next(btn for btn in window.findChildren(QPushButton) if btn.text() == "开始采样")


@pytest.mark.parametrize("state", [SamplingState.INIT, SamplingState.STOPPED, SamplingState.COMPLETED])
def test_start_sampling_opens_task_list_after_terminal_task(window, state):
    fsm = attach_task(window, state)
    start_button(window).click()
    assert window.page_manager.current_page_name() == "task_list"
    assert fsm.current_state == state  # Navigation must not reset or restart hardware.


@pytest.mark.parametrize("state", [SamplingState.INIT, SamplingState.STOPPED, SamplingState.COMPLETED])
def test_return_home_then_start_another_task(window, state):
    fsm = attach_task(window, state)
    page = window._guidance_page
    assert page._btn_home.isVisible()
    assert page._btn_abandon.isVisible() == (state == SamplingState.STOPPED)
    page._btn_home.click()
    assert window.page_manager.current_page_name() == "main"
    assert window.page_manager.pop() is None
    start_button(window).click()
    assert window.page_manager.current_page_name() == "task_list"
    assert fsm.current_state == state


@pytest.mark.parametrize("state", [
    SamplingState.NAVIGATE_TO_POINT, SamplingState.PRESS_AND_SUCTION,
    SamplingState.FORMAL_SAMPLING, SamplingState.RETURN,
])
def test_running_task_remains_accessible(window, state):
    fsm = attach_task(window, state)
    page = window._guidance_page
    assert not page._btn_home.isVisible()
    page._on_return_home()
    assert window.page_manager.current_page_name() == "guidance"
    window.page_manager.push("settings")
    start_button(window).click()
    assert window.page_manager.current_page_name() == "guidance"
    assert fsm.current_state == state


def test_return_completion_unlocks_menu(window):
    fsm = attach_task(window, SamplingState.RETURN)
    assert not window._guidance_page._btn_home.isVisible()
    fsm.transition(SamplingAction.SYSTEM_RETURN_COMPLETE)
    assert window._guidance_page._btn_home.isVisible()
    start_button(window).click()
    assert window.page_manager.current_page_name() == "task_list"


def test_back_history_has_one_entry_per_navigation(qapp):
    stack = QStackedWidget()
    manager = PageManager(stack)
    for name in ("main", "task_list", "guidance"):
        manager.register_page(name, QWidget())
    manager.set_home("main")
    manager.go_home()
    manager.push("task_list")
    manager.push("task_list")
    manager.push("guidance")
    manager.push("guidance")
    assert manager.pop() == "task_list"
    assert manager.pop() == "main"
    assert manager.pop() is None
