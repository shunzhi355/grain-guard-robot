#!/usr/bin/env python3
"""Linux offscreen integration: real UI + CLI/IPC, simulated serial peers only.

Never connects ROS, cloud, a camera or a physical serial port. Temporary task
state is not saved into the operator's real task history.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.name != "posix":
        parser.error("Linux AF_UNIX required")
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    spec = importlib.util.spec_from_file_location("ui_launcher_check", ROOT / "3588/scripts/start_ui.py")
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    launcher.load_qt()
    sys.path[:0] = [str(ROOT / "3588/src"), str(ROOT / "3588"), str(ROOT / "LENOVO/src")]

    import serial
    from grain_sampling_devices.mechanism_driver import PCA9685
    from grain_sampling_workflow.robot_bridge import RobotClient
    from grain_sampling_workflow.state_machine import SamplingState
    from PySide2.QtWidgets import QApplication
    from grain_sampling_ui import main as ui
    from grain_sampling_ui.pages.main_page import CameraWidget
    from grain_sampling_ui.theme import THEME_QSS

    def forbidden(*_args, **_kwargs):
        raise AssertionError("UI integration check attempted physical hardware/cloud I/O")

    serial.Serial = forbidden
    PCA9685.__init__ = forbidden
    CameraWidget._start = lambda *_args: None
    ui.CloudHttpClient.post = forbidden
    ui.save_current_task = lambda *_args, **_kwargs: None
    ui.clear_current_task = lambda *_args, **_kwargs: None
    real_socket = socket.socket

    class LocalOnlySocket(real_socket):
        def __init__(self, family=socket.AF_INET, *values, **kwargs):
            if family != socket.AF_UNIX:
                raise AssertionError("UI attempted external network I/O")
            super().__init__(family, *values, **kwargs)

    socket.socket = LocalOnlySocket
    app = QApplication([])
    app.setStyleSheet(THEME_QSS)
    window = None
    with tempfile.TemporaryDirectory(prefix="grain-ui-check-") as directory:
        runtime = Path(directory)
        config = runtime / "config.json"
        config.write_text(json.dumps({"runtime_dir": directory, "suction_policy": "external"}), encoding="utf-8")
        os.environ.update(GRAIN_LOCAL_RUNTIME="1", GRAIN_ROBOT_SOCKET=str(runtime / "control.sock"),
                          GRAIN_SAMPLING_UI_ENABLE_MECHANISM="1", GRAIN_SAMPLING_UI_FAKE_NAVIGATION="0",
                          GRAIN_SAMPLING_UI_SKIP_MAPPING="1", GRAIN_LOCAL_PREVIEW=str(runtime / "preview.json"))
        client = RobotClient()
        with (args.output / "daemon.log").open("w", encoding="utf-8") as log:
            # The only child entry point is hard-coded --simulate. Physical port
            # names are absent; no user config or live flag can be substituted.
            child = subprocess.Popen([sys.executable, str(ROOT / "LENOVO/scripts/local_robot.py"),
                                      "serve", "--simulate", "--config", str(config)],
                                     stdout=log, stderr=subprocess.STDOUT)

            def wait_for(predicate, timeout=6):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    app.processEvents()
                    if child.poll() is not None:
                        raise AssertionError(f"simulated daemon exited: {child.returncode}")
                    if predicate():
                        return
                    time.sleep(0.01)
                raise AssertionError("UI/IPC integration condition timed out")

            try:
                wait_for(lambda: (runtime / "control.sock").exists())
                assert client.request("status")["simulation"] is True
                window = ui.MainWindow()
                window.show()
                window.start_ros()
                wait_for(lambda: window._rc_mode_mirror == "auto")
                assert window._net_value.text() == "模拟硬件"
                assert window._ros_value.text() == "人工到位"
                assert window._slam_bridge.is_remote is False
                window._on_task_selected({"source": "local", "grain_type": "稻谷",
                                          "depth_list": [0.1], "waypoints": [{"x": 0, "y": 0}]})
                page = window._guidance_page
                assert page._fsm is not None and page._local_operator_navigation
                page._action_buttons["CONFIRM_READY"].click()
                wait_for(lambda: bool((client.request("status").get("navigation") or {}).get("goal_id")))
                wait_for(lambda: page._action_buttons["CONFIRM_LOCAL_ARRIVAL"].isVisible())
                assert page._fsm.current_state == SamplingState.NAVIGATE_TO_POINT
                assert not client.request("status")["chassis"]["motion_armed"]
                assert window.grab().save(str(args.output / "main-window.png"))
                page._action_buttons["CONFIRM_LOCAL_ARRIVAL"].click()
                wait_for(lambda: page._fsm.current_state == SamplingState.ARRIVED_PROMPT)
                assert not client.request("status")["chassis"]["motion_armed"]
                # Do not click the next ready button: startup/IPC is tested here;
                # the complete mechanical cycle is covered by workflow-sim.
                window.close()
                assert client.request("status")["safety_latched"] is True
                window = None
            finally:
                if window is not None:
                    window.close()
                child.terminate()
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.kill()  # this process owns only in-memory simulators
                    child.wait(timeout=3)
                    raise AssertionError("simulated daemon failed graceful shutdown")
            assert child.returncode == 0
            assert not (runtime / "control.sock").exists()
    print("PASS real MainWindow + simulated CLI/Unix IPC: task, RC mode, operator arrival, close-stop, clean shutdown")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
