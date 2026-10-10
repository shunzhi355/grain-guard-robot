import importlib.util
import time
from dataclasses import replace
from pathlib import Path

import pytest
import serial

from grain_sampling_local.config import LocalConfig
from grain_sampling_local.navigation import PointNavigator
from grain_sampling_local.workflow_sim import run_workflow
from grain_sampling_bench.runner import wait_for
from grain_sampling_devices.mechanism_driver import PCA9685

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("local_robot_entry", ROOT / "LENOVO/scripts/local_robot.py")
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


@pytest.fixture(autouse=True)
def forbid_hardware(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("test attempted real serial/PCA9685 I/O")
    monkeypatch.setattr(serial, "Serial", forbidden)
    monkeypatch.setattr(PCA9685, "__init__", forbidden)


@pytest.fixture
def stack(tmp_path):
    config = LocalConfig(runtime_dir=str(tmp_path), suction_policy="external")
    service, peers = entry.create_service(config, simulate=True)
    service.start()
    wait_for(lambda: service.chassis.mode == "auto")
    yield service, *peers
    service.close()


def task(service):
    service.local_request({"action": "status", "client_role": "ui"})
    service.local_request({"action": "mechanism", "name": "set_grain", "grain": "稻谷"})


def test_complete_two_point_three_bin_flow(tmp_path):
    config = LocalConfig(runtime_dir=str(tmp_path), suction_policy="external")
    result = run_workflow(config, entry.create_service)
    assert result["passed"], result["error"]
    for state in ("ADD_PIPE_PROMPT", "PRESS_AND_SUCTION", "OPEN_BIN", "CONVEY_1", "EXTRACT_PIPE", "RELEASE_PIPE", "RETURN", "COMPLETED"):
        assert state in result["states"]
    assert result["states"].count("OPEN_BIN") == 6
    assert result["physical_hardware_verified"] is False
    assert result["external_suction_not_automated"] is True
    assert result["servo_triggers"] > 0


def test_ui_heartbeat_loss_stops_conveyor(stack):
    service, stm32, _ = stack
    task(service)
    service.local_request({"action": "mechanism", "name": "start_convey"})
    assert 2 in stm32.active
    service._ui_at = time.monotonic() - 2
    wait_for(lambda: service.latched)
    wait_for(lambda: not stm32.active)
    with pytest.raises(RuntimeError):
        service.local_request({"action": "mechanism", "name": "clamp"})


def test_disk_full_cannot_prevent_emergency_stop(stack, monkeypatch):
    service, stm32, _ = stack
    task(service)
    service.local_request({"action": "mechanism", "name": "start_convey"})
    original = Path.open
    def broken_log(path, *args, **kwargs):
        if path.name == "events.jsonl":
            raise OSError("simulated full disk")
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, "open", broken_log)
    service.trip("operator stop despite disk failure")
    assert service.latched
    assert not stm32.active


def test_uart_failure_is_not_replayed(stack):
    service, stm32, _ = stack
    task(service)
    stm32.drop_ack = True
    with pytest.raises(RuntimeError):
        service.local_request({"action": "mechanism", "name": "clamp"})
    assert service.latched
    assert len([frame for frame in stm32.frames if frame["kind"] == 0x38]) == 1


def test_operator_navigation_never_arms_base(stack):
    service, stm32, _ = stack
    task(service)
    reply = service.local_request({"action": "test_goal", "pose": {"x_m": 1, "y_m": 2}})
    assert service.chassis.epoch is None
    with pytest.raises(RuntimeError):
        service.local_request({"action": "test_arrive", "goal_id": "stale"})
    service.local_request({"action": "test_arrive", "goal_id": reply["goal_id"]})
    assert service.navigation["result"] == "SUCCEEDED"
    assert service.navigation["test_only"]
    assert not [frame for frame in stm32.frames if frame["kind"] == 14]


def nav_ready(service):
    service.config = replace(service.config, navigation_mode="local", map_id="test-map")
    token = service.local_request({"action": "nav_register"})["token"]
    now = time.monotonic()
    inputs = dict(x_m=0, y_m=0, yaw_rad=0, odom_at=now, cloud_at=now,
                  localization_at=now, localization_valid=True, blocked=False, frame_id="camera_init")
    service.local_request(dict(action="nav_inputs", token=token, sequence=1, inputs=inputs))
    return token, inputs


def test_local_navigation_uses_epoch_and_rejects_old_goal(stack):
    service, stm32, _ = stack
    task(service)
    token, inputs = nav_ready(service)
    reply = service.local_request({"action": "goal", "goal": {"frame_id": "map", "map_id": "test-map", "pose": dict(x_m=1, y_m=0, yaw_rad=0)}})
    assert service.chassis.epoch is not None
    service.local_request(dict(action="nav_velocity", token=token, sequence=2, goal_id=reply["goal_id"],
                               created_at=time.monotonic(), forward=0.1, turn=0))
    wait_for(lambda: stm32.effort != (0, 0))
    with pytest.raises(RuntimeError):
        service.local_request(dict(action="nav_velocity", token=token, sequence=3, goal_id="previous",
                                  created_at=time.monotonic(), forward=0.1, turn=0))
    assert service.latched
    assert service.chassis.epoch is None


@pytest.mark.parametrize("fault", ["blocked", "localization", "odom", "cloud"])
def test_unsafe_sensor_inputs_cannot_arm(stack, fault):
    service, stm32, _ = stack
    task(service)
    token, inputs = nav_ready(service)
    if fault == "blocked":
        inputs["blocked"] = True
    elif fault == "localization":
        inputs["localization_valid"] = False
    else:
        inputs[f"{fault}_at"] -= 2
    service.local_request(dict(action="nav_inputs", token=token, sequence=2, inputs=inputs))
    with pytest.raises(RuntimeError):
        service.local_request({"action": "goal", "goal": {"frame_id": "map", "map_id": "test-map", "pose": dict(x_m=1, y_m=0, yaw_rad=0)}})
    assert service.chassis.epoch is None


def test_unclamp_interlock_kept(stack):
    service, _, _ = stack
    task(service)
    service.mechanism.controller._lift_cycle_origin = 0
    with pytest.raises(RuntimeError, match="松夹"):
        service.local_request({"action": "mechanism", "name": "move_lift", "args": {"direction": "return", "distance_cm": 20}})
    assert service.latched
    assert service.mechanism.requires_mechanical_reset()


def test_recovery_marker_survives_process_restart(tmp_path):
    config = LocalConfig(runtime_dir=str(tmp_path), suction_policy="external")
    service, _ = entry.create_service(config, simulate=True)
    service.mechanism._save_recovery(True)
    service.close()
    restarted, _ = entry.create_service(config, simulate=True)
    try:
        assert restarted.latched
        assert restarted.mechanism.requires_mechanical_reset()
    finally:
        restarted.close()


def test_absent_suction_cannot_silently_start_full_task(stack):
    service, _, _ = stack
    service.config = replace(service.config, suction_policy="required")
    with pytest.raises(RuntimeError, match="suction"):
        task(service)
    assert not service._task_active


def test_live_entry_refuses_unconfirmed_actuation(tmp_path):
    with pytest.raises(SystemExit) as exc:
        entry.main(["serve", "--live", "--runtime-dir", str(tmp_path)])
    assert exc.value.code == 2


def test_point_controller_requires_three_stationary_frames():
    navigation = PointNavigator()
    goal = {"goal_id": "one", "pose": dict(x_m=0, y_m=0, yaw_rad=0)}
    pose = dict(x_m=0, y_m=0, yaw_rad=0)
    assert navigation.step(goal, pose) == (0, 0, False)
    assert navigation.step(goal, pose) == (0, 0, False)
    assert navigation.step(goal, pose) == (0, 0, True)
