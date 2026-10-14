import importlib.util
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
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


def test_operator_reset_waits_for_mcu_clear_status(stack):
    service, _, _ = stack
    service.trip("operator emergency stop")
    assert service.chassis.estop_latched
    assert service.local_request({
        "action": "clear_estop", "mechanical_reset_confirmed": True,
    })["ok"]
    assert not service.chassis.estop_latched
    assert not service.latched
    task(service)


def test_unverified_stm32_link_keeps_console_up_and_prohibits_motion(tmp_path):
    """No STM32 status must not remove the daemon: latch instead of aborting."""
    config = LocalConfig(runtime_dir=str(tmp_path), suction_policy="external")
    service, _peers = entry.create_service(config, simulate=True)

    class UnverifiedChassis:
        link = None
        epoch = None
        last_command = None
        estop_latched = False

        def __init__(self):
            self.lock = threading.RLock()
            self.mode = "unknown"

        def start(self):
            pass

        def status(self):
            return {"rc_mode": "unknown", "rc_valid": False, "faults": None,
                    "motion_armed": False, "chassis_armed": False,
                    "chassis_link": "offline", "estop_latched": False}

        def stop(self, reason="stop"):
            pass

        def estop(self):
            pass

        def set_obstacle(self, value):
            pass

        def arm(self, goal_id):
            raise AssertionError("must not arm while the STM32 link is unverified")

        def clear_estop(self, **kwargs):
            raise AssertionError("must not clear a latch without fresh MCU telemetry")

        def close(self):
            pass

    service.chassis = UnverifiedChassis()
    service.start()
    try:
        assert service.latched
        assert "unverified" in service.last_error
        status = service.local_request({"action": "status"})
        assert status["safety_latched"] is True
        assert status["recovery"]["reset_required"] is True
        with pytest.raises(RuntimeError):
            service.local_request({"action": "goal", "goal": {
                "map_id": "test-map", "frame_id": "camera_init",
                "pose": {"x_m": 1.0, "y_m": 1.0}}})
        with pytest.raises(RuntimeError):
            service.local_request({"action": "mechanism", "name": "set_grain", "grain": "稻谷"})
    finally:
        service.close()


def test_host_estop_latch_is_not_reported_as_current_mcu_fault(stack):
    service, stm32, _ = stack
    assert not stm32.estop
    service.chassis.estop_latched = True
    with pytest.raises(RuntimeError, match="local chassis emergency-stop latch active"):
        service.check()


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


def test_uart_ack_loss_retries_only_same_request_id(stack):
    service, stm32, _ = stack
    task(service)
    stm32.drop_ack = True
    with pytest.raises(RuntimeError):
        service.local_request({"action": "mechanism", "name": "clamp"})
    assert service.latched
    frames = [frame for frame in stm32.frames if frame["kind"] == 0x38]
    assert len(frames) == 3
    assert len({frame["sequence"] for frame in frames}) == 1
    assert sum(key[1] == frames[0]["sequence"] for key in stm32.accepted) == 1
    assert len([event for event in service.events
                if event["kind"] == "stm32_ack_timeout"]) == 3


def test_lost_ack_recovers_without_repeating_clamp_action(stack):
    service, stm32, _ = stack
    task(service)
    stm32.drop_ack = True
    with ThreadPoolExecutor(max_workers=1) as pool:
        action = pool.submit(service.local_request, {
            "action": "mechanism", "name": "clamp",
        })
        wait_for(lambda: len([frame for frame in stm32.frames
                              if frame["kind"] == 0x38]) == 1)
        stm32.drop_ack = False
        assert action.result(timeout=2)["ok"]
    frames = [frame for frame in stm32.frames if frame["kind"] == 0x38]
    assert len(frames) == 2
    assert frames[0]["sequence"] == frames[1]["sequence"]
    assert sum(key[1] == frames[0]["sequence"] for key in stm32.accepted) == 1
    assert not service.latched
    assert any(event["kind"] == "stm32_ack_recovered" for event in service.events)


def test_emergency_stop_prevents_ack_retry(stack):
    service, stm32, _ = stack
    task(service)
    stm32.drop_ack = True
    with ThreadPoolExecutor(max_workers=1) as pool:
        action = pool.submit(service.local_request, {
            "action": "mechanism", "name": "clamp",
        })
        wait_for(lambda: len([frame for frame in stm32.frames
                              if frame["kind"] == 0x38]) == 1)
        service.trip("operator emergency stop")
        with pytest.raises(RuntimeError):
            action.result(timeout=2)
    assert len([frame for frame in stm32.frames if frame["kind"] == 0x38]) == 1


def test_rc_dropout_during_lift_matches_3588_policy(stack):
    service, stm32, servo = stack
    task(service)
    servo.leg_time_s = 0.8
    with ThreadPoolExecutor(max_workers=1) as pool:
        move = pool.submit(service.local_request, {
            "action": "mechanism", "name": "move_lift",
            "args": {"direction": "down", "distance_cm": 2.0},
        })
        wait_for(lambda: servo.triggers == 1)
        stm32.mode = 0
        stm32.faults = 1  # firmware FAULT_RC, not an independent ESTOP
        wait_for(lambda: service.chassis.link.mode_telemetry.snapshot()[0]["faults"] == 1)
        time.sleep(0.15)
        assert not service.latched
        stm32.mode = 2
        stm32.faults = 0
        wait_for(lambda: service.chassis.link.mode_telemetry.snapshot()[0]["faults"] == 0)
        assert move.result(timeout=3)["ok"]
    assert not service.latched
    assert servo.triggers == 1


@pytest.mark.parametrize("stop", ["estop", "non_rc_fault"])
def test_hard_stop_still_cancels_running_lift(stack, stop):
    service, stm32, servo = stack
    task(service)
    servo.leg_time_s = 2.0
    with ThreadPoolExecutor(max_workers=1) as pool:
        move = pool.submit(service.local_request, {
            "action": "mechanism", "name": "move_lift",
            "args": {"direction": "down", "distance_cm": 2.0},
        })
        wait_for(lambda: servo.triggers == 1)
        if stop == "estop":
            stm32.estop = True
        else:
            stm32.faults = 2
        wait_for(lambda: service.latched)
        with pytest.raises(RuntimeError, match="STM32 emergency stop/fault"):
            move.result(timeout=3)
    assert servo.triggers == 1


def test_mcu_fault_takes_priority_over_rc_mode_error(stack):
    service, stm32, _ = stack
    stm32.mode = 0
    stm32.faults = 1
    wait_for(lambda: service.chassis.link.mode_telemetry.snapshot()[0]["faults"] == 1)
    with pytest.raises(RuntimeError, match="STM32 emergency stop/fault:.*faults=1"):
        service.check()


def test_new_action_still_requires_auto_rc(stack):
    service, stm32, servo = stack
    stm32.mode = 0
    wait_for(lambda: service.chassis.link.mode_telemetry.snapshot()[0]["mode"] == 0)
    with pytest.raises(RuntimeError, match="fresh RC automatic mode required"):
        service.local_request({"action": "mechanism", "name": "clamp"})
    assert servo.triggers == 0


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


def test_point_navigation_can_explicitly_disable_cloud_gate(stack):
    service, _, _ = stack
    task(service)
    service.config = replace(service.config, navigation_mode="local", map_id="test-map",
                             require_pointcloud_for_navigation=False)
    token = service.local_request({"action": "nav_register"})["token"]
    now = time.monotonic()
    inputs = dict(x_m=0, y_m=0, yaw_rad=0, odom_at=now, cloud_at=now - 10,
                  localization_at=now, localization_valid=True, blocked=True,
                  frame_id="camera_init")
    service.local_request(dict(action="nav_inputs", token=token, sequence=1, inputs=inputs))
    reply = service.local_request({"action": "goal", "goal": {"frame_id": "map", "map_id": "test-map",
                                                                  "pose": dict(x_m=1, y_m=0, yaw_rad=0)}})
    assert reply["ok"]

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


def test_local_navigation_goal_requires_xy_but_not_target_yaw(stack):
    service, _, _ = stack
    task(service)
    nav_ready(service)
    reply = service.local_request({"action": "goal", "goal": {
        "frame_id": "camera_init", "map_id": "test-map",
        "pose": {"x_m": 1.0, "y_m": -0.5},
    }})
    assert reply["ok"]
    assert service._goal["pose"] == {"x_m": 1.0, "y_m": -0.5}


def test_local_navigation_service_rejects_simultaneous_forward_and_turn(stack):
    service, stm32, _ = stack
    task(service)
    token, _ = nav_ready(service)
    goal = service.local_request({"action": "goal", "goal": {
        "frame_id": "camera_init", "map_id": "test-map",
        "pose": {"x_m": 1.0, "y_m": 0.0},
    }})
    sequence_before = len(stm32.frames)
    with pytest.raises(ValueError, match="mutually exclusive"):
        service.local_request({
            "action": "nav_velocity", "token": token, "sequence": 2,
            "goal_id": goal["goal_id"], "created_at": time.monotonic(),
            "forward": 0.1, "turn": 0.1,
        })
    assert not [frame for frame in stm32.frames[sequence_before:] if frame["kind"] == 14]
    assert service.latched


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


def test_live_entry_refuses_unconfirmed_actuation(tmp_path, monkeypatch):
    # The entry point exports GRAIN_LOCAL_RUNTIME for its own process; restore
    # the caller environment so later tests do not inherit local-runtime mode.
    monkeypatch.delenv("GRAIN_LOCAL_RUNTIME", raising=False)
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


def test_point_controller_effort_is_configurable_up_to_protocol_limit():
    navigation = PointNavigator(far_forward=1.0, near_forward=0.8, max_effort_step=0.4)
    goal = {"goal_id": "drive", "pose": dict(x_m=2.0, y_m=0.0, yaw_rad=0)}
    pose = dict(x_m=0, y_m=0, yaw_rad=0)
    assert navigation.step(goal, pose)[:2] == pytest.approx((0.8, 0.0))
    assert navigation.step(goal, pose)[:2] == pytest.approx((1.0, 0.0))
    assert navigation.step(goal, pose)[:2] == pytest.approx((1.0, 0.0))


def test_point_controller_turns_before_forward_and_never_mixes_axes():
    navigation = PointNavigator(far_forward=0.9, near_forward=0.8, max_effort_step=0.4)
    goal = {"goal_id": "segment", "pose": dict(x_m=2.0, y_m=1.0, yaw_rad=0.0)}
    pose = dict(x_m=0.0, y_m=0.0, yaw_rad=0.0)
    forward, turn, arrived = navigation.step(goal, pose)
    assert (forward, turn, arrived) == pytest.approx((0.0, 0.18, False))
    pose["yaw_rad"] = math.atan2(1.0, 2.0)
    forward, turn, arrived = navigation.step(goal, pose)
    assert forward > 0.0 and turn == 0.0 and not arrived


def test_point_controller_arrival_ignores_target_yaw():
    navigation = PointNavigator(xy_tolerance=0.12)
    goal = {"goal_id": "xy-only", "pose": dict(x_m=0.0, y_m=0.0, yaw_rad=2.5)}
    pose = dict(x_m=0.05, y_m=0.0, yaw_rad=0.0)
    assert navigation.step(goal, pose) == (0.0, 0.0, False)
    assert navigation.step(goal, pose) == (0.0, 0.0, False)
    assert navigation.step(goal, pose) == (0.0, 0.0, True)


def test_point_controller_rejects_effort_above_protocol_limit():
    with pytest.raises(ValueError, match="0..1"):
        PointNavigator(far_forward=1.01)


@pytest.mark.parametrize("heading", [.201, .3, 1., -1.])
def test_turn_floor_survives_repeated_slew(heading):
    nav = PointNavigator(turn_min=.24, turn_max=.3)
    goal = {"goal_id":"turn-floor", "pose":{"x_m": math.cos(heading), "y_m":math.sin(heading)}}
    pose = {"x_m":0., "y_m":0., "yaw_rad":0.}
    for _ in range(20):
        forward, turn, arrived = nav.step(goal,pose)
        assert forward == 0 and not arrived
        assert .24 <= abs(turn) <= .30
        assert turn * heading > 0


def test_forward_starts_above_running_floor_and_heading_hysteresis_prevents_chatter():
    nav = PointNavigator(far_forward=.9, near_forward=.9, forward_deadzone=.8, forward_min=.9)
    goal = {"goal_id":"floor", "pose":{"x_m":.3,"y_m":0.}}
    for heading in [.19,.21,.19,.21]:
        pose = {"x_m":0.,"y_m":0.,"yaw_rad":-heading}
        assert nav.step(goal,pose)[:2] == (.9,0.)
    pose["yaw_rad"] = -.31
    assert nav.step(goal,pose)[0] == 0
    pose["yaw_rad"] = -.25
    assert nav.step(goal,pose)[0] == 0  # stay aligning until lower threshold
    pose["yaw_rad"] = -.19
    assert nav.step(goal,pose)[:2] == (.9,0.)


def test_arrival_hysteresis_stays_neutral_without_relaxing_acceptance():
    nav = PointNavigator(xy_tolerance=.12,arrival_hysteresis=.04)
    goal={"goal_id":"settle", "pose":{"x_m":0.,"y_m":0.}}
    for distance in [.119,.121,.118]:
        assert nav.step(goal,{"x_m":distance,"y_m":0.,"yaw_rad":0.}) == (0.,0.,False)
    assert nav.step(goal,{"x_m":.117,"y_m":0.,"yaw_rad":0.}) == (0.,0.,False)
    assert nav.step(goal,{"x_m":.116,"y_m":0.,"yaw_rad":0.}) == (0.,0.,True)
    assert nav.step(goal,{"x_m":.14,"y_m":0.,"yaw_rad":0.}) == (0.,0.,True)


def test_minimum_turn_is_not_deadzone_and_invalid_config_is_rejected():
    with pytest.raises(ValueError,match="turn_deadzone"):
        PointNavigator(turn_deadzone=.18,turn_min=.17)
    with pytest.raises(ValueError,match="tolerance"):
        PointNavigator(heading_realign_tolerance=.19)


class ServiceClient:
    def __init__(self,service): self.service = service
    def request(self,action,**values): return self.service.local_request({"action":action,**values})


def fresh_inputs():
    now=time.monotonic()
    return dict(x_m=0.,y_m=0.,yaw_rad=0.,odom_at=now,cloud_at=now,
                localization_at=now,localization_valid=True,blocked=False,frame_id="camera_init")


def test_navigation_worker_continues_pose_when_fault_latched_and_never_rearms(stack):
    from grain_sampling_local.navigation_worker import NavigationWorker
    service,stm32,_=stack
    service.config=replace(service.config,navigation_mode="local",map_id="test-map")
    service.trip("original navigation fault")
    worker=NavigationWorker(ServiceClient(service),PointNavigator())
    worker.tick(fresh_inputs())
    assert service.local_request({"action":"status"})["pose"]["localization_valid"]
    assert service.chassis.epoch is None
    assert service.last_error == "original navigation fault"
    worker.tick(fresh_inputs())
    assert not [f for f in stm32.frames if f["kind"]==14]


def test_mcu_reboot_is_reported_and_explicit_reset_never_restores_old_goal(stack):
    from grain_sampling_local.navigation_worker import NavigationWorker
    service,stm32,_=stack
    service.config=replace(service.config,navigation_mode="local",map_id="test-map")
    worker=NavigationWorker(ServiceClient(service),PointNavigator())
    worker.tick(fresh_inputs())
    old=service.local_request({"action":"goal","goal":{"map_id":"test-map","frame_id":"camera_init",
                                                        "pose":{"x_m":1.,"y_m":0.}}})
    worker.tick(fresh_inputs())
    stm32.boot += 1
    wait_for(lambda:service.latched)
    worker.tick(fresh_inputs())
    status=service.local_request({"action":"status"})
    assert status["recovery"]["reset_detected"]
    assert status["recovery"]["automatic_motion_resume"] is False
    assert service._goal is None and service.chassis.epoch is None
    assert status["navigation"]["goal_id"] == old["goal_id"]
    assert status["navigation"]["result"] == "FAILED"
    wait_for(lambda:service.chassis.status()["rc_mode"]=="auto")
    service.local_request({"action":"clear_estop","mechanical_reset_confirmed":True})
    worker.tick(fresh_inputs())
    assert not service.latched and not service._recovery_status()["reset_detected"]
    assert service.chassis.epoch is None and service._goal is None
    assert service.local_request({"action":"status"})["pose"]["localization_valid"]


def test_worker_control_error_stops_without_exiting_sensor_feed(stack):
    from grain_sampling_local.navigation_worker import NavigationWorker
    service,_,_=stack
    service.config=replace(service.config,navigation_mode="local",map_id="test-map")
    navigator=PointNavigator()
    worker=NavigationWorker(ServiceClient(service),navigator)
    worker.tick(fresh_inputs())
    service.local_request({"action":"goal","goal":{"map_id":"test-map","frame_id":"camera_init",
                                                    "pose":{"x_m":1.,"y_m":0.}}})
    navigator.step=lambda *args: (_ for _ in ()).throw(RuntimeError("stuck test"))
    worker.tick(fresh_inputs())
    assert service.latched and service.chassis.epoch is None
    assert "stuck test" in service.last_error
    worker.tick(fresh_inputs())
    assert service.local_request({"action":"status"})["pose"]["localization_valid"]


def test_service_preserves_first_fault_reason_after_shutdown_stop(stack):
    service,_,_=stack
    service.trip("fresh RC timed out")
    service.local_request({"action":"estop"})
    assert service.last_error == "fresh RC timed out"


@pytest.mark.parametrize("turn",[False,True])
def test_candidate_running_floor_reaches_production_serial_frame_without_extra_scaling(stack,turn):
    service,stm32,_=stack
    task(service)
    token,_=nav_ready(service)
    config=replace(service.config,point_far_forward=.9,point_near_forward=.9,
                   point_forward_deadzone=.8,point_forward_min=.9,
                   point_turn_min=.40,point_turn_max=.55)
    nav=PointNavigator.from_config(config)
    target={"x_m":-.3,"y_m":0.} if turn else {"x_m":.3,"y_m":0.}
    response=service.local_request({"action":"goal","goal":{
        "map_id":"test-map","frame_id":"camera_init","pose":target}})
    goal={"goal_id":response["goal_id"],"pose":target}
    forward,steer,_=nav.step(goal,{"x_m":0.,"y_m":0.,"yaw_rad":0.})
    service.local_request(dict(action="nav_velocity",token=token,sequence=2,
                               goal_id=response["goal_id"],created_at=time.monotonic(),
                               forward=forward,turn=steer))
    expected=(0,400) if turn else (900,0)
    wait_for(lambda:stm32.effort == expected)
    assert service.chassis.effort == expected
    if turn:
        forward,steer,_=nav.step(goal,{"x_m":0.,"y_m":0.,"yaw_rad":0.})
        service.local_request(dict(action="nav_velocity",token=token,sequence=3,
                                   goal_id=response["goal_id"],created_at=time.monotonic(),
                                   forward=forward,turn=steer))
        wait_for(lambda:stm32.effort == (0,550))


def test_navigation_worker_reconnect_does_not_claim_old_goal(stack):
    from grain_sampling_local.navigation_worker import NavigationWorker
    service,_,_=stack
    service.config=replace(service.config,navigation_mode="local",map_id="test-map")
    worker=NavigationWorker(ServiceClient(service),PointNavigator())
    worker.tick(fresh_inputs())
    service.trip("connection lost")
    service._nav_token=None
    worker.tick(fresh_inputs())
    assert worker.token is None
    worker.tick(fresh_inputs())
    assert worker.token is not None
    assert service._goal is None and service.chassis.epoch is None
    assert service.latched



def test_idle_mcu_reboot_is_detected_without_needing_motion_request(stack):
    service,stm32,_=stack
    old=service._baseline_boot
    stm32.boot += 1
    wait_for(lambda:service.latched)
    recovery=service._recovery_status()
    assert recovery["reset_detected"]
    assert recovery["mcu_boot"] != old
    assert recovery["reset_required"]
    assert service.chassis.epoch is None


def test_serial_reenumeration_updates_baseline_only_after_explicit_reset(stack):
    service,stm32,_=stack
    original=service._baseline_link
    stm32.disconnect()
    wait_for(lambda:service.chassis.link is None)
    stm32.reconnect()
    wait_for(lambda:service.chassis.link is not None and service.chassis.link is not original)
    wait_for(lambda:service.latched)
    assert service._baseline_link is original
    wait_for(lambda:service._recovery_status()["reset_ready"]
             and service.chassis.status()["rc_mode"] == "auto")
    service.local_request({"action":"clear_estop","mechanical_reset_confirmed":True})
    assert service._baseline_link is service.chassis.link
    assert not service._recovery_status()["reset_required"]
    assert service.chassis.epoch is None


@pytest.mark.parametrize("heading", [.201,.3,1.,-1.,-.201,math.pi])
def test_active_live_turn_profile_has_higher_floor_until_alignment(heading):
    config=LocalConfig.load(ROOT/"LENOVO/config/local_robot.local-live.json")
    assert config.point_turn_min == .50
    assert config.point_turn_max == .70
    nav=PointNavigator.from_config(config)
    goal={"goal_id":"higher-turn", "pose":{"x_m":math.cos(heading),"y_m":math.sin(heading)}}
    pose={"x_m":0.,"y_m":0.,"yaw_rad":0.}
    for _ in range(12):
        forward,turn,arrived=nav.step(goal,pose)
        assert forward == 0 and not arrived
        assert .50 <= abs(turn) <= .70
        assert turn*heading > 0
    pose["yaw_rad"] = heading
    forward,turn,arrived=nav.step(goal,pose)
    assert forward >= config.point_forward_min
    assert turn == 0 and not arrived


def test_transient_status_gap_stops_revokes_goal_without_global_estop(stack, monkeypatch):
    service, stm32, _ = stack
    task(service)
    token, inputs=nav_ready(service)
    goal=service.local_request({"action":"goal","goal":{
        "frame_id":"camera_init","map_id":"test-map","pose":{"x_m":1,"y_m":0}}})
    link=service.chassis.link
    snapshot=link.mode_telemetry.snapshot
    report, _=snapshot()
    frame_count=len(stm32.frames)
    monkeypatch.setattr(link.mode_telemetry,"snapshot",lambda:(report,time.monotonic()-1.01))
    wait_for(lambda:service._goal is None)
    assert not service.latched and not service.chassis.estop_latched
    assert service.navigation["new_goal_required"] is True
    assert service.navigation["automatic_resume"] is False
    assert service.chassis.epoch is None
    controls=[f["payload"] for f in stm32.frames[frame_count:] if f["kind"]==15]
    assert [6] in controls and [7] not in controls
    with pytest.raises(RuntimeError,match="authorized"):
        service.local_request({"action":"nav_velocity","token":token,"sequence":2,
          "goal_id":goal["goal_id"],"created_at":time.monotonic(),"forward":.9,"turn":0})
    monkeypatch.setattr(link.mode_telemetry,"snapshot",snapshot)
    wait_for(lambda:service.chassis.mode=="auto")
    assert service._goal is None and service.chassis.epoch is None
    # Healthy transport does not resume motion. A new explicit goal is needed.
    now=time.monotonic();inputs.update(odom_at=now,cloud_at=now,localization_at=now)
    service.local_request({"action":"nav_inputs","token":token,"sequence":3,"inputs":inputs})
    service.local_request({"action":"status","client_role":"ui"})
    new=service.local_request({"action":"goal","goal":{
        "frame_id":"camera_init","map_id":"test-map","pose":{"x_m":1,"y_m":0}}})
    assert new["goal_id"] != goal["goal_id"]
    assert not service.latched
    service.local_request({"action":"cancel"})


def test_already_revoked_status_gap_does_not_race_into_global_latch(stack):
    service, stm32, _ = stack
    from grain_sampling_local.service import TransientStatusLoss
    before=len(stm32.frames)
    service._goal=None;service._task_active=False
    service._abort_navigation_for_status_gap(TransientStatusLoss("TEST_ONLY delayed status failure"))
    assert not service.latched and not service.chassis.estop_latched
    assert not [f for f in stm32.frames[before:] if f["kind"]==15 and f["payload"]==[7]]


def test_corrupt_status_burst_keeps_crc_gate_and_requires_new_goal(stack):
    service, stm32, _ = stack
    from grain_sampling_devices import chassis_protocol as wire
    task(service);token,inputs=nav_ready(service)
    goal=service.local_request({"action":"goal","goal":{
        "frame_id":"camera_init","map_id":"test-map","pose":{"x_m":1,"y_m":0}}})
    # Pause only the simulated reports, not USB transport. Feed invalid CRC
    # bytes through the production RX parser throughout the status timeout.
    stm32.reports=False
    link=service.chassis.link
    values={k:0 for k in wire.STATUS_KEYS}
    values.update(boot=stm32.boot,mode=2,flags=4,rc_age_ms=0)
    corrupted=bytearray(wire.encode(wire.STATUS,0,999,
        wire.STATUS_STRUCT.pack(*(values[k] for k in wire.STATUS_KEYS))))
    corrupted[-1]^=1
    started=time.monotonic();sequence=1
    while service._goal is not None and time.monotonic()-started<1.5:
        with stm32.lock:link.serial.rx.extend(corrupted)
        now=time.monotonic();inputs.update(odom_at=now,cloud_at=now,localization_at=now)
        sequence+=1
        service.local_request({"action":"nav_inputs","token":token,"sequence":sequence,"inputs":inputs})
        service.local_request({"action":"status","client_role":"ui"})
        # Refresh the existing lease with NEUTRAL; never physical I/O.
        if service._epoch is not None:
            sequence+=1
            try:
                service.local_request({"action":"nav_velocity","token":token,"sequence":sequence,
                  "goal_id":goal["goal_id"],"created_at":now,"forward":0,"turn":0})
            except RuntimeError:pass  # timeout must reject/revoke this request
        time.sleep(.02)
    assert service._goal is None and service.chassis.epoch is None
    assert not service.latched and not service.chassis.estop_latched
    assert link.rx_diagnostics()["crc_errors"]>0
    assert service.navigation["new_goal_required"] is True
    stm32.reports=True
    wait_for(lambda:service.chassis.mode=="auto")
    assert service._goal is None and service.chassis.epoch is None


def test_explicit_reset_claims_replacement_before_clear_wait(stack,monkeypatch):
    service,stm32,_=stack
    service.trip("TEST_ONLY replacement requires explicit reset")
    # Freeze only the supervisor to deterministically test the reset edge;
    # the production UART workers still confirm fresh STATUS.
    service.running.clear();service.worker.join(timeout=1)
    service._stop_link=object()
    original=service.chassis.clear_estop
    def assert_current_handle(**kw):
        assert service._stop_link is service.chassis.link
        return original(**kw)
    monkeypatch.setattr(service.chassis,"clear_estop",assert_current_handle)
    service.local_request({"action":"clear_estop","mechanical_reset_confirmed":True})
    assert not service.latched and not service.chassis.estop_latched
    assert service._goal is None and service.chassis.epoch is None


def test_restore_control_hands_back_control_without_resuming_goal(stack):
    service,_,_=stack
    # A finished goal must never be replayed by handing control back.
    service.navigation={"goal_id":"finished","result":"SUCCEEDED"}
    service._goal,service._epoch=None,None
    result=service.local_request({"action":"restore_control","mechanical_reset_confirmed":True})
    assert result["ok"] is True
    assert result["requires_new_goal"] is True and result["resumed"] is False
    assert service._goal is None and service.chassis.epoch is None
    assert not service.latched and not service.chassis.estop_latched
    assert service.local_request({"action":"status"})["last_error"]==""


def test_restore_control_requires_explicit_operator_confirmation(stack):
    service,_,_=stack
    with pytest.raises(RuntimeError,match="confirmation"):
        service.local_request({"action":"restore_control"})
    assert not service.latched


def test_restore_control_refuses_while_motion_is_armed(stack):
    service,_,_=stack
    service.chassis.epoch=12345
    with pytest.raises(RuntimeError,match="no faults/motion"):
        service.local_request({"action":"restore_control","mechanical_reset_confirmed":True})
    assert service.chassis.epoch==12345


def test_transient_stop_write_failure_is_latched_and_never_claims_physical_stop(stack,monkeypatch):
    service,stm32,_=stack
    from grain_sampling_local.service import TransientStatusLoss
    task(service);nav_ready(service)
    service.local_request({"action":"goal","goal":{
        "frame_id":"camera_init","map_id":"test-map","pose":{"x_m":1,"y_m":0}}})
    def unavailable(packet):raise OSError("TEST_ONLY stop write failed")
    monkeypatch.setattr(service.chassis.link.serial,"write",unavailable)
    service._abort_navigation_for_status_gap(TransientStatusLoss("TEST_ONLY status gap"))
    assert service.latched and service.chassis.epoch is None and service._goal is None
    stopped=next(e for e in service.events if e["kind"]=="navigation_stopped_status_timeout")
    assert stopped["stop_request_submitted"] is False
    assert stopped["physical_stop_confirmed"] is False
    assert "stop write failed/offline" in service.last_error
