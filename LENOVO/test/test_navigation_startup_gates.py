import time
from pathlib import Path
from types import SimpleNamespace
import grain_sampling_workflow

from grain_sampling_local.config import LocalConfig

grain_sampling_workflow.__path__.append(str(Path(__file__).resolve().parents[1] / "src" / "grain_sampling_workflow"))
from grain_sampling_workflow.navigation_session import _status_gates


ROOT = Path(__file__).resolve().parents[2]


def test_local_live_profile_matches_current_map_and_sensor_policy():
    config = LocalConfig.load(ROOT / "LENOVO/config/local_robot.local-live.json")
    assert config.navigation_mode == "local"
    assert config.odom_topic == "/task043_newmap/Odometry"
    assert config.map_frame == "camera_init"
    assert config.map_id == "warehouse_live_20261012_newmap_a"
    assert config.localization_health_mode == "odom_stable"
    assert config.require_pointcloud_for_navigation is False
    assert config.point_near_forward > 0.8
    assert config.point_far_forward >= config.point_near_forward
    assert config.point_far_forward <= 1.0
    assert config.point_turn_deadzone < config.point_turn_min == .50
    assert config.point_turn_max == .70


def _ready_status(config):
    now = time.monotonic()
    return {
        "runtime": "lenovo-local",
        "test_navigation_mode": False,
        "lenovo_online": True,
        "localization_health_mode": "odom_stable",
        "pointcloud_required": False,
        "pose": {
            "x_m": 0.2,
            "y_m": 1.5,
            "yaw_rad": 0.0,
            "localization_valid": True,
            "odom_at": now,
            "localization_at": now,
            "frame_id": config.map_frame,
        },
        "slam": {"map_id": config.map_id, "frame": config.map_frame},
        "chassis": {"rc_mode": "auto", "faults": 0},
    }


def test_local_preflight_accepts_fresh_odom_only_profile():
    config = SimpleNamespace(map_id="current-map", map_frame="camera_init")
    gates = _status_gates(_ready_status(config), require_local=True, config=config)
    assert all(gate.state == "PASS" for gate in gates), gates


def test_local_preflight_fails_closed_on_stale_pose_map_or_unknown_policy():
    config = SimpleNamespace(map_id="current-map", map_frame="camera_init")
    status = _ready_status(config)
    status["pose"]["odom_at"] -= 1.0
    status["slam"]["map_id"] = "old-map"
    status["pointcloud_required"] = None
    gates = _status_gates(status, require_local=True, config=config)
    failed = {gate.name for gate in gates if gate.state != "PASS"}
    assert {"pose_freshness", "map_identity", "pointcloud_policy"} <= failed
