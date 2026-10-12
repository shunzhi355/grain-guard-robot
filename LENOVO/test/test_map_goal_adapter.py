from __future__ import annotations

from grain_sampling_workflow.robot_bridge import RobotBridge


class FakeClient:
    def __init__(self):
        self.calls = []
        self._status_count = 0

    def request(self, action, **values):
        self.calls.append((action, values))
        if action == "status":
            self._status_count += 1
            if self._status_count == 1:
                return {"safety_latched": False, "test_navigation_mode": False,
                        "slam": {"map_id": "warehouse_ab12", "map_version": 3,
                                 "frame": "camera_init"},
                        "pose": {"localization_valid": True}}
            return {"lenovo_online": True,
                    "navigation": {"goal_id": "goal-1", "result": "SUCCEEDED"}}
        if action == "goal":
            assert values["goal"] == {
                "frame_id": "camera_init",
                "map_id": "warehouse_ab12",
                "map_version": 3,
                "pose": {"x_m": 1.2, "y_m": -0.4, "yaw_rad": 0.0},
            }
            return {"goal_id": "goal-1"}
        raise AssertionError(action)


def test_map_goal_carries_active_map_identity_and_frame():
    client = FakeClient()
    bridge = RobotBridge(client=client)
    assert bridge.call_navigate(1.2, -0.4)
    assert [name for name, _ in client.calls if name == "goal"] == ["goal"]
