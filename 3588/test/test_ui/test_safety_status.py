"""Safety interlocks shown by the UI must not be mistaken for normal status."""

import pytest

from grain_sampling_ui.safety_status import chassis_safety_message


@pytest.mark.parametrize(("fields", "expected"), [
    ({"chassis_link": "offline"}, "串口离线"),
    ({"estop_latched": True}, "急停已锁定"),
    ({"faults": 1}, "故障码 1"),
    ({"obstacle_stop": True}, "障碍物保护"),
    ({}, ""),
])
def test_chassis_safety_message(fields, expected):
    chassis = {"chassis_link": "online", "estop_latched": False,
               "faults": 0, "obstacle_stop": False, "obstacle": False}
    chassis.update(fields)
    assert expected in chassis_safety_message(chassis)
