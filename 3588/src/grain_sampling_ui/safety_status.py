"""Pure chassis safety-status formatter shared by the UI and tests."""


def chassis_safety_message(chassis: dict) -> str:
    """Return the highest-priority active chassis interlock for the UI."""
    if chassis.get("chassis_link") != "online":
        return "底盘串口离线，请检查连接"
    if chassis.get("estop_latched"):
        return "底盘急停已锁定；请现场检查并按设备规程复位"
    if chassis.get("faults"):
        return f"底盘故障码 {chassis['faults']}；禁止继续作业"
    if chassis.get("obstacle_stop") or chassis.get("obstacle"):
        return "底盘障碍物保护已触发（禁止行驶，不阻止机构联调）"
    return ""
