#!/usr/bin/env python3
"""X2P 升降机构单次手动微调工具（编码器闭环相对距离控制）。

每次执行只移动一次，结束后程序立即退出。这里的距离是相对当前位置，
不是相对于机械原点的绝对位置。

用法：
  python3 scripts/move_lift.py up 5       # 从当前位置提升 5 cm
  python3 scripts/move_lift.py down 2.5  # 从当前位置下压 2.5 cm

默认单次最多移动 20 cm。需要改变限制时可设置环境变量，例如：
  MANUAL_LIFT_MAX_CM=10 python3 scripts/move_lift.py up 3
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import rospy


_DIRECTION_ALIASES = {
    "up": "up",
    "上": "up",
    "上升": "up",
    "提升": "up",
    "down": "down",
    "下": "down",
    "下降": "down",
    "下压": "down",
}


def _positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("距离必须是数字，单位为 cm") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("距离必须是大于 0 的有限数字")
    return parsed


def _max_distance_cm() -> float:
    raw = os.environ.get("MANUAL_LIFT_MAX_CM", "20")
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError("MANUAL_LIFT_MAX_CM 必须是数字") from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError("MANUAL_LIFT_MAX_CM 必须大于 0")
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="X2P 升降机构单次手动微调（相对当前位置）",
    )
    parser.add_argument(
        "direction",
        help="移动方向：up/down，也支持 上/下、提升/下压",
    )
    parser.add_argument(
        "distance_cm",
        type=_positive_float,
        help="移动距离，单位 cm，必须大于 0",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    direction = _DIRECTION_ALIASES.get(args.direction.strip().lower())
    if direction is None:
        print("错误：方向必须是 up/down（也支持 上/下、提升/下压）", file=sys.stderr)
        return 2

    try:
        max_distance_cm = _max_distance_cm()
    except ValueError as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    distance_cm = args.distance_cm
    if distance_cm > max_distance_cm:
        print(
            f"拒绝执行：单次距离 {distance_cm:g} cm 超过安全限制 "
            f"{max_distance_cm:g} cm",
            file=sys.stderr,
        )
        return 2

    direction_text = "上升" if direction == "up" else "下降"
    print(
        f"准备执行：从当前位置{direction_text} {distance_cm:g} cm；"
        "该命令不会自动寻找机械零点。"
    )

    try:
        rospy.init_node("move_lift_once", anonymous=True)
        rospy.wait_for_service("/mechanism/move_lift", timeout=5)
        from mechanism_node.srv import MoveLift

        move = rospy.ServiceProxy("/mechanism/move_lift", MoveLift)
        response = move(direction=direction, distance_cm=distance_cm)
    except (rospy.ROSException, rospy.ServiceException, ImportError) as exc:
        print(f"执行失败：{exc}", file=sys.stderr)
        return 1

    print(
        f"执行结果：success={response.success}，message={response.message}"
    )
    return 0 if response.success else 1


if __name__ == "__main__":
    sys.exit(main())
