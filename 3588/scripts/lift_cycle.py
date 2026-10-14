#!/usr/bin/env python3
"""扦样周期运动：下5cm → 上2cm → 下5cm → 上2cm ... 直到深度达标。

每周期净下压 = down_cm - up_cm（默认 5-2=3cm），往复推进防粮压实。

用法:
  python3 lift_cycle.py <target_cm> [down_cm] [up_cm]
  例:
  python3 lift_cycle.py 40        # 净下压到 40cm（第1节）
  python3 lift_cycle.py 100 5 2   # 净下压到 100cm，每周期下5上2
  python3 lift_cycle.py 60 5 2    # 净下压到 60cm

安全:
  Ctrl+C 立即停止（当前周期动作完成后退出）。
"""
import os
import sys
import time

import rospy

from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sampling_params import PRESS_DOWN_CM, PRESS_UP_CM, PRESS_PAUSE_S
INTER_MOVE_PAUSE_S = float(os.environ.get("LIFT_PAUSE_S", PRESS_PAUSE_S))


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target_cm = float(sys.argv[1])
    down_cm = float(sys.argv[2]) if len(sys.argv) > 2 else PRESS_DOWN_CM
    up_cm = float(sys.argv[3]) if len(sys.argv) > 3 else PRESS_UP_CM
    if down_cm <= up_cm:
        print("down_cm 必须大于 up_cm（否则净下压 <= 0）")
        sys.exit(1)

    rospy.init_node("lift_cycle", anonymous=True)
    rospy.wait_for_service("/mechanism/move_lift", timeout=5)
    from mechanism_node.srv import MoveLift
    move = rospy.ServiceProxy("/mechanism/move_lift", MoveLift)

    net = 0.0
    cycle = 0
    print(f"周期运动: 目标 {target_cm}cm, 下{down_cm}cm/上{up_cm}cm, 净 {down_cm-up_cm}cm/周期")
    try:
        while net < target_cm:
            cycle += 1
            distance = min(down_cm, target_cm - net)
            r1 = move(direction="down", distance_cm=distance)
            if not r1.success:
                print(f"[周期{cycle}] 下压失败: {r1.message}")
                break
            net += distance
            if net >= target_cm - 1e-9:
                break
            time.sleep(INTER_MOVE_PAUSE_S)
            r2 = move(direction="up", distance_cm=up_cm)
            if not r2.success:
                print(f"[周期{cycle}] 上移失败: {r2.message}")
                break
            time.sleep(INTER_MOVE_PAUSE_S)
            net -= up_cm
            print(f"[周期{cycle}] 净下压 {net:.1f}cm / {target_cm}cm")
    except KeyboardInterrupt:
        print("\n用户中断")
    print(f"结束: 净下压 {net:.1f}cm (目标 {target_cm}cm)")


if __name__ == "__main__":
    main()
