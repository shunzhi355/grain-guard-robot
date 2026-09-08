#!/usr/bin/env python3
"""单次 move_lift 移动工具（编码器闭环精确距离）。

用法:
  python3 move_lift.py up 15      # 提升 15cm
  python3 move_lift.py down 5     # 下压 5cm
"""
import sys
import rospy


def main() -> None:
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    direction = sys.argv[1]
    distance_cm = float(sys.argv[2])
    if direction not in ("up", "down"):
        print("direction 必须是 up 或 down")
        sys.exit(1)

    rospy.init_node("move_lift_once", anonymous=True)
    rospy.wait_for_service("/mechanism/move_lift", timeout=5)
    from mechanism_node.srv import MoveLift
    move = rospy.ServiceProxy("/mechanism/move_lift", MoveLift)
    r = move(direction=direction, distance_cm=distance_cm)
    print(f"move_lift {direction} {distance_cm}cm: success={r.success}, msg={r.message}")


if __name__ == "__main__":
    main()
