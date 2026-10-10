#!/usr/bin/env python3
"""Acknowledge UI navigation goals without moving the chassis.

Run this beside the real UI during upper-mechanism commissioning.  Every
``/move_base_simple/goal`` is printed and acknowledged as ``arrive`` only
after the operator presses Enter.  No velocity command is published.
"""
from __future__ import annotations

import argparse
import queue
import sys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print usage without importing ROS or publishing messages",
    )
    return parser


def run_ros() -> int:
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
    except ImportError as exc:
        print(f"ROS Python modules unavailable: {exc}", file=sys.stderr)
        return 2

    goals: queue.Queue[tuple[float, float]] = queue.Queue()

    def on_goal(msg: PoseStamped) -> None:
        goals.put((msg.pose.position.x, msg.pose.position.y))

    rospy.init_node("fake_navigation_events", anonymous=True)
    publisher = rospy.Publisher("/waypoint_task_done", String, queue_size=10)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, on_goal)

    print("假导航已启动：只反馈到达，不发布 /cmd_vel。")
    print("UI 的记录起点会自动完成，不需要伪造建图完成消息。")
    print("等待 UI 发布导航目标……按 Ctrl+C 退出。", flush=True)

    while not rospy.is_shutdown():
        try:
            x, y = goals.get(timeout=0.2)
        except queue.Empty:
            continue

        print(f"\n收到导航目标：x={x:.3f}, y={y:.3f}")
        try:
            input("确认底盘无需移动且现场安全后，按 Enter 发送 arrive：")
        except EOFError:
            print("标准输入已关闭，未发送 arrive。", file=sys.stderr)
            return 1
        if rospy.is_shutdown():
            break
        publisher.publish(String(data="arrive"))
        print("已发送 /waypoint_task_done: arrive，继续等待下一个目标。", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.dry_run:
        print("DRY RUN: would wait for each /move_base_simple/goal and publish")
        print("         /waypoint_task_done = arrive after Enter; no /cmd_vel output.")
        return 0
    try:
        return run_ros()
    except KeyboardInterrupt:
        print("\n假导航已退出。")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
