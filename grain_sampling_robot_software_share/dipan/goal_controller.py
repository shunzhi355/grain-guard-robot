#!/usr/bin/env python3
"""Point-goal controller for the encoderless differential tracked chassis.

The node closes the position loop with FAST-LIO odometry and sends normalized
forward/turn efforts to motor_driver.py's UDP daemon.  The values sent to the
daemon are dimensionless; they are deliberately not advertised as m/s or rad/s.
"""

from __future__ import annotations

import math
import socket
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
from tf.transformations import euler_from_quaternion


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def same_frame(left: str, right: str) -> bool:
    return left.lstrip("/") == right.lstrip("/")


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class GoalController:
    def __init__(self) -> None:
        # ROS interfaces.
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.cancel_topic = rospy.get_param("~cancel_topic", "/cancel_goal")
        self.control_rate = float(rospy.get_param("~control_rate", 10.0))

        # Heading offset (rad). FAST-LIO odom yaw may transiently initialize
        # 180 deg off during pure straight-line startup and self-correct once
        # enough geometry is seen; leave 0 and calibrate per mapping session if
        # the chassis heading disagrees with odom yaw.
        self.heading_offset = float(rospy.get_param("~heading_offset", 0.0))

        # UDP motor daemon.
        self.motor_host = rospy.get_param("~motor_host", "127.0.0.1")
        self.motor_port = int(rospy.get_param("~motor_port", 8765))

        # Point controller. All efforts below are normalized, not physical speed.
        self.arrival_distance = float(rospy.get_param("~arrival_distance", 0.12))
        self.arrival_resume_distance = float(
            rospy.get_param("~arrival_resume_distance", 0.13)
        )
        self.arrival_confirm_time = float(rospy.get_param("~arrival_confirm_time", 1.0))
        self.final_yaw_tolerance = float(
            rospy.get_param("~final_yaw_tolerance", math.radians(175.0))
        )
        self.final_yaw_resume_tolerance = float(
            rospy.get_param("~final_yaw_resume_tolerance", math.radians(178.0))
        )
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.80))
        self.rotate_threshold = float(
            rospy.get_param("~rotate_threshold", math.radians(45.0))
        )
        self.drive_abort_angle = float(
            rospy.get_param("~drive_abort_angle", math.radians(175.0))
        )
        self.k_turn = float(rospy.get_param("~k_turn", 0.3))
        # 默认力度按履带车实机标定(2026-09-03): 0.15 起步太弱无法克服静摩擦,
        # 会在 2s 内被 stuck 检测误报 FAULT; 0.5 左右才有明显起步。
        self.far_forward = float(rospy.get_param("~far_forward", 0.55))
        self.near_forward = float(rospy.get_param("~near_forward", 0.40))
        self.max_rotate_turn = float(rospy.get_param("~max_rotate_turn", 0.20))
        self.max_drive_turn = float(rospy.get_param("~max_drive_turn", 0.12))
        self.max_effort_step = float(rospy.get_param("~max_effort_step", 0.15))

        # Safety checks.
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.30))
        self.max_pose_jump = float(rospy.get_param("~max_pose_jump", 1.0))
        self.max_yaw_jump = float(rospy.get_param("~max_yaw_jump", math.radians(75.0)))
        self.jump_check_max_dt = float(rospy.get_param("~jump_check_max_dt", 0.5))
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 2.0))
        # 0.05m/2s 对低起步速度偏严, 放低到 0.03m 降低误报。
        self.stuck_min_motion = float(rospy.get_param("~stuck_min_motion", 0.03))
        self.stuck_forward_threshold = float(
            rospy.get_param("~stuck_forward_threshold", 0.05)
        )
        self.allow_goal_frame_mismatch = parse_bool(
            rospy.get_param("~allow_goal_frame_mismatch", False)
        )

        self._validate_params()

        # Runtime state.
        self.pose: Optional[Tuple[float, float, float]] = None
        self.odom_frame = ""
        self.last_odom_receive = rospy.Time(0)
        self.last_odom_stamp = rospy.Time(0)
        self.goal: Optional[Tuple[float, float]] = None
        self.goal_yaw: Optional[float] = None
        self.goal_frame = ""
        self.state = "WAIT_ODOM"
        self.fault_reason = ""
        self.arrival_since: Optional[rospy.Time] = None
        self.position_locked = False
        self.final_yaw_locked = False
        self.last_forward = 0.0
        self.last_turn = 0.0
        self.stuck_reference: Optional[Tuple[rospy.Time, float, float]] = None
        self.last_status = ""
        # 已对其发出 stop 的"停状态"; 用于只在状态切换时停一次, 避免空闲时 spam stop
        self._stop_state: Optional[str] = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_addr = (self.motor_host, self.motor_port)

        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.cmd_debug_pub = rospy.Publisher("~normalized_cmd", Twist, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=5)
        rospy.Subscriber(self.cancel_topic, Empty, self.cancel_callback, queue_size=5)
        rospy.Subscriber("/obstacle_detected", Bool, self.obstacle_callback, queue_size=5)

        rospy.on_shutdown(self.on_shutdown)
        self.publish_status("WAIT_ODOM")
        rospy.loginfo(
            "goal_controller ready: odom=%s goal=%s motor=udp://%s:%d",
            self.odom_topic,
            self.goal_topic,
            self.motor_host,
            self.motor_port,
        )
        rospy.logwarn(
            "Controller outputs normalized motor effort, not physical velocity. "
            "Start testing with tracks lifted or in a clear area."
        )
        self._obstacle_blocked = False
        self._pre_obstacle_state = "WAIT_GOAL"

    def _validate_params(self) -> None:
        if self.control_rate <= 0.0:
            raise ValueError("~control_rate must be > 0")
        if not 0.0 < self.arrival_distance < self.arrival_resume_distance < self.approach_distance:
            raise ValueError(
                "require 0 < arrival_distance < arrival_resume_distance < approach_distance"
            )
        if not 0.0 < self.final_yaw_tolerance < self.final_yaw_resume_tolerance < math.pi:
            raise ValueError(
                "require 0 < final_yaw_tolerance < final_yaw_resume_tolerance < pi"
            )
        if not 0.0 < self.rotate_threshold < self.drive_abort_angle < math.pi:
            raise ValueError("require 0 < rotate_threshold < drive_abort_angle < pi")
        for name, value in (
            ("far_forward", self.far_forward),
            ("near_forward", self.near_forward),
            ("max_rotate_turn", self.max_rotate_turn),
            ("max_drive_turn", self.max_drive_turn),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"~{name} must be in [0, 1]")
        if self.near_forward > self.far_forward:
            raise ValueError("~near_forward must be <= ~far_forward")

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation
        quaternion_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(quaternion_norm) or quaternion_norm < 0.5:
            self.enter_fault("invalid_odometry_quaternion")
            return

        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_to_pi(yaw + self.heading_offset)  # nose heading in odom frame
        new_pose = (float(pose.position.x), float(pose.position.y), float(yaw))
        if not all(math.isfinite(value) for value in new_pose):
            self.enter_fault("non_finite_odometry")
            return

        now = rospy.Time.now()
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else now
        if self.pose is not None and self.last_odom_stamp != rospy.Time(0):
            dt = (stamp - self.last_odom_stamp).to_sec()
            if 0.0 < dt <= self.jump_check_max_dt:
                jump = math.hypot(new_pose[0] - self.pose[0], new_pose[1] - self.pose[1])
                yaw_jump = abs(wrap_to_pi(new_pose[2] - self.pose[2]))
                if jump > self.max_pose_jump or yaw_jump > self.max_yaw_jump:
                    self.enter_fault(
                        f"odometry_jump position={jump:.2f}m yaw={math.degrees(yaw_jump):.1f}deg"
                    )
                    return

        self.pose = new_pose
        self.odom_frame = msg.header.frame_id
        self.last_odom_receive = now
        self.last_odom_stamp = stamp

        if self.state == "WAIT_ODOM":
            self.state = "WAIT_GOAL" if self.goal is None else "ROTATE"
            self.publish_status(self.state)

    def goal_callback(self, msg: PoseStamped) -> None:
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        if not math.isfinite(gx) or not math.isfinite(gy):
            rospy.logerr("Rejected non-finite goal")
            return

        q = msg.pose.orientation
        quaternion_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(quaternion_norm) or quaternion_norm < 0.5:
            rospy.logerr("Rejected goal with invalid orientation quaternion")
            return
        _, _, goal_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if not math.isfinite(goal_yaw):
            rospy.logerr("Rejected goal with non-finite yaw")
            return

        goal_frame = msg.header.frame_id
        if (
            self.pose is not None
            and goal_frame
            and self.odom_frame
            and not same_frame(goal_frame, self.odom_frame)
            and not self.allow_goal_frame_mismatch
        ):
            rospy.logerr(
                "Rejected goal frame '%s': /Odometry frame is '%s'. "
                "Set RViz Fixed Frame to %s or explicitly set "
                "~allow_goal_frame_mismatch:=true only when both frames are identical.",
                goal_frame,
                self.odom_frame,
                self.odom_frame,
            )
            return

        self.goal = (gx, gy)
        self.goal_yaw = float(goal_yaw)
        self.goal_frame = goal_frame
        self.fault_reason = ""
        self.arrival_since = None
        self.position_locked = False
        self.final_yaw_locked = False
        self.stuck_reference = None
        self.last_forward = 0.0
        self.last_turn = 0.0
        self.state = "WAIT_ODOM" if self.pose is None else "ROTATE"
        self.publish_status(self.state)
        rospy.loginfo(
            "Accepted goal x=%.3f y=%.3f yaw=%.1fdeg frame=%s",
            gx,
            gy,
            math.degrees(goal_yaw),
            goal_frame or "<empty>",
        )

    def cancel_callback(self, _msg: Empty) -> None:
        self.goal = None
        self.goal_yaw = None
        self.goal_frame = ""
        self.arrival_since = None
        self.position_locked = False
        self.final_yaw_locked = False
        self.stuck_reference = None
        self.state = "WAIT_ODOM" if self.pose is None else "WAIT_GOAL"
        self.send_stop()
        self.publish_status(self.state)
        rospy.logwarn("Goal cancelled; motors stopped")

    def obstacle_callback(self, msg: Bool) -> None:
        rospy.loginfo("OBSTACLE_CALLBACK fired: %s blocked=%s", msg.data, self._obstacle_blocked)
        if msg.data and not self._obstacle_blocked:
            self._obstacle_blocked = True
            self._pre_obstacle_state = self.state
            self.publish_status("OBSTACLE_STOP")
        elif not msg.data and self._obstacle_blocked:
            self._obstacle_blocked = False
            self.state = self._pre_obstacle_state
            self.publish_status(self.state)

    def enter_fault(self, reason: str) -> None:
        if self.state != "FAULT" or reason != self.fault_reason:
            rospy.logerr("Goal controller fault: %s", reason)
        self.fault_reason = reason
        self.state = "FAULT"
        self.send_stop()
        self.publish_status(f"FAULT:{reason}")

    def compute_command(self, now: rospy.Time) -> Tuple[float, float]:
        assert self.pose is not None
        assert self.goal is not None
        assert self.goal_yaw is not None

        dx = self.goal[0] - self.pose[0]
        dy = self.goal[1] - self.pose[1]
        distance = math.hypot(dx, dy)
        goal_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(goal_heading - self.pose[2])

        # Reaching the position is a one-way transition for the current goal.
        # FAST-LIO pose noise while rotating must not reactivate translation.
        if distance <= self.arrival_distance:
            self.position_locked = True
        elif self.position_locked and distance > self.arrival_resume_distance:
            self.position_locked = False

        if self.position_locked:
            final_yaw_error = wrap_to_pi(self.goal_yaw - self.pose[2])
            # Entering the 5 degree tolerance completes the goal permanently.
            # Do not restart rotation if inertia or odometry noise later moves
            # the reported yaw outside the tolerance.
            if abs(final_yaw_error) <= self.final_yaw_tolerance:
                self.final_yaw_locked = True
                self.state = "ARRIVED"
                self.publish_status(
                    f"ARRIVED x={self.goal[0]:.3f} y={self.goal[1]:.3f} "
                    f"position_error={distance:.3f} "
                    f"yaw_error={math.degrees(final_yaw_error):.1f}deg"
                )
                return 0.0, 0.0

            self.state = "FINAL_ROTATE"
            self.publish_status(
                f"FINAL_ROTATE position_error={distance:.3f} "
                f"yaw_error={math.degrees(final_yaw_error):.1f}deg"
            )
            return 0.0, clamp(
                self.k_turn * final_yaw_error,
                -self.max_rotate_turn,
                self.max_rotate_turn,
            )

        self.arrival_since = None
        self.final_yaw_locked = False
        if self.state == "ARRIVED":
            return 0.0, 0.0

        if abs(heading_error) > self.rotate_threshold:
            self.state = "ROTATE"
            forward = 0.0
            turn = clamp(
                self.k_turn * heading_error,
                -self.max_rotate_turn,
                self.max_rotate_turn,
            )
        else:
            self.state = "APPROACH" if distance <= self.approach_distance else "DRIVE"
            forward = self.near_forward if self.state == "APPROACH" else self.far_forward
            if abs(heading_error) >= self.drive_abort_angle:
                forward = 0.0
            turn = clamp(
                self.k_turn * heading_error,
                -self.max_drive_turn,
                self.max_drive_turn,
            )

        self.publish_status(
            f"{self.state} distance={distance:.3f} heading_error={math.degrees(heading_error):.1f}deg"
        )
        return forward, turn

    def limit_effort_step(self, forward: float, turn: float) -> Tuple[float, float]:
        # Stopping is always immediate. Nonzero commands ramp to reduce track shock.
        if forward == 0.0 and turn == 0.0:
            return 0.0, 0.0
        step = max(0.0, self.max_effort_step)
        if step == 0.0:
            return forward, turn
        forward = clamp(forward, self.last_forward - step, self.last_forward + step)
        turn = clamp(turn, self.last_turn - step, self.last_turn + step)
        return forward, turn

    def check_stuck(self, now: rospy.Time, forward: float) -> bool:
        assert self.pose is not None
        if abs(forward) < self.stuck_forward_threshold:
            self.stuck_reference = None
            return False

        if self.stuck_reference is None:
            self.stuck_reference = (now, self.pose[0], self.pose[1])
            return False

        started, start_x, start_y = self.stuck_reference
        motion = math.hypot(self.pose[0] - start_x, self.pose[1] - start_y)
        if motion >= self.stuck_min_motion:
            self.stuck_reference = (now, self.pose[0], self.pose[1])
            return False
        if (now - started).to_sec() >= self.stuck_timeout:
            self.enter_fault(f"stuck motion={motion:.3f}m in {self.stuck_timeout:.1f}s")
            return True
        return False

    def send_motor_command(self, forward: float, turn: float) -> None:
        forward = clamp(forward, -1.0, 1.0)
        turn = clamp(turn, -1.0, 1.0)
        self._stop_state = None  # 一旦再次驱动, 下次进入停状态需重新发 stop
        payload = f"cmd {forward:.4f} {turn:.4f}"
        try:
            self.sock.sendto(payload.encode("utf-8"), self.motor_addr)
        except OSError as exc:
            rospy.logerr_throttle(1.0, "Failed to send motor command: %s", exc)

        debug = Twist()
        debug.linear.x = forward
        debug.angular.z = turn
        self.cmd_debug_pub.publish(debug)
        self.last_forward = forward
        self.last_turn = turn

    def send_stop(self) -> None:
        try:
            self.sock.sendto(b"stop", self.motor_addr)
        except OSError:
            pass
        self.last_forward = 0.0
        self.last_turn = 0.0

    def _stop_once(self, key: str) -> None:
        """Send a single ``stop`` per distinct stop-state entry."""
        if self._stop_state != key:
            self.send_stop()
            self._stop_state = key

    def publish_status(self, text: str) -> None:
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            self.last_status = text

    def control_step(self) -> None:
        now = rospy.Time.now()

        # 只在进入"停"状态那一刻发一次 stop; 空闲(WAIT_GOAL/WAIT_ODOM/ARRIVED)时
        # 绝不再反复发 stop, 否则会与手动遥控(rc_node→cmd_vel_to_motor)在同一个
        # UDP daemon 上抢命令, 导致遥控卡顿/无反应(2026-09-03 实测)。
        if self.state == "FAULT":
            self._stop_once("FAULT")
            return
        if self.pose is None or self.last_odom_receive == rospy.Time(0):
            self.state = "WAIT_ODOM"
            self._stop_once("WAIT_ODOM")
            self.publish_status(self.state)
            return
        odom_age = (now - self.last_odom_receive).to_sec()
        if odom_age > self.odom_timeout:
            self.enter_fault(f"odometry_timeout age={odom_age:.3f}s")
            return
        if self._obstacle_blocked:
            self._stop_once("OBSTACLE")
            return
        if self.goal is None:
            self.state = "WAIT_GOAL"
            self._stop_once("WAIT_GOAL")
            self.publish_status(self.state)
            return
        if self.state == "ARRIVED":
            self._stop_once("ARRIVED")
            return


        forward, turn = self.compute_command(now)
        forward, turn = self.limit_effort_step(forward, turn)
        if self.check_stuck(now, forward):
            return
        self.send_motor_command(forward, turn)

    def spin(self) -> None:
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.control_step()
            rate.sleep()

    def on_shutdown(self) -> None:
        for _ in range(3):
            self.send_stop()
        self.sock.close()


def main() -> None:
    rospy.init_node("goal_controller", anonymous=False)
    GoalController().spin()


if __name__ == "__main__":
    main()
