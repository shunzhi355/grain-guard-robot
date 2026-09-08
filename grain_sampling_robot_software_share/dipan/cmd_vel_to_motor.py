#!/usr/bin/env python3
"""
ROS1 /cmd_vel bridge for the tracked chassis motor driver.

Subscribes:
  /cmd_vel  geometry_msgs/Twist

Sends UDP commands to:
  /home/orangepi/底盘/motor_driver.py daemon

The motor daemon expects normalized commands:
  cmd LINEAR ANGULAR

This bridge converts ROS physical-ish cmd_vel values to normalized values using
ROS private parameters.
"""

from __future__ import annotations

import socket

import rospy
from geometry_msgs.msg import Twist


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CmdVelToMotor:
    def __init__(self) -> None:
        self.host = rospy.get_param("~host", "127.0.0.1")
        self.port = int(rospy.get_param("~port", 8765))
        self.max_linear_mps = float(rospy.get_param("~max_linear_mps", 0.3))
        self.max_angular_rps = float(rospy.get_param("~max_angular_rps", 0.8))
        self.linear_scale = float(rospy.get_param("~linear_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))
        self.invert_linear = bool(rospy.get_param("~invert_linear", False))
        self.invert_angular = bool(rospy.get_param("~invert_angular", False))
        self.deadband = float(rospy.get_param("~deadband", 0.02))

        if self.max_linear_mps <= 0.0:
            raise ValueError("~max_linear_mps must be > 0")
        if self.max_angular_rps <= 0.0:
            raise ValueError("~max_angular_rps must be > 0")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (self.host, self.port)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.on_cmd_vel, queue_size=10)

        rospy.on_shutdown(self.stop)
        rospy.loginfo(
            "cmd_vel_to_motor ready: /cmd_vel -> udp://%s:%d, max_linear=%.3f m/s, max_angular=%.3f rad/s",
            self.host,
            self.port,
            self.max_linear_mps,
            self.max_angular_rps,
        )

    def normalize(self, linear_mps: float, angular_rps: float) -> tuple[float, float]:
        linear = linear_mps / self.max_linear_mps * self.linear_scale
        angular = angular_rps / self.max_angular_rps * self.angular_scale

        if self.invert_linear:
            linear = -linear
        if self.invert_angular:
            angular = -angular

        if abs(linear) < self.deadband:
            linear = 0.0
        if abs(angular) < self.deadband:
            angular = 0.0

        return clamp(linear, -1.0, 1.0), clamp(angular, -1.0, 1.0)

    def send(self, text: str) -> None:
        self.sock.sendto(text.encode("utf-8"), self.addr)

    def on_cmd_vel(self, msg: Twist) -> None:
        linear, angular = self.normalize(msg.linear.x, msg.angular.z)
        self.send(f"cmd {linear:.4f} {angular:.4f}")
        rospy.logdebug(
            "cmd_vel linear=%.3f angular=%.3f -> cmd %.4f %.4f",
            msg.linear.x,
            msg.angular.z,
            linear,
            angular,
        )

    def stop(self) -> None:
        try:
            self.send("stop")
        except OSError:
            pass
        self.sock.close()


def main() -> None:
    rospy.init_node("cmd_vel_to_motor")
    CmdVelToMotor()
    rospy.spin()


if __name__ == "__main__":
    main()
