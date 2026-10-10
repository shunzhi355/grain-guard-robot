"""ROS1 navigation -> PC USB-TTL -> STM32 USART2, 115200 8N1.

One-way setpoints with passive mode feedback; MCU owns RC and command watchdog.
Local arm gates navigation commands; it is not MCU enable confirmation.
"""
import json
import math
import os
import threading
import time

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices.chassis_serial import ChassisSerial, ChassisError, DEFAULT_SERIAL_PORT


def normalized_effort(forward, turn):
    if not all(math.isfinite(v) for v in (forward, turn)):
        raise ValueError("non-finite chassis command")
    return tuple(round(max(-1.0, min(1.0, v)) * 1000) for v in (forward, turn))


class ChassisNode:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String, Empty, Bool
        from std_srvs.srv import Trigger, TriggerResponse
        self.ros = rospy
        self.String, self.TriggerResponse = String, TriggerResponse
        self.port = rospy.get_param("~port", os.getenv("CHASSIS_SERIAL_PORT", DEFAULT_SERIAL_PORT))
        self.baud = int(rospy.get_param("~baud", 115200))
        self.max_linear = float(rospy.get_param("~max_linear_mps", 0.3))
        self.max_angular = float(rospy.get_param("~max_angular_rps", 0.8))
        if not all(math.isfinite(v) and v > 0 for v in (self.max_linear, self.max_angular)):
            raise ValueError("velocity scales must be finite and positive")
        self.lock = threading.RLock()
        self.link = None
        self.armed = False
        self.command = None
        self.command_time = 0.0
        self.arm_time = 0.0
        self.obstacle = False
        self.status_pub = rospy.Publisher("/chassis/status", String, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher("/rc_mode", String, queue_size=1, latch=True)
        self.cancel_pub = rospy.Publisher("/cancel_goal", Empty, queue_size=1)
        self.Empty = Empty
        rospy.Subscriber("/chassis/effort", Twist, self.on_effort, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.on_velocity, queue_size=1)
        rospy.Subscriber("/cancel_goal", Empty, lambda _: self.stop(), queue_size=1)
        rospy.Subscriber("/obstacle_detected", Bool, self.on_obstacle, queue_size=1)
        self.services = [rospy.Service("/chassis/" + name, Trigger,
                         lambda _, k=kind: self.service(k)) for name, kind in (
                             ("arm", p.AUTO_ARM), ("stop", p.AUTO_STOP),
                             ("estop", p.ESTOP), ("clear_estop", p.CLEAR_ESTOP),
                             ("recover", p.RECOVER))]

    def on_effort(self, msg):
        self.set_command(msg.linear.x, msg.angular.z)

    def on_velocity(self, msg):
        self.set_command(msg.linear.x / self.max_linear, msg.angular.z / self.max_angular)

    def set_command(self, forward, turn):
        with self.lock:
            try:
                effort = normalized_effort(forward, turn)
            except ValueError:
                self.stop()
                return
            # Drop queued motion while disarmed; it cannot survive a later arm.
            if self.armed and not self.obstacle:
                self.command = effort
                self.command_time = time.monotonic()
                if effort == (0, 0) and self.link:
                    try:
                        self.link.stream_control(p.AUTO_STOP)
                    except (OSError, ChassisError):
                        self.disconnect()

    def on_obstacle(self, msg):
        with self.lock:
            self.obstacle = bool(msg.data)
            if self.obstacle:
                self.stop()
                self.cancel_pub.publish(self.Empty())

    def stop(self):
        with self.lock:
            self.armed = False
            self.command = None
            if self.link:
                try:
                    self.link.stream_control(p.AUTO_STOP)
                except (OSError, ChassisError):
                    self.disconnect()

    def service(self, kind):
        with self.lock:
            try:
                if not self.link:
                    raise ChassisError("serial link unavailable")
                if kind == p.AUTO_ARM:
                    if self.obstacle:
                        raise ChassisError("obstacle blocks navigation")
                    self.link.stream_control(p.AUTO_STOP)
                    self.armed = True
                    self.command = None
                    self.arm_time = time.monotonic()
                    self.link.stream_effort(0, 0)
                else:
                    self.armed = False
                    self.command = None
                    self.link.stream_control(kind)
                    self.cancel_pub.publish(self.Empty())
                return self.TriggerResponse(success=True,
                    message="local request accepted; MCU execution is unconfirmed")
            except (OSError, ChassisError) as exc:
                self.disconnect()
                return self.TriggerResponse(success=False, message=str(exc))

    def disconnect(self):
        self.armed = False
        self.command = None
        if self.link:
            try:
                self.link.close()
            except (OSError, ChassisError):
                pass
        self.link = None
        self.mode_pub.publish(self.String(data=""))
        self.status_pub.publish(self.String(data=json.dumps({"connected": False})))
        self.cancel_pub.publish(self.Empty())

    def tick(self):
        import serial
        with self.lock:
            if self.link is None:
                port = serial.Serial(self.port, self.baud, timeout=0.01, write_timeout=0.05,
                                     exclusive=True)
                self.link = ChassisSerial(port)
                port.reset_input_buffer()
                self.armed = False
                self.command = None
                self.connected_time = time.monotonic()
            now = time.monotonic()
            mode = self.link.poll_mode()
            self.mode_pub.publish(self.String(data=mode))
            self.status_pub.publish(self.String(data=json.dumps({
                "port_open": True, "communication": "stream_with_mode_feedback",
                "execution_confirmed": False, "rc_mode": mode,
                "navigation_enabled": self.armed})))
            # Feedback is display-only. Do not reinstate handshake/ACK gating.
            if self.armed:
                if now - (self.command_time if self.command is not None else self.arm_time) > 0.2:
                    # A completed goal already sent zero; preserve ARRIVED for
                    # the sampling workflow instead of publishing cancellation.
                    cancel = self.command != (0, 0)
                    self.stop()
                    if cancel:
                        self.cancel_pub.publish(self.Empty())
                elif self.command is not None:
                    if self.command == (0, 0):
                        self.link.stream_control(p.AUTO_STOP)
                    else:
                        self.link.stream_effort(*self.command)

    def run(self):
        try:
            while not self.ros.is_shutdown():
                started = time.monotonic()
                try:
                    self.tick()
                except (OSError, ChassisError) as exc:
                    self.ros.logerr_throttle(2, "Chassis serial: %s", exc)
                    with self.lock:
                        self.disconnect()
                    time.sleep(0.5)
                time.sleep(max(0, 0.05 - (time.monotonic() - started)))
        finally:
            with self.lock:
                self.disconnect()


def main():
    import rospy
    rospy.init_node("chassis_serial")
    ChassisNode().run()


if __name__ == "__main__":
    main()
