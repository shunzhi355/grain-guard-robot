#!/usr/bin/env python3
"""Same-host ROS1 sensor/point-navigation adapter; never opens a serial port.

Run in a separate Noetic process/container sharing only control.sock with the
host daemon. A real localization-quality Bool heartbeat is REQUIRED; odometry
arrival alone is not used to invent localization validity.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "3588/src"), str(ROOT / "3588"), str(ROOT / "LENOVO/src")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--allow-point-navigation", action="store_true")
    args = parser.parse_args()
    from grain_sampling_local.config import LocalConfig
    from grain_sampling_local.localization_health import LocalizationHealthGate, startup_status_ready
    from grain_sampling_local.navigation import PointNavigator, wrap
    from grain_sampling_local.navigation_worker import NavigationWorker, NavigationSocketClient
    from grain_sampling_workflow.robot_bridge import RobotClient
    config = LocalConfig.load(args.config)
    if not args.allow_point_navigation or config.navigation_mode != "local":
        parser.error("explicit --allow-point-navigation and navigation_mode=local required")
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    if config.require_pointcloud_for_navigation:
        import sensor_msgs.point_cloud2 as pc2
        from sensor_msgs.msg import PointCloud2
    rospy.init_node("grain_lenovo_local_navigation", anonymous=True)
    client = NavigationSocketClient(str(config.socket_path))
    lock = threading.Lock()
    inputs = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "odom_at": 0.0,
              "cloud_at": 0.0, "localization_at": 0.0, "localization_valid": False,
              "blocked": True, "frame_id": config.map_frame}
    startup_status = {}
    preview = []
    sensor_fault = threading.Event()
    health_gate = LocalizationHealthGate(frame_id=config.map_frame)
    health_pub = None
    if config.localization_health_mode == "odom_stable":
        health_pub = rospy.Publisher(config.localization_valid_topic, Bool, queue_size=1, latch=True)

    def publish_health(value):
        if health_pub is not None:
            health_pub.publish(Bool(data=bool(value)))
    def odom(msg):
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0 <= age <= 0.2 or not 0.9 <= norm <= 1.1 or msg.header.frame_id.lstrip("/") != config.map_frame.lstrip("/"):
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        now = time.monotonic()
        with lock:
            if time.monotonic() - inputs["odom_at"] < 0.5 and (math.hypot(x-inputs["x_m"], y-inputs["y_m"]) > 1 or abs(wrap(yaw-inputs["yaw_rad"])) > math.radians(75)):
                inputs["localization_valid"] = False
                inputs["odom_at"] = 0.0
                sensor_fault.set()  # a later Bool heartbeat cannot erase a jump
                publish_health(False)
                return
            inputs.update(x_m=x, y_m=y, yaw_rad=yaw, odom_at=now-age)
            if config.localization_health_mode == "odom_stable":
                healthy = health_gate.accept(now=now, stamp=now-age, x=x, y=y, yaw=yaw,
                                             frame_id=msg.header.frame_id)
                if config.localization_startup_status_topic:
                    healthy = healthy and startup_status_ready(
                        startup_status, now=time.time(), map_id=config.map_id,
                        map_version=config.localization_startup_map_version,
                        manifest_sha256=config.localization_startup_manifest_sha256)
                inputs["localization_valid"] = healthy
                inputs["localization_at"] = now
                publish_health(healthy)
    def cloud(msg):
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if not 0 <= age <= 0.5 or msg.header.frame_id.lstrip("/") not in ("body", "base_link"):
            return  # no unverified map-frame -> body-frame substitution
        count, blocked, points = 0, False, []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point[:3]
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            count += 1
            blocked |= 0.3 <= x <= 1.0 and -0.3 <= y <= 0.3 and -0.5 <= z <= 0.8
            if count % 20 == 0 and len(points) < 3000:
                points.append([round(x, 3), round(y, 3)])
        with lock:
            inputs.update(blocked=blocked or count == 0, cloud_at=time.monotonic()-age)
            preview[:] = points
    def localization(msg):
        with lock:
            inputs.update(localization_valid=bool(msg.data) and not sensor_fault.is_set(), localization_at=time.monotonic())
    def startup_health(msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            value = {}
        with lock:
            startup_status.clear()
            if isinstance(value, dict):
                startup_status.update(value)
    if config.localization_startup_status_topic:
        rospy.Subscriber(config.localization_startup_status_topic, String, startup_health, queue_size=1)
    rospy.Subscriber(config.odom_topic, Odometry, odom, queue_size=1)
    if config.require_pointcloud_for_navigation:
        rospy.Subscriber(config.cloud_topic, PointCloud2, cloud, queue_size=1)
    if config.localization_health_mode == "external":
        rospy.Subscriber(config.localization_valid_topic, Bool, localization, queue_size=1)
    navigator = PointNavigator.from_config(config)
    worker = NavigationWorker(client, navigator)
    last_preview = 0.0
    try:
        while not rospy.is_shutdown():
            with lock:
                if config.localization_health_mode == "odom_stable":
                    health_gate.tick(now=time.monotonic())
                    startup_ok = (not config.localization_startup_status_topic or startup_status_ready(
                        startup_status, now=time.time(), map_id=config.map_id,
                        map_version=config.localization_startup_map_version,
                        manifest_sha256=config.localization_startup_manifest_sha256))
                    if not health_gate.valid or not startup_ok:
                        inputs["localization_valid"] = False
                        inputs["localization_at"] = 0.0
                        publish_health(False)
            with lock:
                snapshot, points = dict(inputs), list(preview)
            worker.tick(snapshot)
            if worker.last_error:
                rospy.logwarn_throttle(2.0, "local navigation waiting/recovery: " + worker.last_error)
            if time.monotonic() - last_preview > 0.25:
                temporary = config.directory / "preview.tmp"
                temporary.write_text(json.dumps({"at": time.monotonic(), "points": points}), encoding="utf-8")
                temporary.replace(config.directory / "preview.json")
                last_preview = time.monotonic()
            time.sleep(0.04)
    finally:
        # Daemon watchdog remains authoritative if this request cannot arrive.
        try:
            client.close()
            RobotClient(str(config.socket_path)).request("estop")
        except (OSError, RuntimeError, ValueError) as exc:
            rospy.logerr("adapter shutdown stop unconfirmed: %s", exc)


if __name__ == "__main__":
    main()
