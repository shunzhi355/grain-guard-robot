"""ROS integration thread for the grain sampling robot UI.

Runs rospy in a background QThread, bridging ROS topics → PySide6 Signals.
Designed to degrade gracefully when rospy is not available (e.g. on a dev machine).
"""

from __future__ import annotations

import json
import logging
import os
from typing import Any, Optional

import numpy as np
from PySide2.QtCore import QThread, Signal, QObject
from PySide2.QtGui import QImage

logger = logging.getLogger(__name__)

#: 是否把 /cloud_registered_body 实时转成 2D 点云图(UI 地图页)。Orange Pi 上
#: 该转换 + Qt 重绘开销大, 且与同一进程内的遥控 busy-wait 线程争抢 GIL, 会让
#: UI/遥控卡顿。需要时设置 GRAIN_SAMPLING_UI_POINTCLOUD=1 重新开启。
_POINTCLOUD_RENDER_ENABLED = os.getenv(
    "GRAIN_SAMPLING_UI_POINTCLOUD", "1"
).strip().lower() not in ("0", "false", "no", "off")
if not _POINTCLOUD_RENDER_ENABLED:
    logger.info("点云渲染已禁用 (GRAIN_SAMPLING_UI_POINTCLOUD=0)")


# ── ROS import (optional) ─────────────────────────────────
try:
    import rospy  # type: ignore[import-untyped]
    from nav_msgs.msg import Odometry, OccupancyGrid  # type: ignore[import-untyped]
    from sensor_msgs.msg import PointCloud2  # type: ignore[import-untyped]
    from std_msgs.msg import String  # type: ignore[import-untyped]
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    logger.warning("rospy not available — ROS integration disabled")


# ── Thread worker object (moveToThread pattern) ────────────

class _ROSWorker(QObject):
    """Worker that owns rospy subscriptions and lives on the background thread."""

    # Signals emitted to the main thread
    odometry_updated = Signal(float, float, float)  # x, y, yaw
    mechanism_status_updated = Signal(dict)
    map_updated = Signal(dict)
    laser_map_updated = Signal(int)  # point count
    cloud_registered_updated = Signal(QImage)  # 2D top-down projection
    rc_mode_updated = Signal(str)  # manual/auto mirror
    connection_changed = Signal(bool)
    ROS_error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._running = False
        self._latest_laser_map: PointCloud2 | None = None

    def init_node(self) -> None:
        """Initialise rospy node and subscribe to ROS topics."""
        if not HAS_ROS:
            self.ROS_error.emit("rospy not installed — cannot create node")
            return

        try:
            # rospy.init_node handles both init and node creation
            # This worker is initialised inside QThread.run(), not Python's
            # main thread.  rospy signal handlers may only be installed from
            # the main thread and could make the UI node disappear or leave
            # the connection indicator stale.
            rospy.init_node("grain_sampling_ui", disable_signals=True)

            # ── Subscriptions ──────────────────────────────────
            # FastLIO odometry (primary — frame_id=camera_init)
            rospy.Subscriber(
                "/Odometry",
                Odometry,
                self._on_odometry,
            )
            # NavStack / mock odometry (fallback — frame_id=odom)
            rospy.Subscriber(
                "/odometry/filtered",
                Odometry,
                self._on_odometry,
            )
            rospy.Subscriber(
                "/mechanism/status",
                String,
                self._on_mechanism_status,
            )
            rospy.Subscriber(
                "/map",
                OccupancyGrid,
                self._on_map,
            )
            # S-FAST_LIO laser map (PointCloud2)
            rospy.Subscriber(
                "/Laser_map",
                PointCloud2,
                self._on_laser_map,
            )
            # Point cloud body frame for 2D map view
            rospy.Subscriber(
                "/cloud_registered_body",
                PointCloud2,
                self._on_cloud_registered,
            )
            # 独立 rc_node 发布的手动/自动模式, 供 UI 镜像显示(避免 UI 自建采样器)
            rospy.Subscriber(
                "/rc_mode",
                String,
                self._on_rc_mode,
            )

            self.connection_changed.emit(True)
            logger.info("ROS node 'grain_sampling_ui' created and subscribed")

        except Exception as exc:
            logger.exception("Failed to create ROS node")
            self.ROS_error.emit(f"Failed to create ROS node: {exc}")

    def spin_once(self) -> None:
        """Called periodically from the QThread run — rospy callbacks are
        processed on internal threads, so we just sleep briefly to yield."""
        if HAS_ROS and not rospy.is_shutdown() and self._running:
            try:
                rospy.sleep(0.01)
            except Exception:
                pass

    def stop(self) -> None:
        """Signal the spin loop to stop."""
        self._running = False

    # ── Callbacks ──────────────────────────────────────────

    def _on_odometry(self, msg: Odometry) -> None:
        """Handle /Odometry (FastLIO) and /odometry/filtered messages."""
        try:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            yaw = self._quat_to_yaw(pose.orientation)
            self.odometry_updated.emit(x, y, yaw)
        except Exception:
            logger.exception("Failed to parse odometry message")

    def _on_mechanism_status(self, msg: Any) -> None:
        """Handle /mechanism/status messages (JSON)."""
        try:
            data = json.loads(msg.data)
            self.mechanism_status_updated.emit(data)
        except (json.JSONDecodeError, AttributeError):
            logger.warning("Invalid mechanism/status message: %s", getattr(msg, "data", ""))

    def _on_rc_mode(self, msg: Any) -> None:
        """Handle /rc_mode (manual/auto) — mirror the standalone rc_node."""
        try:
            self.rc_mode_updated.emit(str(msg.data))
        except Exception:
            logger.exception("Failed to parse rc_mode message")

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Handle /map messages (OccupancyGrid)."""
        try:
            data = {
                "info": {
                    "resolution": msg.info.resolution,
                    "width": msg.info.width,
                    "height": msg.info.height,
                    "origin": {
                        "position": {
                            "x": msg.info.origin.position.x,
                            "y": msg.info.origin.position.y,
                        },
                        "orientation": {
                            "w": msg.info.origin.orientation.w,
                            "z": msg.info.origin.orientation.z,
                        },
                    },
                },
                "data": list(msg.data),
            }
            self.map_updated.emit(data)
        except Exception:
            logger.exception("Failed to parse /map message")

    def _on_laser_map(self, msg: PointCloud2) -> None:
        """Handle /Laser_map (PointCloud2) — emit point count and store msg."""
        self._latest_laser_map = msg
        try:
            self.laser_map_updated.emit(msg.width)
        except Exception:
            logger.exception("Failed to parse /Laser_map message")

    def get_latest_laser_map(self) -> PointCloud2 | None:
        """Return latest /Laser_map message for cross-section extraction."""
        return self._latest_laser_map

    def _on_cloud_registered(self, msg: PointCloud2) -> None:
        """Convert /cloud_registered_body to 2D top-down QImage."""
        if not _POINTCLOUD_RENDER_ENABLED:
            return  # 点云渲染已禁用(省 CPU); 需要时设 GRAIN_SAMPLING_UI_POINTCLOUD=1
        try:
            pts = list(rospy.msg.deserialize_messages(msg, PointCloud2))
            # Actually use numpy approach
            import struct
            w, h = 200, 200
            grid = np.zeros((h, w), dtype=np.uint8)
            # Parse PointCloud2 manually for speed
            data = msg.data
            offset_x = [f.offset for f in msg.fields if f.name == 'x'][0]
            offset_y = [f.offset for f in msg.fields if f.name == 'y'][0]
            if msg.point_step == 0:
                return
            count = min(len(data) // msg.point_step, 5000)
            xs = np.frombuffer(data, dtype=np.float32, count=count, offset=offset_x)
            ys = np.frombuffer(data, dtype=np.float32, count=count, offset=offset_y)
            # Map to 0..200 pixel range for area -10..10m
            xi = ((xs + 10.0) * 10).clip(0, 199).astype(int)
            yi = ((10.0 - ys) * 10).clip(0, 199).astype(int)
            np.add.at(grid, (yi, xi), 1)
            grid = np.clip(grid * 15, 0, 255).astype(np.uint8)
            img = QImage(grid.data, w, h, w, QImage.Format.Format_Grayscale8)
            self.cloud_registered_updated.emit(img.copy())
        except Exception:
            logger.exception("Failed to parse point cloud")

    @staticmethod
    def _quat_to_yaw(orientation: Any) -> float:
        """Convert quaternion to yaw angle."""
        import math
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


# ── QThread wrapper ────────────────────────────────────────

class ROSNodeThread(QThread):
    """Background thread that runs rospy and bridges to Qt signals.

    Usage:
        ros_thread = ROSNodeThread()
        ros_thread.odometry_updated.connect(self.on_odometry)
        ros_thread.start()
    """

    # Proxy signals — pass through from the worker so callers
    # can connect directly to the thread object.
    odometry_updated = Signal(float, float, float)
    mechanism_status_updated = Signal(dict)
    map_updated = Signal(dict)
    connection_changed = Signal(bool)
    ROS_error = Signal(str)
    cloud_registered_updated = Signal(QImage)
    rc_mode_updated = Signal(str)

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._worker: Optional[_ROSWorker] = None
        self._spinning = False
        self._init_worker()

    def _init_worker(self) -> None:
        """Create worker object (belongs to this QThread)."""
        self._worker = _ROSWorker()
        self._worker.moveToThread(self)

        # Wire worker signals → ROSNodeThread signals
        # pyright: ignore[reportAttributeAccessIssue]
        self._worker.odometry_updated.connect(self.odometry_updated)
        self._worker.mechanism_status_updated.connect(self.mechanism_status_updated)
        self._worker.map_updated.connect(self.map_updated)
        self._worker.connection_changed.connect(self.connection_changed)
        self._worker.ROS_error.connect(self.ROS_error)
        self._worker.cloud_registered_updated.connect(self.cloud_registered_updated)
        self._worker.rc_mode_updated.connect(self.rc_mode_updated)

    def run(self) -> None:
        """QThread run loop — keeps the thread alive for rospy callbacks."""
        self._spinning = True
        if self._worker:
            # Initialise the one worker created by __init__.  Recreating it
            # here used to overwrite the signal-connected instance and made
            # connection_changed(True) race with UI startup.
            self._worker.init_node()
            self._worker._running = True  # noqa: SLF001
            while self._spinning and self._worker._running:
                self._worker.spin_once()
                self.msleep(10)  # ~100 Hz

        logger.info("ROS thread spin loop exited")

    def stop(self) -> None:
        """Gracefully stop the ROS spin loop and thread."""
        self._spinning = False
        if self._worker:
            self._worker.stop()
        self.quit()
        if not self.wait(3000):
            logger.warning("ROS thread did not stop gracefully; terminating")
            self.terminate()
            self.wait(1000)
        logger.info("ROS thread stopped")

    def get_latest_laser_map(self):
        """Return latest /Laser_map PointCloud2 for cross-section extraction."""
        if self._worker:
            return self._worker.get_latest_laser_map()
        return None
