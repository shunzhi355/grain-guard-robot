"""Point cloud collector ROS node for Livox Mid-360 LiDAR.

Subscribes to ``/livox/lidar`` (``sensor_msgs/PointCloud2``), maintains a
thread-safe ring buffer of recent frames, and provides methods to access
and persist point cloud data.

Typical usage::

    from grain_sampling_pointcloud.collector import PointCloudCollector

    collector = PointCloudCollector()
    rospy.spin_once()
    frame = collector.get_latest_frame()
    if frame is not None:
        collector.save_frame("/tmp/scan.pcd")
"""

from __future__ import annotations

import os
import struct
from collections import deque
from threading import Lock
from typing import Optional

import rospy
from sensor_msgs.msg import PointCloud2

__all__ = ["PointCloudCollector"]


class PointCloudCollector:
    """ROS node that collects Livox point cloud frames into a ring buffer.

    Parameters
    ----------
    node_name : str
        Name of the ROS node (default ``"point_cloud_collector"``).
    buffer_size : int
        Maximum number of frames kept in the ring buffer (default ``10``).
    topic : str
        Point cloud topic to subscribe to (default ``"/livox/lidar"``).
    """

    def __init__(
        self,
        node_name: str = "point_cloud_collector",
        buffer_size: int = 10,
        topic: str = "/livox/lidar",
    ) -> None:
        try:
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass  # already initialized (e.g. in test)

        self._node_name = node_name  # store for testability
        self._buffer: deque = deque(maxlen=buffer_size)
        self._lock = Lock()
        self._frame_count: int = 0

        self._subscription = rospy.Subscriber(
            topic,
            PointCloud2,
            self._lidar_callback,
        )

        rospy.loginfo(
            f"PointCloudCollector initialized — subscribing to {topic}"
        )

    # ── internal helpers ───────────────────────────────────────────────

    def _lidar_callback(self, msg: PointCloud2) -> None:
        """Store an incoming point cloud frame in the ring buffer."""
        with self._lock:
            self._buffer.append(msg)
            self._frame_count += 1
        rospy.loginfo(
            f"Point cloud frame received. Total frames: {self._frame_count}"
        )

    @staticmethod
    def _parse_points(msg: PointCloud2) -> list[tuple[float, float, float]]:
        """Extract ``(x, y, z)`` tuples from a ``PointCloud2`` message.

        Reads only the first three ``FLOAT32`` fields (x, y, z) from each
        point.  This is compatible with both the standard Livox PointCloud2
        format (PointXYZRTLT) and the custom format.
        """
        points: list[tuple[float, float, float]] = []
        step = msg.point_step
        data = bytes(msg.data)  # ensure contiguous copy
        # Locate offsets for x, y, z fields
        offsets: dict[str, int] = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z") and field.datatype == 7:  # 7 = FLOAT32
                offsets[field.name] = field.offset

        if len(offsets) < 3:
            # Fallback: assume contiguous xyz at offset 0
            for i in range(0, len(data), step):
                if i + 12 > len(data):
                    break
                x, y, z = struct.unpack_from("<fff", data, i)
                points.append((x, y, z))
            return points

        for i in range(0, len(data), step):
            if i + max(offsets.values()) + 4 > len(data):
                break
            try:
                x = struct.unpack_from("<f", data, i + offsets["x"])[0]
                y = struct.unpack_from("<f", data, i + offsets["y"])[0]
                z = struct.unpack_from("<f", data, i + offsets["z"])[0]
                points.append((x, y, z))
            except struct.error:
                continue
        return points

    # ── public API ─────────────────────────────────────────────────────

    def get_latest_frame(self) -> Optional[PointCloud2]:
        """Return the most recent point cloud frame, or ``None`` if empty.

        Returns
        -------
        Optional[PointCloud2]
            The latest frame, or ``None`` if the buffer is empty.
        """
        with self._lock:
            if not self._buffer:
                return None
            return self._buffer[-1]

    def get_frame_at(self, height: float) -> Optional[PointCloud2]:
        """Return the frame closest to the given robot height.

        This is a simplified implementation that returns the latest frame
        regardless of the *height* argument.  A future version may correlate
        frame timestamps with vertical position data.

        Parameters
        ----------
        height : float
            Robot height in metres (currently unused).

        Returns
        -------
        Optional[PointCloud2]
            The latest frame, or ``None`` if the buffer is empty.
        """
        _ = height  # reserved for future use
        return self.get_latest_frame()

    def save_frame(self, filepath: str) -> bool:
        """Save the latest frame as an ASCII PCD file.

        Parameters
        ----------
        filepath : str
            Destination path for the ``.pcd`` file.

        Returns
        -------
        bool
            ``True`` if a frame was saved, ``False`` if the buffer was empty.
        """
        with self._lock:
            if not self._buffer:
                rospy.logwarn("No frames in buffer to save")
                return False
            msg = self._buffer[-1]

        points = self._parse_points(msg)

        # Ensure parent directory exists
        parent = os.path.dirname(filepath)
        if parent:
            os.makedirs(parent, exist_ok=True)

        with open(filepath, "w", encoding="utf-8") as f:
            f.write("# .PCD v.7 - Point Cloud Data file\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        rospy.loginfo(f"Saved {len(points)} points to {filepath}")
        return True

    # ── properties (convenience) ───────────────────────────────────────

    @property
    def frame_count(self) -> int:
        """Total number of frames received since startup."""
        return self._frame_count

    @property
    def buffer_size(self) -> int:
        """Maximum capacity of the ring buffer."""
        return self._buffer.maxlen
