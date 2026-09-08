"""Unit tests for :mod:`grain_sampling_pointcloud.collector`.

All tests mock the ROS ``Node`` base class so that no actual ROS
infrastructure is required.
"""

from __future__ import annotations

import os
import struct
import tempfile
from unittest.mock import MagicMock, patch

import pytest

# ── Module-level guard: skip all tests if ROS is not available ──────
try:
    from grain_sampling_pointcloud.collector import PointCloudCollector
except (ImportError, ModuleNotFoundError):
    pytest.skip(
        "ROS (rospy) not available — skipping pointcloud collector tests",
        allow_module_level=True,
    )

# ═══════════════════════════════════════════════════════════════════════
# Helpers — mock PointCloud2 messages
# ═══════════════════════════════════════════════════════════════════════


def _make_mock_pointcloud2(points: list[tuple[float, float, float]]) -> MagicMock:
    """Create a mock ``sensor_msgs/PointCloud2`` with the given points.

    Each point carries only ``x``, ``y``, ``z`` (``FLOAT32`` each) so
    ``point_step == 12``.

    Parameters
    ----------
    points : list of (float, float, float)
        Point coordinates to encode.

    Returns
    -------
    MagicMock
        A mock object that passes :meth:`isinstance` checks against
        ``PointCloud2`` via ``spec``.
    """
    # Lazy import so tests can run without ROS installed.
    # Use a module-level fallback — if even ``sensor_msgs`` is missing,
    # pass no spec so that attribute access still works via ``MagicMock``.
    # 注意：test_integration 会向 sys.modules 注入 sensor_msgs MagicMock，
    # 此时 PointCloud2 本身是 MagicMock，spec=MagicMock 会抛 InvalidSpecError，
    # 因此检测到 MagicMock 时回退 spec=None。
    try:
        import sensor_msgs.msg  # type: ignore[import-untyped]
        spec = sensor_msgs.msg.PointCloud2
        if isinstance(spec, MagicMock):
            spec = None
    except (ImportError, ModuleNotFoundError):
        spec = None

    msg = MagicMock(spec=spec)
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = "livox_frame"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        MagicMock(name="x", datatype=7, offset=0, count=1),
        MagicMock(name="y", datatype=7, offset=4, count=1),
        MagicMock(name="z", datatype=7, offset=8, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = struct.pack(f"<{3 * len(points)}f", *[v for p in points for v in p])
    msg.is_dense = True
    return msg


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════


@pytest.fixture
def mock_ros():
    """Mock rospy methods so PointCloudCollector can be instantiated without
    a running ROS context.
    
    ROS1 (rospy) version — patches ``rospy.init_node``, ``rospy.Subscriber``,
    and ``rospy.loginfo`` instead of the ROS2 ``Node`` class."""
    patchers = [
        patch("rospy.init_node"),
        patch("rospy.Subscriber"),
        patch("rospy.loginfo"),
    ]
    for p in patchers:
        p.start()
    yield
    for p in patchers:
        p.stop()


@pytest.fixture
def collector(mock_ros):
    """Return a fresh :class:`PointCloudCollector` instance."""
    return PointCloudCollector()


def _push_frames(collector, count: int, *, start: int = 0):
    """Push *count* mock frames into the collector's buffer."""
    for i in range(start, start + count):
        pt = (float(i), float(i), float(i))
        msg = _make_mock_pointcloud2([pt])
        collector._lidar_callback(msg)


# ═══════════════════════════════════════════════════════════════════════
# Instantiation
# ═══════════════════════════════════════════════════════════════════════


class TestInstantiation:
    """Verify that the node is constructed with sane defaults."""

    def test_node_name_default(self, collector):
        """Default node name should be ``"point_cloud_collector"``."""
        assert collector._node_name == "point_cloud_collector"

    def test_buffer_maxlen(self, collector):
        """Default ring-buffer capacity should be 10."""
        assert collector._buffer.maxlen == 10

    def test_custom_buffer_size(self, mock_ros):
        """``buffer_size`` parameter should be honoured."""
        c = PointCloudCollector(buffer_size=5)
        assert c._buffer.maxlen == 5

    def test_custom_node_name(self, mock_ros):
        """``node_name`` parameter should be honoured."""
        c = PointCloudCollector(node_name="test_collector")
        assert c._node_name == "test_collector"

    def test_subscription_created(self, mock_ros):
        """A subscription to ``/livox/lidar`` should be created."""
        with patch(
            "rospy.Subscriber"
        ) as mock_sub:
            PointCloudCollector()
            mock_sub.assert_called_once()
            args, _ = mock_sub.call_args
            topic = args[0]  # ROS1: rospy.Subscriber(topic, msg_type, callback)
            assert topic == "/livox/lidar", f"Expected /livox/lidar, got {topic}"


# ═══════════════════════════════════════════════════════════════════════
# Ring buffer
# ═══════════════════════════════════════════════════════════════════════


class TestRingBuffer:
    """Ring-buffer behaviour (thread-safe deque with maxlen)."""

    def test_buffer_starts_empty(self, collector):
        """Buffer should contain no frames on construction."""
        assert len(collector._buffer) == 0

    def test_single_frame_stored(self, collector):
        """Pushing one frame should increase buffer size to 1."""
        _push_frames(collector, 1)
        assert len(collector._buffer) == 1

    def test_buffer_fills_to_max(self, collector):
        """Pushing 10 frames should fill the buffer without overflow."""
        _push_frames(collector, 10)
        assert len(collector._buffer) == 10

    def test_buffer_does_not_exceed_maxlen(self, collector):
        """Pushing 11 frames should discard the oldest (maxlen=10)."""
        _push_frames(collector, 11)
        assert len(collector._buffer) == 10

    def test_buffer_discards_oldest(self, collector):
        """After 11 pushes, the oldest frame (id=0) should be gone."""
        _push_frames(collector, 11)
        # The oldest frame currently in the buffer should be id=1
        oldest = collector._buffer[0]
        x = struct.unpack_from("<f", bytes(oldest.data), oldest.fields[0].offset)[0]
        assert x == 1.0, f"Expected oldest x=1.0, got x={x}"

    def test_thread_safety_lock_exists(self, collector):
        """The lock should be present and have acquire/release methods."""
        assert collector._lock is not None
        assert hasattr(collector._lock, "acquire")
        assert hasattr(collector._lock, "release")


# ═══════════════════════════════════════════════════════════════════════
# get_latest_frame
# ═══════════════════════════════════════════════════════════════════════


class TestGetLatestFrame:
    """``get_latest_frame()`` behaviour."""

    def test_returns_none_when_empty(self, collector):
        """No frames → ``None``."""
        assert collector.get_latest_frame() is None

    def test_returns_most_recent_frame(self, collector):
        """After multiple pushes, the latest frame should be returned."""
        _push_frames(collector, 3)
        latest = collector.get_latest_frame()
        assert latest is not None
        # The 3rd point has id=2
        x = struct.unpack_from("<f", bytes(latest.data), latest.fields[0].offset)[0]
        assert x == 2.0, f"Expected latest x=2.0, got x={x}"

    def test_returns_after_overflow(self, collector):
        """Even after buffer overflow, the newest frame should be accessible."""
        _push_frames(collector, 15)
        latest = collector.get_latest_frame()
        assert latest is not None
        x = struct.unpack_from("<f", bytes(latest.data), latest.fields[0].offset)[0]
        assert x == 14.0, f"Expected latest x=14.0, got x={x}"


# ═══════════════════════════════════════════════════════════════════════
# get_frame_at
# ═══════════════════════════════════════════════════════════════════════


class TestGetFrameAt:
    """``get_frame_at(height)`` — simplified return of latest frame."""

    def test_returns_none_when_empty(self, collector):
        """Empty buffer → ``None`` regardless of height."""
        assert collector.get_frame_at(1.5) is None

    def test_returns_latest_frame(self, collector):
        """Should return the latest frame (simplified implementation)."""
        _push_frames(collector, 5)
        frame = collector.get_frame_at(1.5)
        assert frame is not None
        x = struct.unpack_from("<f", bytes(frame.data), frame.fields[0].offset)[0]
        assert x == 4.0

    def test_height_param_ignored(self, collector):
        """Different height values should return the same latest frame."""
        _push_frames(collector, 3)
        a = collector.get_frame_at(0.0)
        b = collector.get_frame_at(10.0)
        assert a is b  # same object reference


# ═══════════════════════════════════════════════════════════════════════
# save_frame
# ═══════════════════════════════════════════════════════════════════════


class TestSaveFrame:
    """``save_frame(filepath)`` PCD persistence."""

    def test_save_returns_false_when_empty(self, collector):
        """No frames → ``False`` and no file created."""
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "empty.pcd")
            result = collector.save_frame(path)
            assert result is False
            assert not os.path.isfile(path)

    def test_save_writes_valid_pcd(self, collector):
        """A PCD file should be written with ASCII point data."""
        _push_frames(collector, 1)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test.pcd")
            result = collector.save_frame(path)
            assert result is True
            assert os.path.isfile(path)

            with open(path, "r") as f:
                content = f.read()

            # Validate PCD header
            assert content.startswith("# .PCD v.7")
            assert "FIELDS x y z" in content
            assert "WIDTH 1" in content
            assert "POINTS 1" in content
            # The single point pushed has id=0 → (0.0, 0.0, 0.0)
            assert "0.000000 0.000000 0.000000" in content

    def test_save_multiple_points(self, collector):
        """A PCD with 3 points should contain all coordinates."""
        pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
        msg = _make_mock_pointcloud2(pts)
        collector._lidar_callback(msg)

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "multi.pcd")
            collector.save_frame(path)

            with open(path, "r") as f:
                content = f.read()

            assert "WIDTH 3" in content
            assert "POINTS 3" in content
            assert "1.000000 2.000000 3.000000" in content
            assert "4.000000 5.000000 6.000000" in content
            assert "7.000000 8.000000 9.000000" in content

    def test_save_creates_parent_dirs(self, collector):
        """Parent directories should be created automatically."""
        _push_frames(collector, 1)
        with tempfile.TemporaryDirectory() as tmpdir:
            nested = os.path.join(tmpdir, "sub", "nested", "scan.pcd")
            result = collector.save_frame(nested)
            assert result is True
            assert os.path.isfile(nested)


# ═══════════════════════════════════════════════════════════════════════
# Frame count
# ═══════════════════════════════════════════════════════════════════════


class TestFrameCount:
    """``frame_count`` property and logging."""

    def test_frame_count_starts_at_zero(self, collector):
        """Initial count should be 0."""
        assert collector.frame_count == 0

    def test_frame_count_increments(self, collector):
        """Each frame pushed should increment the counter."""
        _push_frames(collector, 5)
        assert collector.frame_count == 5

    def test_frame_count_on_overflow(self, collector):
        """Counter should keep incrementing even after buffer overflow."""
        _push_frames(collector, 15)
        assert collector.frame_count == 15


# ═══════════════════════════════════════════════════════════════════════
# Thread safety
# ═══════════════════════════════════════════════════════════════════════


class TestThreadSafety:
    """Concurrent access to the ring buffer should not corrupt state."""

    def test_concurrent_push_and_read(self, collector):
        """Simulated concurrent access should not raise."""
        import threading

        errors: list[Exception] = []

        def pusher():
            try:
                for _ in range(50):
                    msg = _make_mock_pointcloud2([(1.0, 2.0, 3.0)])
                    collector._lidar_callback(msg)
            except Exception as e:
                errors.append(e)

        def reader():
            try:
                for _ in range(50):
                    collector.get_latest_frame()
            except Exception as e:
                errors.append(e)

        threads = [
            threading.Thread(target=pusher),
            threading.Thread(target=reader),
            threading.Thread(target=pusher),
            threading.Thread(target=reader),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert len(errors) == 0, f"Concurrent access errors: {errors}"
