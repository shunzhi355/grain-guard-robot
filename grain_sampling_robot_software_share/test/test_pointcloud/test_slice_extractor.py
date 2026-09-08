"""Unit tests for :mod:`grain_sampling_pointcloud.slice_extractor`,
:mod:`~.preview_generator`, and :mod:`~.uploader`.

All tests use synthetic ``PointCloud2`` data — no ROS runtime required.
"""

from __future__ import annotations

import struct
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer


# ═══════════════════════════════════════════════════════════════════════
# Helpers — synthetic PointCloud2
# ═══════════════════════════════════════════════════════════════════════

def _make_pointcloud2(points: list[tuple[float, float, float]]) -> MagicMock:
    """Return a mock ``sensor_msgs/PointCloud2`` with the given ``(x,y,z)``.

    Points use ``FLOAT32`` fields (``point_step == 12``).  ``spec`` is
    only applied when ``sensor_msgs.msg.PointCloud2`` is a real class —
    ``test_integration`` injects a MagicMock into ``sys.modules``, and
    ``spec=MagicMock`` would raise ``InvalidSpecError``.
    """
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
def slicer() -> PointCloudSlicer:
    return PointCloudSlicer(default_tolerance=0.1)


@pytest.fixture
def flat_cloud() -> MagicMock:
    """10×10 grid at z=0.5, spaced 0.1 m."""
    pts: list[tuple[float, float, float]] = []
    for i in range(10):
        for j in range(10):
            pts.append((i * 0.1, j * 0.1, 0.5))
    return _make_pointcloud2(pts)


@pytest.fixture
def multi_layer_cloud() -> MagicMock:
    """Points at z=0.0, 0.5, 1.0."""
    pts: list[tuple[float, float, float]] = []
    for z in (0.0, 0.5, 1.0):
        for i in range(5):
            for j in range(5):
                pts.append((i * 0.2, j * 0.2, z))
    return _make_pointcloud2(pts)


@pytest.fixture
def square_points() -> list[dict]:
    """31 points forming a 1×1 m square perimeter."""
    pts = []
    # top
    for x in np.linspace(0, 1, 10):
        pts.append({"x": float(x), "y": 1.0})
    # right
    for y in np.linspace(1, 0, 10):
        pts.append({"x": 1.0, "y": float(y)})
    # bottom
    for x in np.linspace(1, 0, 10):
        pts.append({"x": float(x), "y": 0.0})
    # left
    for y in np.linspace(0, 1, 2):  # just corner
        pts.append({"x": 0.0, "y": float(y)})
    return pts


@pytest.fixture
def circle_points() -> list[dict]:
    """60 points forming a circle of radius 1.0 at origin."""
    angles = np.linspace(0, 2 * np.pi, 60)
    return [
        {"x": float(np.cos(a)), "y": float(np.sin(a))}
        for a in angles
    ]


# ═══════════════════════════════════════════════════════════════════════
# Tests — extract_slice
# ═══════════════════════════════════════════════════════════════════════

class TestExtractSlice:
    """Tests for ``PointCloudSlicer.extract_slice``."""

    def test_extract_slice_at_height_0_5(self, slicer, flat_cloud):
        """All points at z=0.5, extract at height=0.5 returns all."""
        result = slicer.extract_slice(flat_cloud, height=0.5)
        assert len(result) == 100  # 10×10 grid

    def test_extract_slice_with_tolerance(self, slicer, flat_cloud):
        """Points at z=0.5, extract at height=0.4 with tol=0.2 captures all."""
        result = slicer.extract_slice(flat_cloud, height=0.4, tolerance=0.2)
        assert len(result) == 100

    def test_extract_slice_miss(self, slicer, flat_cloud):
        """Points at z=0.5, extract at height=2.0 returns empty."""
        result = slicer.extract_slice(flat_cloud, height=2.0)
        assert result == []

    def test_multi_layer_extract_middle(self, slicer, multi_layer_cloud):
        """Extract only the z=0.5 layer from a multi-layer cloud."""
        result = slicer.extract_slice(multi_layer_cloud, height=0.5)
        assert len(result) == 25  # 5×5 at z=0.5

    def test_multi_layer_with_tight_tolerance(self, slicer, multi_layer_cloud):
        """Tight tolerance excludes adjacent layers."""
        result = slicer.extract_slice(
            multi_layer_cloud, height=0.5, tolerance=0.01
        )
        # Points at 0.5 ± 0.01 → only z=0.5
        assert len(result) == 25

    def test_empty_pointcloud(self, slicer):
        """Empty point cloud returns empty list."""
        result = slicer.extract_slice(_make_pointcloud2([]), height=0.5)
        assert result == []

    def test_pointcloud_is_none(self, slicer):
        """None point cloud returns empty list."""
        result = slicer.extract_slice(None, height=0.5)  # type: ignore[arg-type]
        assert result == []

    def test_projection_drops_z(self, slicer, flat_cloud):
        """Result dicts contain only 'x' and 'y' keys."""
        result = slicer.extract_slice(flat_cloud, height=0.5)
        for pt in result:
            assert set(pt.keys()) == {"x", "y"}
            assert "z" not in pt

    def test_default_tolerance_used(self, slicer, flat_cloud):
        """When tolerance is not passed, default_tolerance is used."""
        # default=0.1, points at z=0.5 → height=0.4 captures all
        result = slicer.extract_slice(flat_cloud, height=0.4)
        assert len(result) == 100


# ═══════════════════════════════════════════════════════════════════════
# Tests — extract_contour
# ═══════════════════════════════════════════════════════════════════════

class TestExtractContour:
    """Tests for ``PointCloudSlicer.extract_contour``."""

    def test_few_points_returned_as_is(self, slicer):
        """< 10 points → all returned unchanged."""
        pts = [{"x": float(i), "y": float(i)} for i in range(5)]
        result = slicer.extract_contour(pts, grid_resolution=0.1)
        assert len(result) == 5

    def test_square_contour(self, slicer, square_points):
        """Square perimeter → contour approximates the square."""
        result = slicer.extract_contour(square_points, grid_resolution=0.05)
        # Should have boundary points (not all input points)
        assert len(result) > 0
        # All returned points should be near the 0-1 range
        for pt in result:
            assert -0.1 <= pt["x"] <= 1.15
            assert -0.1 <= pt["y"] <= 1.15

    def test_circle_contour(self, slicer, circle_points):
        """Circle perimeter → contour approximates the circle."""
        result = slicer.extract_contour(circle_points, grid_resolution=0.1)
        assert len(result) > 0
        # Points on a circle of radius ~1.0
        for pt in result:
            r = (pt["x"] ** 2 + pt["y"] ** 2) ** 0.5
            assert 0.8 < r < 1.3

    def test_contour_points_contain_x_y(self, slicer, square_points):
        """Contour points are dicts with only 'x' and 'y'."""
        result = slicer.extract_contour(square_points, grid_resolution=0.05)
        for pt in result:
            assert isinstance(pt, dict)
            assert set(pt.keys()) == {"x", "y"}

    def test_contour_ordered_clockwise(self, slicer, circle_points):
        """Contour points are ordered clockwise around centroid."""
        result = slicer.extract_contour(circle_points, grid_resolution=0.1)
        if len(result) < 3:
            pytest.skip("Not enough contour points for ordering test")
        # Compute centroid
        cx = sum(p["x"] for p in result) / len(result)
        cy = sum(p["y"] for p in result) / len(result)
        # Angles from centroid should be strictly decreasing
        # (clockwise = decreasing angle from +x axis, where angle goes π→-π)
        prev = float("inf")
        import math
        for pt in result:
            angle = math.atan2(pt["y"] - cy, pt["x"] - cx)
            assert angle <= prev + 1e-9, f"Not clockwise at {pt}, angle={angle}, prev={prev}"
            prev = angle

    def test_single_point_contour(self, slicer):
        """A single point as input returns that point."""
        pts = [{"x": 1.0, "y": 2.0}]
        result = slicer.extract_contour(pts)
        assert result == pts

    def test_empty_points_contour(self, slicer):
        """Empty point list returns empty list."""
        result = slicer.extract_contour([])
        assert result == []


# ═══════════════════════════════════════════════════════════════════════
# Tests — PreviewGenerator
# ═══════════════════════════════════════════════════════════════════════

class TestPreviewGenerator:
    """Tests for ``PreviewGenerator.generate_preview``."""

    @pytest.fixture
    def gen(self):
        from grain_sampling_pointcloud.preview_generator import PreviewGenerator
        return PreviewGenerator(width=800, height=600)

    def test_generate_preview_returns_bytes(self, gen, square_points):
        """Preview generation returns PNG bytes."""
        result = gen.generate_preview(square_points, (0, 1, 0, 1))
        assert isinstance(result, bytes)
        assert len(result) > 0
        # PNG signature
        assert result[:8] == b"\x89PNG\r\n\x1a\n"

    def test_empty_points_placeholder(self, gen):
        """No contour → placeholder image."""
        result = gen.generate_preview([], (0, 1, 0, 1))
        assert isinstance(result, bytes)
        assert len(result) > 0
        assert result[:8] == b"\x89PNG\r\n\x1a\n"

    def test_single_point_bounds_equal(self, gen):
        """Zero-extent bounds handled without division by zero."""
        pts = [{"x": 0.0, "y": 0.0}]
        result = gen.generate_preview(pts, (0, 0, 0, 0))
        assert isinstance(result, bytes)
        assert result[:8] == b"\x89PNG\r\n\x1a\n"

    def test_large_bounds(self, gen, square_points):
        """Large coordinate range scales correctly."""
        result = gen.generate_preview(square_points, (-10, 10, -10, 10))
        assert isinstance(result, bytes)
        assert result[:8] == b"\x89PNG\r\n\x1a\n"


# ═══════════════════════════════════════════════════════════════════════
# Tests — MapUploader (HTTP)
# ═══════════════════════════════════════════════════════════════════════

class TestMapUploader:
    """Tests for ``MapUploader.upload``."""

    @pytest.fixture
    def mock_http(self):
        client = MagicMock()
        client.mac_address = "00:00:00:00:00:00"
        client.post.return_value = {"success": True}
        return client

    @pytest.fixture
    def uploader(self, mock_http):
        from grain_sampling_pointcloud.uploader import MapUploader
        return MapUploader(http_client=mock_http)

    @pytest.fixture
    def sample_points(self):
        return [{"x": 1.0, "y": 2.0}, {"x": 1.5, "y": 2.5}]

    def test_upload_returns_true(self, uploader, mock_http, sample_points):
        result = uploader.upload(sample_points, height=0.5, aojian_id="a0e1")
        assert result is True

    def test_upload_calls_post(self, uploader, mock_http, sample_points):
        uploader.upload(sample_points, height=0.5, aojian_id="a0e1")
        mock_http.post.assert_called_once()
        args, _ = mock_http.post.call_args
        assert "/api/maps" in args[0]

    def test_upload_failure_returns_false(self, uploader, mock_http, sample_points):
        mock_http.post.return_value = {"success": False}
        result = uploader.upload(sample_points, height=0.5, aojian_id="a0e1")
        assert result is False

    def test_upload_connection_error_returns_false(self, uploader, mock_http, sample_points):
        from grain_sampling_cloud.http_client import CloudConnectionError
        mock_http.post.side_effect = CloudConnectionError("Timeout")
        result = uploader.upload(sample_points, height=0.5, aojian_id="a0e1")
        assert result is False

    def test_upload_with_preview(self, uploader, mock_http, sample_points):
        # preview is local-only now, just delegates to upload
        result = uploader.upload_with_preview(
            sample_points, height=0.5, aojian_id="a0e1", preview_bytes=b"fake"
        )
        assert result is True
        mock_http.post.assert_called_once()

    def test_upload_with_preview_not_connected(
        self, uploader, mock_http, sample_points
    ):
        from grain_sampling_cloud.http_client import CloudConnectionError
        mock_http.post.side_effect = CloudConnectionError("Timeout")
        result = uploader.upload_with_preview(
            sample_points, height=0.5, aojian_id="a0e1", preview_bytes=b"fake"
        )
        assert result is False
