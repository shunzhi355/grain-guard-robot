"""Unit tests for :mod:`grain_sampling_pointcloud.map_data_uploader`,
:mod:`~.pcd_reader`, and :mod:`grain_sampling_cloud.protocol.BoundaryMapRequest`.

Covers ASCII PCD parsing, height-slice boundary building, upload with local
atomic fallback, pending flush, and request assembly.  All HTTP interactions
are mocked — no real network or ROS runtime required.
"""

from __future__ import annotations

import json
import os
import tempfile
from unittest.mock import MagicMock, patch

import pytest

from grain_sampling_cloud.http_client import CloudConnectionError, CloudHttpClient
from grain_sampling_cloud.protocol import BoundaryMapRequest
from grain_sampling_pointcloud.map_data_uploader import MapDataUploader
from grain_sampling_pointcloud.pcd_reader import read_pcd_points


# ═══════════════════════════════════════════════════════════════════════
# Helpers & shared fixtures
# ═══════════════════════════════════════════════════════════════════════

# 4 corners of a 2m×2m square at z=1.0 (for boundary-building tests)
SQUARE_CORNERS = [
    (1.0, 1.0, 1.0),
    (3.0, 1.0, 1.0),
    (3.0, 3.0, 1.0),
    (1.0, 3.0, 1.0),
]


def _ascii_pcd(points: list[tuple[float, float, float]]) -> str:
    """Build an ASCII PCD file body from ``(x, y, z)`` tuples."""
    lines = [
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    lines += [f"{x} {y} {z}" for x, y, z in points]
    return "\n".join(lines) + "\n"


def _make_request(**overrides) -> BoundaryMapRequest:
    """Build a valid ``BoundaryMapRequest`` with sensible defaults."""
    fields = {
        "mac": "AA:BB:CC:DD:EE:FF",
        "aojian_id": "aojian-01",
        "aojian": "1号廒间",
        "boundary": [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}],
        "map_bounds": {"xmin": 0.0, "xmax": 2.0, "ymin": 0.0, "ymax": 2.0},
    }
    fields.update(overrides)
    return BoundaryMapRequest(**fields)


@pytest.fixture
def pcd_file():
    """Factory fixture: write PCD text content to a temp file, return its path."""
    created: list[str] = []

    def _write(content: str) -> str:
        fh = tempfile.NamedTemporaryFile(
            mode="w", suffix=".pcd", delete=False, encoding="utf-8"
        )
        fh.write(content)
        fh.close()
        created.append(fh.name)
        return fh.name

    yield _write
    for path in created:
        try:
            os.unlink(path)
        except OSError:
            pass


# ═══════════════════════════════════════════════════════════════════════
# Tests — read_pcd_points
# ═══════════════════════════════════════════════════════════════════════

class TestReadPcd:
    """Tests for ``read_pcd_points`` (ASCII PCD parsing)."""

    def test_read_ascii_pcd(self, pcd_file):
        path = pcd_file(
            _ascii_pcd([(1.5, 2.5, 0.0), (-3.2, 4.1, 0.5), (0.0, -1.0, 2.0)])
        )
        points = read_pcd_points(path)
        assert len(points) == 3
        assert points[0] == {"x": 1.5, "y": 2.5, "z": 0.0}
        assert points[1] == {"x": -3.2, "y": 4.1, "z": 0.5}
        assert points[2] == {"x": 0.0, "y": -1.0, "z": 2.0}

    def test_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            read_pcd_points("does_not_exist_xyz_123.pcd")

    def test_binary_pcd(self, pcd_file):
        content = (
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "POINTS 1\n"
            "DATA binary\n"
            "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
        )
        path = pcd_file(content)
        points = read_pcd_points(path)
        assert len(points) == 1
        assert points[0]["x"] == 0.0
        assert points[0]["y"] == 0.0
        assert points[0]["z"] == 0.0

    def test_binary_compressed_error(self, pcd_file):
        content = (
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "POINTS 1\n"
            "DATA binary_compressed\n"
        )
        path = pcd_file(content)
        with pytest.raises(ValueError, match="仅支持 ASCII 或 binary"):
            read_pcd_points(path)

    def test_missing_xyz_fields(self, pcd_file):
        content = (
            "VERSION 0.7\n"
            "FIELDS intensity rgb\n"
            "SIZE 4 4\n"
            "TYPE F F\n"
            "COUNT 1 1\n"
            "WIDTH 1\n"
            "HEIGHT 1\n"
            "POINTS 1\n"
            "DATA ascii\n"
            "12 3400\n"
        )
        path = pcd_file(content)
        with pytest.raises(ValueError, match="点云数据缺少坐标字段"):
            read_pcd_points(path)

    def test_empty_pcd(self, pcd_file):
        """Valid ASCII header with zero data rows yields an empty list."""
        content = (
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 0\n"
            "HEIGHT 1\n"
            "POINTS 0\n"
            "DATA ascii\n"
        )
        path = pcd_file(content)
        assert read_pcd_points(path) == []


# ═══════════════════════════════════════════════════════════════════════
# Tests — MapDataUploader.build_boundary
# ═══════════════════════════════════════════════════════════════════════

class TestBuildBoundary:
    """Tests for ``MapDataUploader.build_boundary``."""

    @pytest.fixture
    def uploader(self) -> MapDataUploader:
        return MapDataUploader()

    def test_normalize_to_origin(self, uploader, pcd_file):
        """Points in the target z-band are translated so min is (0, 0)."""
        pts = SQUARE_CORNERS + [(-5.0, 7.0, 2.0), (9.0, -4.0, 2.0)]
        path = pcd_file(_ascii_pcd(pts))
        boundary, _ = uploader.build_boundary(path, height=1.0)
        assert len(boundary) == 4  # z=2.0 noise filtered out
        assert all(p["x"] >= 0 for p in boundary)
        assert all(p["y"] >= 0 for p in boundary)
        assert min(p["x"] for p in boundary) == pytest.approx(0.0, abs=1e-9)
        assert min(p["y"] for p in boundary) == pytest.approx(0.0, abs=1e-9)

    def test_empty_slice(self, uploader, pcd_file):
        """No points in the height band -> ([], {})."""
        path = pcd_file(_ascii_pcd([(1.0, 1.0, 3.0), (2.0, 2.0, 3.0)]))
        boundary, map_bounds = uploader.build_boundary(path, height=1.0)
        assert boundary == []
        assert map_bounds == {}

    def test_map_bounds(self, uploader, pcd_file):
        path = pcd_file(_ascii_pcd(SQUARE_CORNERS))
        _, map_bounds = uploader.build_boundary(path, height=1.0)
        assert map_bounds == {
            "xmin": 0.0,
            "xmax": 2.0,
            "ymin": 0.0,
            "ymax": 2.0,
        }


# ═══════════════════════════════════════════════════════════════════════
# Tests — MapDataUploader.upload / flush_pending
# ═══════════════════════════════════════════════════════════════════════

class TestUploadFallback:
    """Tests for ``MapDataUploader.upload`` and ``flush_pending``."""

    @pytest.fixture
    def pending_dir(self):
        with tempfile.TemporaryDirectory() as tmp:
            yield tmp

    @pytest.fixture
    def mock_client(self):
        return MagicMock(spec=CloudHttpClient)

    @pytest.fixture
    def uploader(self, mock_client):
        with patch(
            "grain_sampling_pointcloud.map_data_uploader.CloudHttpClient"
            ".from_app_config",
            return_value=mock_client,
        ):
            yield MapDataUploader()

    def test_upload_success(self, uploader, mock_client, pending_dir):
        mock_client.post.return_value = True
        result = uploader.upload(_make_request(), pending_dir=pending_dir)
        assert result is True
        assert os.listdir(pending_dir) == []

    def test_upload_failure_saves_fallback(self, uploader, mock_client, pending_dir):
        mock_client.post.side_effect = CloudConnectionError("connection timeout")
        result = uploader.upload(_make_request(), pending_dir=pending_dir)
        assert result is False
        files = [f for f in os.listdir(pending_dir) if f.endswith(".json")]
        assert len(files) == 1
        with open(
            os.path.join(pending_dir, files[0]), "r", encoding="utf-8"
        ) as fh:
            raw = json.load(fh)
        assert "boundary" in raw
        assert raw["mac"] == "AA:BB:CC:DD:EE:FF"
        assert raw["aojian"] == "1号廒间"

    def test_flush_pending_cleans(self, uploader, mock_client, pending_dir):
        req = _make_request()
        safe_ts = req.timestamp.replace(":", "-").replace("+", "-")
        path = os.path.join(pending_dir, f"{req.aojian_id}_{safe_ts}.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(req.__dict__, fh)
        mock_client.post.return_value = True
        flushed = uploader.flush_pending(pending_dir=pending_dir)
        assert flushed == 1
        assert not os.path.exists(path)


# ═══════════════════════════════════════════════════════════════════════
# Tests — MapDataUploader.build_request / BoundaryMapRequest
# ═══════════════════════════════════════════════════════════════════════

class TestBuildRequest:
    """Tests for ``MapDataUploader.build_request`` and ``BoundaryMapRequest``."""

    @pytest.fixture
    def uploader(self) -> MapDataUploader:
        return MapDataUploader()

    def test_request_fields(self, uploader, pcd_file):
        path = pcd_file(_ascii_pcd(SQUARE_CORNERS))
        req = uploader.build_request(
            path,
            height=1.0,
            mac="AA:BB:CC:DD:EE:FF",
            aojian_id="aojian-01",
            aojian="1号廒间",
        )
        assert req is not None
        assert req.mac == "AA:BB:CC:DD:EE:FF"
        assert req.aojian_id == "aojian-01"
        assert req.aojian == "1号廒间"
        assert len(req.boundary) > 0
        assert req.map_bounds["xmin"] == 0.0
        assert req.map_bounds["ymin"] == 0.0
        assert req.map_bounds["xmax"] > 0.0
        # timestamp auto-filled by __post_init__ as an ISO 8601 UTC string
        assert isinstance(req.timestamp, str)
        assert req.timestamp != ""
        assert "T" in req.timestamp
        assert req.timestamp.endswith("+00:00")

    def test_request_none_on_empty(self, uploader, pcd_file):
        """Empty slice (no contour) -> build_request returns None."""
        path = pcd_file(_ascii_pcd([(1.0, 1.0, 3.0), (2.0, 2.0, 3.0)]))
        req = uploader.build_request(
            path,
            height=1.0,
            mac="AA:BB:CC:DD:EE:FF",
            aojian_id="aojian-01",
            aojian="1号廒间",
        )
        assert req is None
