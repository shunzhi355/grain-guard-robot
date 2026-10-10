"""Unit tests for :mod:`grain_sampling_pointcloud.uploader`.

Tests contour-only upload behaviour, payload type field, and backward
compatibility with ``contour_only=False``.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from grain_sampling_pointcloud.uploader import MapUploader


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════


@pytest.fixture
def mock_client() -> MagicMock:
    client = MagicMock()
    client.mac_address = "AA:BB:CC:DD:EE:FF"
    return client


@pytest.fixture
def uploader(mock_client: MagicMock) -> MapUploader:
    return MapUploader(mock_client)


@pytest.fixture
def sample_points() -> list[dict]:
    """100 points forming a 0.9 x 0.9 m square (full slice)."""
    return [{"x": i * 0.1, "y": j * 0.1} for i in range(10) for j in range(10)]


@pytest.fixture
def contour_points() -> list[dict]:
    """Simulated contour output — outer ring only, ~36 points."""
    pts: list[dict] = []
    for i in range(10):
        pts.append({"x": i * 0.1, "y": 0.0})
        pts.append({"x": i * 0.1, "y": 0.9})
    for j in range(1, 9):
        pts.append({"x": 0.0, "y": j * 0.1})
        pts.append({"x": 0.9, "y": j * 0.1})
    return pts


# ═══════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════


def _posted_payload(mock_client: MagicMock) -> dict:
    """Return the dict that was passed to ``mock_client.post()``."""
    return mock_client.post.call_args[0][1]


# ═══════════════════════════════════════════════════════════════════════
# Tests
# ═══════════════════════════════════════════════════════════════════════


class TestMapUploaderContour:
    """Contour extraction and payload type field."""

    def test_contour_type_in_payload(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
    ) -> None:
        """``contour_only=True`` adds ``type: 'contour'`` to the payload."""
        mock_client.post.return_value = {"success": True}
        uploader.upload(sample_points, 0.5, "WH-A01", contour_only=True)
        assert _posted_payload(mock_client)["type"] == "contour"

    def test_full_type_in_payload(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
    ) -> None:
        """``contour_only=False`` adds ``type: 'full'`` to the payload."""
        mock_client.post.return_value = {"success": True}
        uploader.upload(sample_points, 0.5, "WH-A01", contour_only=False)
        assert _posted_payload(mock_client)["type"] == "full"

    def test_contour_only_reduces_points(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
        contour_points: list[dict],
    ) -> None:
        """``contour_only=True`` calls ``extract_contour`` and uploads fewer pts."""
        mock_client.post.return_value = {"success": True}

        with patch(
            "grain_sampling_pointcloud.uploader.PointCloudSlicer.extract_contour",
            return_value=contour_points,
        ) as mock_extract:
            uploader.upload(sample_points, 0.5, "WH-A01", contour_only=True)

        mock_extract.assert_called_once_with(sample_points)
        payload = _posted_payload(mock_client)
        assert len(payload["points"]) == len(contour_points)
        assert len(payload["points"]) < len(sample_points)

    def test_contour_only_false_preserves_points(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
    ) -> None:
        """``contour_only=False`` sends original points without extraction."""
        mock_client.post.return_value = {"success": True}

        with patch(
            "grain_sampling_pointcloud.uploader.PointCloudSlicer.extract_contour",
        ) as mock_extract:
            uploader.upload(sample_points, 0.5, "WH-A01", contour_only=False)

        mock_extract.assert_not_called()
        payload = _posted_payload(mock_client)
        assert payload["points"] == sample_points
        assert payload["type"] == "full"

    def test_default_is_contour(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
        contour_points: list[dict],
    ) -> None:
        """Default behaviour (no ``contour_only`` arg) is ``contour_only=True``."""
        mock_client.post.return_value = {"success": True}

        with patch(
            "grain_sampling_pointcloud.uploader.PointCloudSlicer.extract_contour",
            return_value=contour_points,
        ) as mock_extract:
            uploader.upload(sample_points, 0.5, "WH-A01")

        mock_extract.assert_called_once()
        payload = _posted_payload(mock_client)
        assert payload["type"] == "contour"


class TestMapUploaderBackwardCompat:
    """Backward-compatibility of the :meth:`MapUploader.upload` signature."""

    def test_preserves_required_fields(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
    ) -> None:
        """Required fields are always present regardless of ``contour_only``."""
        mock_client.post.return_value = {"success": True}

        uploader.upload(sample_points, 0.5, "WH-A01", contour_only=False)
        payload = _posted_payload(mock_client)

        assert payload["mac"] == "AA:BB:CC:DD:EE:FF"
        assert payload["aojian_id"] == "WH-A01"
        assert payload["height"] == 0.5
        assert "points" in payload
        assert "type" in payload

    def test_upload_with_preview_delegates(
        self,
        uploader: MapUploader,
        mock_client: MagicMock,
        sample_points: list[dict],
    ) -> None:
        """``upload_with_preview`` delegates to ``upload`` (backward compat)."""
        mock_client.post.return_value = {"success": True}

        with patch.object(
            uploader, "upload", wraps=uploader.upload
        ) as mock_upload:
            uploader.upload_with_preview(sample_points, 0.5, "WH-A01", b"preview")
            mock_upload.assert_called_once_with(
                sample_points, 0.5, "WH-A01",
            )
