"""2D slice extraction from PointCloud2 for grain sampling robot mapping.

Extracts horizontal slices at a given height, projects them to the XY plane,
and computes occupancy-grid-based contours for map preview generation.

Typical usage::

    from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer

    slicer = PointCloudSlicer()
    points = slicer.extract_slice(msg, height=0.5, tolerance=0.1)
    contour = slicer.extract_contour(points, grid_resolution=0.05)
"""

from __future__ import annotations

import math
import struct
from typing import Any, Dict, List, Optional, Tuple


class PointCloudSlicer:
    """Extracts 2D XY slices and contours from sensor_msgs/PointCloud2.

    Parameters
    ----------
    default_tolerance : float
        Default height tolerance in metres used when ``tolerance`` is not
        supplied to ``extract_slice`` (default ``0.1``).
    """

    def __init__(self, default_tolerance: float = 0.1) -> None:
        self.default_tolerance = default_tolerance

    # ── public API ──────────────────────────────────────────────────────

    def extract_slice(
        self,
        pointcloud_msg: Any,
        height: float,
        tolerance: Optional[float] = None,
    ) -> List[Dict[str, float]]:
        """Extract all points within a horizontal band and project to XY.

        Parameters
        ----------
        pointcloud_msg : sensor_msgs.msg.PointCloud2
            The point cloud message (e.g. from ``collector.get_latest_frame()``).
        height : float
            Target height (Z axis) in metres.
        tolerance : float, optional
            Half-width of the Z band in metres.  Points with Z in
            ``[height - tolerance, height + tolerance]`` are included.
            Defaults to ``self.default_tolerance`` (0.1 m).

        Returns
        -------
        list[dict]
            Projected points as ``[{"x": 1.2, "y": 3.4}, ...]``.
            Empty list if the point cloud is empty or no points fall within
            the height band.
        """
        tol = tolerance if tolerance is not None else self.default_tolerance
        z_min = height - tol
        z_max = height + tol

        raw_points = self._parse_xyz(pointcloud_msg)
        if not raw_points:
            return []

        projected: List[Dict[str, float]] = []
        for x, y, z in raw_points:
            if z_min <= z <= z_max:
                projected.append({"x": x, "y": y})

        return projected

    def extract_contour(
        self,
        points: List[Dict[str, float]],
        grid_resolution: float = 0.1,
    ) -> List[Dict[str, float]]:
        """Compute the outermost boundary of a set of 2D points.

        Uses an occupancy-grid approach: points are rasterised onto a
        regular grid and boundary cells (filled cells with at least one
        empty neighbour) are returned as the contour.

        If fewer than 10 points are provided, all points are returned as-is.

        Parameters
        ----------
        points : list[dict]
            List of ``{"x": ..., "y": ...}`` dicts (output of ``extract_slice``).
        grid_resolution : float
            Cell size in metres (default ``0.1``).

        Returns
        -------
        list[dict]
            Boundary points as ``[{"x": ..., "y": ...}, ...]``, ordered
            clockwise around the centroid.
        """
        if len(points) < 10:
            return points

        # Determine bounding box from points
        xs = [p["x"] for p in points]
        ys = [p["y"] for p in points]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        if x_max == x_min:
            x_max = x_min + grid_resolution
        if y_max == y_min:
            y_max = y_min + grid_resolution

        # Build occupancy grid
        cols = max(2, int(math.ceil((x_max - x_min) / grid_resolution)) + 2)
        rows = max(2, int(math.ceil((y_max - y_min) / grid_resolution)) + 2)
        grid = [[False] * cols for _ in range(rows)]

        for p in points:
            col = int((p["x"] - x_min) / grid_resolution)
            row = int((p["y"] - y_min) / grid_resolution)
            if 0 <= col < cols and 0 <= row < rows:
                grid[row][col] = True

        # Find boundary cells (filled cells with ≥1 empty 4-neighbour)
        boundary: List[Tuple[int, int]] = []
        for r in range(rows):
            for c in range(cols):
                if not grid[r][c]:
                    continue
                is_boundary = False
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = r + dr, c + dc
                    if not (0 <= nr < rows and 0 <= nc < cols) or not grid[nr][nc]:
                        is_boundary = True
                        break
                if is_boundary:
                    boundary.append((c, r))

        if not boundary:
            return points

        # Convert grid coords back to world coords (cell centre)
        cx = x_min + sum(c for c, _r in boundary) / len(boundary) * grid_resolution + grid_resolution / 2
        cy = y_min + sum(r for _c, r in boundary) / len(boundary) * grid_resolution + grid_resolution / 2

        # Order cells clockwise by angle from centroid
        def _angle(cell: Tuple[int, int]) -> float:
            col, row = cell
            wx = x_min + (col + 0.5) * grid_resolution
            wy = y_min + (row + 0.5) * grid_resolution
            # math.atan2(y, x) gives angle from +x axis:
            #   +y →  π/2,  -y → -π/2,  +x → 0,  -x → ±π
            # Clockwise ordering: sort descending (π → -π)
            return -math.atan2(wy - cy, wx - cx)

        boundary.sort(key=_angle)

        return [
            {
                "x": x_min + (col + 0.5) * grid_resolution,
                "y": y_min + (row + 0.5) * grid_resolution,
            }
            for col, row in boundary
        ]

    # ── internal helpers ────────────────────────────────────────────────

    @staticmethod
    def _parse_xyz(msg: Any) -> List[Tuple[float, float, float]]:
        """Extract ``(x, y, z)`` tuples from a ``PointCloud2`` message.

        Reads only the first three ``FLOAT32`` fields (x, y, z) from each
        point.  This mirrors ``PointCloudCollector._parse_points``.
        """
        points: List[Tuple[float, float, float]] = []
        if msg is None:
            return points

        step = msg.point_step
        data = bytes(msg.data)

        offsets: Dict[str, int] = {}
        for field in msg.fields:
            if field.name in ("x", "y", "z") and field.datatype == 7:  # FLOAT32
                offsets[field.name] = field.offset

        if len(offsets) < 3:
            # Fallback: contiguous xyz at offset 0
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
