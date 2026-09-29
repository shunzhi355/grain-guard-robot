"""廒间切面地图数据组装器 — 由 PCD 点云构建廒间俯视边界多边形。

从 ASCII PCD 文件中提取指定高度层的水平切面，经 occupancy-grid 轮廓提取
后做纯平移归一化（不旋转），组装成 ``BoundaryMapRequest``，并提供上传
（:meth:`MapDataUploader.upload`）与上传失败时的本地原子兜底/补传
（:meth:`MapDataUploader.flush_pending`）能力。

Typical usage::

    from grain_sampling_pointcloud.map_data_uploader import MapDataUploader

    uploader = MapDataUploader()
    boundary, map_bounds = uploader.build_boundary("/path/to/map.pcd", height=1.0)
    request = uploader.build_request(
        "/path/to/map.pcd", height=1.0,
        mac="AA:BB:CC:DD:EE:FF", aojian_id="aojian-01", aojian="1号廒间",
    )
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from typing import Dict, List, Optional, Tuple

from grain_sampling_cloud.http_client import (
    CloudAuthError,
    CloudConnectionError,
    CloudHttpClient,
    CloudProtocolError,
)
from grain_sampling_cloud.protocol import ApiPath, BoundaryMapRequest
from grain_sampling_pointcloud.pcd_reader import read_pcd_points
from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer
from utils.config import AppConfig

logger = logging.getLogger(__name__)

__all__ = ["MapDataUploader"]


class MapDataUploader:
    """将 PCD 点云转换为廒间切面边界地图数据。

    读取 PCD → 按高度过滤出水平切面 → 投影到 XY 平面 → 提取顺时针轮廓 →
    平移归一化（min 角平移到原点，不做旋转）→ 组装 ``BoundaryMapRequest``。
    """

    def __init__(self) -> None:
        self._slicer = PointCloudSlicer()

    # ── public API ──────────────────────────────────────────────────────

    def build_boundary(
        self,
        pcd_path: str,
        height: float,
        tolerance: float = 0.1,
    ) -> Tuple[List[Dict[str, float]], Dict[str, float]]:
        """由 PCD 文件构建指定高度的廒间边界多边形。

        Parameters
        ----------
        pcd_path : str
            ASCII PCD 文件路径。
        height : float
            目标切面高度（Z 轴，米）。
        tolerance : float
            高度带半宽（米），Z 在 ``[height - tolerance, height + tolerance]``
            内的点被纳入切面。默认 ``0.1``。

        Returns
        -------
        (boundary, map_bounds) : tuple[list[dict], dict]
            ``boundary`` 为顺时针轮廓点 ``[{"x": ..., "y": ...}, ...]``，
            已平移归一化使 min 点约为 ``(0, 0)``，所有点 ``x >= 0``、``y >= 0``。
            ``map_bounds`` 为 ``{"xmin": 0, "xmax": ..., "ymin": 0, "ymax": ...}``。
            若该高度层无点（空切面），返回 ``([], {})``。
        """
        points = read_pcd_points(pcd_path)
        logger.debug("Loaded %d points from %s", len(points), pcd_path)

        z_min = height - tolerance
        z_max = height + tolerance
        projected = [
            {"x": p["x"], "y": p["y"]}
            for p in points
            if z_min <= p["z"] <= z_max
        ]
        if not projected:
            logger.warning(
                "No points found in height band [%.2f, %.2f] of %s",
                z_min, z_max, pcd_path,
            )
            return [], {}

        logger.info(
            "Slice at height %.2f contains %d points", height, len(projected)
        )
        contour = self._slicer.extract_contour(projected, grid_resolution=0.1)
        if not contour:
            logger.warning("Empty contour extracted at height %.2f", height)
            return [], {}

        # ── 纯平移归一化：min 角平移到原点，不做旋转 ──────────────────────
        x_offset = min(p["x"] for p in contour)
        y_offset = min(p["y"] for p in contour)
        boundary = [
            {"x": p["x"] - x_offset, "y": p["y"] - y_offset} for p in contour
        ]

        max_x = max(p["x"] for p in boundary)
        max_y = max(p["y"] for p in boundary)
        map_bounds: Dict[str, float] = {
            "xmin": 0.0,
            "xmax": max_x,
            "ymin": 0.0,
            "ymax": max_y,
        }

        logger.info(
            "Boundary built: %d contour points, map_bounds=%s",
            len(boundary), map_bounds,
        )
        return boundary, map_bounds

    def build_request(
        self,
        pcd_path: str,
        height: float,
        mac: str,
        aojian_id: str,
        aojian: str,
        tolerance: float = 0.1,
    ) -> Optional[BoundaryMapRequest]:
        """构建廒间切面地图上报请求。

        Parameters
        ----------
        pcd_path : str
            ASCII PCD 文件路径。
        height : float
            目标切面高度（Z 轴，米）。
        mac : str
            设备 MAC 地址。
        aojian_id : str
            廒间唯一标识。
        aojian : str
            廒间名称。
        tolerance : float
            高度带半宽（米），默认 ``0.1``。

        Returns
        -------
        BoundaryMapRequest | None
            组装好的请求对象；若空切面（无轮廓可提取）则返回 ``None``。
             ``timestamp`` 由 dataclass 自动填充。
        """
        boundary, map_bounds = self.build_boundary(pcd_path, height, tolerance)
        if not boundary:
            return None

        return BoundaryMapRequest(
            mac=mac,
            aojian_id=aojian_id,
            aojian=aojian,
            boundary=boundary,
            map_bounds=map_bounds,
        )

    # ── upload / 本地兜底 ─────────────────────────────────────────────

    def upload(
        self,
        request: BoundaryMapRequest,
        *,
        pending_dir: Optional[str] = None,
    ) -> bool:
        """上传廒间切面地图数据到云端，失败时本地原子兜底。

        复用现有工厂 ``CloudHttpClient.from_app_config(AppConfig())`` 创建
        client，向 ``ApiPath.MAP_DATA_UPLOAD`` POST ``request.__dict__``
        （``post`` 已解包 CommonResult 返回 ``data``）。上传成功后删除该
        request 对应的本地兜底文件（若存在）；上传失败/异常
        （``CloudConnectionError`` 等）时将请求原子写入 ``pending_dir``
        （``.tmp`` + ``os.replace``），供 :meth:`flush_pending` 事后补传。

        Parameters
        ----------
        request : BoundaryMapRequest
            待上传的廒间切面地图请求（由 :meth:`build_request` 组装）。
        pending_dir : str, optional
            本地兜底目录；默认
            ``~/grain_sampling_robot_software/.cache/map_upload_pending``。

        Returns
        -------
        bool
            ``True`` 上传成功；``False`` 上传失败（已写入本地兜底文件）。
        """
        if pending_dir is None:
            pending_dir = self._default_pending_dir()

        client = CloudHttpClient.from_app_config(AppConfig())
        try:
            resp = client.post(ApiPath.MAP_DATA_UPLOAD, request.__dict__)
            if isinstance(resp, dict):
                success = resp.get("success", False)
            else:
                success = bool(resp)
        except (CloudConnectionError, CloudAuthError, CloudProtocolError) as exc:
            logger.warning("Map upload failed: %s", exc)
            success = False

        if success:
            logger.info("Map upload succeeded: aojian=%s", request.aojian)
            pending_file = self._pending_file_path(request, pending_dir)
            if os.path.exists(pending_file):
                try:
                    os.remove(pending_file)
                except OSError as exc:
                    logger.warning(
                        "Failed to remove pending map data %s: %s",
                        pending_file, exc,
                    )
                else:
                    logger.info("Removed pending map data: %s", pending_file)
            return True

        # ── 本地原子兜底：.tmp 临时文件 + os.replace 原子移动 ─────────────
        try:
            os.makedirs(pending_dir, exist_ok=True)
        except OSError as exc:
            logger.error("Cannot create pending dir %s: %s", pending_dir, exc)
            return False

        pending_file = self._pending_file_path(request, pending_dir)
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                dir=pending_dir,
                suffix=".tmp",
                delete=False,
                encoding="utf-8",
            ) as fh:
                tmp_path = fh.name
                json.dump(request.__dict__, fh, ensure_ascii=False)
            os.replace(tmp_path, pending_file)
        except OSError as exc:
            logger.error("Failed to write pending map data %s: %s", pending_file, exc)
            try:
                os.unlink(tmp_path)
            except (OSError, NameError, UnboundLocalError):
                pass
            return False

        logger.warning("Map upload failed, saved to pending: %s", pending_file)
        return False

    def flush_pending(self, pending_dir: Optional[str] = None) -> int:
        """扫描兜底目录，逐个重试补传未上传成功的地图数据。

        仅处理 ``.json`` 文件（过滤目录与其他后缀），逐个反序列化为
        ``BoundaryMapRequest`` 后调用 :meth:`upload` 重试；上传成功即删除
        本地文件。触发时机由调用方决定，本方法不做定时调度、不实现重试
        次数/退避策略。

        Parameters
        ----------
        pending_dir : str, optional
            兜底目录；默认
            ``~/grain_sampling_robot_software/.cache/map_upload_pending``。

        Returns
        -------
        int
            成功补传的文件数量。
        """
        if pending_dir is None:
            pending_dir = self._default_pending_dir()

        if not os.path.isdir(pending_dir):
            logger.info("No pending map data directory: %s", pending_dir)
            return 0

        flushed = 0
        for name in os.listdir(pending_dir):
            if not name.endswith(".json"):
                continue
            path = os.path.join(pending_dir, name)
            if not os.path.isfile(path):
                continue

            try:
                with open(path, "r", encoding="utf-8") as fh:
                    raw = json.load(fh)
                request = BoundaryMapRequest(**raw)
            except (OSError, ValueError, TypeError, KeyError) as exc:
                logger.warning("Skipping invalid pending map data %s: %s", path, exc)
                continue

            if self.upload(request, pending_dir=pending_dir):
                # upload() 成功时已删除对应兜底文件，此处防重删。
                try:
                    os.remove(path)
                except OSError:
                    pass
                flushed += 1
                logger.info("Pending map data flushed: %s", path)
            else:
                logger.warning("Pending map data still failing, kept: %s", path)

        logger.info("Flushed %d pending map data file(s)", flushed)
        return flushed

    # ── 内部工具 ──────────────────────────────────────────────────────

    def _default_pending_dir(self) -> str:
        """板端默认兜底目录：``~/grain_sampling_robot_software/.cache/map_upload_pending``。"""
        return os.path.join(
            os.path.expanduser("~"),
            "grain_sampling_robot_software",
            ".cache",
            "map_upload_pending",
        )

    def _pending_file_path(
        self, request: BoundaryMapRequest, pending_dir: str
    ) -> str:
        """计算 request 对应的兜底文件名：``{aojian_id}_{timestamp}.json``。"""
        safe_ts = request.timestamp.replace(":", "-").replace("+", "-")
        return os.path.join(pending_dir, f"{request.aojian_id}_{safe_ts}.json")
