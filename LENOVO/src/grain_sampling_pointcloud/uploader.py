"""
Cloud uploader for 2D slice contour maps via HTTP.

Serialises extracted contour points to JSON and sends
them to the cloud HTTP API.
"""

from __future__ import annotations

import json
import logging
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from grain_sampling_cloud.http_client import CloudHttpClient, CloudConnectionError
from grain_sampling_cloud.protocol import ApiPath, MapUploadRequest
from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer

logger = logging.getLogger(__name__)


class MapUploader:
    """Uploads 2D contour map data to the cloud via HTTP POST.

    Parameters
    ----------
    http_client : CloudHttpClient
        An initialised ``CloudHttpClient`` instance.
    """

    def __init__(self, http_client: CloudHttpClient) -> None:
        self._client = http_client

    def upload(
        self,
        points: List[Dict[str, float]],
        height: float,
        aojian_id: str,
        contour_only: bool = True,
    ) -> bool:
        """Serialise and POST contour map data to the cloud.

        Parameters
        ----------
        points : list[dict]
            Slice points as ``[{"x": ..., "y": ...}, ...]``.
        height : float
            Slice height in metres.
        aojian_id : str
            间 (room) identifier.
        contour_only : bool
            If ``True`` (default), extract the outer boundary contour from the
            full slice points before uploading.  The uploaded payload carries
            ``"type": "contour"``.  Set to ``False`` to upload the raw points
            as-is with ``"type": "full"``.

        Returns
        -------
        bool
            ``True`` if the upload succeeded, ``False`` otherwise.
        """
        slicer = PointCloudSlicer()
        if contour_only:
            points = slicer.extract_contour(points)
            map_type = "contour"
        else:
            map_type = "full"

        request = MapUploadRequest(
            mac=self._client.mac_address,
            aojian_id=aojian_id,
            height=height,
            points=points,
        )

        payload = request.__dict__
        payload["type"] = map_type

        try:
            # T2: post() unwraps the CommonResult envelope and returns the
            # ``data`` payload directly (a bool from the mock, or a dict on
            # success in a future backend).  Success is no longer signalled
            # by an envelope-level ``success`` field.
            resp = self._client.post(ApiPath.MAP_UPLOAD, payload)
            if isinstance(resp, dict):
                success = resp.get("success", False)
            else:
                success = bool(resp)
            if success:
                logger.info(
                    "Map upload succeeded: aojian=%s, height=%.2f, %d points",
                    aojian_id, height, len(points),
                )
            else:
                logger.warning("Map upload returned falsy data: %s", resp)
            return success
        except CloudConnectionError as exc:
            logger.error("Map upload failed: %s", exc)
            return False

    def upload_with_preview(
        self,
        points: List[Dict[str, float]],
        height: float,
        aojian_id: str,
        preview_bytes: bytes,
    ) -> bool:
        """Upload map data (preview is local-only, not uploaded to cloud).

        The preview image is stored locally and NOT sent to the HTTP API
        (the cloud only receives contour coordinate data).

        This method exists for backward compatibility with the old MQTT API.
        """
        # Preview is local-only in the new HTTP architecture
        return self.upload(points, height, aojian_id)
