"""
Grain Sampling — Cloud Communication Middleware

Provides HTTP-based cloud communication for the grain sampling robot
with dual-channel (5G primary + WiFi fallback) support, a typed message
protocol, and a local persistent cache for offline resilience.
"""

from .http_client import CloudHttpClient, CloudConnectionError, CloudAuthError, CloudProtocolError, detect_mac
from .protocol import (
    TaskListRequest, TaskListResponse, OrderInfo,
    AcceptTaskRequest, AcceptTaskResponse,
    StatusReportRequest, StatusReportResponse, DetectionResult,
    RoomListRequest, RoomListResponse, RoomInfo,
    WarehouseListRequest, WarehouseListResponse, WarehouseInfo,
    MapUploadRequest, MapUploadResponse,
    ApiPath, CloudErrorCode, TaskStatus, ReportStatus,
)
from .dual_channel import DualChannelManager, Channel, ChannelSwitchReason, ChannelSwitchEvent
from .local_cache import LocalCache

__all__ = [
    "CloudHttpClient", "CloudConnectionError", "CloudAuthError", "CloudProtocolError", "detect_mac",
    "TaskListRequest", "TaskListResponse", "OrderInfo",
    "AcceptTaskRequest", "AcceptTaskResponse",
    "StatusReportRequest", "StatusReportResponse", "DetectionResult",
    "RoomListRequest", "RoomListResponse", "RoomInfo",
    "WarehouseListRequest", "WarehouseListResponse", "WarehouseInfo",
    "MapUploadRequest", "MapUploadResponse",
    "ApiPath", "CloudErrorCode", "TaskStatus", "ReportStatus",
    "DualChannelManager",
    "Channel",
    "ChannelSwitchReason",
    "ChannelSwitchEvent",
    "LocalCache",
]
