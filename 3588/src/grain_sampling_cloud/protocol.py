"""HTTP API data models for grain sampling robot cloud communication.

对齐甲方接口文档（原粮远程验收.xlsx Sheet2）。
覆盖 4 个设备接口 + 第 5 点地图参数上报（BoundaryMapRequest）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional

__all__ = [
    "TaskListRequest",
    "OrderInfo",
    "TaskListResponse",
    "AcceptTaskRequest",
    "AcceptTaskResponse",
    "StatusReportRequest",
    "StatusReportResponse",
    "DetectionResult",
    "RoomListRequest",
    "RoomInfo",
    "RoomListResponse",
    "WarehouseListRequest",
    "WarehouseInfo",
    "WarehouseListResponse",
    "MapUploadRequest",
    "MapUploadResponse",
    "BoundaryMapRequest",
    "ApiPath",
    "CloudErrorCode",
    "TaskStatus",
    "ReportStatus",
]


class ApiPath:
    """HTTP API path constants matching the 甲方 (client) specification."""

    TASK_LIST = "/task-list"
    ACCEPT_TASK = "/accept"
    STATUS_REPORT = "/report"
    WAREHOUSE_LIST = "/warehouse-list"
    # Legacy paths kept for backward-compatible tests
    ROOM_LIST = "/api/rooms"
    MAP_UPLOAD = "/api/maps"
    MAP_DATA_UPLOAD = "/map-upload"


class CloudErrorCode:
    """云端错误码 — 对齐甲方接口文档错误码表（1070700000 ~ 1070700012）。"""

    DEVICE_NOT_FOUND = 1070700000          # 设备不存在，请检查MAC地址
    DEVICE_NOT_MOBILE_PLATFORM = 1070700001  # 设备不是移动平台类型
    DEVICE_NOT_BOUND = 1070700002          # 设备未绑定粮库
    DEVICE_BUSY = 1070700003               # 设备正在验收中，无法承接新任务
    TASK_NOT_FOUND = 1070700004            # 任务不存在
    TASK_STATUS_INVALID = 1070700005       # 任务状态不允许该操作
    TASK_WRONG_LIANGKU = 1070700006        # 任务不属于设备绑定的粮库
    TASK_NO_MOBILE_NEEDED = 1070700007     # 该任务无需移动平台参与验收
    TASK_ALREADY_ACCEPTED = 1070700008     # 该任务已被其他移动平台承接
    TASK_NOT_ACCEPTED_BY_DEVICE = 1070700009  # 该任务不是由本设备承接的
    REPORT_STATUS_INVALID = 1070700010     # 上报状态无效（1-完成/2-放弃）
    REPORT_MISSING_RESULTS = 1070700011    # 任务完成时必须上传检测结果数据
    INDICATOR_INVALID = 1070700012         # 检测指标无效或不属于生化/理化类别


class TaskStatus:
    """任务状态 — 0-待验收 ~ 5-验收终止。"""

    PENDING_ACCEPT = 0   # 待验收
    ACCEPTING = 1        # 验收中
    ACCEPTED = 2         # 验收通过
    PENDING_REVIEW = 3   # 待复验
    REJECTED = 4         # 验收未通过
    TERMINATED = 5       # 验收终止


class ReportStatus:
    """任务状态上报状态码。"""

    COMPLETE = 1   # 完成
    ABANDON = 2    # 放弃


@dataclass
class TaskListRequest:
    """1. 任务列表请求 — 设备上传MAC，后台返回粮库及待承接任务。"""

    mac: str

    def to_dict(self) -> Dict[str, str]:
        return {"mac": self.mac}


@dataclass
class OrderInfo:
    """单个任务信息。

    字段来源：API v2 任务列表响应（grain-sampler-api-v2.md orders[]）。
    pinzhong_code/pinzhong 为 v2 新增品种字段（如 "XM"/"小麦"），
    带默认值以便本地任务构造时不传。
    """

    order_id: str
    aojian_id: int
    aojian: str
    depth_list: List[float]
    jiance: List[int]
    points: List[Dict[str, float]]
    pinzhong_code: str = ""
    pinzhong: str = ""


@dataclass
class TaskListResponse:
    """任务列表响应。"""

    liangku_id: int
    liangku: str
    orders: List[OrderInfo]


@dataclass
class AcceptTaskRequest:
    """2. 承接任务请求 — 设备上传MAC和任务ID。"""

    mac: str
    order_id: str


@dataclass
class AcceptTaskResponse:
    """承接任务响应。"""

    success: bool
    message: str = ""


@dataclass
class DetectionResult:
    """检测结果条目 — 对应任务状态上报的 jiance_results[].

    注意：depth 带默认值，Python dataclass 要求带默认字段在后，
    故 value 排在 depth 之前（均支持关键字传参）。
    """

    indicator_id: int
    value: float
    depth: Optional[float] = None


@dataclass
class StatusReportRequest:
    """3. 任务状态上报 — 完成/放弃，完成时必须附带检测结果。

    status: 1-完成、2-放弃（见 ReportStatus）。
    jiance_results: 检测结果列表（仅生化/理化指标）。
    """

    mac: str
    order_id: str
    status: int
    jiance_results: List[DetectionResult] = field(default_factory=list)


@dataclass
class StatusReportResponse:
    """状态上报响应。"""

    success: bool


@dataclass
class RoomListRequest:
    """4. 廒间列表请求 — 设备上传MAC，后台返回廒间列表。"""

    mac: str

    def to_dict(self) -> Dict[str, str]:
        return {"mac": self.mac}


@dataclass
class RoomInfo:
    """廒间信息。"""

    aojian_id: str
    aojian: str


@dataclass
class RoomListResponse:
    """廒间列表响应。

    aojian_list: 对齐新文档 warehouse-list 响应字段。
    rooms: 旧字段，保留以向后兼容。
    """

    liangku: str
    aojian_list: List[RoomInfo] = field(default_factory=list)
    rooms: List[RoomInfo] = field(default_factory=list)


# 别名（对齐新文档命名），与旧 RoomList* 共存
WarehouseListRequest = RoomListRequest
WarehouseListResponse = RoomListResponse
WarehouseInfo = RoomInfo


@dataclass
class MapUploadRequest:
    """5. 地图参数上报 — 设备上传切面轮廓坐标。"""

    mac: str
    aojian_id: str
    height: float
    points: List[Dict[str, float]]
    timestamp: str = ""

    def __post_init__(self) -> None:
        if not self.timestamp:
            self.timestamp = datetime.now(timezone.utc).isoformat()


@dataclass
class MapUploadResponse:
    """地图上传响应。"""

    success: bool


@dataclass
class BoundaryMapRequest:
    """5. 廒间切面地图上报 — 建图完成后上传廒间俯视边界多边形。"""

    mac: str
    aojian_id: str
    aojian: str
    boundary: List[Dict[str, float]] = field(default_factory=list)
    map_bounds: Dict[str, float] = field(
        default_factory=lambda: {"xmin": 0.0, "xmax": 0.0, "ymin": 0.0, "ymax": 0.0}
    )
    timestamp: str = ""

    def __post_init__(self) -> None:
        if not self.timestamp:
            self.timestamp = datetime.now(timezone.utc).isoformat()
