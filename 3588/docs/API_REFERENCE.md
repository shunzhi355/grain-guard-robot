# 粮食扦样机器人 API 参考 v0.1

> 适用人员：开发人员
> 更新日期：2026-07-15

---

## 目录

1. [grain_sampling_cloud — 云端通信](#1-grain_sampling_cloud--云端通信)
2. [grain_sampling_workflow — 扦样流程](#2-grain_sampling_workflow--扦样流程)
3. [grain_sampling_pointcloud — 点云处理](#3-grain_sampling_pointcloud--点云处理)
4. [grain_sampling_camera — 摄像头模块](#4-grain_sampling_camera--摄像头模块)
5. [grain_sampling_devices — 外设对接](#5-grain_sampling_devices--外设对接)
6. [grain_sampling_ui — 操作界面](#6-grain_sampling_ui--操作界面)
7. [utils — 工具模块](#7-utils--工具模块)

---

## 1. grain_sampling_cloud — 云端通信

MQTT 云端通信中间件，支持 5G+WiFi 双模通信、消息协议定义、本地缓存和自动重连。

### 1.1 MQTTClient

线程安全的 MQTT 客户端，支持 JSON 序列化和自动重连。

```python
from grain_sampling_cloud.mqtt_client import MQTTClient, ReceivedMessage

client = MQTTClient(config={
    "broker_url": "mqtt://cloud.example.com:1883",
    "client_id": "robot-001",
    "keepalive": 60,
    "reconnect_min_delay": 1,
    "reconnect_max_delay": 60,
})
client.connect()
client.publish("robot/001/status", {"state": "idle"})
client.subscribe("robot/001/cmd")
client.loop_forever()  # 阻塞运行
```

#### 构造函数

```python
MQTTClient(config: Optional[Dict[str, Any]] = None)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `config` | `dict` | `None` | 配置字典，支持的 key 见下表 |

配置项：

| Key | 默认值 | 说明 |
|-----|--------|------|
| `broker_url` | `"mqtt://cloud-test.client.com:1883"` | MQTT Broker 地址 |
| `client_id` | 自动生成 | 客户端唯一标识 |
| `keepalive` | `60` | 心跳间隔（秒） |
| `reconnect_min_delay` | `1` | 重连最小延迟（秒） |
| `reconnect_max_delay` | `60` | 重连最大延迟（秒） |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `connect` | `connect() -> bool` | 连接到 MQTT Broker，返回是否连接成功 |
| `disconnect` | `disconnect() -> None` | 断开连接 |
| `publish` | `publish(topic: str, payload: Dict[str, Any], qos: int = 1, retain: bool = False) -> None` | 发布消息（线程安全，经内部队列异步发送） |
| `subscribe` | `subscribe(topic: str, qos: int = 1) -> None` | 订阅主题 |
| `unsubscribe` | `unsubscribe(topic: str) -> None` | 取消订阅 |
| `loop_forever` | `loop_forever() -> None` | 阻塞运行事件循环 |
| `loop_start` | `loop_start() -> None` | 启动后台线程运行事件循环 |
| `loop_stop` | `loop_stop() -> None` | 停止后台事件循环 |
| `is_connected` | `is_connected() -> bool` | 检查是否已连接 |

#### 属性与信号

| 属性 | 类型 | 说明 |
|------|------|------|
| `on_connect` | `Callable[[], None]` | 连接成功回调 |
| `on_disconnect` | `Callable[[], None]` | 断开连接回调 |
| `on_message` | `Callable[[ReceivedMessage], None]` | 收到消息回调 |

#### 辅助类

```python
@dataclass
class OutgoingMessage:
    topic: str
    payload: Dict[str, Any]
    qos: int = 1
    retain: bool = False

@dataclass
class ReceivedMessage:
    topic: str
    payload: Dict[str, Any]
    timestamp: float
```

### 1.2 MessageProtocol（消息协议）

定义云端通信的消息类型和编解码方式。

```python
from grain_sampling_cloud.protocol import (
    MessageType,
    WorkOrderRequest,
    WorkOrderResponse,
    TaskStatusReport,
    make_topic,
)
```

#### 消息类型常量

```python
class MessageType:
    WORK_ORDER_REQUEST  = "work_order_request"    # 工单请求
    WORK_ORDER_RESPONSE = "work_order_response"   # 工单响应
    TASK_STATUS_REPORT  = "task_status_report"    # 任务状态上报
    MAP_UPLOAD          = "map_upload"            # 地图上传
```

#### 消息数据类

```python
@dataclass
class WorkOrderRequest:
    """工单请求"""
    warehouse_id: str

    def to_json(self) -> str: ...
    @classmethod
    def from_json(cls, data: str) -> "WorkOrderRequest": ...
```

```python
@dataclass
class WorkOrderResponse:
    """工单响应，包含待执行的工单列表"""
    orders: List[Dict[str, Any]]

    def to_json(self) -> str: ...
    @classmethod
    def from_json(cls, data: str) -> "WorkOrderResponse": ...
```

每个 order 字典包含：`order_id`（工单ID）、`warehouse`（仓库名）、`depth_list`（深度列表）、`points`（点位列表）、`status`（状态）。

```python
@dataclass
class TaskStatusReport:
    """任务状态上报"""
    order_id: str
    status: str    # "started" | "completed" | "error"

    def to_json(self) -> str: ...
    @classmethod
    def from_json(cls, data: str) -> "TaskStatusReport": ...
```

#### 工具函数

```python
def make_topic(device_id: str, msg_type: str) -> str:
    """构建 MQTT Topic: robot/{device_id}/{msg_type}"""
```

### 1.3 LocalCache

基于 SQLite 的本地持久化缓存，用于暂存发送失败的消息。

```python
from grain_sampling_cloud.local_cache import LocalCache

cache = LocalCache(db_path="/var/cache/robot/mqtt.db")
# 或使用默认路径：~/.grain_sampling/mqtt_cache.db
cache = LocalCache()
```

#### 构造函数

```python
LocalCache(db_path: Optional[str] = None)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `db_path` | `str` | SQLite 数据库文件路径，默认 `~/.grain_sampling/mqtt_cache.db` |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `store` | `store(topic: str, payload: Dict[str, Any], qos: int = 1) -> int` | 存储消息，返回行 ID |
| `get_all` | `get_all() -> List[Dict[str, Any]]` | 获取所有未发送的消息 |
| `remove` | `remove(row_id: int) -> None` | 删除指定消息（确认已发送后调用） |
| `clear` | `clear() -> None` | 清空所有缓存消息 |
| `count` | `count() -> int` | 获取缓存消息数量 |
| `flush` | `flush(mqtt_client: MQTTClient) -> int` | 通过 MQTT 客户端发送所有缓存消息，返回成功发送的数量 |

### 1.4 DualChannelManager

5G + WiFi 双通道管理器，自动检测网络状态并切换。

```python
from grain_sampling_cloud.dual_channel import DualChannelManager, Channel

dm = DualChannelManager(config={
    "primary": {"broker_url": "mqtt://5g-broker:1883"},
    "fallback": {"broker_url": "mqtt://wifi-broker:1883"},
})
dm.start()
dm.publish("robot/001/status", {"state": "online"})
print(dm.active_channel)  # Channel.PRIMARY 或 Channel.FALLBACK
```

#### 构造函数

```python
DualChannelManager(config: Optional[Dict[str, Any]] = None)
```

配置项：

| Key | 说明 |
|-----|------|
| `primary` | 主通道（5G）MQTT 客户端配置 |
| `fallback` | 备用通道（WiFi）MQTT 客户端配置 |
| `network_check.enabled` | 是否启用网络检测（默认 `True`） |
| `network_check.primary_ping` | 主通道 Ping 地址（默认 `"8.8.8.8"`） |
| `network_check.fallback_ping` | 备用通道 Ping 地址（默认 `"192.168.1.1"`） |
| `network_check.interval` | 检测间隔秒数（默认 `5.0`） |
| `network_check.timeout` | Ping 超时秒数（默认 `2`） |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `start` | `start() -> None` | 启动双通道监听 |
| `stop` | `stop() -> None` | 停止所有通道 |
| `publish` | `publish(topic: str, payload: Dict[str, Any], **kwargs) -> None` | 通过当前活跃通道发布消息 |
| `subscribe` | `subscribe(topic: str, **kwargs) -> None` | 在当前活跃通道订阅主题 |

#### 属性与枚举

```python
class Channel(Enum):
    PRIMARY = "primary"     # 5G 主通道
    FALLBACK = "fallback"   # WiFi 备用通道

class ChannelSwitchReason(Enum):
    PRIMARY_CONNECTED = "primary_connected"
    PRIMARY_DISCONNECTED = "primary_disconnected"
    PRIMARY_RECOVERED = "primary_recovered"
    FALLBACK_ACTIVATED = "fallback_activated"
    MANUAL_SWITCH = "manual_switch"

# 通道切换事件
@dataclass
class ChannelSwitchEvent:
    previous: Channel
    current: Channel
    reason: ChannelSwitchReason
    timestamp: float
```

---

## 2. grain_sampling_workflow — 扦样流程

15 步扦样流程状态机和 ROS 服务调用桥接。

### 2.1 SamplingStateMachine

15 步扦样作业流程状态机，支持暂停/继续/停止安全控制。

```python
from grain_sampling_workflow.state_machine import (
    SamplingStateMachine,
    SamplingState,
    SamplingAction,
)

sm = SamplingStateMachine()
print(sm.current)  # SamplingState.INIT
sm.dispatch(SamplingAction.CONFIRM_READY)
print(sm.current)  # SamplingState.RECORD_START
```

#### 构造函数

```python
SamplingStateMachine()
```

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `dispatch` | `dispatch(action: SamplingAction) -> bool` | 执行一个动作，触发状态转换。返回 `True` 表示成功 |
| `can` | `can(action: SamplingAction) -> bool` | 检查当前状态下是否允许执行该动作 |

#### 属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `current` | `SamplingState` | 当前状态 |
| `is_paused` | `bool` | 是否处于暂停状态 |
| `is_running` | `bool` | 是否正在运行（非终止状态） |
| `on_state_change` | `Callable[[SamplingState, SamplingState], None]` | 状态变化回调 |

#### 状态枚举

```python
class SamplingState(Enum):
    INIT = auto()               # 1: 开机初始化
    RECORD_START = auto()       # 2: 记录起始点
    NAVIGATE_TO_POINT = auto()  # 3: 自主导航
    ARRIVED_PROMPT = auto()     # 4: 到达提示（连接取样管）
    PRESS_AND_SUCTION = auto()  # 5: 下压+吸粮
    ADD_PIPE_PROMPT = auto()    # 6: 提示加管
    REPEAT_UNTIL_DEPTH = auto() # 7: 重复直到达到深度
    DISCHARGE_WASTE = auto()    # 8: 排出废粮
    FORMAL_SAMPLING = auto()    # 9: 正式采样
    CONVEY_1 = auto()           # 10: 输送约2分钟
    OPEN_BIN = auto()           # 11: 打开分仓口
    CONVEY_DONE = auto()        # 12: 输送完成
    NEXT_CHECK = auto()         # 13: 判断下一步
    ALL_DONE_PROMPT = auto()    # 14: 确认返航
    RETURN = auto()             # 15: 返回起始点
    STOPPED = auto()            # 终止：已停止
    COMPLETED = auto()          # 终止：已完成
```

#### 动作枚举

```python
class SamplingAction(Enum):
    # 用户动作
    CONFIRM_READY = auto()              # 已就绪
    CONFIRM_PIPE_ADDED = auto()         # 已加管
    CONFIRM_WASTE_DISCHARGED = auto()   # 废粮已排完
    CONFIRM_DONE = auto()               # 确认
    CONFIRM_RETURN = auto()             # 确认返航
    # 安全控制（仅在 FORMAL_SAMPLING 状态有效）
    PAUSE = auto()
    RESUME = auto()
    STOP = auto()
    # 系统完成事件
    SYSTEM_RECORD_COMPLETE = auto()
    SYSTEM_NAV_COMPLETE = auto()
    SYSTEM_PRESS_COMPLETE = auto()
    SYSTEM_DEPTH_NOT_REACHED = auto()
    SYSTEM_DEPTH_REACHED = auto()
    SYSTEM_SUCTION_COMPLETE = auto()
    SYSTEM_CONVEY_COMPLETE = auto()
    SYSTEM_BIN_OPENED = auto()
    SYSTEM_NEXT_DEPTH = auto()
    SYSTEM_NEXT_POINT = auto()
    SYSTEM_ALL_DONE = auto()
    SYSTEM_RETURN_COMPLETE = auto()
```

### 2.2 SamplingBridge

封装与机器人底盘/机构/导航组件的 ROS 服务通信。

```python
from grain_sampling_workflow.ros_bridge import SamplingBridge

bridge = SamplingBridge()

# 调用导航服务
success = bridge.call_navigate(x=12.5, y=-3.2)

# 调用机构服务
bridge.call_start_suction()
bridge.call_stop_suction()
bridge.call_pause_suction()
bridge.call_resume_suction()
```

#### 构造函数

```python
SamplingBridge(node: Optional[rospy.node.Node] = None)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `node` | `Node` | 现有的 ROS Node。若为 `None` 且 rospy 可用，则创建新节点。若 rospy 不可用，进入 stub 模式（所有调用返回 `True`）。 |

#### 方法

所有方法返回 `bool`（`True` 表示成功，`False` 表示失败/超时）。

| 方法 | 签名 | ROS 服务 |
|------|------|-----------|
| `call_navigate` | `call_navigate(x: float, y: float) -> bool` | `/mechanism/navigate` |
| `call_start_suction` | `call_start_suction() -> bool` | `/mechanism/start_suction` |
| `call_stop_suction` | `call_stop_suction() -> bool` | `/mechanism/stop_suction` |
| `call_pause_suction` | `call_pause_suction() -> bool` | `/mechanism/pause_suction` |
| `call_resume_suction` | `call_resume_suction() -> bool` | `/mechanism/resume_suction` |
| `call_add_pipe` | `call_add_pipe() -> bool` | `/mechanism/add_pipe` |
| `call_convey_belt` | `call_convey_belt() -> bool` | `/mechanism/convey_belt` |
| `call_open_bin` | `call_open_bin(bin_index: int) -> bool` | `/mechanism/open_bin` |
| `call_emergency_stop` | `call_emergency_stop() -> bool` | `/emergency_stop` |

---

## 3. grain_sampling_pointcloud — 点云处理

Livox Mid-360 点云数据采集、2D 切面提取、预览图生成和云端上传。

### 3.1 PointCloudCollector

ROS 节点，订阅 `/livox/lidar` 点云话题，维护环形缓冲区。

```python
from grain_sampling_pointcloud.collector import PointCloudCollector
import rospy

rospy.init()
node = PointCloudCollector(node_name="my_collector", buffer_size=20)
rospy.spin_once(node)
frame = node.get_latest_frame()
if frame:
    points = node.get_points(frame)  # [(x, y, z), ...]
rospy.shutdown()
```

#### 构造函数

```python
PointCloudCollector(
    node_name: str = "point_cloud_collector",
    buffer_size: int = 10,
    topic: str = "/livox/lidar",
)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `node_name` | `str` | `"point_cloud_collector"` | ROS 节点名称 |
| `buffer_size` | `int` | `10` | 环形缓冲区帧数上限 |
| `topic` | `str` | `"/livox/lidar"` | 点云话题名称 |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `get_latest_frame` | `get_latest_frame() -> Optional[PointCloud2]` | 获取最新一帧点云消息 |
| `get_frame_count` | `get_frame_count() -> int` | 返回已接收帧总数 |
| `get_points` | `get_points(msg: PointCloud2) -> List[Tuple[float, float, float]]` | 从 PointCloud2 消息中提取 `(x, y, z)` 坐标列表 |
| `save_frame` | `save_frame(filepath: str, msg: Optional[PointCloud2] = None) -> bool` | 保存一帧为 PCD 文件 |

### 3.2 PointCloudSlicer

从点云中提取指定高度的 2D XY 切面，并计算轮廓。

```python
from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer

slicer = PointCloudSlicer(default_tolerance=0.1)

# 提取切面
points = slicer.extract_slice(pointcloud_msg, height=0.5, tolerance=0.1)
# 结果: [{"x": 1.2, "y": 3.4}, ...]

# 计算轮廓
contour = slicer.extract_contour(points, grid_resolution=0.05)
```

#### 构造函数

```python
PointCloudSlicer(default_tolerance: float = 0.1)
```

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `extract_slice` | `extract_slice(pointcloud_msg: Any, height: float, tolerance: Optional[float] = None) -> List[Dict[str, float]]` | 提取指定高度范围内的点并投影到 XY 平面 |
| `extract_contour` | `extract_contour(points: List[Dict[str, float]], grid_resolution: float = 0.05) -> List[Dict[str, float]]` | 基于占有率网格计算轮廓点 |

### 3.3 PreviewGenerator

将 2D 轮廓点渲染为 PNG 预览图，供操作员在 UI 中预览。

```python
from grain_sampling_pointcloud.preview_generator import PreviewGenerator

gen = PreviewGenerator(width=800, height=600)
png_bytes = gen.generate_preview(
    contour_points=contour,
    map_bounds=(xmin, xmax, ymin, ymax),
)
# 保存到文件
with open("/tmp/map_preview.png", "wb") as f:
    f.write(png_bytes)
```

#### 构造函数

```python
PreviewGenerator(
    width: int = 800,
    height: int = 600,
    background: str = "#0D1117",
    margin: int = 60,
)
```

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `generate_preview` | `generate_preview(contour_points: List[Dict[str, float]], map_bounds: Tuple[float, float, float, float]) -> bytes` | 渲染 PNG 预览图，返回图像字节数据 |

#### 返回值

返回 PNG 格式的 `bytes`，可直接写入文件或嵌入 UI。图像为深色主题，包含：
- 白色轮廓点
- 灰色网格线
- 蓝色比例尺
- 坐标轴标注

### 3.4 MapUploader

将 2D 地图数据和预览图通过 MQTT 上传到云端。

```python
from grain_sampling_cloud.mqtt_client import MQTTClient
from grain_sampling_pointcloud.uploader import MapUploader

mqtt = MQTTClient()
mqtt.connect()

uploader = MapUploader(mqtt, device_id="robot-001")

# 仅上传地图数据
ok = uploader.upload(points, height=0.5, map_name="warehouse_A")

# 上传地图 + 预览图
ok = uploader.upload_with_preview(
    points, height=0.5, map_name="warehouse_A",
    preview_bytes=png_bytes,
)
```

#### 构造函数

```python
MapUploader(mqtt_client: MQTTClient, device_id: str)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `mqtt_client` | `MQTTClient` | 已连接的 MQTT 客户端实例 |
| `device_id` | `str` | 机器人设备唯一标识 |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `upload` | `upload(points: List[Dict[str, float]], height: float, map_name: str) -> bool` | 上传地图数据（点 + 元数据），返回是否进入发送队列 |
| `upload_with_preview` | `upload_with_preview(points: List[Dict[str, float]], height: float, map_name: str, preview_bytes: bytes) -> bool` | 上传地图数据并附带 Base64 编码的 PNG 预览图 |

---

## 4. grain_sampling_camera — 摄像头模块

USB 摄像头 ROS 节点、RTSP 视频推流和云端流集成。

### 4.1 CameraNode

ROS 节点，从 USB 摄像头采集帧并发布 `sensor_msgs/Image` 消息。

```python
from grain_sampling_camera.camera_node import CameraNode
import rospy

rospy.init()
node = CameraNode(
    node_name="my_camera",
    device="/dev/video0",
    resolution=(640, 480),
    fps=30,
)
rospy.spin(node)
```

#### 构造函数

```python
CameraNode(
    node_name: str = "camera_node",
    device: Union[int, str] = 0,
    resolution: Tuple[int, int] = (1280, 720),
    fps: int = 15,
    image_topic: str = "/camera/image_raw",
    start_service: str = "/camera/start_stream",
    stop_service: str = "/camera/stop_stream",
)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `node_name` | `str` | `"camera_node"` | ROS 节点名 |
| `device` | `int` 或 `str` | `0` | 摄像头设备索引或路径（如 `"/dev/video0"`） |
| `resolution` | `(int, int)` | `(1280, 720)` | 采集分辨率 |
| `fps` | `int` | `15` | 发布帧率 |
| `image_topic` | `str` | `"/camera/image_raw"` | 图像话题名 |
| `start_service` | `str` | `"/camera/start_stream"` | 启动推流服务名 |
| `stop_service` | `str` | `"/camera/stop_stream"` | 停止推流服务名 |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `start_publishing` | `start_publishing() -> bool` | 开始采集并发布图像 |
| `stop_publishing` | `stop_publishing() -> bool` | 停止发布图像 |

### 4.2 RTSPServer

基于 GStreamer 的 RTSP H264 推流服务，支持 MJPEG HTTP 回退。

```python
from grain_sampling_camera.rtsp_server import RTSPServer

server = RTSPServer(
    device="/dev/video0",
    port=8554,
    mount_point="/camera",
    host="192.168.1.100",
    width=1280,
    height=720,
    framerate=15,
    bitrate=2000,
)
server.start()
print(server.stream_url)  # rtsp://192.168.1.100:8554/camera
server.stop()
```

#### 构造函数

```python
RTSPServer(
    device: str = "/dev/video0",
    port: int = 8554,
    mount_point: str = "/camera",
    host: str = "localhost",
    width: int = 1280,
    height: int = 720,
    framerate: int = 15,
    bitrate: int = 2000,
)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `device` | `str` | `"/dev/video0"` | V4L2 设备路径 |
| `port` | `int` | `8554` | RTSP / HTTP 端口 |
| `mount_point` | `str` | `"/camera"` | 流路径 |
| `host` | `str` | `"localhost"` | 主机名（用于构造 URL） |
| `width` | `int` | `1280` | 采集宽度 |
| `height` | `int` | `720` | 采集高度 |
| `framerate` | `int` | `15` | 帧率 |
| `bitrate` | `int` | `2000` | H264 编码码率（kbps） |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `start` | `start() -> bool` | 启动 RTSP 推流。若 GStreamer 不可用，自动回退到 MJPEG HTTP。 |
| `stop` | `stop() -> None` | 停止推流 |
| `is_streaming` | `is_streaming() -> bool` | 检查是否正在推流 |

#### 属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `stream_url` | `str` | RTSP 流地址，如 `rtsp://host:8554/camera` |
| `http_url` | `str` | MJPEG HTTP 地址（回退模式），如 `http://host:8554/camera` |

### 4.3 StreamManager

整合 RTSPServer 和 MQTT，提供云端可控的视频推流。

```python
from grain_sampling_camera.stream_integration import StreamManager
from grain_sampling_cloud.mqtt_client import MQTTClient

mqtt = MQTTClient()
mqtt.connect()

mgr = StreamManager(
    device_id="robot-001",
    mqtt_client=mqtt,
    config={"host": "192.168.1.100", "port": 8554},
)
mgr.start_streaming()
status = mgr.get_stream_status()
mgr.stop_streaming()
```

#### 构造函数

```python
StreamManager(
    device_id: str = "grain-robot-001",
    mqtt_client: Optional[MQTTClient] = None,
    config: Optional[Dict[str, Any]] = None,
    rtsp_server: Optional[RTSPServer] = None,
)
```

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `start_streaming` | `start_streaming() -> bool` | 启动 RTSP 推流并向云端上报状态 |
| `stop_streaming` | `stop_streaming() -> bool` | 停止推流并向云端上报状态 |
| `get_stream_status` | `get_stream_status() -> Dict[str, Any]` | 获取当前推流状态 |
| `update_config` | `update_config(config: Dict[str, Any]) -> None` | 动态更新推流配置 |

---

## 5. grain_sampling_devices — 外设对接

生化/理化检测设备的 TCP 适配器，提供统一的抽象接口。

### 5.1 BaseDeviceAdapter

所有外设适配器的抽象基类，定义了统一的接口规范。

```python
from grain_sampling_devices.base_adapter import (
    BaseDeviceAdapter,
    DeviceError,
    DeviceConnectionError,
    DeviceTimeoutError,
)
```

#### 抽象属性

| 属性 | 类型 | 说明 |
|------|------|------|
| `device_name` | `str` | 设备名称（如 `"BioChem-2000"`） |
| `device_type` | `str` | 设备类型（如 `"biochemical"` 或 `"physicochemical"`） |
| `ip_address` | `str` | 设备 IPv4 地址 |
| `port` | `int` | 设备 TCP 端口 |

#### 抽象方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `connect` | `connect() -> bool` | 建立 TCP 连接，返回是否成功 |
| `disconnect` | `disconnect() -> None` | 断开连接 |
| `is_connected` | `is_connected() -> bool` | 检查连接状态 |
| `send_command` | `send_command(cmd: bytes) -> bytes` | 发送命令，返回设备响应 |

#### 异常类

```python
class DeviceError(Exception):
    """设备错误基类"""
    device_name: Optional[str]

class DeviceConnectionError(DeviceError):
    """连接失败"""

class DeviceTimeoutError(DeviceError):
    """操作超时"""
```

### 5.2 TCPClient

线程安全的 TCP 客户端，支持自动重连，是所有设备适配器的底层传输层。

```python
from grain_sampling_devices.tcp_client import TCPClient

client = TCPClient(host="192.168.1.200", port=5001, timeout=5.0, retry_count=3)
client.connect()
resp = client.send(b"STATUS\r\n")
client.disconnect()
```

#### 构造函数

```python
TCPClient(
    host: str,
    port: int,
    timeout: float = 5.0,
    retry_count: int = 3,
)
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `host` | `str` | — | 目标设备 IP |
| `port` | `int` | — | 目标端口 |
| `timeout` | `float` | `5.0` | 套接字超时（秒） |
| `retry_count` | `int` | `3` | 发送失败时最大自动重连次数 |

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `connect` | `connect() -> None` | 建立 TCP 连接，失败则抛出异常 |
| `disconnect` | `disconnect() -> None` | 关闭连接 |
| `is_connected` | `is_connected() -> bool` | 检查是否已连接 |
| `send` | `send(data: bytes) -> bytes` | 发送字节数据并接收响应 |

### 5.3 BiochemicalAdapter

生化检测设备适配器（当前为 JSON-over-TCP Stub，待供应商提供真实协议后替换）。

```python
from grain_sampling_devices.biochemical_adapter import BiochemicalAdapter

adapter = BiochemicalAdapter(host="192.168.1.200", port=5001)
adapter.connect()

# 发送 JSON 命令（Stub 协议）
resp = adapter.send_command(b'{"cmd":"start_test"}')

# 获取设备信息
info = adapter.get_device_info()

adapter.disconnect()
```

#### 构造函数

```python
BiochemicalAdapter(host: str, port: int = 5001)
```

#### 方法（继承自 BaseDeviceAdapter + 扩展）

| 方法 | 签名 | 说明 |
|------|------|------|
| `connect` | `connect() -> bool` | 建立 TCP 连接 |
| `disconnect` | `disconnect() -> None` | 断开连接 |
| `is_connected` | `is_connected() -> bool` | 检查连接 |
| `send_command` | `send_command(cmd: bytes) -> bytes` | 发送原始命令 |
| `get_device_info` | `get_device_info() -> Dict[str, Any]` | 获取设备信息（名称、固件版本等） |
| `start_test` | `start_test() -> Dict[str, Any]` | 启动检测 (Stub) |
| `get_result` | `get_result() -> Dict[str, Any]` | 获取检测结果 (Stub) |

### 5.4 PhysicochemicalAdapter

理化检测设备适配器（当前为 JSON-over-TCP Stub）。

```python
from grain_sampling_devices.physicochemical_adapter import PhysicochemicalAdapter

adapter = PhysicochemicalAdapter(host="192.168.1.201", port=5002)
adapter.connect()
result = adapter.get_result()
adapter.disconnect()
```

#### 构造函数

```python
PhysicochemicalAdapter(host: str, port: int = 5002)
```

接口与 `BiochemicalAdapter` 相同，仅 `device_type` 返回 `"physicochemical"`。

---

## 6. grain_sampling_ui — 操作界面

基于 PySide6 的 1024×600 操作界面，包含 6 个页面和 4 个通用组件。

### 6.1 MainWindow

应用主窗口。

```python
from grain_sampling_ui.main import MainWindow, main

# 方式一：代码启动
from PySide6.QtWidgets import QApplication
import sys

app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()

# 方式二：命令行启动
# $ grain-sampling-ui
```

### 6.2 PageManager

基于 QStackedWidget 的页面导航管理器，支持 push/pop 导航栈。

```python
from grain_sampling_ui.page_manager import PageManager

manager = PageManager(stacked_widget)
manager.register_page("main", main_page_widget)
manager.register_page("guidance", guidance_page_widget)
manager.push("guidance")   # 导航到扦样引导页
manager.pop()              # 返回上一页
manager.go_home()          # 回到主页
```

#### 构造函数

```python
PageManager(stacked_widget: QStackedWidget, parent: Optional[QObject] = None)
```

#### 方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `register_page` | `register_page(name: str, widget: QWidget) -> int` | 注册页面，返回栈索引 |
| `push` | `push(name: str) -> None` | 推入新页面 |
| `pop` | `pop() -> Optional[str]` | 返回上一页，返回页面名 |
| `go_home` | `go_home() -> None` | 回到主页 |
| `current_page_name` | `current_page_name() -> Optional[str]` | 获取当前页面名 |

#### 信号

```python
page_changed = Signal(str)  # 页面切换时发射，参数为新页面名称
```

### 6.3 UI 页面

6 个页面类，每个继承自 `QWidget`：

| 页面 | 类名 | 文件 | 用途 |
|------|------|------|------|
| 主操作页 | `MainPage` | `pages/main_page.py` | 机器人状态总览、地图、快捷操作 |
| 扦样引导页 | `GuidancePage` | `pages/guidance_page.py` | 15 步扦样流程逐步引导 |
| 廒间地图 | `MapPage` | `pages/map_page.py` | 仓库地图列表，上传、预览 |
| 任务列表 | `TaskListPage` | `pages/task_list_page.py` | 工单列表，显示状态（未完成/进行中/已完成） |
| 建图设置 | `MappingPage` | `pages/mapping_page.py` | 启动/停止/保存 SLAM 建图 |
| 系统设置 | `SettingsPage` | `pages/settings_page.py` | 网络、设备、系统参数设置 |

每个页面通常实现以下方法：

```python
class SomePage(QWidget):
    def __init__(self, parent=None): ...
    def refresh(self) -> None:      # 刷新页面数据
    def on_activate(self) -> None:  # 页面被激活时调用
    def on_deactivate(self) -> None: # 页面被隐藏时调用
```

### 6.4 UI 组件

4 个可复用组件：

| 组件 | 类名 | 文件 | 用途 |
|------|------|------|------|
| 状态栏 | `StatusBar` | `widgets/status_bar.py` | 顶部状态栏（时间、网络、电量、设备编号） |
| 控制面板 | `ControlPanel` | `widgets/control_panel.py` | 底部导航栏（6 个页面切换按钮） |
| 地图组件 | `MapWidget` | `widgets/map_widget.py` | 仓库地图可视化显示 |
| 告警条 | `AlarmBar` | `widgets/alarm_bar.py` | 告警信息展示 |

### 6.5 ROSNodeThread

ROS 后台线程，负责在主线程外运行 ROS spin。

```python
from grain_sampling_ui.ros_thread import ROSNodeThread

thread = ROSNodeThread()
thread.start()
# ... UI 运行中 ...
thread.stop()
```

### 6.6 Theme

UI 主题定义（深色工业风格）。

```python
from grain_sampling_ui.theme import THEME_QSS

app.setStyleSheet(THEME_QSS)
```

---

## 7. utils — 工具模块

### 7.1 AppConfig

集中式应用配置数据类，提供类型安全的配置管理和工厂方法。

```python
from utils.config import AppConfig

# 使用默认配置
config = AppConfig()
print(config.device_id)  # "robot-001"

# 从字典加载
config = AppConfig.from_dict({"device_id": "robot-002", "mqtt_broker": "mqtt://prod:1883"})

# 从 JSON 文件加载
config = AppConfig.from_json_file("/etc/robot/config.json")
```

#### 属性

| 属性 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mqtt_broker` | `str` | `"mqtt://localhost:1883"` | MQTT Broker 地址 |
| `device_id` | `str` | `"robot-001"` | 设备唯一标识 |
| `rtsp_port` | `int` | `8554` | RTSP / HTTP 端口 |
| `camera_device` | `str` | `"/dev/video0"` | 摄像头设备路径 |
| `livox_config_path` | `str` | `"config/livox_config.json"` | Livox 配置文件路径 |
| `slice_default_height` | `float` | `0.5` | 默认切面高度（米） |
| `biochemical_port` | `int` | `5001` | 生化检测设备 TCP 端口 |
| `physicochemical_port` | `int` | `5002` | 理化检测设备 TCP 端口 |

#### 工厂方法

| 方法 | 签名 | 说明 |
|------|------|------|
| `from_dict` | `from_dict(data: Dict[str, Any]) -> AppConfig` | 从字典创建，未知 key 被忽略 |
| `from_json_file` | `from_json_file(path: str) -> AppConfig` | 从 JSON 文件加载配置 |

---

## 附录

### A. 项目入口点

| 命令 | Python 入口 | 说明 |
|------|------------|------|
| `grain-sampling-ui` | `grain_sampling_ui.main:main` | 启动操作界面 |
| `mock-robot` | `mock_robot.main:main` | 启动模拟机器人节点 |

### B. Mock Robot

MockRobotNode 为开发和测试提供模拟的机器人数据：

- 订阅话题：无（纯发布者）
- 发布话题：`/odometry/filtered`（圆形模拟轨迹，10Hz）、`/map`（10×10m 静态地图，0.05m 分辨率）、`/tf`（坐标变换）、`/mechanism/status`（JSON 状态，1Hz）

```python
from mock_robot.mock_robot_node import MockRobotNode
import rospy

rospy.init()
node = MockRobotNode()
rospy.spin(node)
```

---

*文档版本：v0.1 | 最后更新：2026-07-15*
