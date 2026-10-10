# ROS 接口对齐表 —— 粮食扦样机器人

> 本文档用于各团队（底盘/导航/SLAM/机构/UI）对齐 ROS 接口。
> 请逐项确认 Topic/Service 名称、类型、提供方。如有冲突请标注修改。

---

## 一、我方（UI团队）订阅的 Topic

| Topic | 消息类型 | 频率 | 提供方 | 用途 |
|:------|:---------|:----:|:------|:-----|
| `/odometry/filtered` | `nav_msgs/Odometry` | ~50Hz | 导航团队(EKF) | UI显示机器人位置 |
| `/map` | `nav_msgs/OccupancyGrid` | 0.1~1Hz | SLAM团队 | UI显示仓库栅格地图 |
| `/tf` | `tf2_msgs/TFMessage` | ~50Hz | 导航团队 | 坐标变换 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10Hz | LiDAR驱动 | 点云采集与切面提取 |
| `/camera/image_raw` | `sensor_msgs/Image` | 15fps | 摄像头驱动 | 实时视频流 |
| `/mechanism/status` | `std_msgs/String`(JSON) | 1~5Hz | 机构团队 | 机构状态(深度/负压/重量) |

### 确认 ☐ （请各提供方确认以上 Topic 名称和类型无误）

---

## 二、我方（UI团队）调用的 Service

| Service | 类型 | 说明 | 提供方 |
|:--------|:-----|:-----|:-------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 发送导航目标点(X,Y) | 导航团队 |
| `/mechanism/start_sampling` | `std_srvs/Trigger` | 启动扦样 | 机构团队 |
| `/mechanism/stop_sampling` | `std_srvs/Trigger` | 停止扦样 | 机构团队 |
| `/mechanism/pause_sampling` | `std_srvs/Trigger` | 暂停吸粮 | 机构团队 |
| `/mechanism/resume_sampling` | `std_srvs/Trigger` | 恢复吸粮 | 机构团队 |
| `/slam/start_mapping` | `std_srvs/Trigger` | 开始建图 | SLAM团队 |
| `/slam/stop_mapping` | `std_srvs/Trigger` | 停止建图 | SLAM团队 |
| `/slam/save_map` | `std_srvs/Trigger` | 保存地图 | SLAM团队 |
| `/emergency_stop` | `std_srvs/Trigger` | 全系统急停 | 全系统 |

### 确认 ☐ （请各提供方确认以上 Service 名称和类型无误）

---

## 三、坐标系约定

| Frame | 父 Frame | 说明 |
|:------|:---------|:-----|
| `map` | — | 世界坐标系原点 |
| `odom` | `map` | 里程计坐标系 |
| `base_link` | `odom` | 机器人本体中心 |
| `laser` | `base_link` | Livox Mid-360 激光雷达 |
| `camera` | `base_link` | USB 摄像头 |

### 确认 ☐

---

## 四、`/mechanism/status` JSON 格式约定

```json
{
  "state": "idle",
  "depth": 1.5,
  "pressure": -3.2,
  "bin_weights": [2.3, 0, 1.8],
  "pipe_index": 3,
  "error_code": 0
}
```

字段说明：
- `state`: `idle` | `pressing` | `suctioning` | `conveying` | `error`
- `depth`: 当前扦样深度（米）
- `pressure`: 负压吸粮系统压力值（kPa）
- `bin_weights`: 三个分仓各自重量（kg），顺序[1号仓, 2号仓, 3号仓]
- `pipe_index`: 当前已连接管节序号
- `error_code`: 0=正常，非0=故障码

### 确认 ☐

---

## 五、确认回执

| 团队 | 确认人 | 日期 | 修改意见 |
|:----|:------|:----:|:---------|
| 底盘驱动 | | | |
| 导航/SLAM | | | |
| 机构控制 | | | |
| UI/操作界面 | | | |
