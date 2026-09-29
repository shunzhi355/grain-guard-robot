# 粮食扦样机器人部署指南 v0.1

> 适用人员：技术员 / 现场部署工程师
> 更新日期：2026-07-15

---

## 一、系统要求

### 1.1 硬件要求

| 项目 | 规格 |
|------|------|
| 主控板 | RK3588（8核 ARM，4GB+ RAM） |
| 存储 | 64GB eMMC + 可扩展 TF 卡 |
| 显示屏 | 7 寸触摸屏，分辨率 1024×600 |
| 激光雷达 | Livox Mid-360（通过网口连接） |
| 摄像头 | USB 摄像头（支持 V4L2） |
| 网络 | 5G 模组 + WiFi 模组（双模通信） |

### 1.2 软件要求

| 软件 | 版本 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 20.04 LTS (arm64) | RK3588 刷入 Armbian 或官方 Ubuntu |
| ROS | Noetic Ninjemys | 机器人框架 |
| Python | 3.10+ | 运行环境 |
| GStreamer | 1.20+ | RTSP 视频推流 |
| Open3D | 0.18+ | 点云处理（可选，用于预览） |

---

## 二、环境安装

### 2.1 刷入 Ubuntu 系统

1. 从 RK3588 厂商获取 Ubuntu 20.04 镜像并刷入 eMMC。
2. 首次启动后，完成基础设置（语言、时区、用户名等）。
3. 建议创建一个名为 `robot` 的用户，后续所有操作在此用户下进行。

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础工具
sudo apt install -y git curl wget vim net-tools
```

### 2.2 安装 ROS Noetic

```bash
# 设置 locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros.list > /dev/null

# 安装 ROS Noetic（基础版）
sudo apt update
sudo apt install -y ros-noetic-ros-base

# 安装额外依赖
sudo apt install -y ros-noetic-cv-bridge ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs python3-catkin-tools
```

### 2.3 安装系统依赖

```bash
# Python 虚拟环境
sudo apt install -y python3-venv python3-pip

# GStreamer（RTSP 推流用）
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# 点云处理依赖
sudo apt install -y libopen3d-dev  # 可选，如编译失败可跳过
```

### 2.4 配置 ROS 环境

将以下内容添加到 `~/.bashrc`：

```bash
# ROS Noetic
source /opt/ros/noetic/setup.bash

# 本项目的 workspace
source ~/grain_sampling_ws/install/setup.bash 2>/dev/null
```

---

## 三、项目部署

### 3.1 获取代码

```bash
# 克隆仓库到机器人
cd ~
git clone <仓库地址> grain_sampling_robot_software
cd grain_sampling_robot_software
```

### 3.2 创建虚拟环境并安装

```bash
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装核心依赖
pip install --upgrade pip
pip install -r requirements.txt

# 以开发模式安装项目
pip install -e .
```

### 3.3 验证安装

```bash
# 运行测试
pytest

# 预期结果：所有测试通过（可能有 1~2 个 skip，因无实际 ROS 环境）

# 测试 UI 能否启动（需要有显示器和桌面环境）
grain-sampling-ui
```

---

## 四、LiDAR 配置

### 4.1 Livox Mid-360 网络配置

Livox Mid-360 通过以太网连接 RK3588 的网口。

```bash
# 1. 设置机器人网口 IP 为 Livox 默认网段
sudo ip addr add 192.168.1.50/24 dev eth0
# 或者持久化配置：
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.1.50/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

### 4.2 安装 Livox SDK 驱动

```bash
# 克隆并编译 Livox ROS 驱动
cd ~
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd livox_ros_driver
source /opt/ros/noetic/setup.bash
colcon build --packages-select livox_ros_driver
```

### 4.3 验证 LiDAR

```bash
# 启动 Livox 驱动
source ~/livox_ros_driver/install/setup.bash
ros launch livox_ros_driver livox_lidar_launch.py

# 在新终端中验证点云数据
ros topic echo /livox/lidar --once
```

### 4.4 配置文件

确保 `config/livox_config.json` 中的 IP 地址与实际上设置一致：

```json
{
  "lidar_net_info": {
    "cmd_data_port": 56000,
    "push_msg_port": 0,
    "point_data_port": 57000,
    "imu_data_port": 58000
  },
  "host_net_info": {
    "cmd_data_ip": "192.168.1.50",
    "cmd_data_port": 56000
  }
}
```

---

## 五、网络配置

### 5.1 5G 模组配置

```bash
# 确认 5G 模组被识别
lsusb | grep -i quectel
# 或者
ls /dev/ttyUSB*

# 使用 ModemManager 配置连接
sudo apt install -y modemmanager
sudo systemctl enable ModemManager
sudo systemctl start ModemManager

# 添加 APN 配置（联系运营商获取正确的 APN）
sudo nmcli con add type gsm ifname "*" con-name "5G-Connection" \
    apn "your-apn-here" connection.autoconnect yes
```

### 5.2 WiFi 配置

```bash
# 扫描可用 WiFi
nmcli dev wifi list

# 连接到 WiFi
sudo nmcli dev wifi connect "SSID名称" password "密码"
```

### 5.3 MQTT Broker 配置

MQTT 消息中间件作为机器人与云端通信的桥梁。

```bash
# 安装 Mosquitto MQTT Broker（可选，如果云端已有则跳过）
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

配置 MQTT 连接参数（通过系统配置文件）：

```yaml
# config/mqtt_config.yaml
primary:        # 5G 通道
  broker_url: "mqtt://cloud-broker.example.com:1883"
  client_id: "robot-001"
  keepalive: 60
fallback:       # WiFi 通道
  broker_url: "mqtt://local-broker.example.com:1883"
  client_id: "robot-001-wifi"
  keepalive: 60
network_check:
  enabled: true
  primary_ping: "8.8.8.8"
  fallback_ping: "192.168.1.1"
  interval: 5.0
  timeout: 2
```

---

## 六、运行验证

### 6.1 启动系统

有两种启动方式：

**方式一：一键启动（推荐）**

```bash
# 运行启动脚本
bash scripts/bringup.sh
```

**方式二：分步启动**

```bash
# 终端 1：启动 Livox LiDAR 驱动
source /opt/ros/noetic/setup.bash
source ~/livox_ros_driver/install/setup.bash
ros launch livox_ros_driver livox_lidar_launch.py

# 终端 2：启动点云采集节点
source ~/grain_sampling_ws/install/setup.bash
ros run grain_sampling_pointcloud collector

# 终端 3：启动相机节点（发布 ROS 图像话题）
ros run grain_sampling_camera camera_node

# 终端 4：启动 RTSP 推流
python3 -c "
from grain_sampling_camera.stream_integration import StreamManager
mgr = StreamManager()
mgr.start_streaming()
"

# 终端 5：启动 UI
source ~/grain_sampling_robot_software/venv/bin/activate
grain-sampling-ui
```

### 6.2 验证清单

启动后逐项检查：

| 检查项 | 验证方法 | 预期结果 |
|--------|----------|----------|
| UI 显示正常 | 观察 7 寸屏幕 | 主操作页显示，1024×600 分辨率 |
| LiDAR 数据 | `ros topic echo /livox/lidar --once` | 输出点云数据 |
| 相机图像 | `ros topic echo /camera/image_raw --once` | 输出图像消息 |
| MQTT 连接 | 查看 UI 状态栏网络图标 | 显示已连接（绿色） |
| RTSP 推流 | 用 VLC 打开 `rtsp://<机器人IP>:8554/camera` | 显示实时视频 |
| 导航服务 | `ros service call /navigate_to_pose std_srvs/srv/Trigger` | 返回 success |
| 机构控制 | `ros service call /mechanism/start_sampling std_srvs/srv/Trigger` | 返回 success |

### 6.3 开发模式（模拟运行）

在没有实际硬件的情况下，可以使用 Mock Robot 进行开发和测试：

```bash
# 终端 1：启动模拟机器人
mock-robot

# 终端 2：启动 UI
grain-sampling-ui
```

Mock Robot 会发布模拟的里程计、地图、TF 变换和机构状态，方便在不连接实际传感器的情况下测试 UI 和流程。

---

## 七、故障排查

### 7.1 LiDAR 未检测到

**现象**：`ros topic list` 中没有 `/livox/lidar` 话题。

**排查步骤**：

1. 检查网线连接是否牢固。
2. 确认 IP 配置正确：`ip addr show eth0`。
3. 尝试 ping LiDAR：`ping 192.168.1.1XX`（根据 LiDAR 实际 IP）。
4. 检查 Livox 驱动是否正常运行：`ros node list | grep livox`。
5. 重启 Livox 驱动：重新运行 launch 文件。

### 7.2 摄像头未识别

**现象**：无 `/camera/image_raw` 话题，或 RTSP 推流失败。

**排查步骤**：

```bash
# 检查摄像头设备
ls /dev/video*
# 预期输出：/dev/video0, /dev/video1 等

# 测试摄像头是否能采集图像
v4l2-ctl --device=/dev/video0 --list-formats-ext

# 检查 GStreamer 是否安装
gst-launch-1.0 --version
```

如果 `/dev/video0` 不存在：
- 检查 USB 接口是否松动。
- 尝试更换 USB 口。
- 确认摄像头在 `lsusb` 中可见。

### 7.3 MQTT 连接失败

**现象**：状态栏显示"未连接"，任务列表为空。

**排查步骤**：

```bash
# 1. 检查网络连接
ping 8.8.8.8                # 测试外网
ping 192.168.1.1             # 测试局域网

# 2. 测试 MQTT Broker 连接
mosquitto_pub -h <broker地址> -p 1883 -t "test" -m "hello"

# 3. 检查配置文件
cat config/mqtt_config.yaml

# 4. 查看日志
journalctl -u grain-sampling -f
```

常见原因：
- Broker 地址或端口配置错误。
- 防火墙阻止了 1883 端口。
- 5G/WiFi 网络未正常连接。

### 7.4 UI 无法启动

**现象**：运行 `grain-sampling-ui` 无反应或报错。

**排查步骤**：

```bash
# 确认依赖已安装
pip list | grep PySide6

# 确认虚拟环境已激活
which python3

# 手动运行 Python
python3 -c "from grain_sampling_ui.main import main; main()"

# 检查显示器环境变量
echo $DISPLAY
# 应为 :0 或 :0.0

# 如果无显示器，设置：
export DISPLAY=:0
```

### 7.5 Python 包导入错误

**现象**：`ModuleNotFoundError: No module named 'grain_sampling_xxx'`。

**解决方法**：

```bash
# 确认已安装为 editable 模式
pip install -e .

# 或设置 PYTHONPATH
export PYTHONPATH=~/grain_sampling_robot_software/src:$PYTHONPATH
```

---

## 八、附录

### 8.1 项目文件结构

```
grain_sampling_robot_software/
├── src/
│   ├── grain_sampling_ui/       # PySide6 UI（6页面 + 4组件）
│   ├── grain_sampling_cloud/    # MQTT云端通信
│   ├── grain_sampling_workflow/ # 15步扦样流程状态机
│   ├── grain_sampling_pointcloud/ # 点云处理（采集/切面/预览/上传）
│   ├── grain_sampling_camera/   # USB摄像头 + RTSP推流
│   ├── grain_sampling_devices/  # 外设适配器（生化/理化检测）
│   ├── mock_robot/              # 模拟机器人（开发用）
│   └── utils/                   # 共享工具
├── config/                      # 配置文件
├── docs/                        # 文档
├── scripts/                     # 启动脚本
├── test/                        # 测试文件
├── setup.py
└── requirements.txt
```

### 8.2 常用命令速查

| 命令 | 说明 |
|------|------|
| `grain-sampling-ui` | 启动操作界面 |
| `mock-robot` | 启动模拟机器人 |
| `pytest` | 运行所有测试 |
| `ros topic list` | 查看所有 ROS 话题 |
| `ros service list` | 查看所有 ROS 服务 |
| `ros node list` | 查看所有 ROS 节点 |
| `pip install -e .` | 以开发模式安装项目 |
| `bash scripts/bringup.sh` | 一键启动全系统 |

---

*文档版本：v0.1 | 最后更新：2026-07-15*
