# 粮食扦样机器人 — 软件系统

> 基于 PySide6 + Debian ROS1 + MQTT 的全栈工业机器人控制软件
> 运行平台：RK3588 / Ubuntu 22.04 aarch64

---

## 项目概述

工控机 USB1 经 USB-TTL 连接 STM32 USART2 的导航控制见 [串口底盘说明](docs/CHASSIS_SERIAL.md)，
启动入口为 `scripts/start_industrial_pc.sh`。底盘 PWM 和遥控由 STM32 负责，机构仍使用 PCA9685。

粮食扦样机器人是一套面向粮库质量检测场景的自动化控制系统。系统通过云端工单下发扦样任务，机器人自动导航到仓库指定位置，将扦样管插入粮堆完成取样，再将样本送至化验设备。软件层面整合了触摸屏操作界面、MQTT 远程通信、ROS 运动控制、激光点云处理、视频监控、以太网设备通信七个模块，为粮库一线操作工和技术员提供完整的闭环工作流。

---

## 系统架构

软件共包含七个核心子包，各司其职：

| 模块 | 技术栈 | 职责 |
|------|--------|------|
| **操作界面** (`grain_sampling_ui`) | PySide6 | 7 寸触摸屏（1024×600）人机交互界面，含主操作页、任务列表、设备状态、视频预览、点云视图、系统设置六个页面 |
| **云端通信** (`grain_sampling_cloud`) | MQTT / JSON | 5G + WiFi 双模通信中间件，支持工单接收、状态上报、消息缓存与自动重连 |
| **扦样流程** (`grain_sampling_workflow`) | ROS / FSM | 有限状态机驱动的 ROS 节点编排，管理导航、取样、送检全过程的状态流转与异常处理 |
| **点云处理** (`grain_sampling_pointcloud`) | Livox ROS Driver | Livox Mid-360 激光雷达点云采集、2D 切面提取、轮廓预览与上传 |
| **摄像头模块** (`grain_sampling_camera`) | GStreamer / RTSP | USB 摄像头驱动与 RTSP 视频推流，支持 MJPEG 软解回退 |
| **外设对接** (`grain_sampling_devices`) | TCP / Ethernet | 生化检测设备、理化检测设备的以太网通信适配（接口文档待甲方提供） |
| **工具模块** (`utils`) | — | 配置管理、日志系统、数据校验、坐标转换等跨模块共享工具 |

---

## 目录结构

```
grain_sampling_robot_software/
├── src/                          # 源代码
│   ├── grain_sampling_ui/        # PySide6 操作界面
│   ├── grain_sampling_cloud/     # MQTT 云端通信
│   ├── grain_sampling_workflow/  # ROS 扦样流程
│   ├── grain_sampling_pointcloud/# Livox 点云处理
│   ├── grain_sampling_camera/    # 摄像头驱动
│   ├── grain_sampling_devices/   # 外设控制
│   ├── mock_robot/               # 仿真机器人（离线开发用）
│   └── utils/                    # 共享工具
├── test/                         # 测试用例（pytest，248+ 用例）
├── docs/                         # 项目文档
├── config/                       # 配置文件
├── scripts/                      # 部署与运维脚本
├── .github/workflows/            # CI 流水线
├── setup.py                      # 包定义与入口
├── requirements.txt              # Python 依赖
├── pytest.ini                    # 测试配置
├── .flake8                       # 代码风格配置
├── mypy.ini                      # 类型检查配置
└── README.md                     # 本文件
```

---

## 快速开始

### 环境要求

| 项目 | 要求 |
|------|------|
| 硬件平台 | RK3588（8 核 ARM，4GB+ RAM） |
| 操作系统 | Ubuntu 22.04 LTS (aarch64) |
| ROS | Debian packaged ROS1 1.15.x |
| Python | 3.10 及以上 |
| GStreamer | 1.20+（RTSP 推流，可选，无 GStreamer 时使用 MJPEG 回退） |

### 安装步骤

```bash
# 1. 克隆仓库
git clone <仓库地址>
cd grain_sampling_robot_software

# 2. 创建虚拟环境
python -m venv venv
source venv/bin/activate

# 3. 安装依赖
pip install -r requirements.txt

# 4. 以可编辑模式安装本项目
pip install -e .

# 5. 验证安装
python -c "import grain_sampling_ui; print('安装成功')"
```

### 启动

```bash
# 启动操作界面（需要显示屏）
python -m grain_sampling_ui.main

# 或通过控制台入口启动
grain-sampling-ui
```

---

## 核心功能

| 功能 | 说明 |
|------|------|
| **操作界面** | 7 寸触摸屏 UI，提供任务操作、设备监控、视频预览、点云可视化等完整交互体验 |
| **云端通信** | 通过 MQTT 协议与云平台双向通信，接收工单、上报设备状态与作业结果，支持 5G/WiFi 双网切换 |
| **扦样作业** | FSM 状态机管理全流程：工单解析 → 自主导航 → 粮面感知 → 扦样管插入 → 吸粮取样 → 样本送检 |
| **点云切面** | Livox Mid-360 激光雷达采集点云，Open3D 执行粮面拟合与切面计算，自动生成取样坐标 |
| **视频监控** | USB 摄像头通过 GStreamer 推流 RTSP 视频，支持实时预览、截图保存与录像回放 |
| **外设对接** | 统一抽象 GPIO 和 TCP 设备接口，控制扦样管电机、吸粮风机、化验仪器等外部设备 |

---

## 开发指南

### 运行测试

```bash
# 运行全部测试（含覆盖率报告）
pytest

# 运行指定模块测试
pytest test/test_ui/           # 界面测试
pytest test/test_devices/ -v   # 外设测试（详细输出）

# 覆盖率阈值检查（默认 ≥70%）
pytest --cov-fail-under=70
```

### 代码规范

```bash
# 代码风格检查（flake8）
flake8 src/ test/

# 类型检查（mypy）
mypy src/
```

### 离线开发

本项目提供了 `mock_robot` 仿真模块，在无实际机器人硬件的环境下也能进行开发与调试：

```bash
# 启动仿真机器人
mock-robot
```

---

## 文档索引

| 文档 | 适用人员 | 说明 |
|------|----------|------|
| [USER_MANUAL.md](docs/USER_MANUAL.md) | 粮库操作工 | 开机、任务操作、日常维护的操作手册 |
| [DEPLOY_GUIDE.md](docs/DEPLOY_GUIDE.md) | 现场部署工程师 | 环境安装、系统配置、网络部署指南 |
| [API_REFERENCE.md](docs/API_REFERENCE.md) | 开发人员 | 各模块 API 详细参考与代码示例 |
| [ros-interfaces.md](docs/ros-interfaces.md) | 开发人员 | ROS 话题、服务、动作接口定义 |
| [X2P手动调整初始位置使用说明.md](docs/X2P手动调整初始位置使用说明.md) | 现场操作/调试人员 | 上电后把升降机构手动挪回物理最高点的完整操作说明 |

---

## 依赖项

### 运行依赖

| 包名 | 版本 | 用途 |
|------|------|------|
| PySide6 | ≥6.5.0 | GUI 框架 |
| paho-mqtt | ≥1.6.1 | MQTT 通信 |
| rospy | ≥5.3.0 | ROS Python 客户端 |
| numpy | ≥1.24.0 | 数值计算 |
| opencv-python | ≥4.8.0 | 图像处理 |
| PyYAML | ≥6.0 | 配置文件解析 |

### 开发依赖

| 包名 | 版本 | 用途 |
|------|------|------|
| pytest | ≥7.0 | 测试框架 |
| pytest-cov | ≥4.0 | 覆盖率报告 |
| pytest-timeout | ≥2.0 | 测试超时控制 |
| flake8 | ≥6.0 | 代码风格检查 |
| mypy | ≥1.0 | 静态类型检查 |

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v0.1.0 | 2026-07-15 | 初始版本，完成七个子包的全栈开发，含操作界面、云端通信、扦样流程、点云处理、摄像头、外设对接与工具模块，配套 248+ 测试用例 |

---

## 许可证

MIT License
