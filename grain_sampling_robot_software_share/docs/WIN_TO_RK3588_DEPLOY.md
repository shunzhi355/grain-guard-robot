# 从 Windows 开发环境迁移到 RK3588 部署指南

## 概述

本文档指导将粮食扦样机器人软件从 Windows 开发环境迁移到 RK3588（Ubuntu 20.04 / ROS Noetic）上运行。

## 1. 代码传输

### 方式一：Zip 压缩包（推荐）
1. 将 `grain_sampling_robot_software.zip` 拷贝到 U 盘
2. 插入 RK3588，拷贝到用户目录下：
   ```bash
   cp /media/usb/grain_sampling_robot_software.zip ~/
   cd ~/
   unzip grain_sampling_robot_software.zip -d grain_sampling_robot_software
   cd grain_sampling_robot_software
   ```

### 方式二：SCP 网络传输
```bash
# 在 RK3588 上执行（替换 IP 为 Windows 机器 IP）
scp user@192.168.1.xxx:"E:/青赋驭境/项目/粮食扦样/grain_sampling_robot_software.zip" ~/
unzip grain_sampling_robot_software.zip -d grain_sampling_robot_software
```

## 2. 环境准备

### 2.1 安装 Python 3.10
Ubuntu 20.04 默认是 Python 3.8，代码需要 3.10+。

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.10 python3.10-dev python3.10-venv
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
sudo update-alternatives --config python3
python3 -m ensurepip --upgrade
python3 --version  # 应显示 Python 3.10.x
```

### 2.2 运行一键部署脚本
```bash
cd ~/grain_sampling_robot_software
chmod +x scripts/deploy.sh
sudo ./scripts/deploy.sh
```

此脚本会自动安装：
- ROS Noetic 基础包
- Python 依赖（PySide6、numpy、Pillow 等）
- 环境变量配置（~/.bashrc）

### 2.3 PySide6 在 arm64 上的安装
PySide6 官方没有 arm64 的 wheel，在 RK3588 上需要额外处理。

**备选方案：**

```bash
# 方案一：从源码编译（耗时约1-2小时）
sudo apt install -y qt6-base-dev qt6-tools-dev qt6-tools-dev-tools
pip3 install PySide6 --no-binary PySide6

# 方案二：降级使用 PySide2
pip3 install PySide2
# 需将代码中 PySide6 导入改为 PySide2

# 方案三：搜索社区 arm64 预编译 wheel
```

## 3. 环境验证

```bash
python3 --version  # 应 >= 3.10
source /opt/ros/noetic/setup.bash
python3 -c "import rospy; print('ROS OK')"
python3 -c "from PySide6.QtCore import *; print('PySide OK')" || \
  python3 -c "from PySide2.QtCore import *; print('PySide2 OK')"
cd ~/grain_sampling_robot_software
python3 -m pytest --tb=short -q
```

## 4. 启动运行

### 4.1 Mock 机器人（无需硬件）
```bash
source /opt/ros/noetic/setup.bash
cd ~/grain_sampling_robot_software
python3 src/mock_robot/mock_robot_node.py &

# 验证 ROS 数据
rostopic list
rostopic echo /odometry/filtered -n 1
```

### 4.2 启动 UI 界面（需 7 寸屏）
```bash
export DISPLAY=:0
cd ~/grain_sampling_robot_software
python3 -m grain_sampling_ui.main
```

### 4.3 运行环境检查脚本
```bash
cd ~/grain_sampling_robot_software
./scripts/env_check.sh
```

## 5. 配置 HTTP 云端

等甲方提供服务器地址后配置：
```bash
echo "export CLOUD_BASE_URL=http://甲方提供的地址" >> ~/.bashrc
source ~/.bashrc
```

当前占位地址为 `http://cloud-api.xxx.com`，需替换为实际地址。

## 6. 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| No module named rospy | 未 source ROS 环境 | source /opt/ros/noetic/setup.bash |
| PySide6 导入失败 | arm64 无官方 wheel | 改用 PySide2 或源码编译 |
| cannot connect to X server | 未设置 DISPLAY | export DISPLAY=:0 |
| pytest 找不到模块 | 未在项目根目录 | cd 到项目目录 |
| 摄像头找不到 | 权限不足 | sudo usermod -a -G video $USER |

## 7. 部署检查清单

- [ ] 代码已解压到 RK3588
- [ ] Python 3.10+ 已安装并设为默认
- [ ] ROS Noetic 已安装
- [ ] Python 依赖已安装
- [ ] pytest 全部通过
- [ ] ROS 环境已验证
- [ ] Mock 节点能正常发布数据
- [ ] HTTP 云端地址已配置

## 8. 附录：详细部署文档

完整的部署说明（Livox 驱动、5G 模块、网络配置等）请参考：
- docs/DEPLOY_GUIDE.md — 详细部署指南
- scripts/env_check.sh — 环境检查脚本
- scripts/acceptance_checklist.md — 验收清单
