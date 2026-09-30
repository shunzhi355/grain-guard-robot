# 3588 假导航、真实机构台架联调

此模式用于“人工将车停在采样位 → UI 正常创建任务 → 终端人工确认假到位 → 真实机构/三仓流程”。默认关闭，不属于生产导航方案。联想主机不需要启动，假导航终端与 UI 均运行在 3588。它不会发 GRICP 速度、PWM 或底盘运动许可。

## 前置安全条件

1. 首次测试架空履带，清空机构周围人员与障碍物，准备可用的物理急停。
2. STM32 底盘串口在线、遥控器处于自动档、底盘未获运动许可、无急停/故障；机构动作前守护进程还会复核底盘状态。
3. X2P、PCA9685、电调和机构传感器按实机配置；不要在无人值守时开启真实机构。
4. 3588 服务端仍需本机固定 IP 和双向 TLS 证书以正常启动，但联想客户端不必连接。终端与 UI 须能访问同一个 Unix Socket 路径。

## 三个终端的启动顺序

在 `3588/` 目录执行。按现场部署配置加载 `config/industrial_pc.env`；不要把假导航环境变量写入生产 service。

终端 1，启动无 ROS 的 3588 守护进程：

```bash
source config/industrial_pc.env
bash scripts/start_industrial_pc.sh
```

终端 2，启动假导航人工确认程序：

```bash
source config/industrial_pc.env
python3 scripts/fake_navigation_terminal.py
```

终端 3，显式打开假导航和真实机构，再启动 UI：

```bash
source config/industrial_pc.env
export GRAIN_SAMPLING_UI_FAKE_NAVIGATION=1
export GRAIN_SAMPLING_UI_ENABLE_MECHANISM=1
python3 scripts/start_ui.py
```

如果 `/run/grain-robot/fake_navigation.sock` 无写权限，给终端 2 和 3 同时设置 `GRAIN_FAKE_NAV_SOCKET` 为一个仅当前用户可访问的绝对路径。终端会将套接字权限设为 `0600`，且不会覆盖已存在的路径。

## 操作流程

1. 在 UI 创建本地任务并点击开始。选“跳过地图”不是必需条件；假导航模式本身不依赖联想定位。
2. UI 显示“假导航联调：等待人工确认”时，终端 2 打印目标坐标。确认底盘静止、车已人工放在安全采样位后按 Enter；输入 `n` 拒绝。
3. 进入“已到达指定点位”后，继续按照 UI 提示进行接管、下压、排废粮、吸粮、输送和三仓动作。多点位任务每次导航都会再等待一次人工确认。
4. 最后的“返回起点”也只会在终端等待人工确认，车不会自动返航。需先人工处理车的位置，再按 Enter。

终端未启动、拒绝、断开或 300 秒内未确认时，UI 不会进入机构阶段。假导航仅替换导航完成信号；真实机构仍由当前守护进程执行。停止联调时先停 UI 任务/机构，再关闭假导航终端；恢复生产运行前取消 `GRAIN_SAMPLING_UI_FAKE_NAVIGATION` 和 `GRAIN_SAMPLING_UI_ENABLE_MECHANISM` 的联调设置。

旧 ROS 假导航脚本通过 `/waypoint_task_done` 话题工作，不适用于当前非 ROS 的 3588 运行入口。
