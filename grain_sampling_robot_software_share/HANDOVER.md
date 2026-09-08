# 粮食扦样机器人 — 技术交接说明

> 更新日期：2026-09-08　|　青赋驭境科技有限公司

## 1. 项目概述

粮食扦样机器人全栈系统，实现：遥控/导航移动底盘 + SLAM 建图 + 多点扦样（1m/3m/6m 深度）+ UI 控制。

## 2. 硬件平台

| 部件 | 型号 / 说明 |
|---|---|
| 主控板 | Orange Pi 5 Max，IP `192.168.43.60`，账号 `orangepi` / `orangepi` |
| 激光雷达 | Livox MID360（**网口**连接，非 USB） |
| 底盘 | `dipan/`（motor_driver + goal_controller） |
| 扦样升降 | X2P 伺服（USB-RS485 Modbus RTU，FTDI FT231X，`/dev/x2p_lift`，导程 5mm） |
| 多通道执行器 | PCA9685（16 通道 PWM，`/dev/i2c-2`，0x40，50Hz） |
| 扦样管 | 第 1 节 40cm（夹持在中间），后续每节 1m |

## 3. 软件架构（ROS Noetic）

```
src/
  grain_sampling_ui/         UI 界面（状态机：idle/relocalize/mapping）
  grain_sampling_workflow/   mechanism_node（机构服务）、slam_bridge（SLAM 进程管理）
  grain_sampling_devices/    mechanism_driver（PCA9685）、x2p_lift（X2P 伺服）、rc_receiver（遥控）
  grain_sampling_camera/     相机/流媒体
  grain_sampling_cloud/      云端通信
  grain_sampling_pointcloud/ 点云采集/上传
ros/mechanism_node/          ROS 服务定义（MoveLift.srv / SetGrain.srv）
dipan/                       底盘（motor_driver、goal_controller、pca9685）
scripts/                     启动/部署/调试脚本
sampling_params.py           统一标定参数（唯一数据源）
```

## 4. PCA9685 通道映射与标定

| 通道 | 功能 | 标定 |
|---|---|---|
| CH0/1 | 螺旋输送 | 开=1200us，关=断电释放 |
| CH2/3/4 | 三仓（浅/中/深） | 开=1200us，关=1800us |
| CH5 | 夹紧 | 夹=**2000us**（代码 CLAMP_PULSE_CLOSE=1900，口述 2000，待统一）/ 松=1200us |
| CH6 | 拧紧 | 紧=1300us，松=1900us |
| CH7 | 负压风机 | 未接线（占位） |

时长（DEFAULT_GRAIN_PARAMS）：夹紧 2s / 松开 1s / 开仓 5s / 关仓 3s / 输送 120s / 拧紧 10s。

## 5. X2P 伺服升降参数

| 项 | 值 |
|---|---|
| 端口 | `/dev/x2p_lift`（udev 稳定符号链接，会随 FTDI 重枚举变 ttyUSBx） |
| 协议 | Modbus RTU，从站 2，9600 波特 |
| 导程 | 5mm |
| 转速 | `X2P_RPM = 1000`（sampling_params.py） |
| 位置容差 | `tolerance_mm = 15.0`（mechanism_driver.py） |

## 6. 扦样下压流程（sampling_press.py）

1. 夹紧扦样管（CH5 2000us 保持 1s 断电）
2. 周期运动下压：下 3cm → 上 2cm（净 1cm/周期），周期间停顿 1s（`LIFT_PAUSE_S` 可调）
3. 段内累计净下压 20cm 后：松开（CH5 1200us 0.5s）→ 伺服单独回位 → 再夹紧
4. 重复直到目标深度（第 1 节 40cm → 加管 → 继续到 1m/3m/6m）

## 7. 启动方式

```bash
# 全套启动（10 组件，幂等；已 source mechanism_ws + unset QT_QPA_PLATFORM）
bash scripts/start_all_full.sh

# 单独启动扦样机构（move_lift/set_grain 服务依赖 ~/mechanism_ws）
bash scripts/start_mechanism.sh

# 单通道手动控制（SSH 调试用）
python3 scripts/ch_control.py <通道0-15> <脉宽us|off|init|read>
```

## 8. 当前状态（2026-09-08）

| 子系统 | 状态 |
|---|---|
| UI + SLAM 状态机 | ✅ 完成（默认 idle，选任务才重定位，点开始才建图） |
| 导航 | ✅ 测试通过（点1/点2 ARRIVED，误差 ~0.12m） |
| 底盘 + 遥控 | ✅ 顺滑 |
| 扦样机构 | ⚠️ 位置误差 -19.9mm（up 方向过冲超容差 15mm），待现场排查 |
| 里程计 | ❌ /Odometry 无数据（暂搁置） |

## 9. 已知问题与待办

1. **位置误差 -19.9mm**：sampling_press 周期运动时 up 2cm 过冲约 20mm（down 正常）。疑似升/降负载不对称或停止刹车惯性，需现场复测调容差/减速。
2. **USB 串口断连**：FTDI FT231X 连续快速往复时整芯片掉电重连（ttyUSB0→ttyUSB1）。已通过周期停顿 + move_lift 失败自动重连双重缓解，根因（供电/振动）未根治。
3. **扦样全流程**：1m/3m/6m 深度完整测试未完成（目前只测到 40cm 第 1 节入口）。
4. **CH5 夹紧脉宽**：代码 1900us vs 口述 2000us 未统一。
5. **里程计恢复**：goal_controller FAULT:odometry_timeout。

## 10. 板端部署要点

- 代码位置：板端 `/home/orangepi/grain_sampling_robot_software`，本地 `E:\青赋驭境\项目\粮食扦样\grain_sampling_robot_software`
- 同步：`pscp` 上传（`scripts/sync_board.bat`），板端 IP `192.168.43.60`
- X2P 依赖：`~/mechanism_ws`（ROS 工作空间，含 MoveLift.srv/SetGrain.srv），启动前必须 `source ~/mechanism_ws/devel/setup.bash`
- x2p 包：板端 `src/x2p`（dais516，USB-RS485 Modbus RTU），依赖 pyserial
