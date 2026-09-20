# 粮食扦样机器人 — 技术交接说明

> 更新日期：2026-09-20　|　青赋驭境科技有限公司

## 1. 项目概述

粮食扦样机器人全栈系统，实现：遥控/导航移动底盘 + SLAM 建图 + 多点扦样（1m/3m/6m 深度）+ UI 控制。

## 2. 硬件平台

| 部件 | 型号 / 说明 |
|---|---|
| 主控板 | 临滴 NEARDI LPA3588 工控机（RK3588），板型 `neardi,lpb3588-linux-f0`，账号 `neardi` |
| 激光雷达 | Livox MID360（**网口**连接，非 USB） |
| 底盘 | `dipan/`（motor_driver + goal_controller） |
| 扦样升降 | X2P 伺服（Modbus RTU，导程 5mm）。3588 工控机走板载 RS485 `/dev/ttyS0`；早期 USB-RS485（FTDI FT231X）方案使用 `/dev/x2p_lift` |
| 多通道执行器 | PCA9685（16 通道 PWM，TP I2C4 → LPA3588 实机确认为 `/dev/i2c-2`，0x40，50Hz） |
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
| 端口 | `/dev/ttyS0`（LPA3588 板载 RS485，实机已确认）；早期 USB-RS485 方案为 `/dev/x2p_lift`（udev 稳定符号链接，会随 FTDI 重枚举变 ttyUSBx） |
| 协议 | Modbus RTU，从站 2，9600 波特 |
| 导程 | 5mm |
| 转速 | `X2P_RPM = 200`（sampling_params.py） |
| 位置容差 | `X2P_POSITION_TOLERANCE_MM = 2.0`（sampling_params.py） |
| 方向翻转 | `X2P_FORWARD_SIGN = 1` |
| 编码器 | 131072 计数/电机圈，**无断电保持多圈绝对编码器**：重新上电后计数从零开始，驱动器不知道机构停在哪里 |
| OFF 判据 | 用 Un058（`Register.SERVO_ENABLE_STATUS`，0=未使能）+ 实际转速判断。本机固件在未使能且静止时上报 `status=3`，按 `status==1` 判断 OFF 会误报互锁并阻止所有移动 |

### 5.1 手动调整初始位置

由于断电后编码器不保持位置，如果上电时机构不在物理最高点，自动流程的相对动作起点就是错的。此时先停下机构节点、释放串口，再用手动程序把机构挪回最高点：

```bash
sudo systemctl stop grain-sampling
cd "/home/neardi/project/grain guard robot/grain-guard-robot/grain_sampling_robot_software_share"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python3 scripts/manual_lift_adjust.py            # 交互模式，默认 /dev/ttyS0
python3 scripts/manual_lift_adjust.py up 0.5     # 单次：上升 0.5 cm
python3 scripts/manual_lift_adjust.py --status   # 只读位置
```

交互命令：`up 0.5` / `down 1.2` 移动，`s` 看位置，`m` 记物理最高点，`clear` 清除，`q` 退出。该程序直连串口，不依赖 ROS 和 `/mechanism/move_lift`，但**必须**先停掉 `grain-sampling`（串口独占）。调整完成后 `sudo systemctl start grain-sampling`。

完整使用文档见 `docs/X2P手动调整初始位置使用说明.md`（命令表、参数与环境变量、真实输出示例、常见故障排查）。

只读确认驱动器状态：

```bash
python3 -m x2p.cli --port /dev/ttyS0 status
python3 -m x2p.cli --port /dev/ttyS0 hybrid-status
```

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

## 8. 当前状态（2026-09-20）

| 子系统 | 状态 |
|---|---|
| UI + SLAM 状态机 | ✅ 完成（默认 idle，选任务才重定位，点开始才建图） |
| 导航 | ✅ 测试通过（点1/点2 ARRIVED，误差 ~0.12m） |
| 底盘 + 遥控 | ✅ 顺滑 |
| 扦样机构 | ⚠️ 早期位置误差 -19.9mm 记录基于旧容差 15mm；现容差已收到 `X2P_POSITION_TOLERANCE_MM = 2.0` 并加入低速接近段，待现场复测 |
| X2P 手动调整 | ✅ 新增 `scripts/manual_lift_adjust.py`（交互式直连串口，不依赖 ROS） |
| 里程计 | ❌ /Odometry 无数据（暂搁置） |

## 9. 已知问题与待办

1. **位置误差 -19.9mm**：sampling_press 周期运动时 up 2cm 过冲约 20mm（down 正常）。疑似升/降负载不对称或停止刹车惯性，需现场复测调容差/减速。
2. **USB 串口断连（历史）**：早期 FTDI FT231X 方案连续快速往复时整芯片掉电重连（ttyUSB0→ttyUSB1）。改用 LPA3588 板载 RS485 `/dev/ttyS0` 后该问题不再适用。
3. **扦样全流程**：1m/3m/6m 深度完整测试未完成（目前只测到 40cm 第 1 节入口）。
4. **CH5 夹紧脉宽**：代码 1900us vs 口述 2000us 未统一。
5. **里程计恢复**：goal_controller FAULT:odometry_timeout。
6. **无断电保持编码器**：X2P 不记忆断电前位置，上电后若机构不在物理最高点，需先用手动程序调整（§5.1）。
7. **无机械零点/软限位**：系统尚未实现回零，也没有全行程软限位；手动程序只做单次距离上限 + 可选的本次运行内软限位。

## 10. 板端部署要点

- 代码位置：板端 `/home/neardi/project/grain guard robot/grain-guard-robot/grain_sampling_robot_software_share`（登录用户 `neardi`）
- X2P 依赖：`~/mechanism_ws`（ROS 工作空间，含 MoveLift.srv/SetGrain.srv），启动前必须 `source ~/mechanism_ws/devel/setup.bash`
- ROS1 环境：**不要**写死 `/opt/ros/noetic/setup.bash`，该机没有这个路径。用 `scripts/start_industrial_pc.sh` 的探测逻辑，或直接 `source ~/mechanism_ws/devel/setup.bash`
- x2p 包：板端 `src/x2p`（dais516，Modbus RTU），依赖 pyserial
- 硬件配置：`config/industrial_pc.env`（`X2P_PORT=/dev/ttyS0`、`PCA9685_I2C_DEVICE=/dev/i2c-2`、`RC_SERIAL_PORT=/dev/rc_receiver`）
- 手动调整：`scripts/manual_lift_adjust.py`（交互式直连串口），见 §5.1
