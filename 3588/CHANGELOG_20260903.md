# 变更记录 — 2026-09-03

> 粮食扦样机器人（Orange Pi 5 Max）导航/遥控/UI 卡顿排查与一键启动固化。

## 一、问题现象

1. **遥控不流畅**：手动档推杆时底盘一顿一顿 / 甚至无反应。
2. **导航开环走偏**：发 0.5m 目标点，车往反方向一路开、停不下来。
3. **UI 进程 100% CPU**：Qt 界面近乎冻结。

## 二、根因与修复

### 1. 遥控卡顿（三层叠加）
| 层 | 根因 | 修复 | 文件 |
|---|---|---|---|
| 双控制器抢发 | UI 内建遥控与独立 `rc_node` 同时以 10Hz 发布 `/cmd_vel`，指令互相打架 | 检测到独立 `rc_control_node`（或 `GRAIN_SAMPLING_UI_RC_PUBLISH=0`）时 UI 不发布，仅镜像 | `src/grain_sampling_ui/main.py` |
| UI 忙等采样锁 GIL | UI 自建 `RCReceiver` 忙等采样线程占满单核、锁死进程 GIL | UI 在独立 rc_node 模式下**不自建接收机**，改订阅 `/rc_mode` 镜像 | `main.py`、`src/grain_sampling_ui/ros_thread.py` |
| goal_controller 空闲 spam stop | `WAIT_GOAL` 空闲态每个 tick 都 `send_stop()`，与 rc forward 命令在同一 UDP daemon 上打架 | 新增 `_stop_once()`，只在状态切换那一刻停一次，空闲不碰电机 | `dipan/goal_controller.py` |

### 2. 导航走偏/停不下来
- **根因**：S-FAST_LIO 建图启动初期航向可能临时偏 180°（直线运动中航向弱可观测，随后自校正）。此前按 `yaw+π` 硬编码修正，自校正后反而加反，控制器以为在靠近目标、实际在远离 → 永不到达。
- **修复**：`heading_offset` 改为可配置参数、**默认 0**（与稳定后里程计一致）。启动后建议先遥控小动一下、确认 `/Odometry` 航向稳定再下导航点。

### 3. goal_controller 力度默认值
- 原 `far_forward=0.15 / near_forward=0.10` 太弱，克服不了履带静摩擦，2 秒内被 stuck 检测误报 FAULT。
- 调优为：`far_forward=0.55`、`near_forward=0.40`、`max_effort_step=0.15`、`stuck_min_motion=0.03`。

### 4. UI 点云渲染可关
- `/cloud_registered_body` 实时转 2D 点云图开销大；新增开关 `GRAIN_SAMPLING_UI_POINTCLOUD=0` 关闭（默认关闭，需要时设 1）。

### 5. 雷达网口路由持久化
- Livox MID360 走网口（`192.168.1.116`），板端 eth 需到 `192.168.1.0/24` 的路由。
- 新建 systemd 服务 `livox-route.service`（幂等 `ip route replace`），开机自启，重启不丢。

## 三、启动方式

```bash
bash ~/grain_sampling_robot_software/scripts/start_all_full.sh
```

一键拉起：雷达路由 → GPIO → roscore → Livox 驱动 → S-FAST_LIO 建图 → mechanism_node → rc_node → esc-pwm → motor_driver daemon → cmd_vel_to_motor → stop_on_obstacle → goal_controller → UI。
脚本幂等，已在跑的自动跳过；自动带 `GRAIN_SAMPLING_UI_RC_PUBLISH=0` 与 `GRAIN_SAMPLING_UI_POINTCLOUD=0`。

## 四、验证结果（实测）

- 满配下遥控顺畅，`/cmd_vel` 单一来源（rc_node）10Hz、无抖动。
- 导航 0.5m 目标点闭环：`APPROACH → ARRIVED`，到位误差 0.105m、航向误差 -1.7°，自动停车。
- 避障联动：`OBSTACLE_STOP` 自动刹停，移开障碍自动恢复。
- UI CPU 100% → ~22%；窗口可见、ROS 已连。
- 本地/板端 5 个改动文件 md5 一致；`py_compile` 全通过。

## 五、遗留说明

1. 建图启动后头几秒 FAST-LIO 航向可能临时偏 180°，自校正后恢复；导航前建议先遥控小动确认航向。
2. 旧地图已备份到 `fastlio2_ws/src/S-FAST_LIO/PCD/backup_20260903/`；正式导航需在作业区重录地图（当前 PCD 目录已清空，UI 不会误触发自动重定位）。
