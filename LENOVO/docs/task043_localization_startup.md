# task043 定位启动与恢复

沿用 `bash LENOVO/scripts/start_mapping_ui.sh --task043-gate start`。
启动成功只表示服务可用，不能等同于定位通过或运动已授权。

## 启动位置要求

当前 Golden `laserMapping_re.cpp` 从 `mapping/init_pos` 和
`mapping/init_rot` 初始化滤波器状态。当前冻结配置是 [0,0,0] 和
[0,0,0,1]，未发现全局任意位置自动重定位入口。

最简办法：每次冷启动回到本地图的建图起点，保持建图时朝向和雷达安装条件。
不是一定要物理位置分毫不差；但目前没有实测的可接受偏差范围，不能承诺
离原点几米或旋转任意角度都能初始化。

若在其他位置启动，必须给出与本地图一致的可信初始 xyz 和 yaw。
这里是 Golden 原生 body/IMU 状态参考点，不是下游经过 B1 补偿后的车体中心。
不要把坏定位最后一帧当作初始位置，不要再次加 B1 安装偏移。
只要求终点 XY 到达，不代表初始化 yaw 可以随意。

## 一次明确确认

仅当车已静止在原建图起点、方向和安装条件一致时：

```bash
cd /home/ps02/grain_robot/staging/lenovo-local-20261010
bash LENOVO/scripts/task043_initialize_localization.sh --map-origin-confirmed
```

其他位置使用 `--pose-confirmed X Y Z YAW_RAD`，数值必须来自可信地图参考。
这一步不清除底盘锁存、不授权运动、不恢复旧目标。确认仅限当前主机启动和
当前地图，5 分钟内有效，使用后归档；再次初始化需要新的确认。

## 自动启动步骤

1. 复用原容器、原驱动、Golden 二进制及只读地图。先检查 ROS Master 和
   Livox 接收端口不存在其他占用，再建立唯一接收者。
2. 校验 manifest、两份 PCD、冻结配置和二进制 SHA256。
   manifest 中的源代码身份与二进制身份分别记录，不互相冒充。
3. 等待原始 LiDAR/IMU 连续稳定 3 秒，核对消息帧、有效值、时效、频率和
   单调时间戳。当前驱动原始加速度按 g 判断，不改 Golden 内部单位转换。
4. 没有本次初始位姿确认时停在 `WAIT_INITIAL_POSE`。
5. 通过独立生成的 launch 覆盖本次初始状态，冻结配置和旧地图不变。
6. 保持静止，30 秒预热后再评估 30 秒，沿用 042 R2 限值：xy 跨度各
   <=0.08 m，首尾 xy<=0.05 m，yaw 跨度<=0.04 rad，步进速度 p95<=0.15 m/s，
   最大单步<=0.03 m，最大间隔<=0.5 s。
7. 启动通过后继续监控输入、Odometry 时效/跳变、地图边界、进程和异常日志。
   启动健康心跳缺失、过期或跨地图时，导航适配器不能判定定位有效。

日志保存在 task-owned runtime 的 `startup_<时间>_<随机ID>/`，状态是
`/task043/out/localization_startup_status.json` 及 ROS String topic
`/task043/localization_startup_status`。

## 有限恢复

启动阶段只有原始输入中断，且输入重新稳定、初始确认仍有效时，允许重试
Golden 一次；保持原 Livox/ROS Master，不重复启动传感器接收者。
地图不符、发散、R2 不通过、持续 No Effective Points 或 VoxelGrid 异常直接
`BLOCKED`，保留证据并停止本次 Golden 子进程。启动后失效不盲目重启。

恢复物理条件和已知初始位姿后，再执行上面的明确确认命令，可以开始新的
定位验证。不会自动恢复旧导航任务。输入健康和 R2 仅是准入证据，匹配点数
当前未接入，绝对定位准确度及停车精度仍需要现场独立验证。

## 2026-10-14 运动阶段输入门控修正

`READY` 前保留原静态前提：IMU 原始加速度模长 0.8–1.2 g、角速度不超过
0.08 rad/s，随后进行 30 秒预热和 30 秒 R2 检查。
`READY` 后，平移加速度与转动本身不等同于传感器断流；不再用静态重力
模长或静止角速度要求判断运动输入。仍严格检查消息帧、有限值、时间戳
新鲜度和顺序、LiDAR/IMU 频率、数据间隔、发布者唯一性、Odometry 跳变、
地图边界与 Golden 匹配异常。失效后继续锁止，不自动恢复旧目标。

状态 JSON 新增 `raw_streams` 和 `fault_diagnostics`。故障首次出现时单独
保存 `localization_fault_<时间戳>.json`，冻结当时的具体流、拒绝原因、频率、
时间差、输入摘要及发布者身份；后续原始数据恢复不会覆盖该证据。
发布者身份冲突单独报 `publisher_ownership_conflict`，不再冒充断流。

此前失败仅记录通用 `raw_stream_interrupted`，触发帧没有留存；因此静态
条件误用于运动的代码缺陷已经复现，但不把未记录的历史帧认定为根因。
修复后的实机运动跟踪需以人工推车再次验收。当前车已离开起点时，
必须先回到对应活动地图起点并静止确认，不能用过期末帧或默认零位初始化。
