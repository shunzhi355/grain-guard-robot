# 3588 假导航、真实机构台架联调

此模式用于“人工将车停在采样位 → UI 正常创建任务 → 终端人工确认假到位 → 真实机构/三仓流程”。默认关闭，不属于生产导航方案。联想主机不需要启动，假导航终端与 UI 均运行在 3588。它不会发 GRICP 速度、PWM 或底盘运动许可。

## 前置安全条件

1. 首次测试架空履带，清空机构周围人员与障碍物，准备可用的物理急停。
2. STM32 底盘串口在线、遥控器处于自动档、底盘未获运动许可、无急停/故障；机构动作前守护进程还会复核底盘状态。
3. X2P、PCA9685、电调和机构传感器按实机配置；不要在无人值守时开启真实机构。
4. 3588 服务端仍需本机固定 IP 和双向 TLS 证书以正常启动，但联想客户端不必连接。终端与 UI 须能访问同一个 Unix Socket 路径。

## 三个终端的启动顺序

### 一键启动（推荐）

在 3588 的桌面终端中执行一次：

```bash
cd "/home/neardi/project/grain guard robot/grain-guard-robot/3588"
bash scripts/start_fake_navigation_bench.sh
```

脚本会先停止同一项目的旧联调进程（UI、假导航终端和本机守护进程），保留历史日志与工单数据，再创建仅本机回环使用的临时 TLS 证书、启动无 ROS 守护进程、打开新终端和 UI，并把本次日志保存到 `log/bench/`。对于已停止的本地测试，即使旧 UI 窗口仍在，也会在静止检查通过后归档残留工单至 `~/.grain_robot/abandoned_bench_tasks/` 并切换会话；旧 UI 窗口已关闭且停在“已到位等待”时也可自动收尾。只有日志明确表明“机构动作前因遥控/串口条件失败，软件因此触发急停”，且遥控恢复有效自动档、故障码为 0、底盘无运动授权时，才通过守护进程的保护接口自动清除这一类急停，不伪造机械复位确认。机构动作中、回程不完整、其他急停来源或状态无法核实时，脚本拒绝自动清理；旧生产服务或其他程序占用硬件串口时也不会抢占。它不会自动开始任务、确认假到位或执行机械复位。关闭 UI 前应先安全结束任务；UI 退出后脚本会停止自己启动的进程。

如果工控机重启导致旧进程消失而只留下工单文件，脚本仅在最近一轮 UI 日志明确记录本地任务于机构动作前进入 `STOPPED` 时自动归档该文件。已经进入机构动作的任务不会被自动抹去。启动脚本能打开服务和 UI，但遥控无效、STM32 故障、串口离线或未完成机械复位仍会阻止真实测试，必须先解决对应现场故障。

当前一键联调脚本默认升降转速为现场指定的 **800 r/min**；可用 `--rpm 30` 等参数降低本轮速度。它不改变生产流程默认的 30 r/min 指令，也不会把 800 写进驱动器永久参数。开始任何真实动作前，须按设备铭牌、机构负载与实际起点确认适用，保持现场监护。脚本会显示 STM32 遥控状态；如果不是有效自动档、故障码不为 0 或急停锁定，UI 虽可打开，真实任务仍会被安全检查拦截。当前手动拆分三终端的方式保留如下，便于单独排障。

### DRV8701E 夹爪单项验证（首次接线后）

先把本次代码同步到 3588，在板端执行 `python3 -m py_compile sampling_params.py src/grain_sampling_devices/mechanism_driver.py`，确认服务能导入。按上述方式启动联调会话，**暂不在 UI 开始任务**。在 `3588/` 的另一个终端执行：

```bash
source config/industrial_pc.env
export PYTHONPATH="$PWD/src:$PWD${PYTHONPATH:+:$PYTHONPATH}"
python3 - <<'PY'
from grain_sampling_workflow.robot_bridge import RobotClient
s = RobotClient().request("status")["chassis"]
print(s)
assert s["chassis_link"] == "online" and s["rc_mode"] == "auto"
assert s["motion_armed"] is False and s["estop_latched"] is False
assert s["faults"] == 0
PY
```

确认夹爪周围安全、可用物理急停后，分别发送夹紧和松开；每次等待动作自动停止并观察方向，不能在夹爪仍运动时切换：

```bash
python3 -c 'from grain_sampling_workflow.robot_bridge import RobotClient; RobotClient().request("mechanism", name="clamp")'
# 观察：PH/CH8 低，EN/CH9 为 50 Hz、约 9.5% PWM，NS/CH10 高约 2 秒后变低。
python3 -c 'from grain_sampling_workflow.robot_bridge import RobotClient; RobotClient().request("mechanism", name="unclamp")'
# 观察：PH/CH8 高，EN/CH9 为 50 Hz、约 6% PWM，NS/CH10 高约 5 秒后变低。
```

命令返回只表示动作已启动；必须按上述时长等待并观察 CH10 回低。若方向相反、持续转动、驱动报警或不能保持夹紧，停止联调并检查接线及机构，再进入下述 UI 全流程。EN 在待机时仍是 PWM，由 NS 低电平使驱动休眠。旧 CH5 脉宽只是 EN 起测占空比的换算依据，实际速度须现场标定。

在 `3588/` 目录执行。按现场部署配置加载 `config/industrial_pc.env`；不要把假导航环境变量写入生产 service。

终端 1，启动无 ROS 的 3588 守护进程：

```bash
source config/industrial_pc.env
bash scripts/start_industrial_pc.sh
```

仅在已完成机械复位、现场监护并确认无故障时，可为一次受控测试在终端 1 启动前设置 `export GRAIN_LIFT_RPM=800`，将往复下压的目标转速从默认 30 r/min 提高到 800 r/min。该覆盖只对本次守护进程有效，代码限制在 1–800 r/min，并同步设定本轮 X2P 位置控制的软件上限；不要把它写入生产 service。启动后要核对守护进程日志中的速度覆盖提示和 X2P 实测峰值转速。高速短行程可能受 500 ms 加减速时间限制，实际动作时间与位置误差须现场核查。800 r/min 高于参数文件中记录的额定转速，必须由现场按设备铭牌、减速机构和负载条件确认。

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

## 2026-09-30 300 r/min 实机结果

首根管的 20 cm 往复下压与回程完成，实测峰值 300 r/min。第二根管下压时，第三段到位后在第四段回退的使能检查处出现 X2P `E37`（本项目 X2P 手册为“霍尔接线错误”）；流程进入 `STOPPED`，自动安全停机锁定。不能据此单独判定转速是根因。在现场确认伺服轴完全静止、检查霍尔/电机线束与接插件、查明报警原因并完成机械复位前，不得重复 300 r/min 测试或清锁继续旧任务。
