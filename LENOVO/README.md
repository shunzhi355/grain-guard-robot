# 联想导航主机代码

## 联想直连完整取样流程（当前迁移入口）

新增 `scripts/local_robot.py`：联想本机 UI → 本机 Unix Socket → STM32 USB 串口 / X2P USB-RS485，不运行 3588 服务、不使用 GRICP/TLS、不初始化 PCA9685。复用原状态机、取样编排和升降标定，支持多点、多深度三仓、加管、取管及返航阶段。

```bash
python LENOVO/scripts/local_robot.py inventory
python LENOVO/scripts/local_robot.py check
python LENOVO/scripts/local_robot.py workflow-sim --runtime-dir .codex-tmp/local-flow --report .codex-tmp/local-flow/report.json
```

实机模式必须明确指定两个稳定 USB 设备身份并通过现场确认；默认配置不能驱动硬件。导航有“人工静止到位确认”和“本机 ROS1 点位导航”两种显式模式，前者不代表自主导航验收。见 [联想单机部署与完整流程验证](docs/LOCAL_RUNTIME.md)。

## 单机迁移可行性验证（新增，尚未切换生产部署）

当前增加了 **仅代码与模拟测试** 的验证入口：

```bash
python -m pip install -r LENOVO/requirements-bench.txt
python LENOVO/scripts/bench_hardware.py --simulate --cycles 20 --report .codex-tmp/lenovo-bench.json
python -m pytest -o addopts= -q LENOVO/test/test_hardware_bench.py
```

请从仓库根目录执行。验证复用 `3588/` 的 STM32 串口、底盘授权/超时停车及 X2P 驱动，新增机构适配不初始化 PCA9685。CLI 不接受真实串口或实机执行参数，也不会连接 `192.168.1.200`。报告文件必须使用新名字。

此入口不是完整的 UI/Nav2 单机部署包；模拟通过不代表全部机械动作、联想系统兼容性或 USB-RS485 稳定性已经通过。实机两项验收通过后才决定全迁；否则保留 3588 硬件守护进程。详见 [硬件迁移验证说明](docs/HARDWARE_FEASIBILITY.md)。

## 原双机部署基线（保留作为回退方案）

联想主机负责 MID360 驱动、SLAM/重定位、地图管理、路径规划、Nav2、障碍检测和速度生成。3588 是 GRICP 服务端，联想是客户端；联想不直接操作底盘串口、PCA9685、X2P 或电调 PWM。

当前已从原工程迁入：`deploy/mapping/`、`config/livox_config.json`、`src/grain_sampling_pointcloud/`、雷达探测及建图维护脚本、旧障碍检测节点、旧 SLAM 进程管理器与旧目标控制器。`src/grain_sampling_interhost/protocol.py` 是与3588一致的 GRICP v1 帧格式副本。

当前文件主要是 ROS1 历史实现和迁移基线；ROS2/Nav2 与 GRICP 的联想侧运行程序尚未实现。详见 [TASKS.md](TASKS.md) 与根目录 [双机通信协议.md](../双机通信协议.md)。因此此目录目前不能作为联想主机的完整导航部署包。
