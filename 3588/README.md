# LPA3588 工控机代码

3588 负责本地 UI、工单与扦样状态机、PCA9685/X2P 机构、STM32 底盘串口、GRICP 服务端及停车安全逻辑。导航、SLAM、MID360 和速度规划由 `../LENOVO/` 负责。

## 当前运行入口

- `scripts/start_industrial_pc.sh`：前台启动 `grain_sampling_interhost.server`，无需 `roscore`。
- `scripts/start_ui.py`：启动 PySide UI，通过 `/run/grain-robot/control.sock` 访问本机守护进程。
- `config/industrial_pc.env`：底盘串口、PCA9685、X2P、固定 IP、双向 TLS 证书及本机 Socket 路径。
- `scripts/grain-sampling.service`：systemd 服务模板，`scripts/install-service.sh` 按安装目录生成正式服务。

速度链：`GRICP MOTION_COMMAND → ChassisController → ChassisSerial.stream_effort → STM32`。3588 只接收 `vx/wz` 物理速度，`vy` 必须为零；STM32 继续负责遥控优先、PWM 和自身断流保护。3588 本地运动指令超时为 200 ms。

机构链：UI/工作流 → 本机 Socket → `MechanismRuntime` → 原 PCA9685/X2P，同时复用底盘串口发送 STM32 `0x36` 机构动作帧。机构动作只在底盘未授权运动时放行。急停同时锁定底盘与机构；仅本机显式 `clear_estop` 请求可复位。编号、关仓语义和 MCU 本地时序限制见[机构串口接入](docs/MECHANISM_SERIAL.md)。

STM32 串口采用独立收发：TX 只发指令，不握手、不等 ACK 或动作完成回传；RX 由独立线程持续接收单片机主动发送的 CH8 自动／手动状态。急停复位同样发送后返回。状态超时显示“未知”并禁止自动动作，不因缺少应答反复重开串口；实际 I/O 故障仍会重连。原帧格式和 200 ms 运动指令超时保护保留。

GRICP 协议见根目录 [双机通信协议.md](../双机通信协议.md)。TLS 证书与实机串口/I2C 设备必须现场配置；本仓库没有默认生产证书。首次上电前必须架空履带测试手动优先、断网、串口断开、超时与急停。

不接联想主机、只测 3588 后续扦样流程时，使用默认关闭的[假导航联调模式](docs/FAKE_NAVIGATION_BENCH_TEST.md)。新终端程序不依赖 ROS，不发送任何底盘速度；每个采样点和返航都须人工按 Enter 确认。旧 `legacy_ros/scripts/fake_navigation_events.py` 不能用于此运行入口。

`legacy_ros/` 保留旧启动脚本、ROS 服务定义和旧底盘输出实现用于追溯；生产服务不调用它。`src/grain_sampling_workflow/` 中的若干旧 ROS1 包装模块仍为历史测试保留，当前 UI 与守护进程入口不导入它们。

## 尚未完成的联调

联想侧 ROS2/Nav2 的 GRICP 客户端、地图下载与云端点云切片上传、TLS 证书部署及实车验收仍需完成。当前地图上传按钮会明确提示该功能应由联想地图服务执行，不会读取3588本地旧 PCD。
