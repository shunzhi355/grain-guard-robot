# LPA3588（临滴工控机）任务清单

> 目标架构：联想主机负责 MID360、SLAM/重定位和 Nav2；LPA3588 负责 UI、任务流程、扦样机构、底盘安全网关和 STM32 通信。  
> 双机协议：[LENOVO_LPA3588_COMM_PROTOCOL.md](docs/LENOVO_LPA3588_COMM_PROTOCOL.md)  
> 文档状态：待实现/待实机验收

## 结论

LPA3588 在拆分后的核心职责不是做路径规划，而是做机器人上的**业务主机与安全执行网关**：

```text
联想 Nav2/S-FAST_LIO
        |
        | GRICP（目标、状态、vx/wz）
        v
LPA3588 安全网关
        |
        | ChassisController + ChassisSerial（USB-TTL 串口唯一占用者）
        v
STM32（遥控优先、PWM、300 ms 看门狗）
```

LPA3588 必须在联想掉线、网络延迟、旧指令重放、手动遥控接管、急停和 STM32 链路故障时，不依赖联想主机的进一步命令就能安全停车。

## 一、职责边界

### 3588 继续保留

- [ ] PySide6 触摸屏 UI。
- [ ] 云端工单、MQTT/HTTP 和本地任务缓存。
- [ ] 扦样工作流状态机。
- [ ] PCA9685 扦样机构输出。
- [ ] X2P 升降伺服和 `/dev/ttyS0`。
- [ ] STM32 底盘串口和非 ROS 的底盘控制器。
- [ ] 物理急停、软件急停、遥控模式和底盘故障状态。
- [ ] 联想—3588 GRICP 服务端、非 ROS 本地业务接口和 200 ms 运动看门狗。

### 从 3588 迁出

- [ ] Livox MID360 驱动。
- [ ] S-FAST_LIO 建图和重定位进程。
- [ ] Nav2/路径规划/局部控制。
- [ ] 高频点云障碍处理。
- [ ] 导航过程中的物理速度计算。
- [ ] 3588 生产运行时的 `roscore`、`rospy` 和 ROS1 话题/服务适配层。

### 3588 不允许做的事

- [ ] 不同时启动本地 `goal_controller` 和远端 Nav2 输出。
- [ ] 不将 GRICP 速度值直接当成 PWM。
- [ ] 不因收到心跳而续期旧速度。
- [ ] 不因网络重连自动恢复旧导航。
- [ ] 不允许联想主机解除本地急停。
- [ ] 不在一个进程中混用物理速度和归一化力度。
- [ ] 不让 UI、GRICP 网关和其他进程同时打开 STM32 底盘串口。

## 二、开发任务

### S01．确认实机接线和软件基线（P0）

- [ ] 确认底盘最终执行链路是 `3588 USB-TTL → STM32 USART2 → 底盘 PWM`。
- [ ] 确认 PCA9685 只用于扦样机构，还是实机仍有底盘电调通道。
- [ ] 核对 STM32 固件版本，确认已支持 `STREAM_EFFORT`、`STREAM_CONTROL` 和档位回传。
- [ ] 核对 `CHASSIS_SERIAL_PORT`、`X2P_PORT`、`PCA9685_I2C_DEVICE`，确保互不冲突。
- [ ] 记录当前可回滚的 3588 代码 commit、STM32 HEX 和硬件配置。
- [ ] 架空履带，完成本地手动遥控、停车、急停和 STM32 300 ms 断流停车基线测试。

**交付物**

- 《3588 实机接线与软件基线表》。
- 可回滚的完整备份和版本号。

**验收**

- 联想主机不存在时，手动遥控和急停仍可独立工作。
- 实机接线与 `README.md`、`CHASSIS_SERIAL.md` 及启动配置一致。

### S02．建立双机控制网络（P0）

- [ ] 使用固定 IP，建议 3588=`192.168.50.1`、联想=`192.168.50.2`。
- [ ] 控制网段不使用 DHCP，不配默认网关。
- [ ] 确认不与 MID360 的 `192.168.1.0/24` 网段冲突。
- [ ] 关闭 3588 网口的节能、自动挂起和不必要的网络自动切换。
- [ ] 放行 TCP 29100、UDP 29101；如需地图下载，放行联想 TCP 29102。
- [ ] 防火墙只允许联想固定 IP 访问控制端口。
- [ ] 部署 Chrony/NTP，对时偏差要求小于 50 ms。
- [ ] 加入链路自检：接口存在、IP 正确、对端可达、时钟同步。

**验收**

- 连续 `ping` 2 小时无异常断链。
- 拔掉网线后 3588 不依赖 TCP 超时，通过运动看门狗在 200 ms 内发出停车。
- 恢复网线后只恢复通信，不恢复旧目标。

### S03．实现 GRICP 共享协议库（P0）

- [ ] 实现 32 字节帧头的编码和解码。
- [ ] 实现 big-endian 字段转换。
- [ ] 实现 CRC-32C，测试向量 `123456789 -> 0xE3069283`。
- [ ] 实现 UDP HMAC-SHA256 截断校验。
- [ ] 实现 `session_id`、`sequence`、`motion_epoch` 检查。
- [ ] 实现消息长度上限和 JSON schema 检查。
- [ ] 所有非法帧必须“丢弃+计数+限频日志”，不得引发运动。
- [ ] 双端共用同一组黄金测试向量，不各自凭理解手写格式。

**交付物**

- `gricp` Python 协议包。
- 帧字节级黄金样本。
- 单元测试和模糊测试用例。

### S04．实现 3588 GRICP 服务端（P0）

- [ ] 在 `0.0.0.0:29100` 或指定控制 IP 上启动 TCP/TLS 服务。
- [ ] 在 `0.0.0.0:29101` 上启动 UDP 速度接收。
- [ ] 只允许一个有效联想会话。
- [ ] 新会话建立前先执行本地停车和旧会话失效。
- [ ] 实现 `HELLO/HELLO_ACK`、版本协商和能力列表。
- [ ] 每次会话生成新的非零 `session_id` 和 32 字节 UDP 会话密钥。
- [ ] 实现 500 ms 心跳和 1500 ms 会话超时。
- [ ] 明确心跳不刷新运动指令时间。
- [ ] TCP 断开、TLS 失败、心跳超时时立即调用本地停车。
- [ ] 日志中不得输出私钥、证书私钥或 `udp_session_key`。

### S05．实现运动安全状态机（P0）

- [ ] 实现 `DISARMED -> ARMED_IDLE -> ACTIVE` 状态机。
- [ ] 实现独立的 `ESTOP_LATCHED` 锁定状态。
- [ ] 处理 `MOTION_ARM_REQUEST`时检查：
  - [ ] RC 数据有效。
  - [ ] CH8 处于自动档。
  - [ ] 遥控摇杆符合 STM32 起步中位要求。
  - [ ] 急停未锁定。
  - [ ] 无本地障碍停车。
  - [ ] STM32 底盘串口在线。
  - [ ] 当前没有其他自动速度源。
- [ ] 授权通过后生成新的非零 `motion_epoch`。
- [ ] 手动档、RC 失效、急停、障碍、串口断开、取消、断网和超时均必须立即使 `motion_epoch` 失效。
- [ ] 旧 `motion_epoch` 的非零指令必须拒绝。
- [ ] 当前会话认证通过的 `MOTION_STOP` 即使 epoch 不匹配也必须停车。
- [ ] 急停只能由 3588 本地符合条件的显式操作解除。

### S06．实现 UDP 速度接收与 200 ms 看门狗（P0）

- [ ] 只接受当前联想 IP、`session_id`、HMAC 和 `motion_epoch` 均有效的速度包。
- [ ] 严格检查新的 `sequence`，拒绝重复和乱序包。
- [ ] 检查 `timestamp_ms + valid_for_ms`，过期指令不执行。
- [ ] 检查 `linear_x_mm_s` 在 `-300..300`。
- [ ] 强制 `linear_y_mm_s == 0`，非零时整帧拒绝。
- [ ] 检查 `angular_z_mrad_s` 在 `-800..800`。
- [ ] 不对越界值做裁剪后执行，必须拒绝并停止当前授权。
- [ ] 只有“新的、完整验证通过的 `MOTION_COMMAND`”可刷新运动时间。
- [ ] 实现 200 ms 本地单调时钟看门狗。
- [ ] 超时后立即通过 `ChassisSerial.stream_control(AUTO_STOP)` 发停车，撤销 `motion_epoch`并取消目标。
- [ ] 看门狗线程不得被地图下载、UI 渲染、日志写入或 TCP JSON 处理阻塞。

**验收**

- TCP 心跳正常但 UDP 速度停止时，200 ms 内必须发出停车。
- 重放旧速度包不得使底盘运动或延长运动有效期。

### S07．实现非 ROS 底盘控制器并直连 STM32（P0）

- [ ] 新建常驻 `grain_robot_daemon`；由其中的 `ChassisController` 唯一占用底盘 USB-TTL 串口。
- [ ] 复用现有 `grain_sampling_devices.chassis_serial.ChassisSerial`，不再启动 `chassis_node`。
- [ ] 授权仅是 3588 本地 `MotionGuard` 状态，不能被解释为 STM32 已确认开始运动。
- [ ] 验证 GRICP 物理速度后转换为 STM32 归一化力度：
  - [ ] `forward = round(linear_x_mm_s / 300.0 * 1000)`。
  - [ ] `turn = round(angular_z_mrad_s / 800.0 * 1000)`。
  - [ ] `linear_y_mm_s` 必须为 `0`；两项结果再次检查在 `-1000..1000`。
- [ ] 调用 `ChassisSerial.stream_effort(forward, turn)`，不把网络数据直接解释为 PWM。
- [ ] `MOTION_STOP`、取消、超时和故障调用 `ChassisSerial.stream_control(AUTO_STOP)`。
- [ ] 全系统急停调用 `ChassisSerial.stream_control(ESTOP)`，同时调用机构急停。
- [ ] 守护进程正常退出和异常退出路径均先尽力停车并关闭串口。
- [ ] 保留 STM32 300 ms 运动断流、遥控优先、急停锁定和摇杆回中互锁。

### S08．实现底盘和遥控状态上报（P0）

- [ ] 通过 `ChassisSerial.poll_mode()` 和串口回包解析获得 `manual/auto/unknown`，不得依赖 `/rc_mode`。
- [ ] 由 `ChassisController` 维护串口在线、导航放行、最后回包时间和 STM32 回传状态。
- [ ] 订阅/集成本地急停和机构急停状态。
- [ ] 上报 `ROBOT_STATUS`，包含 RC 模式、RC 年龄、串口在线、授权、急停、障碍、最后速度年龄和故障。
- [ ] 状态变化立即发送，正常期间以 10 Hz 上报。
- [ ] 状态链路断开时上报未知/离线，不得继续使用最后一次的 `auto`。
- [ ] 本地停车事件上报 `SAFETY_EVENT/MOTION_EVENT`，但上报失败不得阻止已经开始的停车。

### S09．移除 3588 ROS1 运行依赖并实现本地业务接口（P0）

3588 生产模式不启动 `roscore`，UI、任务流程、底盘和机构通过普通 Python API 或本机 Unix Socket 与 `grain_robot_daemon` 通信。

- [ ] 用 `RobotBridge` 替换现有 `SamplingBridge/ros_bridge.py`，发送 `NAV_GOAL_REQUEST` 和 `NAV_CANCEL`。
- [ ] 用事件回调/本机 IPC 将 `NAV_STATUS`、`NAV_RESULT`、`POSE_STATUS`、`OBSTACLE_STATUS` 和 `ROBOT_STATUS` 分发给 UI 与任务状态机。
- [ ] 成功的 `NAV_RESULT` 直接触发工作流的“到位”事件，不再伪装成 `/waypoint_task_done`。
- [ ] 失败、取消、手动接管和超时必须向 UI/状态机提供明确原因。
- [ ] 用 `RobotEventThread` 替换 UI 的 `ros_thread.py`；地图、位姿和遥控模式均来自 GRICP/本地状态模型。
- [ ] 用本机 Unix Socket（建议 `/run/grain-robot/control.sock`）隔离 UI 与安全守护进程；IPC 断开不得影响安全看门狗。
- [ ] 远程建图/重定位改由 `SLAM_COMMAND` 完成，删除 UI 中 `rostopic` 和本机 ROS 进程管理调用。
- [ ] 生产启动配置不安装也不检查 `rospy`、ROS 消息包或 ROS Master。
- [ ] 旧 ROS1 包只作为回滚参考，不进入生产启动链。

### S10．改造 UI 导航与建图流程（P1）

- [ ] UI 增加“联想导航主机在线/离线”状态。
- [ ] UI 显示当前地图 ID、SLAM 模式、定位是否有效和里程计年龄。
- [ ] 建图、停止、保存、重定位按钮改为发送 `SLAM_COMMAND`。
- [ ] 不再使用 3588 上的 `pgrep/pkill` 管理联想进程。
- [ ] 地图列表来自 `MAP_LIST_RESPONSE`，不再直接枚举 3588 本地 PCD 目录。
- [ ] 根据 HTTPS 地图栅格/缩略图更新 UI 地图。
- [ ] 下载完成后校验文件长度和 SHA-256。
- [ ] 机器人运动时禁止或限速下载大型 PCD。
- [ ] 导航失败时显示可执行的错误，不只显示“失败”。
- [ ] 联想掉线不得导致 UI 中 PCA9685/X2P 机构手动维护页一起崩溃。

### S11．改造扦样任务状态机（P1）

- [ ] 工作流发送目标时生成唯一 `goal_id`。
- [ ] 工作流在收到 `NAV_RESULT=SUCCEEDED` 后才进入扦样机构流程。
- [ ] 继续保留最终到点距离小于等于 0.2 m 的校验。
- [ ] `FAILED/CANCELED/超时/手动接管` 不得自动进入扦样动作。
- [ ] 执行扦样机构前再次确认底盘已停车且没有活动 `motion_epoch`。
- [ ] 暂停、停止、急停首先本地停止底盘和机构，网络通知仅为后续状态同步。
- [ ] 重启 UI 不自动恢复中断前的导航和机构动作。

### S12．保留机构子系统本地独立性（P0）

- [ ] 从 `mechanism_node.py` 提取无 ROS 的 `MechanismController`，由工作流通过 Python API/本机 IPC 调用。
- [ ] PCA9685 仍使用 `/dev/i2c-2`/`0x40`，不通过联想主机中转。
- [ ] X2P 仍使用 3588 板载 RS485 `/dev/ttyS0`。
- [ ] 机构急停、软限位、串口错误不依赖联想在线。
- [ ] 联想网络故障不得占用 I2C、RS485 或阻塞机构看门狗。
- [ ] 完成导航网关改造后的 PCA9685/X2P 回归测试。

### S13．改造启动和停止脚本（P0）

在 `config/industrial_pc.env` 或新的双机配置中增加：

```bash
NAVIGATION_MODE=remote
GRICP_BIND_IP=192.168.50.1
GRICP_CONTROL_PORT=29100
GRICP_MOTION_PORT=29101
GRICP_ALLOWED_PEER=192.168.50.2
GRICP_MOTION_TIMEOUT_MS=200
GRICP_HEARTBEAT_TIMEOUT_MS=1500
GRICP_MAX_LINEAR_MPS=0.3
GRICP_MAX_ANGULAR_RPS=0.8
GRICP_TLS_CERT=/etc/grain-robot/tls/3588.crt
GRICP_TLS_KEY=/etc/grain-robot/tls/3588.key
GRICP_TLS_CA=/etc/grain-robot/tls/ca.crt
```

- [ ] `NAVIGATION_MODE=remote` 时只启动 `grain_robot_daemon`、UI 和云端/任务服务，不启动 `roscore`、`mechanism_node` 或 `chassis_node`。
- [ ] `NAVIGATION_MODE=remote` 时禁止启动本地 Livox、S-FAST_LIO、`goal_controller`、旧 `cmd_vel_to_motor`、旧 UDP motor daemon。
- [ ] 启动前检查底盘串口、PCA9685、X2P、网口、证书和端口占用。
- [ ] 启动检查发现底盘串口已被其他进程占用时拒绝启动并报错。
- [ ] 停止服务的顺序必须是：本地停车 → 撤销授权 → 停网关 → 停底盘串口 → 停其他节点。
- [ ] systemd 服务崩溃重启前必须保证 STM32 已回停车状态。

### S14．部署 TLS 证书和权限（P1）

- [ ] 建立项目专用 CA、3588 证书和联想证书。
- [ ] 3588 私钥只允许 GRICP 服务账号读取。
- [ ] 使用双向 TLS，拒绝未信任的客户端证书。
- [ ] 证书错误或过期时保持未授权和停车状态。
- [ ] 开发无认证模式只能在台架网络显式启用，生产启动脚本必须拒绝该模式。
- [ ] 编写证书更换和回滚流程。

### S15．日志、诊断和存储控制（P1）

- [ ] 记录会话建立/断开、对端版本、对端 IP 和 `session_id`，但不记密钥。
- [ ] 记录授权、撤销、停车、手动接管、急停和超时原因。
- [ ] 导出诊断计数：CRC/HMAC 错误、重复包、乱序包、过期包、越界包、看门狗停车次数。
- [ ] 记录最大速度包间隔和最后有效速度年龄。
- [ ] UI 提供“双机通信诊断”页或运维命令。
- [ ] 配置日志轮转、大小限制和磁盘剩余空间告警。
- [ ] 地图下载和日志压缩不得占用安全看门狗线程。

## 三、建议代码结构

```text
3588/
├─ src/grain_sampling_interhost/
│  ├─ __init__.py
│  ├─ protocol.py           # 帧编解码、CRC、HMAC、错误码
│  ├─ schemas.py            # TCP JSON schema
│  ├─ session.py            # TLS 会话、心跳、序号
│  ├─ motion_guard.py       # motion_epoch、200 ms 看门狗
│  ├─ chassis_controller.py # 速度换算、ChassisSerial、串口状态
│  ├─ mechanism_controller.py # PCA9685/X2P/三仓流程的非 ROS 接口
│  ├─ ipc_server.py         # UI/工作流本机 Unix Socket 接口
│  └─ server.py             # 3588 GRICP 与机器人守护进程入口
├─ src/grain_sampling_ui/
│  └─ robot_client.py       # UI 的非 ROS 本机 IPC 客户端
├─ config/interhost.env
├─ scripts/start_interhost_gateway.sh
├─ scripts/stop_interhost_gateway.sh
└─ test/test_interhost_*.py
```

实际实现可调整文件名，但协议、会话、运动安全、底盘串口、机构和 UI IPC 必须分层，不要把网络接收、UI 和串口输出写在一个循环中。

## 四、自动化测试任务

### 协议层

- [ ] 帧编码后再解码字节完全一致。
- [ ] CRC-32C 标准向量通过。
- [ ] HMAC 标准向量通过。
- [ ] TCP 拆包、粘包、空负载、超长负载和断开重连测试。
- [ ] UDP 坏 CRC、坏 HMAC、错 IP、错 session、错 epoch 测试。
- [ ] 重复、乱序、旧会话、过期包全部被拒绝。
- [ ] 任意字节 fuzz 不得导致进程崩溃或运动输出。

### 运动安全层

- [ ] 未授权时收到非零速度不执行。
- [ ] 手动档持续收到有效速度包仍不执行。
- [ ] 急停后持续收到非零包仍保持停车。
- [ ] 心跳正常、速度断流时 200 ms 看门狗生效。
- [ ] 速度正常、TCP 会话断开时立即停车并作废授权。
- [ ] 零速帧可保持导航中暂停，但不能解除任何故障。
- [ ] 越界、`vy != 0`、NaN/溢出转换和时钟失步测试。
- [ ] 新会话成功后重放旧会话指令不执行。
- [ ] 到点时先停车、后发布 `arrive`。

### UI/业务回归

- [ ] 原有 UI 可以选择工单、发送点位、取消和急停。
- [ ] `NAV_RESULT=SUCCEEDED` 能推动扦样状态机，其他结果不能误触发机构。
- [ ] `POSE_STATUS` 断流后 UI 显示离线/无效，不停留在旧位置假装实时。
- [ ] 3588 未安装/未启动 ROS 时，UI、底盘和扦样全流程仍能运行。
- [ ] 联想离线时机构手动维护功能不崩溃。
- [ ] PCA9685、X2P、粮仓、夹紧和输送现有测试全部回归。

## 五、台架与实车验收

| 编号 | 场景 | 合格条件 |
|---|---|---|
| A01 | 联想不启动 | 3588/STM32 保持停止；手动遥控独立可用 |
| A02 | 远端正常发 20 Hz 速度 | 3588 正确换算并调用 `stream_effort`，无明显抖动 |
| A03 | 拔掉双机网线 | 3588 在 200 ms 内发出停车，STM32 300 ms 保护仍有效 |
| A04 | 杀死联想导航进程 | 速度断流导致 3588 看门狗停车 |
| A05 | 杀死 GRICP 网关 | 进程退出前尽力停车；最终由 STM32 超时兜底 |
| A06 | 导航中切手动档 | STM32 手动立即获得最高优先级；3588 撤销远程授权 |
| A07 | 手动切回自动 | 旧目标不恢复，必须重新下发和授权 |
| A08 | 急停后持续收到速度 | 底盘和机构保持急停锁定 |
| A09 | 重放旧 session/epoch 速度包 | 不运动，不刷新 200 ms 超时 |
| A10 | 发送 `vy != 0` 或越界速度 | 整帧拒绝，不裁剪执行 |
| A11 | 定位失效 | 联想发停车；如未发，3588 仍因速度断流停车 |
| A12 | 障碍检测失效 | 按安全故障处理，不视为前方无障碍 |
| A13 | 正常到点 | 先停底盘，再进入扦样流程；距离误差不大于 0.2 m |
| A14 | 联想断线后操作机构 | 本地机构维护界面和急停可用 |
| A15 | 长时运行 | 连续 4 小时无双速度源、内存持续增长或日志占满磁盘 |

> 200 ms 和 300 ms 是通信断流后“发出停车输出”的上限，不代表机械车体必然在相同时间内完全静止。必须实测不同速度、负载和地面条件下的制动距离。

## 六、分阶段交付顺序

### M1．不接电机的协议联调

- S02、S03、S04、S08。
- 使用假联想客户端发送目标、状态和速度包。
- 验收帧格式、会话、心跳、断线和日志，不接入底盘输出。

### M2．非 ROS 本地运行时联调

- S05、S06、S07、S09。
- 将网络速度输出到模拟 `ChassisSerial`，不连接 STM32。
- 验收授权、超时、序号、epoch、速度换算和本机 IPC 状态。

### M3．架空履带联调

- 接入 `ChassisController` 和 STM32。
- 限制低速，先测零速、停车、手动接管和断网，后测前后/转向。
- 通过 A01–A12 后才允许落地。

### M4．UI/建图/任务流程联调

- S10、S11、S12、S13。
- 验收地图选择、远程重定位、多点导航和到点后扦样。

### M5．安全固化与长时验收

- S14、S15 和全部 A 类测试。
- 完成 TLS、证书权限、systemd、日志轮转、故障注入和 4 小时连续运行。

## 七、最终交付物

- [ ] 3588 GRICP 服务端源码。
- [ ] 共享协议库和黄金测试向量。
- [ ] 非 ROS 本地业务桥、底盘控制器和机器人守护进程。
- [ ] UI/工作流/建图管理改造。
- [ ] `industrial_pc.env` 双机配置模板。
- [ ] systemd 启停单元和安全停机脚本。
- [ ] TLS 证书生成与部署说明。
- [ ] 单元测试、集成测试、网络故障测试报告。
- [ ] 架空履带和实车验收记录。
- [ ] 一键回滚到拆分前版本的操作说明。
- [ ] 现场运维手册：断网、证书过期、底盘串口失效、遥控无效和急停恢复。
