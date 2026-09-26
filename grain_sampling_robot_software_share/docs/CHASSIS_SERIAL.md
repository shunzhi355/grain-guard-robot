# 工控机导航串口控制底盘

## 接线与配置

当前固件在仓库根目录 `dipan/`：STM32 USART1 接遥控器 i-BUS，
USART2 经 USB-TTL 与工控机 USB1 通信，参数为 **115200、8数据位、无校验、1停止位、无流控**。
协议以 `dipan/MDK-ARM/chassis/chassis.c` 为准：A5 5A 帧头、小端数据、CRC16-CCITT-FALSE。

`config/industrial_pc.env` 默认 `CHASSIS_BACKEND=serial`。
现场已识别 FT232（序列号 AB3N91KX），当前为 `/dev/ttyUSB0`。默认使用固定路径
`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0`，不会因USB编号改变而误选其他设备。
更换转换器时，用 `ls -l /dev/serial/by-id/` 查询并设置 `CHASSIS_SERIAL_PORT`，程序不会自动猜测端口。
`config/61-grain-serial.rules` 将此转换器的旧遥控别名改为 `/dev/chassis_serial`；遥控器现接STM32 USART1。
请按工控机实际设备节点设置。现有 X2P 使用 `/dev/ttyS0`，两者不能占用同一设备；启动脚本会检查路径及软链接冲突。
连接：USB-TTL TXD→PA3、RXD→PA2、GND→GND，使用3.3V TTL，断开原板载TX2/RX2。
单片机独立供电时不接转换器VCC。

## 启动

在工控机项目目录执行（替换实际底盘设备名）：

```bash
bash scripts/stop_industrial_pc.sh
export CHASSIS_SERIAL_PORT=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0
source config/industrial_pc.env
bash scripts/start_industrial_pc.sh
```

启动机构节点、串口底盘节点和目标点导航节点；串口模式不启动旧的
`motor_driver.py`、`cmd_vel_to_motor.py` 或 `rc_node`。
遥控接收与左右履带混控由 STM32 完成，机构仍由工控机控制。
UI 在加载同一配置的终端中启动，使用 `/rc_mode` 镜像遥控模式。
原 Orange Pi 的 `start.sh`、`start_all_full.sh` 是旧 UDP 部署入口，不要同时运行。

FAST-LIO/激光雷达仍需按现有建图流程启动，提供 `/Odometry`；本脚本不启动定位系统。
本次补齐的是现有目标点控制器的串口执行链路，没有新增路径规划器。

```text
导航目标 /move_base_simple/goal + FAST-LIO /Odometry
  -> goal_controller（位置闭环、归一化 forward/turn）
  -> /chassis/arm 使能握手 + /chassis/effort
  -> chassis_serial（单一串口持有者，20 Hz，指令新鲜度检查）
  -> 工控机 USB1 -> USB-TTL -> STM32 USART2 -> PWM -> 履带
```

单片机需收到有效遥控信号：CH8 自动档（>=1550，稳定500 ms）、CH1/CH3 回中
（1450～1550），没有故障或急停，才允许导航使能。发送一个新导航目标时自动调用
`/chassis/arm`；失败会显示 `FAULT chassis_arm`，不会输出运动命令。

## 状态与控制

### 无 ROS 的独立串口运动测试

测试程序 `scripts/test_chassis_serial.py` 直接复用生产串口协议。先停止正常底盘节点，
避免两个进程抢串口；将履带架空，遥控器开机，CH8 切自动档，CH1/CH3 回中。
在工控机项目目录运行，默认使用上述固定USB路径，可用 `--port` 覆盖：

```bash
bash scripts/stop_industrial_pc.sh
python3 scripts/test_chassis_serial.py status
python3 scripts/test_chassis_serial.py forward --effort 0.2 --seconds 1
python3 scripts/test_chassis_serial.py backward --effort 0.2 --seconds 1
python3 scripts/test_chassis_serial.py left --effort 0.2 --seconds 1
python3 scripts/test_chassis_serial.py right --effort 0.2 --seconds 1
```

每次测试先握手、检查遥控状态并确认使能，再以20Hz发送运动指令，不逐条等待ACK。
结束时发送停车，不等待执行完成反馈。Ctrl+C、SIGTERM或异常退出会尽力发送停车。
运行期间仍接收STATUS监测链路、遥控和故障；位置闭环与到达目标判断由工控机雷达定位完成。
兼容当前固件：单片机仍会发ACK/STATUS，但运动发送不依赖逐条ACK。
默认动作是 `status`，不发送运动指令，但新会话握手会清除此前的自动使能。
`--effort` 是归一化力度（0.01～1.0），不是速度；如果0.2无法克服静摩擦，架空确认
方向及停止正常后，可逐次试0.3、0.4。`--seconds` 限0.3～10秒，默认1秒。

故障/急停不会自动清除。恢复遥控、摇杆回中后可显式操作：

```bash
python3 scripts/test_chassis_serial.py recover
python3 scripts/test_chassis_serial.py clear-estop
python3 scripts/test_chassis_serial.py estop
python3 scripts/test_chassis_serial.py stop
```

`stop` 仅停止自动控制，`estop` 锁存急停并停止手动输出。
退出码0表示所选操作流程完成，1表示失败，130表示人工中断。运动“发送结束”仅表示
指令已写入串口，不证明单片机已执行、已停车、履带实际转动或导航通过。
单片机没有轮速反馈，需观察履带；测试结束后重新启动正常导航系统验证目标点行驶。
仅需 `pyserial`（项目依赖已包含），不需要ROS。Windows串口可用 `--port COM1`。

### ROS 接口

```bash
rostopic echo /chassis/status
rostopic echo /rc_mode
rostopic echo /goal_controller/status
rosservice call /chassis/stop       # 仅停自动控制，保留单片机手动控制
rosservice call /chassis/estop      # 单片机锁存急停，手动也停止
rosservice call /chassis/recover    # 有效遥控、摇杆回中后，显式恢复 RC 掉线故障
rosservice call /chassis/clear_estop # 无故障、摇杆回中后，显式解除急停
```

恢复故障或解除急停不会继续旧目标，需重新发目标。UI 急停在串口配置下同时调用
`/chassis/estop` 和既有机构急停接口。

`/chassis/status` 是 JSON，包含连接状态、单片机状态、故障位、模式、左右力度、
PWM、遥控年龄和计数器。`state`：0上电安全、1手动、2自动未使能、3自动使能、
4故障停止、5急停锁存；`faults` 位：1遥控超时、2IO故障、4PWM配置故障。
力度不是轮速反馈，不发布虚构的里程计；定位继续使用 FAST-LIO。

串口节点也接收标准 `/cmd_vel`，默认按 0.3 m/s、0.8 rad/s 映射为满量程力度，
通过 `~max_linear_mps`、`~max_angular_rps` 调整。这是开环力度映射，需要实车标定。
接入其他导航器或单独调试 `/cmd_vel` 时，先调用 `/chassis/arm`，然后立即以至少
10 Hz 发布指令（200 ms 内开始）；不要同时让目标点控制器与其他速度源驱动车辆。
`/chassis/effort` 的 Twist.linear.x/angular.z 为 [-1,1] 归一化力度，不使用物理速度单位。

## 停止与验证

工控机指令或单片机状态超过200 ms不更新时停止；单片机另有300 ms运动超时。
心跳只检测通信，不延长运动有效期。遥控手动接管、障碍信号、断线或超时取消旧目标；
重连使用新session，清空旧命令，需要新目标/显式使能。新目标先建立新会话以撤销旧运动，再等待AUTO_ARM返回新epoch，
之后持续发送带令牌的力度；运动、普通停车和心跳不等待ACK。
使能、故障恢复和急停等操作仍等待确认；普通停车服务成功仅代表已发送。

首次实机验证将履带架空，依次检查：手动模式、自动新目标、取消目标、急停及解除、
遥控掉线及恢复、拔掉串口、停止导航发布。确认停止和方向正确后再进行低力度落地测试。
当前开发环境的测试覆盖协议组帧/解析、ACK令牌更新、坏CRC、拆包粘包、通信异常和
超时/接管停机；真实串口、电平、履带方向及动力学尚需现场验证。
