# 底盘串口控制与拨杆状态回传

当前无 ROS 守护进程还通过同一串口发送 `0x38` 可靠机构动作命令，保留 `0x36` 单独调试帧及 PCA9685 输出。机构最终编号、STOP/关仓区别及本地执行时长见 [MECHANISM_SERIAL.md](MECHANISM_SERIAL.md)。下文底盘运动帧与遥控状态格式保持不变，ROS 节点相关段落为历史入口说明。

## 当前行为

工控机根据 FAST-LIO /Odometry 做位置闭环，通过 goal_controller 输出运动量。
串口节点以20Hz发送最新运动量，不握手、不等待ACK，也不等待执行完成。
独立被动接收 STATUS 中的拨杆档位，仅用于 `/rc_mode` 和 UI 显示，不恢复运动握手依赖。
到达目标后控制器发送零运动量并停止更新；串口节点200ms没有新导航指令就停止发送并发停车帧。
单片机每收到一个有效运动帧刷新300ms有效期，断流300ms后在下一控制周期（约10ms）输出1500/1500us。
“执行完毕”由工控机定位闭环判断，单片机没有距离目标，也没有执行完成报文。

新版单片机开启回传（CHASSIS_TELEMETRY_ENABLED=1），每100ms主动发送 STATUS，
无需工控机查询或先发运动指令。旧固件仍可接收运动指令，但没有回传时 UI 显示未知。
CRC、长度、幅值和同一发送会话内的递增序号仍检查，重复帧不能延长运动有效期。
自动控制仍要求遥控有效、CH8自动档、无故障/急停；从停止进入运动时摇杆须回中。
手动档以遥控输出为准，工控机运动帧不覆盖手动驾驶。RC故障或急停不因收到运动帧自动清除。
断流后新的有效运动帧可恢复自动输出。档位反馈不等于某条运动指令的执行确认。
`/chassis/status` 增加真实 `rc_mode`；`execution_confirmed` 仍为 false。

机构指令另有 `0x37` 接受回执。当前无 ROS 守护进程的唯一串口接收线程用同一解析器分发 `STATUS` 和机构回执，发送机构 START/STOP 后按会话及序号等待最多 400 ms；超时则在 10 秒窗口内用相同会话号和序号补发，USB 重连后继续。单片机对 `0x38` 重复帧只补回执，不重复执行动作。明确拒绝或持续无回执才使步骤失败；底盘运动帧仍不等待 ACK。机构回执仅确认 MCU 接受指令，不确认机械到位。

## 必须更新固件

实际固件工程位于 `D:\Project\下位机构\机构\MDK-ARM`，输出 `dipan/dipan.hex`。
需烧录新 HEX 才能启用机构回执；工控机与单片机应成套更新。STREAM_EFFORT/STREAM_CONTROL 与此前可用版本保持一致，
仍不兼容更早的 SET_EFFORT 固件。
旧固件不会执行新运动帧；程序无法自动检测固件版本。默认115200/8N1。
USB-TTL TX→PA3，GND共地；要接收回传，必须连接 STM32 PA2→USB-TTL RX（3.3V TTL）。
默认端口 /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0。

## 无ROS测试

先停止占用底盘串口的程序，遥控自动档、摇杆回中，然后运行：

```bash
python3 scripts/test_chassis_serial.py forward --effort 0.2 --seconds 3
python3 scripts/test_chassis_serial.py stop
python3 scripts/test_chassis_serial.py estop
python3 scripts/test_chassis_serial.py clear-estop
python3 scripts/test_chassis_serial.py recover
```

默认操作为stop，运动时长0.3～10秒。Ctrl+C或异常退出尽力发送停车。
输出“发送结束”或退出码0只代表写串口成功，不代表执行验证通过。无status测试命令。

## 导航

保留原启动配置 CHASSIS_BACKEND=serial 和 CHASSIS_SERIAL_PORT。
/chassis/arm只是工控机本地允许发送新目标指令，不再代表单片机使能成功。
/cmd_vel按0.3m/s、0.8rad/s映射满量程力度；/chassis/effort使用归一化力度。
/chassis/stop、estop、clear_estop、recover仅发送指令，服务返回不确认单片机执行。
障碍、导航指令超过200ms未更新或本地串口写失败会停止发送并取消目标。
雷达定位失效由goal_controller的里程计超时检查停止导航。实际手动档和故障检查在单片机执行。

## 新协议

保持 A5 5A | 01 | type | payload_len:u16 | sender:u32 | seq:u32 | payload | CRC16。
小端，CRC16-CCITT-FALSE覆盖01到payload末尾，sender非零，每进程随机生成，seq递增。
类型14 STREAM_EFFORT：payload为forward:i16、turn:i16，均[-1000,1000]，总帧20字节。
类型15 STREAM_CONTROL：payload为一个字节，6停车、7急停、8解除急停、13恢复RC故障，总帧17字节。
控制帧不需要epoch。新发送进程先发停车帧，再发运动量。不同sender的运动帧在已有运动期间被拒绝。
STATUS 类型10、payload长度52沿用已有布局；旧握手接口仅用于兼容/回归，不是生产运动入口。

## 拨杆状态显示（2026-09-28）

- STM32 判断 CH8：稳定500ms后，mode=1手动、mode=2自动；切换中/无效档位为0。
- STATUS 的 payload[13] 是 mode，payload[14] 的 bit2 表示遥控有效；
  payload[20:24] 为遥控数据年龄（小端毫秒），payload[32:34] 为原始 CH8。
- 回传在手动档也主动发送，帧头 session 可以为0。工控机只对显示用 STATUS 放宽
  session 匹配，原 HELLO/ACK/运动协议校验不变。同一 boot 下拒绝重复/倒序序号。
- 收到有效手动/自动且遥控年龄小于300ms时发布 `/rc_mode` 的 `manual`/`auto`；
  无效或超过1秒没有新状态则发布空字符串，UI 显示“未知”。ROS 发布节点消失时，
  UI 自身1.5秒超时也会清除旧显示。
- 单片机用固定缓冲区和 HAL_UART_Transmit_IT 中断发送，不引用栈内帧。
  发送忙/失败仅累计 pc_tx_dropped，不阻塞PWM循环，不触发接收DMA重启。
- 档位回传只改变显示，不发使能、停车或切换模式命令；MCU 的遥控接管、
  急停及300ms运动指令超时规则不变。

验收：烧录后先保持摇杆中位，启动底盘节点和UI，观察 `/rc_mode` 随 CH8 切换；
拔掉回传线后应在约1秒内变为未知。独立串口抓包工具不能与底盘节点同时占用串口。
编译/软件测试不代表已烧录或实机档位联动已验证。

## 平滑输出与串口恢复

自动运动PWM变化率上限500μs/秒，正常10ms控制周期最多变化5μs；从1500到50%前进1625约0.25秒。
正常减速和反向同样限速；零运动量、明确停车、300ms断流、急停、RC失效及严重故障立即回中位，不经过渐变。
手动遥控不经过自动运动的渐变逻辑。
USART2瞬时接收错误丢弃未完成帧并重启DMA，不清除尚有效的运动指令，也不更新指令时间戳。
即使DMA持续重启失败，300ms截止时间仍有效；USART1遥控错误仍立即停车。
导航控制器到达目标发送零量，串口节点收到零量立即发STREAM_CONTROL停车帧，不依靠300ms超时停车。
诊断计数保存在RAM，可用调试器观察：chassis_app.c的pc_uart_errors（串口2错误回调次数），
controller.pc_rx_recoveries（接收恢复尝试次数）、controller.motion_timeouts（自动指令超时次数），
以及controller.crc_errors/rejects和pwm_left/pwm_right；默认不回传，不影响单向控制。

## 正式导航启动与验证

生产链路：界面/RViz目标 → /move_base_simple/goal → goal_controller读取/Odometry →
/chassis/effort → chassis_serial以20Hz发送 → USB-TTL → F103渐变PWM。
默认后端serial；无需再运行独立串口测试脚本，也不要同时运行旧motor_driver/cmd_vel_to_motor。
导航目标控制器10Hz更新运动量，串口节点20Hz发送；/chassis/arm仅作为本地新目标发送开关。
到达后的零量立即转为停车帧，进入空闲后不再取消已完成目标，保留ARRIVED供采样流程判断。

在项目根目录执行：

```bash
source config/industrial_pc.env
bash scripts/start_industrial_pc.sh
```

若已有旧节点运行，先按原流程停止后再启动。上述脚本还启动机构节点；雷达/FAST-LIO仍按原流程单独启动。
保持遥控自动档、摇杆回中，确认/Odometry持续更新，在与其相同坐标系中通过原界面或RViz发送目标。
查看/goal_controller/status中的DRIVE、APPROACH、ARRIVED或FAULT；/chassis/status仅表示发送端状态。
自动化验证覆盖目标控制器到真实协议编码的链路、到达停车、取消、定位超时、障碍及到达状态保留。
底盘前后/转向已由现场独立测试确认；实际雷达闭环目标行驶仍需按导航流程现场验证。
