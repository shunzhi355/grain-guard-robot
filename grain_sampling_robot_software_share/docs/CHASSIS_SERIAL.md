# 单向串口底盘控制

## 当前行为

工控机根据 FAST-LIO /Odometry 做位置闭环，通过 goal_controller 输出运动量。
串口节点以20Hz发送最新运动量，不握手、不等待ACK、不读取STATUS，也不等待执行完成。
到达目标后控制器发送零运动量并停止更新；串口节点200ms没有新导航指令就停止发送并发停车帧。
单片机每收到一个有效运动帧刷新300ms有效期，断流300ms后在下一控制周期（约10ms）输出1500/1500us。
“执行完毕”由工控机定位闭环判断，单片机没有距离目标，也没有执行完成报文。

单片机默认关闭所有串口回传（CHASSIS_TELEMETRY_ENABLED=0）。无需连接回传才能控制。
CRC、长度、幅值和同一发送会话内的递增序号仍检查，重复帧不能延长运动有效期。
自动控制仍要求遥控有效、CH8自动档、无故障/急停；从停止进入运动时摇杆须回中。
手动档以遥控输出为准，工控机运动帧不覆盖手动驾驶。RC故障或急停不因收到运动帧自动清除。
断流后新的有效运动帧可恢复自动输出。单向通信无法让工控机知道单片机是否接收、是否允许执行或是否已停车。
/chassis/status只报告本地端口和发送状态，不伪造遥控档位和执行状态。

## 必须更新固件

新命令与旧SET_EFFORT不兼容。先烧录仓库根目录 dipan/MDK-ARM/dipan/dipan.hex，再使用新版工控机代码。
旧固件不会执行新运动帧；程序无法自动检测固件版本。默认115200/8N1。
USB-TTL TX→PA3，GND共地；RX→PA2可保留。旧板载串口线断开。
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
默认无任何串口发送；旧协议仅用于代码兼容/回归，不再是生产控制入口。

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
