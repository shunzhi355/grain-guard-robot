# 粮食扦样机器人 — ARM Linux 工控机当前接线

以用户确认的本表为准，替代此前 LVDS 和 Orange Pi 40-pin 接线。

| 工控机/设备 | 连接 |
|---|---|
| TP Pin5 I2C_SDA_TP | PCA9685 SDA |
| TP Pin4 I2C_SCL_TP | PCA9685 SCL |
| TP Pin1 VCC3V0_TOUCH（3.0V） | PCA9685 VCC |
| TP Pin6 GND | PCA9685 GND，与电调信号地共地 |
| USB1 | USB-TTL → 遥控接收机 i-BUS |
| USB3 | USB-RS485 → X2P 伺服 |
| PCA9685 CH0/CH1 | 螺旋输送机1/2 |
| PCA9685 CH2/CH3/CH4 | 浅仓/中仓/深仓 |
| PCA9685 CH5/CH6 | 夹紧/拧紧 |
| PCA9685 CH8/CH9 | 左/右底盘电调 Signal |

CH7 保留未接线占位，CH10–15 未使用。所有 PWM 均由同一块 PCA9685 输出，目标频率 50Hz。
底盘保持原有 1500us 中位、1000–2000us 硬限制和默认半速配置，方向与脉宽待现场验证。
机构各动作的脉宽标定仍以 sampling_params.py 为准。

电调主电源使用原独立供电。PCA9685 V+ 不属于本次确认的 TP 接线，勿把电机主电源接到 VCC/V+。

软件默认遥控 /dev/ttyUSB0、伺服 /dev/ttyUSB1、I²C /dev/i2c-4（TP I2C4，Linux 映射待核实） 地址 0x40，待现场测试修改。
USB1/USB3 不代表 Linux 设备编号。接收机沿用现有 i-BUS：CH1 前后、CH3 转向、CH8 模式。

工控机手册 TP 表（印刷页12/17）标注 Pin1 电源及 Pin4/5 I²C 信号为 3.0V。
PCA9685 模块上拉须匹配 TP 的 3.0V 信号，不能上拉到 5V；Pin2 INT、Pin3 RST 不接 PCA9685。
现场目前没有 /dev/i2c-4，需核实 TP 的 Linux 总线映射或启用设备树；仅设置环境变量不会创建设备节点。

启动与可覆盖配置见 [工控机适配说明](grain_sampling_robot_software_share/docs/INDUSTRIAL_PC.md)。
