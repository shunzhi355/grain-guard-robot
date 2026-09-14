# 粮食扦样机器人 — ARM Linux 工控机当前接线

以用户确认的本表为准，替代此前 Orange Pi 40-pin 接线。

| 工控机/设备 | 连接 |
|---|---|
| LVDS Pin36 I2C_SDA_LVDS | PCA9685 SDA |
| LVDS Pin38 I2C_SCL_LVDS | PCA9685 SCL |
| LVDS Pin1/2/3 VCC_LVDS（用户指定 3.3V） | PCA9685 VCC |
| LVDS Pin4/5/6 GND | PCA9685 GND，与电调信号地共地 |
| USB1 | USB-TTL → 遥控接收机 i-BUS |
| USB3 | USB-RS485 → X2P 伺服 |
| PCA9685 CH0/CH1 | 螺旋输送机1/2 |
| PCA9685 CH2/CH3/CH4 | 浅仓/中仓/深仓 |
| PCA9685 CH5/CH6 | 夹紧/拧紧 |
| PCA9685 CH8/CH9 | 左/右底盘电调 Signal |

CH7 保留未接线占位，CH10–15 未使用。所有 PWM 均由同一块 PCA9685 输出，目标频率 50Hz。
底盘保持原有 1500us 中位、1000–2000us 硬限制和默认半速配置，方向与脉宽待现场验证。
机构各动作的脉宽标定仍以 sampling_params.py 为准。

电调主电源使用原独立供电。PCA9685 V+ 不属于本次确认的 LVDS 接线，勿把电机主电源接到 VCC/V+。

软件默认遥控 /dev/ttyUSB0、伺服 /dev/ttyUSB1、I²C /dev/i2c-2 地址 0x40，待现场测试修改。
USB1/USB3 不代表 Linux 设备编号。接收机沿用现有 i-BUS：CH1 前后、CH3 转向、CH8 模式。

工控机手册 PDF 第14–15页显示 VCC_LVDS 可通过 J27 选择 3.3V/5V，Pin36/38 I²C 信号标注 1.8V。
用户指定的 VCC 3.3V 与信号电平是两回事，需现场核对 J27、I²C 电平及模块上拉，必要时使用电平转换。

启动与可覆盖配置见 [工控机适配说明](grain_sampling_robot_software_share/docs/INDUSTRIAL_PC.md)。
