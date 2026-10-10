# PCA9685 PWM 扩展板使用说明

通道 0 电机电子开关的启动、关闭和接线流程见：

```text
/home/orangepi/dipan/pca9685/PWM_MOTOR_SWITCH.md
```

## 硬件与系统配置

当前接线：

```text
PCA9685       Orange Pi 5 Max
VCC      ---> 物理 Pin 1  (3.3V)
V+       ---> 物理 Pin 2  (5V，电子开关控制端供电)
GND      ---> 物理 Pin 9  (GND)
SDA      ---> 物理 Pin 11 (I2C2_M4 SDA)
SCL      ---> 物理 Pin 13 (I2C2_M4 SCL)
```

系统接口与设备地址：

```text
I2C 设备：/dev/i2c-2
PCA9685：0x40
All Call：0x70
```

`0x70` 是 PCA9685 默认启用的广播地址，通常不是第二块设备。

驱动程序不依赖 `smbus` 或其他第三方 Python 模块，直接使用 Linux `i2c-dev`。

## 安全说明

- 初次测试不要连接舵机、电调或其他负载。
- PCA9685 的 `VCC` 只能接 Orange Pi 的 3.3V。
- PCA9685 的 `V+` 是通道附件供电轨，不是电机电池输入；本项目接 Orange Pi Pin 2 的 5V。
- 舵机等负载使用独立电源，外部电源与 Orange Pi 必须共地。
- PCA9685 的 16 个通道共用同一个 PWM 频率。

## 基本命令

进入目录：

```bash
cd /home/orangepi/dipan/pca9685
```

查看当前状态：

```bash
sudo ./pca9685_driver.py status
```

只查看通道 0：

```bash
sudo ./pca9685_driver.py status 0
```

初始化为 50Hz，并关闭全部通道：

```bash
sudo ./pca9685_driver.py init 50
```

PCA9685 的 25MHz 振荡器和 8位预分频器会产生少量频率误差，程序会显示实际配置频率。

## 微秒脉宽输出

通道 0 输出 1500us、50Hz：

```bash
sudo ./pca9685_driver.py pulse 0 1500
```

通道 1 输出 1000us：

```bash
sudo ./pca9685_driver.py pulse 1 1000
```

指定其他频率：

```bash
sudo ./pca9685_driver.py pulse 0 1500 --frequency 60
```

如果连接电调，应先输出停止/中位脉宽，再给电调上电。具体停止值取决于电调类型和标定方式。

## 占空比输出

通道 0 输出 25% 占空比：

```bash
sudo ./pca9685_driver.py duty 0 25
```

通道 0 持续高电平：

```bash
sudo ./pca9685_driver.py duty 0 100
```

## 关闭输出

关闭通道 0：

```bash
sudo ./pca9685_driver.py off 0
```

关闭全部 16 个通道：

```bash
sudo ./pca9685_driver.py all-off
```

让 PCA9685 进入休眠：

```bash
sudo ./pca9685_driver.py sleep
```

唤醒：

```bash
sudo ./pca9685_driver.py wake
```

## 原始计数模式

PCA9685 每个周期有 4096 个计数。下面让通道 0 从计数 0 开始，在计数 307 关闭：

```bash
sudo ./pca9685_driver.py raw 0 0 307
```

在约 50Hz 下，这大约对应 1500us 脉宽。通常优先使用 `pulse` 命令。

## 默认总线与地址

程序默认使用：

```text
--bus 2
--address 0x40
```

如果以后修改了 PCA9685 地址，可这样指定：

```bash
sudo ./pca9685_driver.py --bus 2 --address 0x41 status
```

## 与现有电机驱动的关系

现有履带电机驱动仍然使用：

```text
左电调：物理 Pin 7，pwm3-m3
右电调：物理 Pin 16，pwm1-m2
```

`pca9685_driver.py` 是独立的扩展板测试和控制程序，不会改变 `motor_driver.py` 的 Pin 7/16 输出。
