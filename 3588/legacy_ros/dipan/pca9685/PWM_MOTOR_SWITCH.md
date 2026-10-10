# PCA9685 通道 0 电机电子开关操作流程

## 已确认的硬件

控制板：Orange Pi 5 Max（RK3588）

PWM 扩展板：PCA9685，I2C 地址 `0x40`

电子开关：欣薇 × Zave 高压版正逻辑电子开关

```text
工作电压：3.0V～30V
标称电流：20A
低于 1.5ms：关闭
高于 1.5ms：开启
LED 慢闪：无输出
LED 长亮：有输出
```

这是二段电子开关，只能控制电机全开或关闭，不能调速，也不能控制正反转。

## PCA9685 接线

操作接线前必须关闭 Orange Pi，并断开电机电池。

```text
PCA9685              Orange Pi 5 Max
VCC（逻辑电源） ---> 物理 Pin 1（3.3V）
V+（通道电源）  ---> 物理 Pin 2（5V）
GND              ---> 物理 Pin 9（GND）
SDA              ---> 物理 Pin 11（I2C2_M4 SDA）
SCL              ---> 物理 Pin 13（I2C2_M4 SCL）
```

本机实测：

```text
Orange Pi Pin 2 -> GND：5.37V
PCA9685 VCC -> GND：3.27V
PCA9685 V+ -> GND：接通后应约为 5.37V
```

PCA9685 参数规定 `V+` 最大为 6V。不得把 3～30V 的电机电池接到 PCA9685 的 `V+`。

## 电子开关接线

电子开关三线插头接 PCA9685 通道 0：

```text
电子开关黑线 ---> CH0 GND
电子开关红线 ---> CH0 V+
电子开关白线 ---> CH0 PWM
```

电机接电子开关的负载输出端。电机电池只接电子开关的大电流电源输入端，不能接 Orange Pi 或 PCA9685。

电子开关切换正极，必须按照产品线序正确连接电池、电机和公共负极。建议在电池回路串联合适的保险丝，并保证电机堵转电流不超过电子开关允许值。

## 每次启动的完整流程

1. 确认电机电池处于断开状态。
2. 确认电子开关三线插头方向正确。
3. 给 Orange Pi 上电并等待系统启动完成。
4. 进入驱动目录：

```bash
cd /home/orangepi/dipan/pca9685
```

5. 初始化 PCA9685 为 50Hz，并关闭全部通道：

```bash
sudo ./pca9685_driver.py init 50
```

6. 在通道 0 持续输出可靠的关闭信号：

```bash
sudo ./pca9685_driver.py pulse 0 1000
```

7. 检查输出状态：

```bash
sudo ./pca9685_driver.py status 0
```

状态中应显示频率约为 `50Hz`、脉宽约为 `1000us`。

8. 将电机或车轮架空，确保周围没有人员、线缆和杂物。
9. 接通电子开关的电机电池。
10. 确认电子开关处于关闭状态，再发送启动命令：

```bash
sudo ./pca9685_driver.py pulse 0 2000
```

电子开关应开启，LED 长亮，电机转动。

## 正常关闭流程

1. 发送可靠的关闭脉宽：

```bash
cd /home/orangepi/dipan/pca9685
sudo ./pca9685_driver.py pulse 0 1000
```

2. 确认电机停止，电子开关 LED 慢闪。
3. 断开电子开关的电机电池。
4. 需要关闭 Orange Pi 时再执行：

```bash
sudo poweroff
```

不要先关闭 Orange Pi 再等待电机停止。正常停机时应先发送 `1000us`，确认电机停止，再断开电机电池。

## 常用命令

开启电机：

```bash
sudo /home/orangepi/dipan/pca9685/pca9685_driver.py pulse 0 2000
```

关闭电机：

```bash
sudo /home/orangepi/dipan/pca9685/pca9685_driver.py pulse 0 1000
```

查看通道 0：

```bash
sudo /home/orangepi/dipan/pca9685/pca9685_driver.py status 0
```

## 注意事项

- 不使用正好 `1500us`，因为它处于电子开关的临界点。
- 使用 `1000us` 作为可靠关闭命令，使用 `2000us` 作为可靠开启命令。
- 不使用 `off 0` 代替正常关闭命令；`off 0` 会完全移除 PWM 脉冲，电子开关对信号丢失的行为不确定。
- PCA9685 会在程序退出后继续保持最后一次输出，因此每次启动和停机都要明确发送对应命令。
- 软件关闭不能代替物理急停。出现失控、冒烟、异味或异常发热时，立即断开电机电池。
- 电子开关标称 20A 不代表所有散热和堵转工况下都能长期承受 20A。

## 故障检查

执行 `2000us` 后电机不转时，按顺序检查：

```text
1. PCA9685 VCC -> GND 是否约为 3.3V
2. PCA9685 V+  -> GND 是否约为 5V
3. status 0 是否显示约 2000us
4. 电子开关 LED 是否长亮
5. 电机电池是否有电
6. 电子开关大电流输入和输出是否接反
7. 电机端是否获得接近电池电压
```

