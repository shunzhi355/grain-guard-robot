# 履带底盘电机驱动说明

驱动文件：

```bash
/home/orangepi/底盘/motor_driver.py
```

当前硬件映射：

```text
左履带/电调1：物理 pin7  -> pwm3-m3
右履带/电调2：物理 pin16 -> pwm1-m2
```

如果实际左右相反，先用 `--invert-left`、`--invert-right` 或交换接线调整。

## 基本命令

进入目录：

```bash
cd /home/orangepi/底盘
```

停止，保持 1500us 中位信号：

```bash
sudo ./motor_driver.py stop
```

查看 PWM 状态：

```bash
sudo ./motor_driver.py status
```

直接控制左右履带，范围是 `-1.0 ~ 1.0`：

```bash
sudo ./motor_driver.py lr 0.2 0.2
sudo ./motor_driver.py lr 0.2 -0.2
sudo ./motor_driver.py stop
```

差速混控，`linear` 和 `angular` 都是归一化值：

```bash
sudo ./motor_driver.py cmd 0.2 0.0
sudo ./motor_driver.py cmd 0.2 0.3
sudo ./motor_driver.py cmd 0.0 0.3
sudo ./motor_driver.py stop
```

含义：

```text
linear > 0：前进
linear < 0：后退，前提是电调支持反转
angular > 0：左转
angular < 0：右转
```

如果电调不支持反转，加 `--forward-only`：

```bash
sudo ./motor_driver.py cmd 0.2 0.3 --forward-only
```

## `lr` 和 `cmd` 的区别

`motor_driver.py` 现在有两种常用控制方式：

```text
lr  LEFT RIGHT
cmd LINEAR ANGULAR
```

它们后面的数字范围都是：

```text
-1.0 ~ 1.0
```

其中：

```text
0.0  = 停止/中位
0.2  = 低速正转
1.0  = 最大正转
-0.2 = 低速反转
-1.0 = 最大反转
```

注意：负数能不能让履带反转，取决于电调是否支持反转。

## 当前起转标定值

实测最小起转命令：

```text
左履带正转： 0.062
右履带正转： 0.054
左履带反转：-0.186
右履带反转：-0.191
```

这些值已经写进 `motor_driver.py` 默认参数：

```text
left_forward_min  = 0.062
right_forward_min = 0.054
left_reverse_min  = 0.186
right_reverse_min = 0.191
```

驱动默认开启“起转补偿”。也就是说，只要你发的命令不是 0，程序会先跨过该方向的死区，再按剩余范围缩放。

例如：

```bash
sudo ./motor_driver.py lr -0.1 0.1
```

不会再直接输出原始的 `-0.1` 和 `0.1`，而是会自动补偿到能起转的区间。

如果想关闭补偿，使用：

```bash
sudo ./motor_driver.py lr -0.1 0.1 --no-start-boost
```

如果后面重新标定了起转值，也可以临时覆盖：

```bash
sudo ./motor_driver.py lr -0.1 0.1 \
  --left-forward-min 0.062 \
  --left-reverse-min 0.186 \
  --right-forward-min 0.054 \
  --right-reverse-min 0.191
```

### `lr LEFT RIGHT`

`lr` 是最底层、最直接的左右履带控制。

```bash
sudo ./motor_driver.py lr -0.3 0.3
```

含义：

```text
left  = -0.3  左履带低速反转
right =  0.3  右履带低速正转
```

如果电调支持反转，这个命令会让底盘原地向左旋转。

常用例子：

```bash
sudo ./motor_driver.py lr 0.2 0.2
```

左右履带都低速正转，底盘直行前进。

```bash
sudo ./motor_driver.py lr 0.2 0.0
```

左履带转、右履带停，底盘会偏转。

```bash
sudo ./motor_driver.py lr -0.3 0.3
```

左履带反转、右履带正转，尝试原地左转。

```bash
sudo ./motor_driver.py lr 0.3 -0.3
```

左履带正转、右履带反转，尝试原地右转。

`lr` 适合做电机、电调、方向、左右一致性的底层调试。

### `cmd LINEAR ANGULAR`

`cmd` 是给上层导航/寻迹使用的差速底盘控制方式。

```bash
sudo ./motor_driver.py cmd 0.2 0.3
```

含义：

```text
linear  = 0.2  前进速度比例
angular = 0.3  左转速度比例
```

程序内部会把 `linear/angular` 换算成左右履带：

```text
左履带 = linear - angular
右履带 = linear + angular
```

例如：

```bash
sudo ./motor_driver.py cmd 0.0 0.3
```

内部换算：

```text
左履带 = 0.0 - 0.3 = -0.3
右履带 = 0.0 + 0.3 =  0.3
```

这和下面这条 `lr` 指令等价：

```bash
sudo ./motor_driver.py lr -0.3 0.3
```

所以 `cmd 0.0 0.3` 的目标是原地左转。

常用例子：

```bash
sudo ./motor_driver.py cmd 0.2 0.0
```

低速直行前进。

```bash
sudo ./motor_driver.py cmd 0.2 0.3
```

前进同时左转。

```bash
sudo ./motor_driver.py cmd 0.2 -0.3
```

前进同时右转。

```bash
sudo ./motor_driver.py cmd 0.0 0.3
```

尝试原地左转。

```bash
sudo ./motor_driver.py cmd 0.0 -0.3
```

尝试原地右转。

后面接 FAST-LIO 和寻迹时，建议上层程序输出 `cmd LINEAR ANGULAR`，不要直接输出 `lr LEFT RIGHT`。

## UDP 常驻服务

后面寻迹程序可以通过 UDP 发速度命令：

```bash
sudo ./motor_driver.py daemon --host 127.0.0.1 --port 8765 --timeout 0.3
```

安全逻辑：

```text
超过 0.3 秒没有收到新命令 -> 自动 stop
```

发送文本命令：

```bash
echo 'cmd 0.2 0.0' | nc -u -w1 127.0.0.1 8765
echo 'cmd 0.2 0.3' | nc -u -w1 127.0.0.1 8765
echo 'lr 0.2 0.2'  | nc -u -w1 127.0.0.1 8765
echo 'stop'        | nc -u -w1 127.0.0.1 8765
```

也可以发送 JSON：

```json
{"linear": 0.2, "angular": 0.0}
```

或：

```json
{"left": 0.2, "right": 0.2}
```

## 给 FAST-LIO/寻迹预留的接口

FAST-LIO 负责输出位姿，不直接控制电机。

后续结构建议：

```text
FAST-LIO odom/pose
    -> 寻迹控制器
    -> linear/angular
    -> motor_driver.py UDP daemon
    -> 左右电调 PWM
```

寻迹程序只需要持续发送：

```text
cmd linear angular
```

例如：

```text
cmd 0.2 0.1
```

如果寻迹程序崩溃或不再发命令，`motor_driver.py daemon` 会在超时后自动停车。
