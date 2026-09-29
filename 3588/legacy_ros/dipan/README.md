# Orange Pi 5 Max 电调 PWM 使用说明

PCA9685 PWM 扩展板驱动：

```text
/home/orangepi/dipan/pca9685/pca9685_driver.py
```

详细接线和命令见：

```text
/home/orangepi/dipan/pca9685/README.md
```

本机使用 `/home/orangepi/esc_pin7_pwm.sh` 控制电调 PWM。

新的履带底盘电机驱动是：

```bash
/home/orangepi/底盘/motor_driver.py
```

详细用法见：

```bash
/home/orangepi/底盘/MOTOR_DRIVER.md
```

后续 FAST-LIO 寻迹建议调用 `motor_driver.py daemon` 的 UDP 接口，而不是直接操作底层 PWM 脚本。

## MID360 单目标点闭环控制（第一版）

本目录现在包含：

```text
goal_controller.py
```

它不依赖轮式编码器，也不使用 `waypoint_tools`。控制链路是：

```text
FAST-LIO /Odometry + /move_base_simple/goal
                    -> goal_controller.py
                    -> UDP: cmd FORWARD TURN
                    -> motor_driver.py daemon
                    -> 左右履带 PWM
```

`FORWARD` 和 `TURN` 是 `-1.0 ~ 1.0` 的归一化电机力度，不是 m/s 和 rad/s。

### 1. 启动电机守护进程

先架空履带或确保车辆周围有足够安全空间：

```bash
cd /home/orangepi/dipan
sudo ./motor_driver.py daemon --host 127.0.0.1 --port 8765 --timeout 0.3
```

### 2. 启动 FAST-LIO 重定位

按 `/home/orangepi/fastlio2_ws/FASTLIO2_RUN_GUIDE.md` 启动 MID360 驱动和重定位，确认：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry/header
```

### 3. 启动目标点控制器

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/dipan
./goal_controller.py
```

首次低风险测试可进一步降低力度：

```bash
./goal_controller.py \
  _far_forward:=0.10 \
  _near_forward:=0.07 \
  _max_rotate_turn:=0.15 \
  _max_drive_turn:=0.08
```

### 4. 发布目标点

目标点的 `frame_id` 默认必须与 `/Odometry.header.frame_id` 完全一致。当前 FAST-LIO 通常是 `camera_init`：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'camera_init'
pose:
  position: {x: 1.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

这表示目标是当前地图坐标中的 `(1.0, 0.0)`，不是“从车辆当前位置向前1米”。到达目标位置后，控制器会原地旋转到 `orientation` 指定的最终朝向。

也可以在 RViz 中把 `Fixed Frame` 设置成 `/Odometry.header.frame_id`（通常为 `camera_init`），再使用 `2D Nav Goal` 点击目标。

### 5. 查看状态和归一化输出

```bash
rostopic echo /goal_controller/status
rostopic echo /goal_controller/normalized_cmd
```

状态包括：

```text
WAIT_ODOM / WAIT_GOAL / ROTATE / DRIVE / APPROACH / FINAL_ROTATE
ARRIVAL_CONFIRM / ARRIVED / FAULT
```

### 6. 取消目标

```bash
rostopic pub -1 /cancel_goal std_msgs/Empty '{}'
```

### 默认控制参数

```text
到达半径：              0.05 m
位置恢复控制半径：      0.10 m
到达连续确认：          0.8 s
最终航向允许误差：      5 deg
航向恢复控制误差：      10 deg
开始低力度靠近距离：    0.80 m
先原地转向的角度阈值：  25 deg
远距离前进力度：        0.15
近距离前进力度：        0.08
最大原地转向力度：      0.20
最大行进转向修正：      0.12
定位超时：              0.30 s
堵转判断时间：          2.0 s
```

安全逻辑：定位超时、位姿异常跳变、持续前进但没有位移、取消目标、程序退出或底层 UDP 命令中断都会停车。发生 `FAULT` 后，排除原因并发送一个新目标才会重新尝试运动。

## 接线

下面说的都是 **物理排针编号**，不是 GPIO 编号。

电调 1：

- 信号线：物理 `pin7`
- 地线：物理 `pin6`

电调 2：

- 信号线：物理 `pin16`
- 地线：物理 `pin14` 或 `pin20`

暂时不要使用物理 `pin32`、`pin35` 和 `pin36`。`pin32` 实测一直是高电平；`pin35` 虽然有 PWM 平均电压变化，但电调不识别；`pin36` 的 PWM15 被系统背光占用。

电调/电机必须使用独立电源供电。电调 GND 必须和 Orange Pi 的 GND 共地。

## 换到 Pin16 后的一次性配置

如果第二个电调已经改接到 `pin16`，先执行一次：

```bash
cd /home/orangepi
sudo ./esc_pin7_pwm.sh enable-overlay
sudo reboot
```

重启后检查 `/boot/orangepiEnv.txt`，里面应该包含：

```text
pwm3-m3 pwm1-m2
```

如果使用 `pin16`，里面不应该再有：

```text
pwm14-m0
pwm14-m2
pwm15-m3
```

## 正常启动电调

进入目录：

```bash
cd /home/orangepi
```

先让两个电调进入中位/停止信号：

```bash
sudo ./esc_pin7_pwm.sh stop
```

低速启动两个电调：

```bash
sudo ./esc_pin7_pwm.sh pulse 1550
```

如果 `1550` 力度不够或转速不稳定，可以逐步提高：

```bash
sudo ./esc_pin7_pwm.sh pulse 1600
sudo ./esc_pin7_pwm.sh pulse 1650
```

停止两个电机，但继续保持有效 PWM 信号：

```bash
sudo ./esc_pin7_pwm.sh stop
```

## 单独测试某一路

只控制电调 1，也就是 `pin7`：

```bash
sudo ./esc_pin7_pwm.sh pulse1 1600
sudo ./esc_pin7_pwm.sh stop
```

只控制电调 2，也就是 `pin16`：

```bash
sudo ./esc_pin7_pwm.sh pulse2 1600
sudo ./esc_pin7_pwm.sh stop
```

## 重要注意

正常停车请使用：

```bash
sudo ./esc_pin7_pwm.sh stop
```

不要把 `off` 当作普通停车命令。

`off` 会彻底关闭 PWM 输出，很多电调会把它当成信号丢失，之后可能需要重新解锁或断电重上电才能再次转动。

脉宽含义：

- `1000us`：最低值
- `1500us`：中位/停止
- `1550us` 以上：当前测试电调的正向油门
- `2000us`：最大值

## 排查命令

查看当前 PWM 输出状态：

```bash
for d in /sys/class/pwm/pwmchip*; do
  echo "$d -> $(readlink -f "$d")"
  for f in "$d"/pwm0/period "$d"/pwm0/duty_cycle "$d"/pwm0/polarity "$d"/pwm0/enable; do
    [ -e "$f" ] && printf '  %s=' "$f" && cat "$f"
  done
done
```

测试 `pin16` 电压：

```bash
sudo ./esc_pin7_pwm.sh test-m2
```

万用表红表笔接 `pin16`，黑表笔接 GND。

## 本机调试记录

这台机器是 Orange Pi 5 Max，设备树型号：

```text
RK3588 OPi 5 Max
```

最终可用方案：

```text
电调 1：物理 pin7  -> pwm3-m3
电调 2：物理 pin16 -> pwm1-m2
```

调试过程中试过这些脚：

### Pin7

物理 `pin7` 对应 `PWM3`，实际使用 overlay：

```text
pwm3-m3
```

实测可正常控制电调。

注意：一开始误用了 `pwm3-m0`，内核报错：

```text
gpio0-28 already requested by fea90000.i2c
```

原因是 `pwm3-m0` 和板载 I2C 复用冲突。后来改成 `pwm3-m3` 后正常。

### Pin32

物理 `pin32` 在排针表里标为 `PWM14`，对应：

```text
pwm14-m2
```

但实测 `pin32` 对 GND 一直约 `3.27V`，不同 duty 下都不变化，说明这根脚实际没有被 PWM 正常控制住。

结论：本机不使用 `pin32` 控制电调。

### Pin35

物理 `pin35` 对应：

```text
pwm14-m0
```

万用表测试时，`pin35` 对 GND 的平均电压会随 duty 改变，例如从低电压变化到接近 `3.3V`，说明这个脚确实有 PWM 相关输出。

但是，把已确认能转的电调从 `pin7` 换到 `pin35` 后，电调仍然不转。也就是说，`pin35` 虽然有平均电压变化，但输出波形或电气状态不被当前电调识别。

结论：本机不使用 `pin35` 控制电调。

### Pin36

物理 `pin36` 对应：

```text
pwm15-m3
```

但启用后，导出 PWM 时出现：

```text
echo: 写错误: 设备或资源忙
```

检查 `/sys/class/pwm` 后发现 `pwm15` 被系统背光占用：

```text
consumer:platform:backlight
```

结论：`pin36` 这路 PWM 被系统占用，不拿来控制电调。

### Pin16

物理 `pin16` 对应：

```text
pwm1-m2
```

把第二个电调接到：

```text
信号线 -> pin16
地线   -> pin14 或 pin20
```

然后启用：

```text
pwm1-m2
```

重启后 `pulse2` 可以正常控制第二个电调。

结论：第二路最终使用 `pin16`。

## 为什么最终选择 Pin7 + Pin16

原因很简单：不是所有排针上标了 PWM 的脚，在当前系统和当前板子状态下都适合电调。

本机实测结果：

- `pin7`：可用，使用 `pwm3-m3`
- `pin16`：可用，使用 `pwm1-m2`
- `pin32`：一直高电平，不可用
- `pin35`：有电压变化，但电调不识别
- `pin36`：被系统背光占用，不可用

所以当前可靠接法是：

```text
电调 1 信号 -> pin7
电调 1 GND  -> pin6

电调 2 信号 -> pin16
电调 2 GND  -> pin14 或 pin20
```

## 开机行为

之前已经安装过 systemd 服务：

```text
esc-pwm.service
```

它开机后会执行：

```bash
/home/orangepi/esc_pin7_pwm.sh pulse 1500
```

也就是说，Linux 启动完成后，系统会自动给电调输出 `1500us` 中位/停止信号。
