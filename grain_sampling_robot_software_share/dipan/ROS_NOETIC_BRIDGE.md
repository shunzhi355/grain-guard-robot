# ROS Noetic 接入 `/cmd_vel`

本目录新增 ROS1 桥接脚本：

```bash
/home/orangepi/底盘/cmd_vel_to_motor.py
```

作用：

```text
/cmd_vel geometry_msgs/Twist
        -> cmd_vel_to_motor.py
        -> UDP: cmd linear angular
        -> motor_driver.py daemon
        -> pin7 / pin16 PWM
        -> 两个电调
```

## 1. 启动 ROS

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

## 2. 启动电机 UDP daemon

终端 2：

```bash
cd /home/orangepi/底盘
sudo ./motor_driver.py daemon --host 127.0.0.1 --port 8765 --timeout 0.3
```

说明：

```text
--timeout 0.3
```

表示 0.3 秒没有收到新速度命令就自动 `stop`。

## 3. 启动 `/cmd_vel` 桥接节点

终端 3：

```bash
source /opt/ros/noetic/setup.bash
cd /home/orangepi/底盘
./cmd_vel_to_motor.py
```

默认参数：

```text
max_linear_mps  = 0.5
max_angular_rps = 1.0
```

也就是说：

```text
/cmd_vel linear.x = 0.5  -> motor_driver cmd linear = 1.0
/cmd_vel angular.z = 1.0 -> motor_driver cmd angular = 1.0
```

可以启动时改参数：

```bash
./cmd_vel_to_motor.py _max_linear_mps:=0.3 _max_angular_rps:=0.8
```

如果方向反了：

```bash
./cmd_vel_to_motor.py _invert_linear:=true
```

如果左右转向反了：

```bash
./cmd_vel_to_motor.py _invert_angular:=true
```

## 4. 用 rostopic 测试

低速前进：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

前进并左转：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3"
```

停止：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## 5. 后续 FAST-LIO/寻迹结构

FAST-LIO 输出定位，不直接控制电机。

推荐链路：

```text
FAST-LIO odom / pose
        -> 寻迹控制器
        -> 发布 /cmd_vel
        -> cmd_vel_to_motor.py
        -> motor_driver.py daemon
        -> PWM / 电调 / 履带
```

寻迹控制器只需要发布标准 ROS `/cmd_vel`：

```text
linear.x  前进速度 m/s
angular.z 转向速度 rad/s
```

底层电机安全超时由 `motor_driver.py daemon --timeout 0.3` 负责。
