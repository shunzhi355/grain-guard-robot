# 联想导航主机代码

联想主机负责 MID360 驱动、SLAM/重定位、地图管理、路径规划、Nav2、障碍检测和速度生成。3588 是 GRICP 服务端，联想是客户端；联想不直接操作底盘串口、PCA9685、X2P 或电调 PWM。

当前已从原工程迁入：`deploy/mapping/`、`config/livox_config.json`、`src/grain_sampling_pointcloud/`、雷达探测及建图维护脚本、旧障碍检测节点、旧 SLAM 进程管理器与旧目标控制器。`src/grain_sampling_interhost/protocol.py` 是与3588一致的 GRICP v1 帧格式副本。

当前文件主要是 ROS1 历史实现和迁移基线；ROS2/Nav2 与 GRICP 的联想侧运行程序尚未实现。详见 [TASKS.md](TASKS.md) 与根目录 [双机通信协议.md](../双机通信协议.md)。因此此目录目前不能作为联想主机的完整导航部署包。
