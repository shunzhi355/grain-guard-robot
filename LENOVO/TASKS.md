## 结论

这个项目应按“两台主机、三层控制”拆分：

| 层级 | 职责 |
|---|---|
| 联想主机 | MID360驱动、SLAM/重定位、地图、路径规划、Nav2、障碍物感知，最终输出速度指令 |
| LPA3588 | UI、任务状态机、扦样机构、PCA9685、X2P伺服、导航指令接入、超时停车、急停 |
| STM32 | 遥控接收、手动/自动仲裁、底盘PWM、最终硬件看门狗 |

联想主机不应输出PWM，也不应操作PCA9685。对当前仓库来说，底盘最终PWM实际上已由STM32负责，3588通过USB-TTL发送运动量；PCA9685主要控制扦样机构。对应代码和说明见 [3588 README](../3588/README.md) 和 [CHASSIS_SERIAL.md](../3588/docs/CHASSIS_SERIAL.md)。

另外有三个必须先纠正的认识：

1. 原工程是ROS1，不是ROS2，建图脚本甚至会主动拒绝ROS2环境，[runtime.sh](deploy/mapping/runtime.sh)。这说明现有代码需要拆出硬件与业务逻辑，但不要求3588继续运行ROS。
2. Nav2是ROS2组件。新架构把ROS2限制在联想主机内部，双机之间只使用GRICP，因此不需要部署ROS1/ROS2桥，也不需要让3588加入联想的ROS Domain。
3. 当前是履带式底盘，只使用`linear.x`和`angular.z`，`vy/linear.y`没有执行通道，[chassis_node.py](../3588/src/grain_sampling_workflow/chassis_node.py)。因此接口应定义为`vx + wz`，强制`vy=0`。

Vault 中未找到足够依据；下面的拆解以当前项目代码、硬件配置和官方ROS/Nav2资料为依据。

---

## 联想主机工作清单

### L01．确定ROS迁移路线

建议分两期完成，不要把“跨主机拆分”和“ROS2/Nav2迁移”一次性混在一起。

- 第一阶段：先完成双机网络、MID360迁移、SLAM迁移和速度指令链路。
- 第二阶段：再接入ROS2 Nav2，替换当前直线点到点`goal_controller`。
- 联想若使用Ubuntu 22.04，ROS2 Humble与MID360官方驱动组合比较合适；Livox官方驱动同时支持MID360和ROS2 Humble，[Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)。
- 联想内部统一采用ROS2/Nav2；3588不运行ROS，两个软件栈只在GRICP消息边界相接。

交付物：

- ROS版本决策文档。
- 联想操作系统、ROS版本、Nav2版本锁定。
- 第三方仓库commit和补丁版本锁定。
- 明确联想内部ROS2话题与GRICP消息的唯一映射点。

### L02．重新设计雷达和双机网络

当前MID360是网口设备，地址为`192.168.1.116`，现有接收主机地址为`192.168.1.200`，[livox_config.json](config/livox_config.json)。

如果联想承担SLAM，最好让联想直接接收MID360原始UDP数据，不要先由3588生成ROS点云再转发，否则双机链路传输的就不只是几十字节的`cmd_vel`。

建议拓扑：

```text
MID360 ─┐
        ├─ 小型以太网交换机 ─ USB2.0网卡 ─ 联想主机
3588 ───┘
```

联想侧工作：

- 配置USB网卡固定名称、固定IP和固定路由。
- 将Livox配置里的`HOST_IP`改为联想雷达网卡地址。
- 避免雷达网段抢占默认路由。
- 关闭USB网卡节能、自动挂起。
- 配置防火墙放行Livox UDP端口及ROS通信。
- 双机配置时间同步，至少使用Chrony。
- 测试连续运行时丢包、抖动、USB网卡重连。
- 记录`ping`、`iperf3`、雷达包率和ROS话题频率基线。

验收：

- `/livox/lidar`和`/livox/imu`连续运行两小时无中断。
- 拔插联想USB网卡后，导航保持停止，网络恢复不会自动恢复运动。
- 雷达数据不需要通过3588进行高带宽ROS转发。

### L03．部署MID360驱动

在联想建立独立工作空间，安装：

- Livox-SDK2。
- `livox_ros_driver2`。
- MID360配置文件。
- 雷达静态地址和主机UDP端口配置。
- systemd启动服务和日志轮转。

需要验证：

- 雷达点云频率、IMU频率、时间戳是否单调。
- `frame_id`统一为`livox_frame`。
- 雷达断电、网线断开、驱动崩溃时能够产生明确故障状态。
- 驱动不能无限自动重启并在定位未恢复时重新放行导航。

### L04．迁移S-FAST_LIO建图和重定位

当前工程使用两种互斥进程：

- `sfastlio_mapping`：建图并输出PCD。
- `fastlio_mapping_re`：加载已有PCD进行重定位。

二者不能同时运行，[slam_bridge.py](src/grain_sampling_workflow/slam_bridge.py)。

联想侧需要：

- 移植仓库中现有S-FAST_LIO补丁。
- 建立建图、停止、保存、重定位四种生命周期。
- 建图启动前检查雷达和IMU。
- 保存时正确向S-FAST_LIO发送SIGINT，等待PCD真正写完。
- 重定位启动前校验地图存在、格式正确。
- 定位丢失时发布故障，立即停止速度输出。
- 发布定位质量指标，例如点云数量、匹配分数、里程计更新时间、跳变检测。
- 禁止建图和重定位同时运行。
- 保存地图时生成地图名称、仓库编号、时间、坐标原点和标定版本等元数据。

首要验收是恢复稳定的`/Odometry`。原交接文件明确记录该话题无数据，这是现阶段导航的直接阻塞项，[HANDOVER.md](../3588/HANDOVER.md)。

### L05．统一坐标系和机器人模型

当前工程同时出现了`camera_init`、`map`、`odom`和`base_link`，必须统一。

Nav2要求完整TF链：

```text
map → odom → base_link → livox_frame
```

这是Nav2的基本输入要求，[Nav2 TF说明](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/)。

联想侧要完成：

- 测量MID360相对底盘旋转中心的XYZ和RPY外参。
- 创建机器人URDF/Xacro。
- 发布`base_link → livox_frame`静态TF。
- 将S-FAST_LIO的`camera_init`输出规范化为`map/odom`体系。
- 发布`nav_msgs/Odometry`，包含合理的速度和协方差。
- 检查TF时间戳、方向和单位。
- 固定机器人正前方为X轴正方向、左侧为Y轴正方向、Z轴向上。

不能只把`camera_init`字符串改成`map`，必须验证其实际语义和变换关系。

### L06．建立Nav2所需的二维地图

当前S-FAST_LIO主要保存三维PCD，而Nav2全局规划通常需要二维占据栅格。

联想侧需要新增：

- PCD地面分割、障碍物高度过滤。
- PCD投影为二维`OccupancyGrid`。
- 生成`map.yaml + map.pgm/png`。
- 地图分辨率、原点和占据阈值配置。
- 地图版本和PCD版本绑定。
- 加载地图后验证与重定位坐标完全一致。
- 通过地图服务/GRICP状态向3588 UI提供低频地图，不跨机发布ROS `/map`。
- 提供地图列表、删除、重命名、选择和下载接口。

原UI直接读取本机PCD目录，现已增加远程地图清单入口；切片与上传仍需在联想地图服务实现，[mapping_page.py](../3588/src/grain_sampling_ui/pages/mapping_page.py)。

### L07．配置Nav2导航栈

如果采用ROS2 Nav2，联想侧需要配置：

- `map_server`
- 定位或S-FAST_LIO适配节点
- global costmap
- local costmap
- planner server
- controller server
- behavior server
- BT navigator
- lifecycle manager
- velocity smoother
- 可选collision monitor

具体参数包括：

- 履带车设为非全向底盘，`vy=0`。
- 机器人真实footprint，不用随意的小圆形代替。
- 最大线速度不超过当前3588默认的`0.3 m/s`。
- 最大角速度不超过当前默认的`0.8 rad/s`。
- 配置最小启动速度，解决履带静摩擦导致的小指令不动。
- 配置线加速度、角加速度、减速度。
- 配置倒车策略、原地旋转和恢复行为。
- 配置目标距离容差不大于当前工作流使用的`0.2 m`。
- 对粮仓过道、粮堆边缘调整膨胀半径和未知区域策略。
- 对控制器进行实车调参，不能直接使用TurtleBot默认参数。

Nav2速度平滑器支持加减速度限制和速度超时归零，可作为联想侧第一层保护，[Nav2 Velocity Smoother](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)。

### L08．实现目标点适配器

3588现有工作流曾使用：

- `/move_base_simple/goal`
- 类型`geometry_msgs/PoseStamped`
- 当前`frame_id`为`camera_init`
- 等待`/goal_controller/status`或`/waypoint_task_done`

对应旧代码见 [ros_bridge.py](../3588/src/grain_sampling_workflow/ros_bridge.py)。

联想需要提供GRICP目标适配器；旧ROS接口仅用于理解现有业务语义，不在3588继续运行：

1. 接收3588的目标点。
2. 校验目标坐标、四元数、地图版本。
3. 转换到`map`坐标系。
4. 调用Nav2的`NavigateToPose` Action。
5. 持续输出规划、控制、恢复、成功、失败、取消状态。
6. 成功后先停止速度输出，再返回`NAV_RESULT=SUCCEEDED`；3588本地状态机自行产生“到位”业务事件。
7. 失败时输出明确原因，例如：
   - 定位无效
   - 目标不可达
   - 规划失败
   - 控制超时
   - 遥控不在自动档
   - 底盘通信异常
   - 障碍物持续阻塞

### L09．实现唯一速度输出链路

建议联想内部链路：

```text
Nav2 controller
  → /cmd_vel_nav
  → velocity_smoother
  → navigation safety gate
  → GRICP MOTION_COMMAND（20～30 Hz）
  → 3588 ChassisController/ChassisSerial
```

输出要求：

- 类型最终兼容`geometry_msgs/Twist`。
- 只使用`linear.x`和`angular.z`。
- 强制`linear.y=0`。
- 频率建议20～30 Hz。
- 限幅为`|vx|≤0.3 m/s`、`|wz|≤0.8 rad/s`。
- 取消、故障、生命周期退出时立即连续发送若干零速。
- 任何非有限数值、过期指令、异常时间戳都变成零速。
- 系统中只能有一个自动速度发布者。
- 禁止旧`goal_controller`与Nav2同时输出控制量。

注意：当前系统没有看到轮速编码器闭环，3588只是将速度比例换算为归一化力度。FAST-LIO位置闭环不等于电机速度闭环。若必须实现真正的底盘速度闭环，需要另外增加编码器和STM32侧速度反馈控制，不能把它列为联想主机已有能力。

### L10．迁移障碍物感知

原项目通过`/cloud_registered_body`检测前方矩形区域障碍物，[stop_on_obstacle.py](src/grain_sampling_navigation/stop_on_obstacle.py)。

建议把点云障碍处理放到联想，避免将完整点云回传3588：

- 地面去除。
- 车体自身点云过滤。
- 盲区过滤。
- 前向硬停车区域。
- Nav2局部代价地图障碍层。
- 障碍状态防抖。
- 输出小数据量`OBSTACLE_STATUS`给3588。
- 无点云、点云超时应按故障处理，不能视为“前方无障碍”。

3588仍保留对`OBSTACLE_STATUS.detected=true`立即停车和取消导航的最终执行权。

### L11．实现导航管理服务

联想需要提供一个独立的导航管理节点，至少包含：

- 启动建图
- 停止建图
- 保存地图
- 启动重定位
- 停止重定位
- 获取地图列表
- 选择地图
- 发送导航目标
- 取消导航
- 查询导航状态
- 查询定位状态
- 查询雷达状态
- 查询节点健康状态

不要让3588 UI通过SSH、`pgrep`或远程文件路径直接管理联想进程。当前`SlamBridge`是本机进程管理器，迁移后需要改成网络服务客户端。

### L12．联想侧安全门控

联想侧可以增加软件保护，但不能代替3588和STM32：

- 接收3588的`ROBOT_STATUS.rc_mode`。
- 只有`rc_mode=auto`才允许输出非零速度。
- 手动、未知、超时立即输出零速并取消Nav2目标。
- `ROBOT_STATUS`表明底盘串口异常时停止导航。
- 联想本地定位/里程计超过规定时间未更新立即停车。
- 雷达点云超时立即停车。
- 规划器、控制器或桥接进程退出立即停车。
- 网络延迟过大时停止，不使用积压的旧速度指令。
- 网络恢复后不能自动继续旧目标，必须重新确认或重新下发目标。

3588已有200 ms指令超时，STM32还有300 ms底层超时和遥控优先，[CHASSIS_SERIAL.md](../3588/docs/CHASSIS_SERIAL.md)。这两层必须保留。

### L13．启动、监控和日志

联想需要提供统一启动服务，例如：

```text
grain-navigation.service
```

启动顺序：

1. 网络和时间同步
2. Livox驱动
3. TF/机器人模型
4. S-FAST_LIO建图或重定位
5. 二维地图
6. Nav2
7. GRICP客户端/网关
8. 目标适配器和安全门
9. 健康监控

必须保存：

- Livox日志
- SLAM日志
- Nav2规划/控制日志
- 桥接日志
- 最后一次目标和失败原因
- 网络断连次数
- cmd_vel频率和最大间隔
- 定位跳变和超时次数

---

## 建议固定的双机接口

| 方向 | 接口 | 说明 |
|---|---|---|
| 3588 → 联想 | `NAV_GOAL_REQUEST` | `map`坐标目标点和唯一`goal_id` |
| 3588 → 联想 | `NAV_CANCEL` | 取消当前导航 |
| 3588 → 联想 | `ROBOT_STATUS` | 遥控模式、串口、急停、底盘和机构状态 |
| 3588 → 联想 | `SLAM_COMMAND` | 建图、保存、重定位 |
| 联想 → 3588 | `MOTION_COMMAND` | 20～30 Hz，只允许`vx/wz`且`vy=0` |
| 联想 → 3588 | `MOTION_STOP` | 到点、取消、定位失效或导航故障时显式停车 |
| 联想 → 3588 | `POSE_STATUS` | 低频位姿，供UI显示和到点复核 |
| 联想 → 3588 | `NAV_STATUS/NAV_RESULT` | 结构化导航状态和最终结果 |
| 联想 → 3588 | `OBSTACLE_STATUS` | 小数据量硬停车状态 |
| 联想 → 3588 | `SLAM_STATUS` | 建图、保存和重定位状态 |
| 双向 | `HEARTBEAT` | 会话健康心跳；不得为旧速度续期 |

`/Laser_map`、`/cloud_registered_body`等大点云原则上留在联想，仅在调试时按需传输；3588不订阅这些ROS话题。

---

## 3588必须配合修改的最小项目

虽然主体工作在联想，但以下3588改动不可省略：

- 生产模式不启动`roscore`、`rospy`、`chassis_node`或`mechanism_node`。
- 增加非ROS的`grain_robot_daemon`，由它唯一占用STM32底盘串口，并通过现有`ChassisSerial`输出控制量。
- 联想启用Nav2后，禁止启动旧`goal_controller`，避免双控制器抢占。
- 新导航目标开始时由本地`MotionGuard`完成授权并生成`motion_epoch`，未授权速度必须丢弃。
- 保留200 ms速度断流停车。
- 保留STM32遥控优先、急停和300 ms看门狗。
- 将UI的`SamplingBridge`、`RosThread`和本地`SlamBridge`替换为守护进程本机IPC客户端。
- 从`mechanism_node.py`提取非ROS的机构控制器，保留PCA9685、X2P和三仓流程的本地独立性。
- 急停必须直接调用本地底盘与机构控制器，不能依赖联想在线。
- 如果实机底盘电调确实仍由PCA9685而不是STM32驱动，必须先完成“代码与实机接线一致性核查”，再决定保留哪个3588底层驱动，不能两套并存。

---

## 最终验收清单

- 联想进程被强制结束，机器人自动停车。
- 拔掉双机网线或USB网卡，机器人自动停车。
- 杀死GRICP进程或断开会话，机器人自动停车。
- `MOTION_COMMAND`停止超过200 ms，3588停车。
- 3588进程崩溃后，STM32在300 ms内停车。
- 自动导航过程中切到手动档，遥控接管。
- 手动档切回自动档后，不自动恢复旧目标。
- 雷达断线、定位丢失、里程计跳变均停车。
- 急停保持锁定，网络恢复不能解除急停。
- `linear.y`始终为零。
- 同一时刻只有一个自动速度发布者。
- 建图和重定位不能同时运行。
- 地图保存后可以重启联想并重新定位。
- UI能够正常显示地图、位置、导航状态和故障原因。
- 到点误差不大于现有流程要求的0.2 m。
- 完成多点任务：导航→到点→扦样→下一点→返回起点。
- 连续运行至少4小时，无速度断流抖动、地图漂移失控或资源持续增长。
