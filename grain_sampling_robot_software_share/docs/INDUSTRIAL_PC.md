# ARM Linux 工控机硬件适配

当前接线以本文件和根目录 WIRING.md 为准。工控机使用 `scripts/start_industrial_pc.sh`，旧 Orange Pi / Jetson 启动脚本仍供历史部署使用。

## 默认映射

| 设备 | 接口与软件默认值 |
|---|---|
| 遥控接收机 | USB1 → USB-TTL → i-BUS；`/dev/ttyUSB0`，115200，8N1 |
| X2P 升降伺服 | USB3 → USB-RS485；`/dev/ttyUSB1`，原 Modbus 参数保持 |
| PCA9685 | LVDS Pin36 SDA、Pin38 SCL；`/dev/i2c-2`，0x40，50Hz |
| 螺旋输送1/2 | CH0/CH1 |
| 浅仓/中仓/深仓 | CH2/CH3/CH4 |
| 夹紧/拧紧 | CH5/CH6 |
| 左/右底盘电调 | CH8/CH9 |

USB 物理位置不对应 ttyUSB 编号；I²C 总线和设备名均为待现场核实的默认值。CH7 仍为未接线占位，CH10–15 未使用。

接收机沿用项目已有 FS-iA10B i-BUS 协议：CH1 前后、CH3 转向、CH8 模式开关（软件内部仍叫 CH5）。USB-TTL 只负责串口传输；若现场测试使用其他协议，需要相应更换解析器。

PCA9685 振荡器校准、各机构动作脉宽、底盘方向、遥控死区与速度标定沿用原值，本次未重新标定。机构初始化只写 CH0–6，底盘由独立驱动写 CH8/CH9。

## 启动与配置

需要 Linux i2c-dev、Python 3、与 Python 版本匹配的 ROS1（rospy）、pyserial，以及已编译的 mechanism_node 消息包。完整项目的 setup.py 声明 Python >=3.10；使用厂商镜像时须确认 Python/ROS1 兼容性。ROS1 从系统或对应源码工作空间安装，不通过 pip 安装 rospy。UI 的 Qt/OpenCV 依赖不影响此硬件节点入口。

进入 `grain_sampling_robot_software_share` 项目目录运行：

```bash
bash scripts/start_industrial_pc.sh
```

启动配置集中在 `config/industrial_pc.env`；环境变量优先于文件默认值，例如：

```bash
RC_SERIAL_PORT=/dev/rc_receiver X2P_PORT=/dev/x2p_lift \
PCA9685_I2C_DEVICE=/dev/i2c-2 bash scripts/start_industrial_pc.sh
```

`ROS_SETUP_BASH` 可指定 ROS1 setup.bash；`MECHANISM_WS_SETUP` 可指定机制消息包工作空间 setup.bash；`GRAIN_HARDWARE_CONFIG` 可指定另一份硬件环境配置。直接运行 Python 节点时，默认参数来自 `sampling_params.py`，也可使用同名环境变量覆盖设备路径。

脚本检查依赖和设备访问权限，启动 roscore、机构节点、遥控节点、底盘 daemon 和 cmd_vel 桥。它不启动 UI、相机、雷达和导航；这些模块仍需各自的 ROS 工作空间与部署配置。

日志默认位于 `/tmp/grain_sampling_robot`。重复运行不会重启已有节点，因此修改硬件配置后需先停止旧进程，再重新启动。旧 `start.sh`、`start_robot_all.sh`、`esc-pwm.service` 和 `esc_pin7_pwm.sh` 针对旧板，不作为本工控机的启动入口。

## 输出与失联行为

- Linux 默认使用 PCA9685 和 USB i-BUS，设备缺失会报错；Windows 开发默认仍为模拟。历史 GPIO/sysfs 后端需显式选择。
- 底盘 daemon 在 CH8/CH9 输出 1500us 中位并保持 3 秒，再接受新运动命令；命令超过 0.3 秒未更新时回中位。
- 打开共用 PCA9685 不再全通道关闭；相同频率不会重新设置振荡器。有活动输出时，拒绝改变不一致的频率配置。
- 驱动用线程锁和 Linux flock 协调项目运行进程；单通道四个寄存器以一次 I²C 写入更新。
- USB 串口断开会立即清除旧指令；无有效帧超过原有 0.5 秒阈值后，手动控制输出零速度。串口重新接入后需重启遥控节点。接收机若在无线失联后继续发送保持值，仍需现场设置接收机自身的 failsafe。

`dipan/pca9685` 中的历史独立诊断工具不参与运行驱动的锁协议，仅在控制节点全部停止后使用；运行中不能使用其 init/all-off/frequency 命令。

## 手册核对与现场测试

工控机手册 `fe1160432813cde211bdbb12b33af6d8.pdf` 的 PDF 第14–15页（印刷页13/17、14/17）确认：

- LVDS Pin36/38 为 I2C2_SDA_M4 / I2C2_SCL_M4，信号电平标为 **1.8V**。
- LVDS Pin1/2/3 的 VCC_LVDS 通过 J27 选择 **3.3V/5V**；按用户要求使用3.3V。
- LVDS Pin4/5/6 为 GND。

VCC 3.3V 与 I²C 信号电平是不同参数，上电前需核对 J27、电平及 PCA9685 模块上拉是否匹配，必要时使用双向电平转换。软件不能改变引脚电压或代替设备树启用 I²C。

后续用 `i2cdetect -l`、`/dev/serial/by-id/` 和 `/dev/serial/by-path/` 核实映射，调整配置与 i2c/串口设备权限。实机仍需验证中位脉宽、左右方向、USB 失联停车和机构/底盘同时运行。

## 本地验证

硬件回归测试位于 `test/test_industrial_hardware.py`，以模拟寄存器与串口验证共用芯片、通道映射和失联行为。可与已有机构、遥控、伺服测试一起运行：

```bash
python -m pytest -o addopts='-q --tb=short' test/test_industrial_hardware.py test/test_mechanism_driver.py test/test_mechanism_node.py test/test_rc_receiver.py test/test_rc_control.py test/test_x2p_lift.py
```

本地测试不能替代 ARM 工控机、ROS1 和实体设备的联调。

## UI 启动与无硬件下潜预览

UI 是工控机桌面上的 Qt 原生窗口，不是网页。通过 HDMI/DP 显示器、已配置的触摸屏或远程桌面查看；普通 SSH 终端不能直接显示窗口。
新增入口会先加载系统安装的 Qt，避开 src/PySide2 与 src/PySide6 兼容文件的循环导入。Ubuntu 22.04 可安装系统 PySide2 的 qtcore、qtgui、qtwidgets、qtnetwork 包。

```bash
# 在工控机桌面终端运行，ROBOT_PROJECT 使用实际项目目录
python3 "$ROBOT_PROJECT/scripts/start_ui.py" --check-qt

# 正常主界面：保持独立 rc_node 唯一读取接收机
export GRAIN_SAMPLING_UI_RC_PUBLISH=0
unset QT_QPA_PLATFORM
python3 "$ROBOT_PROJECT/scripts/start_ui.py"

# 单独的无硬件界面预览，不与真实任务混用
python3 "$ROBOT_PROJECT/scripts/start_ui.py" --preview-descent
```

预览窗口明确标为模拟，点击“已就绪”后显示下压状态，停留在该页面；不启动 ROS 节点、导航、PCA9685、伺服或机构编排器。它只验证界面状态转换，不能证明真实下潜链路正常。
真实流程仍需导航完成反馈以及启用真实机构控制；现有编排器默认机构占位，主界面的任务创建入口尚未接通 enable_mechanism，不能仅凭“正在下压”文字认定伺服已执行。

示波器台架测试保持电机主电源断开，仅启动 roscore、rc_node、cmd_vel_to_motor 和 motor_driver。底盘驱动可使用 --max-offset-us 100 --no-start-boost 限制 CH8/CH9 为约1400–1600us，目标频率50Hz，回中约1500us。不启动机构节点、目标导航或旧独立 PWM 工具。示波器探头测 CH8/CH9 Signal，参考地接 PCA9685 GND。
