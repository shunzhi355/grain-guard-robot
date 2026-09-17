# ARM Linux 工控机硬件适配

当前接线以本文件和根目录 WIRING.md 为准。工控机使用 `scripts/start_industrial_pc.sh`，旧 Orange Pi / Jetson 启动脚本仍供历史部署使用。

## 默认映射

| 设备 | 接口与软件默认值 |
|---|---|
| 遥控接收机 | USB1 → USB-TTL → i-BUS；`/dev/ttyUSB0`，115200，8N1 |
| X2P 升降伺服 | USB3 → USB-RS485；`/dev/ttyUSB1`，原 Modbus 参数保持 |
| PCA9685 | TP Pin5 SDA、Pin4 SCL；暂定 `/dev/i2c-4`（I2C4，Linux 映射待核实），0x40，50Hz |
| 螺旋输送1/2 | CH0/CH1 |
| 浅仓/中仓/深仓 | CH2/CH3/CH4 |
| 夹紧/拧紧 | CH5/CH6 |
| 左/右底盘电调 | CH10/CH9（CH8 实机异常后改线） |

USB 物理位置不对应 ttyUSB 编号；I²C 总线和设备名均为待现场核实的默认值。CH7 仍为未接线占位，CH10–15 未使用。

接收机沿用项目已有 FS-iA10B i-BUS 协议：CH1 前后、CH3 转向、CH8 模式开关（软件内部仍叫 CH5）。USB-TTL 只负责串口传输；若现场测试使用其他协议，需要相应更换解析器。

PCA9685 振荡器校准、各机构动作脉宽、底盘方向、遥控死区与速度标定沿用原值。机构初始化只写 CH0–6，底盘由独立驱动写左 CH10、右 CH9。

## 启动与配置

需要 Linux i2c-dev、Python 3、与 Python 版本匹配的 ROS1（rospy）、pyserial，以及已编译的 mechanism_node 消息包。完整项目的 setup.py 声明 Python >=3.10；使用厂商镜像时须确认 Python/ROS1 兼容性。ROS1 从系统或对应源码工作空间安装，不通过 pip 安装 rospy。UI 的 Qt/OpenCV 依赖不影响此硬件节点入口。

进入 `grain_sampling_robot_software_share` 项目目录运行：

```bash
bash scripts/start_industrial_pc.sh
```

启动配置集中在 `config/industrial_pc.env`；环境变量优先于文件默认值，例如：

```bash
RC_SERIAL_PORT=/dev/rc_receiver X2P_PORT=/dev/x2p_lift \
PCA9685_I2C_DEVICE=/dev/i2c-4 bash scripts/start_industrial_pc.sh
```

`ROS_SETUP_BASH` 可指定 ROS1 setup.bash；`MECHANISM_WS_SETUP` 可指定机制消息包工作空间 setup.bash；`GRAIN_HARDWARE_CONFIG` 可指定另一份硬件环境配置。直接运行 Python 节点时，默认参数来自 `sampling_params.py`，也可使用同名环境变量覆盖设备路径。

脚本检查依赖和设备访问权限，启动 roscore、机构节点、遥控节点、底盘 daemon 和 cmd_vel 桥。它不启动 UI、相机、雷达和导航；这些模块仍需各自的 ROS 工作空间与部署配置。

日志默认位于 `/tmp/grain_sampling_robot`。重复运行不会重启已有节点，因此修改硬件配置后需先停止旧进程，再重新启动。旧 `start.sh`、`start_robot_all.sh`、`esc-pwm.service` 和 `esc_pin7_pwm.sh` 针对旧板，不作为本工控机的启动入口。

## 输出与失联行为

- Linux 默认使用 PCA9685 和 USB i-BUS，设备缺失会报错；Windows 开发默认仍为模拟。历史 GPIO/sysfs 后端需显式选择。
- 底盘 daemon 在 CH10/CH9 输出 1500us 中位并保持 3 秒，再接受新运动命令；命令超过 0.3 秒未更新时回中位。
- 打开共用 PCA9685 不再全通道关闭；相同频率不会重新设置振荡器。有活动输出时，拒绝改变不一致的频率配置。
- 驱动用线程锁和 Linux flock 协调项目运行进程；单通道四个寄存器以一次 I²C 写入更新。
- USB 串口断开会立即清除旧指令；无有效帧超过原有 0.5 秒阈值后，手动控制输出零速度。串口重新接入后需重启遥控节点。接收机若在无线失联后继续发送保持值，仍需现场设置接收机自身的 failsafe。

`dipan/pca9685` 中的历史独立诊断工具不参与运行驱动的锁协议，仅在控制节点全部停止后使用；运行中不能使用其 init/all-off/frequency 命令。

## 手册核对与现场测试

工控机手册 `fe1160432813cde211bdbb12b33af6d8.pdf` 的 TP 表（印刷页12/17）确认：

- TP Pin5 为 I2C4 SDA、Pin4 为 I2C4 SCL，信号电平为 **3.0V**。
- TP Pin1 为 VCC3V0_TOUCH（3.0V），接 PCA9685 VCC；Pin6 为 GND。
- TP Pin2 INT、Pin3 RST 不接 PCA9685。模块上拉必须匹配 3.0V 信号。

软件暂以 `/dev/i2c-4` 作为 TP 默认路径，环境变量仍可覆盖。硬件 I2C4 与 Linux 设备编号需核实，不能仅凭编号确认引脚。

现场反馈：内核 6.1.84，已有多个 `/dev/i2c-*`，但没有 `/dev/i2c-4`，现有节点权限为 root 专用；`i2cdetect` 未安装，`modprobe i2c-dev` 找不到模块。已有设备节点说明不能单凭 modprobe 报错判断 I²C 不可用，驱动可能内建于内核。

先在工控机执行以下诊断（不启动电机、不写 PCA9685）：

```bash
uname -r
for d in /sys/class/i2c-dev/i2c-*; do
    [ -e "$d" ] || continue
    echo "=== ${d##*/} ==="
    cat "$d/name"
    readlink -f "$d/device/of_node"
done
for a in /proc/device-tree/aliases/i2c*; do
    [ -f "$a" ] || continue
    printf '%s: ' "${a##*/}"
    tr -d '\000' < "$a"
    printf '\n'
done
```

根据设备树路径核对 TP 所属控制器及 Linux 总线编号。若控制器未启用，需修改匹配本机镜像的设备树及引脚复用后重启；不要套用 Orange Pi overlay。若已映射为其他编号，用 `PCA9685_I2C_DEVICE=/dev/i2c-X` 覆盖。`export` 不会创建设备节点。

Debian/Ubuntu 可用 `sudo apt-get update && sudo apt-get install i2c-tools` 安装诊断工具，再运行 `i2cdetect -l`。确认总线后单独处理设备权限（i2c 组和 udev 规则）；当前 root 专用权限会阻止普通用户运行。实机仍需验证 PCA9685 通信、中位脉宽、左右方向、USB 失联停车和机构/底盘同时运行。

## TP I2C4 启用镜像准备

现场进一步确认：板型 `neardi,lpb3588-linux-f0,`，I2C4 节点
`/i2c@feac0000` 为 `disabled`，引脚组为 `i2c4m0-xfer`。
启动分区为 `/dev/mmcblk0p3`（64 MiB），`/boot` 为空。
已校验的备份位于工控机 `/home/neardi/tp-i2c4.GEcNa9/boot.original.img`。
该 FIT 使用外部 fdt/kernel/resource 数据及 SHA-256；所贴出的 signature
节点尚未显示签名 value，必须检查实际文件，不能据此认定签名已关闭。

将 `scripts/prepare_tp_i2c4.py` 同步到工控机项目后，在项目目录运行：

```bash
python3 scripts/prepare_tp_i2c4.py \
  /home/neardi/tp-i2c4.GEcNa9/boot.original.img \
  /home/neardi/tp-i2c4.GEcNa9/boot.tp-i2c4-v2.img
```

脚本只依赖 Python 标准库，只接受普通备份文件，并独占创建新文件；不会刷写分区或覆盖已有文件。
它先校验三个数据段的 SHA-256、配置引用、板型、I2C4 别名和引脚组。
存在实际 FIT 签名 value 时拒绝修改，需要对应签名流程；不会删除签名。
修改以 FDT_NOP 填补缩短的 status 属性占用空间，保持镜像大小和所有数据偏移，
第二版同时将 FIT fdt 和 resource 中 `rk-kernel.dtb` 的 I2C4 status 改为 okay。
同时更新资源条目已有的 SHA-1/SHA-256，以及 FIT 的 fdt/resource SHA-256。
仅接受 RSCE v0、单个默认 rk-kernel.dtb；存在多份 DTB 或格式不匹配时停止。
生成后重新解析设备树并核对其他属性、内核、非 DTB 资源和所有允许范围外的字节未改变。

第一版仅修改 FIT fdt；现场确认已写入并重启（分区和镜像 SHA-256 均为
`331756fc8af097b80e76386fa44ba8fe26dd337b78fe7eb7017fd2d64fcdeb20`），
运行设备树仍为 disabled。Rockchip U-Boot 在启用 RESOURCE_IMAGE 时可优先读取
resource 内 DTB，第一版没有覆盖这一路径。
源码依据：https://github.com/rockchip-linux/u-boot/blob/next-dev/arch/arm/mach-rockchip/boot_rkimg.c
资源格式依据：https://github.com/rockchip-linux/u-boot/blob/next-dev/arch/arm/mach-rockchip/resource_img.c

第二版仍应以 `boot.original.img` 为输入，不覆盖第一版或原始备份。
输出应包含 `script_version: 2` 和 `resource_dtb_status: okay`。
本地 13 项合成 FIT 测试通过，不能证明实际固件采用的引导路径或实体机器能启动；
第二版真实备份处理、部署、重启后设备节点及 PCA9685 通信验证仍待完成。

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
