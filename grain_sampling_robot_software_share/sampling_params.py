"""粮食扦样机器人 —— 统一标定参数（唯一数据源）。

所有需要现场标定/调整的参数集中在本文件，一次修改全局生效。
各模块（mechanism_driver / mechanism_config / rc_control / rc_receiver /
x2p_lift / mechanism_node / utils.config）均从本文件导入默认值。

修改约定：
* 脉宽单位 us（合法范围 1000~2300，历史约定已放宽；写驱动前会钳制到该范围）。
* 时长单位 秒。
* 品种键与 mechanism_config.GRAIN_MECHANISM_CONFIG 一致。
* 方向类参数：forward_sign 仅允许 1（正向）或 -1（反向翻转）。

字段分布
--------
1. 机制执行器（PCA9685 通道映射 + 脉宽 + 品种参数）
2. 遥控（RC 通道 GPIO + 死区 + 档位带 + 速度标定）
3. X2P 伺服升降（串口 + Modbus + 转速 + 时长 + 方向）
4. 云端（base URL + 设备 MAC）
"""

from __future__ import annotations

# 正式流程及独立测试共用；修改后重启机构服务。
PRESS_SEGMENT_CM = 20.0
PRESS_DOWN_CM = 5.0
PRESS_UP_CM = 2.0
PRESS_PAUSE_S = 0.5

# ===========================================================================
# 1. 机制执行器 —— PCA9685 通道映射 + 脉宽 + 品种参数
# ===========================================================================

#: 执行器 → PCA9685 通道映射（物理接线确认 2026-08，用户已确认）。
#: CH0=螺旋输送1、CH1=螺旋输送2、CH2=开仓(浅)、CH3=开仓(中)、CH4=开仓(深)、
#: CH5=夹紧、CH6=拧紧、CH7=负压风机（未接线，占位）。
CHANNELS: dict[str, int] = {
    "convey_1": 0,      # 螺旋输送 1
    "convey_2": 1,      # 螺旋输送 2
    "bin_shallow": 2,   # 开仓(浅)
    "bin_mid": 3,       # 开仓(中)
    "bin_deep": 4,      # 开仓(深)
    "clamp": 5,         # 夹紧
    "tighten": 6,       # 拧紧
    "fan": 7,           # 负压风机（未接线，占位）
}

# LPB3588 实机确认 PCA9685 映射到 Linux I2C2，可用环境变量覆盖。
PCA9685_I2C_BUS: int = 2
PCA9685_I2C_ADDRESS: int = 0x40
PCA9685_I2C_DEVICE: str = ""  # 由 config/industrial_pc.env 设置为 /dev/i2c-2
PCA9685_CHASSIS_LEFT: int = 10  # CH8 实机异常，左侧信号改接 CH10（2026-09-16）
PCA9685_CHASSIS_RIGHT: int = 9

#: PCA9685 内部振荡器实际频率（Hz）。标称 25MHz，但实机示波器校准
#: 2026-08-23：PRESCALE=121 时实测 55.25Hz → 振荡器 = 55.25×4096×122
#: = 27,609,088 Hz（偏差 +10.4%）。用实测值计算 PRESCALE 才能得到
#: 精确 50Hz 输出。若换 PCA9685 板需重新校准。
#: 2026-08-31 复校（两次实测取平均）：
#:   PRESCALE=134 → 49.75Hz → 振荡器 27,509,760
#:   PRESCALE=133 → 50.25Hz → 振荡器 27,580,416
#:   平均 = 27,545,088 → PRESCALE=134 得 49.82Hz(20.07ms) 最接近 50Hz。
#: 注：PCA9685 内部 RC 振荡器会温漂，prescale 为整数，±0.5% 内已是最优。
PCA9685_OSCILLATOR_HZ: float = 27_545_088.0

#: PCA9685 PWM 目标频率（Hz）。默认 50（电调/舵机标准）。
PCA9685_FREQUENCY_HZ: float = 50.0

#: 脉宽标定值（us）：板端实机确认 —— 
#: 输送（CH0/1）用全局 open/close（throttle 品种参数默认值）。
#: 三仓（CH2/3/4）为独立标定（见 BIN_OPEN_PULSE/BIN_CLOSE_PULSE，与品种无关）。
#: CH5 夹紧、CH6 拧紧为独立标定（见下），不走全局 open/close。
#: 电调需先收到中位信号初始化（1500us）才能正常响应控制。
#: 注意：停止语义已改为"断电释放"（channel_off），PULSE_STOP 仅作参考保留。
PULSE_OPEN: float = 1200.0    # 开（输送 throttle 默认值，实机标定 1200us）
PULSE_CLOSE: float = 1900.0   # 关（已改用断电释放 channel_off，此默认值保留占位）
PULSE_STOP: float = 1500.0    # 停（初始化中位参考值，实际停止走 channel_off）

#: CH5 夹紧独立标定（us）：夹紧=1900，松开=1200（实测确认）。
CLAMP_PULSE_CLOSE: float = 1900.0
CLAMP_PULSE_OPEN: float = 1200.0

#: CH6 拧紧独立标定（us）：拧紧=1300，拧松=1900（实测确认）。
TIGHTEN_PULSE_CLOSE: float = 1300.0
TIGHTEN_PULSE_OPEN: float = 1900.0

#: 三仓（CH2/3/4）开关仓独立标定（us）：开仓门=1200，关仓门=1800
#: （2026-08-31 用户实机标定）。与品种无关，不走 throttle_open/close。
BIN_OPEN_PULSE: float = 1200.0
BIN_CLOSE_PULSE: float = 1800.0

#: 全局脉宽合法范围（us）：写入校验用（set_pwm/actuate 时钳制到该范围）。
PULSE_MIN_US: float = 1000.0
PULSE_MAX_US: float = 2300.0

#: 支持的品种（与 GRAIN_MECHANISM_CONFIG 键一致，可扩展）。
SUPPORTED_GRAINS: tuple[str, ...] = ("稻谷", "玉米", "黄豆")

#: 未接线通道（press/lift/fan 等）是否真实写 I2C。
#: False=占位 no-op（当前未接线状态）；接好线后改 True 才会真实写。
ENABLE_UNWIRED_CHANNELS: bool = False

#: 未知品种的兜底时序参数（秒）。
DEFAULT_GRAIN_PARAMS: dict[str, float] = {
    "sampling_duration": 120.0,  # 扦样时长，默认 2 min
    "convey_duration": 120.0,    # 输送时长，默认 2 min
    "open_duration": 5.0,        # 开仓保持时长（秒），之后自动关同仓（暂定 5s，以实测为准）
    "close_duration": 3.0,       # 关仓时长
    "clamp_duration": 2.0,       # 夹紧时长（实机确认 2s）
    "unclamp_duration": 3.0,     # 松开时长（实机确认 3s）
    "tighten_duration": 10.0,    # 拧紧时长（用户 2026-09 标定 10s）
    "untighten_duration": 3.0,   # 旋松时长
    "throttle_open": 1200.0,     # 输送/节流开（实机标定：开=1200us）
    "throttle_close": 1400.0,    # 输送/节流关：已改用断电释放(channel_off)，此脉宽值保留占位
    "stop_value": 1500.0,        # 油门/节流停（初始化中位参考值）
}

#: 品种 → 机制参数（在默认参数上覆盖扦样/输送时长）。
GRAIN_MECHANISM_CONFIG: dict[str, dict[str, float]] = {
    "稻谷": {**DEFAULT_GRAIN_PARAMS, "sampling_duration": 120.0, "convey_duration": 120.0},
    "玉米": {**DEFAULT_GRAIN_PARAMS, "sampling_duration": 180.0, "convey_duration": 180.0},
    "黄豆": {**DEFAULT_GRAIN_PARAMS, "sampling_duration": 90.0, "convey_duration": 90.0},
}

# ===========================================================================
# 2. 遥控 —— RC 通道 GPIO + 死区 + 档位带 + 速度标定
# ===========================================================================

#: 旧 GPIO 接收方式的逻辑通道 → sysfs GPIO 号（板端实测）。
#: CH1 → Pin15 gpio-34、CH3 → Pin22 gpio-40、CH5(接收机CH8模式开关) → Pin40 gpio-111。
#: 注：CH5 原接 Pin24 gpio-44（SPI0_CS0）抖动、Pin16 gpio-35 读到摇杆信号，
#: 2026-09-03 最终改接 Pin40 gpio-111（GPIO3_B7，纯 GPIO）。
RC_PINS: dict[str, int] = {"CH1": 34, "CH3": 40, "CH5": 111}

# 工控机 USB1 -> USB-TTL -> FS-iA10B i-BUS；串口失败报错，不回退 GPIO。
# i-BUS 实机扫描确认：物理 CH3=油门，CH1=转向，CH8=手动/自动模式。
RC_RECEIVER_BACKEND: str = "ibus"
RC_SERIAL_PORT: str = "/dev/ttyUSB0"
RC_SERIAL_BAUDRATE: int = 115200

#: 摇杆中心脉宽（us）。
RC_STICK_CENTER: float = 1450.0

#: 摇杆死区（us）：区间内摇杆映射为 0，避免抖动误动。
#: 死区 1350~1750us（2026-08-26 因杜邦线串扰 ~1666us 加宽，包住串扰噪声；中心对齐实测摇杆中心 1410us。）
RC_DEADBAND_LOW: float = 1350.0
RC_DEADBAND_HIGH: float = 1750.0

#: CH5 档位带（us，含端点）：manual≈700~1450us / auto≈1550~2300us。
#: （用户 2026-09-03 重新标定：CH8→Pin40 gpio-111，manual 实测 839/1111 跳动、
#:  auto 实测 1978；阈值 1500 分界，留 1450~1550 死区避免边界抖动。）
RC_MODE_RANGES: dict[str, tuple[float, float]] = {
    "manual": (700.0, 1450.0),
    "auto": (1550.0, 2300.0),
}

#: 档位切换消抖采样数（连续 N 次一致才生效）。
RC_DEBOUNCE_SAMPLES: int = 5

#: 速度全量程标定（与 cmd_vel_to_motor 匹配）。
#: （用户 2026-08-26 重新标定：最大 0.3 m/s / 0.8 rad/s，降低手动控制最高速度更安全。）
RC_MAX_LINEAR_MPS: float = 0.3   # 全速前进 m/s
RC_MAX_ANGULAR_RPS: float = 0.8  # 全速转向 rad/s

# ===========================================================================
# 3. X2P 伺服升降 —— 串口 + Modbus + 转速 + 时长 + 方向
# ===========================================================================

#: LPB3588 板载 RS485；2026-09-17 实机通讯确认为 ttyS0。
X2P_PORT: str = "/dev/ttyS0"

#: Modbus 从站地址（与 x2p_config.json 一致）。
X2P_SLAVE: int = 2

#: 升降转速（r/min）。同时作为 move-timed 距离运动的速度上限（max_rpm）。
#: 注意：X2P 文档标称电机额定 120 r/min，超过需现场确认安全，勿长期超速。
#: 2026-08-27 用户实机标定确认 500 r/min 可用；自动回程靠近顶端时
#: 先使用 200 r/min 的保守速度，减小刹车惯性和冲顶风险。
X2P_RPM: int = 200

#: 升降时长（秒）。
X2P_DURATION_S: float = 2.0

#: 升降方向：1=正向，-1=反向翻转（实机方向不对时改这里）。
X2P_FORWARD_SIGN: int = 1

#: 自动下压循环回程时不贴回顶部机械原点，保留的安全距离（mm）。
#: 该余量必须大于位置容差，防止刹车惯性导致冲顶。
X2P_RETURN_CLEARANCE_MM: float = 5.0

#: 编码器距离动作的停止后位置容差（mm）。
X2P_POSITION_TOLERANCE_MM: float = 2.0

#: x2p 包所在目录（dais516 仓库根）；板端部署后如不在 sys.path 填绝对路径。
X2P_PACKAGE_PATH: str = ""

# ===========================================================================
# 4. 云端 —— base URL + 设备 MAC
# ===========================================================================

#: 云端 API base URL。
CLOUD_BASE_URL: str = "http://124.220.41.27:47070/admin-api/grain/sampler"

#: 设备 MAC（云端识别）。
DEVICE_MAC: str = "C0:F5:35:EE:1D:93"
