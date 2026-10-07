"""机构驱动模块（PCA9685 封装 + 真实/模拟控制器）。

架构（三层）：
- :class:`PCA9685`：真实硬件封装。直接通过 Linux i2c-dev（``fcntl.ioctl``
   I2C_SLAVE=0x0703）访问 ``/dev/i2c-2``, 0x40，无 adafruit 等第三方依赖。
   顶层不打开任何设备（Windows 开发机无 I2C，import 必须安全）。
- :class:`MechanismController`：真实控制器。I2C 写操作统一加线程锁、
  失败自动重试 2 次、duration 后台 daemon 线程自动回停、急停抢断
  （只停运行中通道）、品种参数来自 ``mechanism_config.get_grain_params``。
- :class:`MockMechanismController`：mock 模式。兼容 test/conftest.py 的
  ``mock_pca9685``（``set_pwm(channel, on, off)`` 签名，side_effect 把 off
  记录到 ``register_history``），全部动作同时记录到 ``action_history``。

设计约定（mechanism-driver 计划）：
- PCA9685：本机断电/上电对照确认 /dev/i2c-2，地址 0x40，50Hz，4096 计数/周期
- 执行器映射：CH0/1=螺旋输送、CH2/3/4=开仓(浅/中/深)、CH5=旧电调中位、
  CH6=拧紧、CH7=负压风机；夹爪 DRV8701E 接 CH8/9/10；伺服升降独立控制
- 脉宽标定从 sampling_params 读取；停止固定为持续1500us中位，不切断PWM。
  写入前钳制到全局合法范围 1000~2300us（sampling_params.PULSE_MIN_US/MAX_US）。
- 接口差异说明：真实 ``PCA9685.set_pwm(channel, pulse_us)`` 直接写脉宽；
  MockMechanismController 调用注入的 mock 时用 ``set_pwm(channel, 0, pulse_us)``
  以兼容 conftest 的 3 参数 mock 签名——两者并存，互不冲突。
"""

from __future__ import annotations

import logging
import math
import os
from contextlib import contextmanager
from functools import wraps
import threading
import time

try:
    import fcntl
except ImportError:  # pragma: no cover - Windows 开发机
    fcntl = None

from grain_sampling_workflow.mechanism_config import get_grain_params
from grain_sampling_devices.tp_i2c import validate_tp_bus
from utils.sampling_params import (
    BIN_CLOSE_PULSE,
    BIN_OPEN_PULSE,
    CHANNELS,
    CLAMP_PH_CHANNEL,
    CLAMP_EN_CHANNEL,
    CLAMP_NS_CHANNEL,
    CLAMP_EN_DUTY_CLOSE_PERCENT,
    CLAMP_EN_DUTY_OPEN_PERCENT,
    CLAMP_PULSE_CLOSE,
    CLAMP_PULSE_OPEN,
    ENABLE_UNWIRED_CHANNELS,
    PCA9685_FREQUENCY_HZ,
    PCA9685_I2C_ADDRESS,
    PCA9685_I2C_BUS,
    PCA9685_I2C_DEVICE,
    PCA9685_OSCILLATOR_HZ,
    PULSE_CLOSE,
    PULSE_MAX_US,
    PULSE_MIN_US,
    PULSE_OPEN,
    PULSE_STOP,
    SUPPORTED_GRAINS,
    TIGHTEN_PULSE_CLOSE,
    TIGHTEN_PULSE_OPEN,
    X2P_POSITION_TOLERANCE_MM,
    X2P_RETURN_CLEARANCE_MM,
    PRESS_DOWN_CM,
    PRESS_UP_CM,
    PRESS_PAUSE_S,
)

logger = logging.getLogger(__name__)


def _press_cycle_depths(distance_mm: float) -> list[float]:
    """Bounded insertion path; extraction traverses these targets in reverse."""
    down_mm, up_mm = PRESS_DOWN_CM * 10.0, PRESS_UP_CM * 10.0
    if (not all(math.isfinite(v) for v in (down_mm, up_mm, PRESS_PAUSE_S))
            or not down_mm > up_mm >= 0 or PRESS_PAUSE_S < 0):
        raise ValueError("往复参数要求 down > up >= 0，pause >= 0")
    depths = []
    progress = 0.0
    while progress < distance_mm - 1e-9:
        bottom = min(progress + down_mm, distance_mm)
        depths.append(bottom)
        progress = bottom
        if bottom < distance_mm - 1e-9 and up_mm > 0:
            progress = bottom - up_mm
            depths.append(progress)
    return depths

# ---------------------------------------------------------------------------
# PCA9685 寄存器 / I2C 常量（复用 dipan/pca9685/pca9685_driver.py 的取值与公式）
# ---------------------------------------------------------------------------
I2C_SLAVE = 0x0703

MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
PRESCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_EXTCLK = 0x40
MODE1_AUTO_INCREMENT = 0x20
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01
MODE2_OUTDRV = 0x04
MODE2_INVRT = 0x10
FULL_ON_OFF_BIT = 0x10

CHANNEL_COUNT = 16
COUNTS_PER_CYCLE = 4096
#: 按用户要求恢复最早版本的旧板实测校准值，统一从参数文件读取。
OSCILLATOR_HZ = PCA9685_OSCILLATOR_HZ
DEFAULT_FREQUENCY_HZ = PCA9685_FREQUENCY_HZ


def frequency_to_prescale(frequency_hz: float) -> int:
    """频率 -> PRESCALE 寄存器值（校准振荡器频率 / (4096 * hz) - 1）。"""
    if frequency_hz <= 0:
        raise ValueError("frequency must be greater than zero")
    prescale = int(round(OSCILLATOR_HZ / (COUNTS_PER_CYCLE * frequency_hz) - 1.0))
    if not 3 <= prescale <= 255:
        raise ValueError(
            f"frequency {frequency_hz:g} Hz is outside the PCA9685 prescale range"
        )
    return prescale


def prescale_to_frequency(prescale: int) -> float:
    """PRESCALE 寄存器值 -> 实际频率。"""
    return OSCILLATOR_HZ / (COUNTS_PER_CYCLE * (prescale + 1))


def pulse_us_to_counts(pulse_us: float, frequency_hz: float = DEFAULT_FREQUENCY_HZ) -> int:
    """把脉宽(us)换算成 4096 周期内的 off 计数（on=0）。

    公式（dipan set_pulse_us 同源）：``counts = pulse_us * hz * 4096 / 1e6``。
    结果钳制在 1..4095。
    """
    period_us = 1_000_000.0 / frequency_hz
    if not 0.0 < pulse_us < period_us:
        raise ValueError(
            f"pulse_us must be greater than 0 and less than the "
            f"{period_us:g} us period, got {pulse_us}"
        )
    counts = int(round(pulse_us * frequency_hz * COUNTS_PER_CYCLE / 1_000_000.0))
    return max(1, min(COUNTS_PER_CYCLE - 1, counts))


def clamp_pulse_us(pulse_us: float) -> float:
    """把脉宽钳制到全局合法范围 ``[PULSE_MIN_US, PULSE_MAX_US]``（us）。

    写硬件前的最后防线：防止标定错误/外部输入把非 1000~2300 的脉宽
    写入 PCA9685，避免电机长时间处于非预期油门导致堵转发热。
    """
    return min(max(float(pulse_us), PULSE_MIN_US), PULSE_MAX_US)


# ---------------------------------------------------------------------------
# 执行器通道映射 / 脉宽标定 / 品种 / 未接线通道 —— 统一参数源：
# utils.sampling_params（现场标定只需改该文件）
# ---------------------------------------------------------------------------



def _serialized_i2c(method):
    """Serialize complete I2C operations across threads and cooperating processes."""
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._transaction():
            return method(self, *args, **kwargs)
    return wrapped


class PCA9685:
    """PCA9685 16 通道 PWM 驱动（Linux i2c-dev 直连，无 adafruit）。

    默认硬件：本机已验证 /dev/i2c-2, 0x40, 旧板校准27.545088MHz、目标50Hz。
    注意：模块顶层不打开设备；``open()`` 是显式的（Windows 上会抛 RuntimeError）。

    高层接口 ``set_pwm(channel, pulse_us)`` 直接写脉宽(us)——
    与 MockMechanismController 注入的 conftest mock（``set_pwm(channel, on, off)``）
    签名不同，二者分属真实/模拟两条链路，互不影响。
    """

    def __init__(self, bus: int = PCA9685_I2C_BUS, address: int = PCA9685_I2C_ADDRESS,
                 device: str = PCA9685_I2C_DEVICE) -> None:
        self.bus = int(os.environ.get("PCA9685_I2C_BUS", str(bus)))
        self.address = int(os.environ.get("PCA9685_I2C_ADDRESS", str(address)), 0)
        if not 0x03 <= self.address <= 0x77:
            raise ValueError(
                f"I2C address must be 0x03..0x77, got 0x{self.address:02x}"
            )
        self.device = (device or os.environ.get("PCA9685_I2C_DEVICE", "").strip()
                       or f"/dev/i2c-{self.bus}")
        self.fd = None  # type: int | None
        self.frequency_hz = DEFAULT_FREQUENCY_HZ
        self._io_lock = threading.RLock()
        self._io_depth = 0

    # -- 打开 / 关闭 ------------------------------------------------------
    def open(self) -> "PCA9685":
        """打开 ``/dev/i2c-N`` 并通过 ioctl 设置从机地址。返回 self。"""
        if self.fd is not None:
            return self
        if fcntl is None:
            raise RuntimeError(
                f"I2C is not supported on this platform (no fcntl module); "
                f"cannot open {self.device}"
            )
        validate_tp_bus(self.device)
        try:
            self.fd = os.open(self.device, os.O_RDWR)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"{self.device} does not exist. Check `i2cdetect -l` and set "
                f"PCA9685_I2C_DEVICE=/dev/i2c-X (address 0x{self.address:02x})."
            ) from exc
        except PermissionError as exc:
            raise RuntimeError(
                f"permission denied for {self.device}; run with sudo or add "
                f"your user to the i2c group"
            ) from exc
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
            # PCA9685 may retain its last output when only the host restarts.
            self.set_level(CLAMP_NS_CHANNEL, False)
            # Restore first-revision oscillator initialization on every open.
            self.set_frequency(DEFAULT_FREQUENCY_HZ)
            self.all_stop()  # 保留用户要求：初始化为持续中位，不恢复FULL_OFF。
            self.set_level(CLAMP_PH_CHANNEL, False)
            # EN is a continuous speed-control PWM; nSLEEP controls motor stop.
            self.set_duty_cycle(CLAMP_EN_CHANNEL, CLAMP_EN_DUTY_CLOSE_PERCENT)
        except Exception:
            self.close()
            raise
        return self

    def close(self) -> None:
        """关闭设备文件描述符（幂等）。"""
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self) -> "PCA9685":
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    # -- 寄存器读写 -------------------------------------------------------
    def _require_open(self) -> int:
        if self.fd is None:
            raise RuntimeError("I2C device is not open")
        return self.fd

    @contextmanager
    def _transaction(self):
        # Device symlinks resolve to the same inode, hence share this flock.
        # The legacy standalone diagnostic CLI must not run alongside nodes.
        with self._io_lock:
            fd = self._require_open()
            outer = self._io_depth == 0
            if outer:
                fcntl.flock(fd, fcntl.LOCK_EX)
            self._io_depth += 1
            try:
                yield
            finally:
                self._io_depth -= 1
                if outer:
                    fcntl.flock(fd, fcntl.LOCK_UN)

    @_serialized_i2c
    def write_register(self, register: int, value: int) -> None:
        """写单个寄存器（2 字节：寄存器地址 + 值）。"""
        if not 0x00 <= register <= 0xFF:
            raise ValueError(f"register must be 0x00..0xFF, got 0x{register:02x}")
        if not 0x00 <= value <= 0xFF:
            raise ValueError(f"register value must be 0x00..0xFF, got 0x{value:02x}")
        written = os.write(self._require_open(), bytes((register, value)))
        if written != 2:
            raise RuntimeError(f"short I2C write: expected 2 bytes, wrote {written}")

    @_serialized_i2c
    def read_register(self, register: int) -> int:
        """读单个寄存器。"""
        if not 0x00 <= register <= 0xFF:
            raise ValueError(f"register must be 0x00..0xFF, got 0x{register:02x}")
        fd = self._require_open()
        if os.write(fd, bytes((register,))) != 1:
            raise RuntimeError("failed to select PCA9685 register")
        data = os.read(fd, 1)
        if len(data) != 1:
            raise RuntimeError("short I2C read from PCA9685")
        return data[0]

    # -- 频率 -------------------------------------------------------------
    @_serialized_i2c
    def set_frequency(self, frequency_hz: float) -> float:
        """写入 PRESCALE 设置 PWM 频率（默认 50Hz），返回实际频率。"""
        prescale = frequency_to_prescale(frequency_hz)
        old_mode = self.read_register(MODE1)
        sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP
        awake_mode = (
            (old_mode & ~MODE1_SLEEP) | MODE1_AUTO_INCREMENT | MODE1_ALLCALL
        )

        self.write_register(MODE1, sleep_mode)
        self.write_register(PRESCALE, prescale)
        self.write_register(MODE1, awake_mode)
        time.sleep(0.005)
        self.write_register(MODE1, awake_mode | MODE1_RESTART)
        self.write_register(MODE2, (self.read_register(MODE2) | MODE2_OUTDRV) & ~MODE2_INVRT)
        self.frequency_hz = frequency_hz
        return prescale_to_frequency(prescale)

    # -- 通道 PWM ---------------------------------------------------------
    @staticmethod
    def _channel_base(channel: int) -> int:
        if not 0 <= channel < CHANNEL_COUNT:
            raise ValueError(
                f"channel must be 0..{CHANNEL_COUNT - 1}, got {channel}"
            )
        return LED0_ON_L + 4 * channel

    @_serialized_i2c
    def set_pwm(self, channel: int, pulse_us: float) -> int:
        """向指定通道写脉宽(us)：on=0、off=counts（50Hz 周期 4096 计数）。

        写入前把脉宽钳制到全局合法范围 ``[PULSE_MIN_US, PULSE_MAX_US]``
        （见 :func:`clamp_pulse_us`），防止异常脉宽导致电机堵转发热。
        返回换算后的 counts。
        """
        counts = pulse_us_to_counts(clamp_pulse_us(pulse_us), self.frequency_hz)
        base = self._channel_base(channel)
        self.write_register(base, 0)
        self.write_register(base + 1, 0)
        self.write_register(base + 2, counts & 0xFF)
        self.write_register(base + 3, (counts >> 8) & 0x0F)
        return counts

    @_serialized_i2c
    def set_level(self, channel: int, high: bool) -> None:
        """Use the PCA9685 full-on/full-off bits for a stable logic level."""
        base = self._channel_base(channel)
        if high:
            # FULL_OFF dominates FULL_ON; release it last.
            self._write_channel(base, 0, FULL_ON_OFF_BIT, 0, 0)
        else:
            # Assert FULL_OFF first, before changing the other registers.
            self.write_register(base + 3, FULL_ON_OFF_BIT)
            self.write_register(base + 1, 0)
            self.write_register(base, 0)
            self.write_register(base + 2, 0)

    @_serialized_i2c
    def set_duty_cycle(self, channel: int, percent: float) -> None:
        """Set ordinary PWM duty, with true constant output at 0% and 100%."""
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            raise ValueError("percent must be between 0 and 100")
        if percent == 0:
            self.set_level(channel, False)
            return
        if percent == 100:
            self.set_level(channel, True)
            return
        counts = max(1, min(COUNTS_PER_CYCLE - 1, round(percent * COUNTS_PER_CYCLE / 100)))
        base = self._channel_base(channel)
        # When changing a running PWM, retain PWM output; nSLEEP stays low
        # during clamp direction/duty changes.
        self.write_register(base, 0)
        self.write_register(base + 1, 0)
        self.write_register(base + 2, counts & 0xFF)
        self.write_register(base + 3, (counts >> 8) & 0x0F)

    @_serialized_i2c
    def channel_stop(self, channel: int) -> None:
        """停止电机而不停止信号：持续输出固定 1500us 中位。"""
        base = self._channel_base(channel)
        counts = pulse_us_to_counts(PULSE_STOP, self.frequency_hz)
        # 与原始逐寄存器写入逻辑一致；停止时不写FULL_OFF。
        self._write_channel(base, 0, 0, counts & 255, (counts >> 8) & 15)

    def channel_off(self, channel: int) -> None:
        """旧接口兼容：语义已改为1500us中位，不再设置 FULL_OFF。"""
        self.channel_stop(channel)

    def _write_channel(self, base: int, *values: int) -> None:
        # Restore the first revision's separate register transactions.
        for offset, value in enumerate(values):
            self.write_register(base + offset, value)

    @_serialized_i2c
    def all_stop(self) -> None:
        """旧电调 CH0–7 回到持续中位；夹爪由 NS 另行控制。"""
        for channel in sorted(set(CHANNELS.values())):
            self.channel_stop(channel)

    def all_off(self) -> None:
        """旧接口兼容：回中位，保持PWM。"""
        self.all_stop()


class _RecordingPCA9685:
    """mock 模式内部替换件：记录 ``set_pwm(channel, pulse_us)`` 写入。

    仅用于 ``MechanismController(pca9685=None, mock_mode=True)``，
    让真实控制器在 Windows / 无 I2C 环境下也能安全运行与测试。
    签名与真实 ``PCA9685.set_pwm`` 一致（2 参数）。
    """

    def __init__(self) -> None:
        self.register_history: dict[int, list] = {}
        self.level_history: dict[int, list[bool]] = {}
        self.duty_history: dict[int, list[float]] = {}

    def set_pwm(self, channel: int, pulse_us: float, off: float | None = None) -> None:
        if off is not None:  # MockMechanismController's legacy three-argument call
            pulse_us = off
        self.register_history.setdefault(channel, []).append(pulse_us)

    def set_frequency(self, frequency_hz: float) -> None:
        pass

    def set_level(self, channel: int, high: bool) -> None:
        self.level_history.setdefault(channel, []).append(bool(high))

    def set_duty_cycle(self, channel: int, percent: float) -> None:
        self.duty_history.setdefault(channel, []).append(float(percent))

    def channel_off(self, channel: int) -> None:
        self.set_pwm(channel, PULSE_STOP)

    channel_stop = channel_off

    def all_off(self) -> None:
        for channel in sorted(set(CHANNELS.values())):
            self.channel_stop(channel)


class _BaseMechanismController:
    """共享控制逻辑：I2C 线程锁、失败重试、运行通道跟踪、急停、品种参数。

    子类只需实现 :meth:`_write_hw`（低层脉宽写入方式）：
    - :class:`MechanismController`：``pca9685.set_pwm(channel, pulse_us)``
    - :class:`MockMechanismController`：``pca9685.set_pwm(channel, 0, pulse_us)``
    """

    #: action 名 -> 脉宽属性名（品种参数可覆盖的字段）
    ACTION_TO_ATTR = {
        "open": "throttle_open",
        "close": "throttle_close",
        "stop": "stop_value",
    }
    #: 总写入尝试次数 = 1 次正常 + 2 次重试
    WRITE_ATTEMPTS = 3
    #: 重试间隔（秒）
    RETRY_INTERVAL = 0.01

    def __init__(self, pca9685=None, mock_mode: bool = True) -> None:
        self.pca9685 = pca9685
        self.mock_mode = bool(mock_mode)
        self._lock = threading.RLock()       # 保护 I2C 写 + _running + 仓门定时器
        self._bin_timers: dict[int, threading.Timer] = {}
        self._running: set = set()           # 运行中通道集合
        self._stop_flag = threading.Event()  # 急停/关闭标志
        self._shutdown = False
        # [(action_name, kwargs), ...] 按调用顺序记录
        self.action_history: list = []
        self.current_grain = None
        # 油门/停止脉宽（默认标定值，set_grain 按品种覆盖）
        self.throttle_open = PULSE_OPEN
        self.throttle_close = PULSE_CLOSE
        self.stop_value = PULSE_STOP
        #: 可选 X2P 伺服升降驱动器（USB-RS485）。提供时 press/lift 走真实
        #: 伺服（需暴露位置控制接口；兼容入口为
        #: ``run_speed(direction, rpm, duration_s)`` 与 ``stop()``，由
        #: dais516 x2p.MotionController 转换为内部位置段）；缺省为占位 no-op。
        self.lift_drive = None
        #: 升降默认转速（r/min）与时长（s），可现场标定覆盖。
        self.lift_rpm = 30
        self.lift_duration = 2.0
        # A paired automatic press cycle remembers its absolute encoder origin.
        # Keeping these fields in the shared base also makes mock controllers
        # exercise exactly the same safety state machine as real hardware.
        self._lift_motion_lock = threading.Lock()
        self._lift_cycle_origin: int | None = None
        self._lift_extraction_targets: tuple[int, ...] | None = None
        self._lift_extraction_ready = False

    # -- 低层：脉宽写入（子类实现） ---------------------------------------
    def _write_hw(self, channel: int, pulse_us: float) -> None:
        raise NotImplementedError

    def _write_hw_off(self, channel: int) -> None:
        """低层持续1500us中位写入（历史方法名保留）。"""
        raise NotImplementedError

    def _write_pulse(self, channel: int, pulse_us: float) -> None:
        """加锁写入脉宽，失败重试 ``WRITE_ATTEMPTS - 1`` 次后抛 RuntimeError。"""
        last_error = None
        for attempt in range(self.WRITE_ATTEMPTS):
            try:
                with self._lock:
                    if self._stop_flag.is_set():
                        raise RuntimeError("controller is in emergency-stop state")
                    self._write_hw(channel, pulse_us)
                    self._running.add(channel)
                return
            except Exception as exc:  # noqa: BLE001 - 重试语义捕获一切 I2C 错误
                last_error = exc
                if attempt < self.WRITE_ATTEMPTS - 1:
                    time.sleep(self.RETRY_INTERVAL)
        raise RuntimeError(
            f"failed to write pulse {pulse_us}us to channel {channel} after "
            f"{self.WRITE_ATTEMPTS} attempts: {last_error}"
        ) from last_error

    def _write_off(self, channel: int) -> None:
        """加锁回中位并保持PWM，失败重试后抛 RuntimeError。"""
        last_error = None
        for attempt in range(self.WRITE_ATTEMPTS):
            try:
                with self._lock:
                    self._write_hw_off(channel)
                    self._running.discard(channel)
                return
            except Exception as exc:  # noqa: BLE001 - 重试语义捕获一切 I2C 错误
                last_error = exc
                if attempt < self.WRITE_ATTEMPTS - 1:
                    time.sleep(self.RETRY_INTERVAL)
        raise RuntimeError(
            f"failed to write neutral to channel {channel} after "
            f"{self.WRITE_ATTEMPTS} attempts: {last_error}"
        ) from last_error

    def set_pulse(self, channel: int, pulse_us: float) -> None:
        """向指定通道写入脉宽（含重试），并记录历史。"""
        self._write_pulse(channel, pulse_us)
        self.action_history.append(
            ("set_pulse", {"channel": channel, "pulse_us": pulse_us})
        )

    def set_stop(self, channel: int) -> None:
        """停止电机，通道保持1500us中位PWM。"""
        self._write_off(channel)
        self.action_history.append(("set_stop", {"channel": channel}))

    # -- 三态控制 ---------------------------------------------------------
    def actuate(self, channel: int, action: str, duration: float | None = None) -> None:
        """对通道做 open/close/stop 三态控制。

        - ``open``/``close`` 按品种油门脉宽写入；``duration>0`` 时
          duration 秒后自动回停（daemon 线程，期间通道记入 ``_running``）。
        - ``stop`` 立即回到1500us并持续输出PWM，从 ``_running`` 移除。
        - 急停（``emergency_stop``）后拒绝新动作，抛 RuntimeError。
        """
        if action not in self.ACTION_TO_ATTR:
            raise ValueError(
                f"unknown action {action!r}; expected open/close/stop"
            )
        if self._stop_flag.is_set():
            raise RuntimeError(
                "controller is in emergency-stop state; call reset() to resume"
            )
        if action == "stop":
            self._write_off(channel)
            self.action_history.append(
                ("actuate", {"channel": channel, "action": action,
                             "pulse_us": PULSE_STOP, "duration": duration})
            )
            with self._lock:
                self._running.discard(channel)
            return
        pulse = int(getattr(self, self.ACTION_TO_ATTR[action]))
        self.set_pulse(channel, pulse)
        self.action_history.append(
            ("actuate", {"channel": channel, "action": action,
                         "pulse_us": pulse, "duration": duration})
        )
        if duration is not None and duration > 0:
            with self._lock:
                self._running.add(channel)
            threading.Thread(
                target=self._auto_stop, args=(channel, duration), daemon=True
            ).start()

    def _auto_stop(self, channel: int, duration: float) -> None:
        """后台 daemon：duration 秒后回到1500us并移出运行集合。

        急停/关闭会 set ``_stop_flag``，本线程立即被唤醒并放弃
        （停止写入由 emergency_stop/shutdown 完成，避免重复写）。
        """
        if self._stop_flag.wait(duration):
            return
        try:
            self._write_off(channel)
        except Exception:  # noqa: BLE001 - 回停尽力而为
            pass
        with self._lock:
            self._running.discard(channel)

    # -- 急停 / 恢复 ------------------------------------------------------
    def emergency_stop(self) -> None:
        """急停：锁定新动作，所有机构回1500us；保留伺服停止命令。"""
        self._stop_flag.set()
        with self._lock:
            for timer in self._bin_timers.values():
                timer.cancel()
            self._bin_timers.clear()
            running = sorted(set(self._running) | set(CHANNELS.values()))
            self._running.clear()
        self.action_history.append(("emergency_stop", {"channels": running}))
        for ch in running:
            try:
                self._write_off(ch)
            except Exception:  # noqa: BLE001 - 急停写入尽力而为
                pass

        drive = getattr(self, "lift_drive", None)
        if drive is not None:
            drive.stop()

    def reset(self, *, mechanical_reset_confirmed: bool = False) -> None:
        """解锁前处理未完成的升降原点；机械复位必须由操作员显式确认。"""
        with self._lift_motion_lock:
            if self._lift_cycle_origin is not None:
                if not mechanical_reset_confirmed:
                    raise RuntimeError(
                        "上次自动回程未完成；须现场机械复位后显式确认，"
                        "不能仅清急停或自动覆盖原点"
                    )
                logger.warning(
                    "LIFT_CYCLE_ORIGIN_DISCARDED_AFTER_MANUAL_RESET origin=%d",
                    self._lift_cycle_origin,
                )
                self._lift_cycle_origin = None
                self._lift_extraction_targets = None
                self._lift_extraction_ready = False
        self._stop_flag.clear()

    # -- 品种参数 ---------------------------------------------------------
    def set_grain(self, grain: str) -> bool:
        """设置当前品种，从 mechanism_config 覆盖油门/停止脉宽。

        未知品种回退默认参数，返回是否受支持。
        """
        params = get_grain_params(grain)
        self.current_grain = grain
        self.throttle_open = params["throttle_open"]
        self.throttle_close = params["throttle_close"]
        self.stop_value = params["stop_value"]
        self.action_history.append(("set_grain", {"grain": grain}))
        return grain in SUPPORTED_GRAINS

    # -- 语义动作 ---------------------------------------------------------
    def _act(self, name: str, channel: int, action: str, duration=None) -> None:
        """执行语义动作：经 actuate（含重试/运行跟踪）并追加语义历史。"""
        self.actuate(channel, action, duration=duration)
        self.action_history.append(
            (name, {"channel": channel, "action": action, "duration": duration})
        )

    def _act_pulse(self, name: str, channel: int, pulse_us: float,
                   duration=None) -> None:
        """用独立标定脉宽执行动作（如 CH5 夹紧/CH6 拧紧），含自动回停。

        不走品种参数 ``throttle_open/close`` —— 这些通道的标定值与
        三仓/输送不同（见 sampling_params 的 ``*_PULSE_*`` 常量）。
        """
        self.set_pulse(channel, pulse_us)
        self.action_history.append(
            (name, {"channel": channel, "pulse_us": pulse_us, "duration": duration})
        )
        if duration is not None and duration > 0:
            with self._lock:
                self._running.add(channel)
            threading.Thread(
                target=self._auto_stop, args=(channel, duration), daemon=True
            ).start()

    def convey(self, duration=None, direction: int = 1) -> None:
        """螺旋输送：direction>0 送料(开=1200us)，direction<=0 回1500us停料。

        同时驱动 CH0（convey_1）与 CH1（convey_2）两个输送通道——
        实机确认两个螺旋输送需同时开启才正常出粮。
        """
        action = "open" if direction > 0 else "stop"
        for key in ("convey_1", "convey_2"):
            self._act("convey", CHANNELS[key], action, duration=duration)

    def open_bin(self, depth: str = "mid", duration=None) -> None:
        """开仓，depth 取值 shallow/mid/deep。独立标定 BIN_OPEN_PULSE=1200us。

        ``duration>0`` 时，duration 秒后自动关**同一个仓**（写
        BIN_CLOSE_PULSE=1800us）——开仓保持时长由品种参数 open_duration 提供。
        """
        key = f"bin_{depth}"
        if key not in CHANNELS:
            raise ValueError(
                f"unknown bin depth {depth!r}; expected shallow/mid/deep"
            )
        channel = CHANNELS[key]
        with self._lock:
            self._cancel_bin_timer(channel)
            self._act_pulse("open_bin", channel, BIN_OPEN_PULSE, None)
            if duration is not None and duration > 0:
                timer = threading.Timer(
                    duration, lambda: self._auto_close_bin(channel, timer)
                )
                timer.daemon = True
                self._bin_timers[channel] = timer
                timer.start()

    def _cancel_bin_timer(self, channel: int) -> None:
        timer = self._bin_timers.pop(channel, None)
        if timer is not None:
            timer.cancel()

    def hold_bin_open(self, depth: str) -> None:
        """正式流程：目标仓开门、其他两仓同时关门，到时各自回1500us。"""
        if depth not in ("shallow", "mid", "deep"):
            raise ValueError(f"unknown bin depth {depth!r}; expected shallow/mid/deep")
        params = get_grain_params(self.current_grain or "")
        with self._lock:
            try:
                for name in ("shallow", "mid", "deep"):
                    selected = name == depth
                    self._move_bin(
                        "hold_bin_open" if selected else "close_bin",
                        CHANNELS[f"bin_{name}"],
                        BIN_OPEN_PULSE if selected else BIN_CLOSE_PULSE,
                        float(params["open_duration" if selected else "close_duration"]),
                    )
            except Exception:
                self.emergency_stop()
                raise

    def _move_bin(self, action: str, channel: int, pulse: float, duration) -> None:
        """仓门定时回中位；新动作取消旧回调，避免重试/换仓被旧定时器截断。"""
        with self._lock:
            self._cancel_bin_timer(channel)
            self.set_pulse(channel, pulse)
            self.action_history.append(
                (action, {"channel": channel, "pulse_us": pulse, "duration": duration})
            )
            if duration is not None and duration > 0:
                timer = threading.Timer(
                    duration, lambda: self._finish_bin_move(channel, timer)
                )
                timer.daemon = True
                self._bin_timers[channel] = timer
                timer.start()

    def _finish_bin_move(self, channel: int, timer) -> None:
        with self._lock:
            if self._stop_flag.is_set() or self._bin_timers.get(channel) is not timer:
                return
            self._bin_timers.pop(channel, None)
            try:
                self.set_stop(channel)
            except Exception:
                logger.exception("Failed to return bin channel %s to neutral", channel)
                self.emergency_stop()

    def close_all_bins(self, duration=None) -> None:
        """三仓同时按关门方向运行，动作到时保持1500us中位。"""
        if duration is None:
            duration = float(get_grain_params(self.current_grain or "")["close_duration"])
        with self._lock:
            try:
                for depth in ("shallow", "mid", "deep"):
                    self.close_bin(depth, duration=duration)
            except Exception:
                self.emergency_stop()
                raise

    def _auto_close_bin(self, channel: int, timer=None) -> None:
        """旧服务自动关仓；过期或急停后的回调不再输出。"""
        with self._lock:
            if self._stop_flag.is_set() or self._bin_timers.get(channel) is not timer:
                return
            self._bin_timers.pop(channel, None)
            duration = float(get_grain_params(self.current_grain or "")["close_duration"])
            self._move_bin("close_bin_auto", channel, BIN_CLOSE_PULSE, duration)

    def close_bin(self, depth: str = "mid", duration=None) -> None:
        """关仓，depth 取值 shallow/mid/deep。独立标定 BIN_CLOSE_PULSE=1800us。"""
        key = f"bin_{depth}"
        if key not in CHANNELS:
            raise ValueError(
                f"unknown bin depth {depth!r}; expected shallow/mid/deep"
            )
        with self._lock:
            self._cancel_bin_timer(CHANNELS[key])
            if duration is None:
                duration = float(get_grain_params(self.current_grain or "")["close_duration"])
            self._move_bin("close_bin", CHANNELS[key], BIN_CLOSE_PULSE, duration)

    def clamp(self, duration=None) -> None:
        """夹紧（CH5 独立标定：1900us）。"""
        self._act_pulse("clamp", CHANNELS["clamp"], CLAMP_PULSE_CLOSE, duration)

    def unclamp(self, duration=None) -> None:
        """松开（CH5 独立标定：1000us）。"""
        self._act_pulse("unclamp", CHANNELS["clamp"], CLAMP_PULSE_OPEN, duration)

    def tighten(self, duration=None) -> None:
        """拧紧（CH6 独立标定：1300us）。"""
        self._act_pulse("tighten", CHANNELS["tighten"], TIGHTEN_PULSE_CLOSE, duration)

    def untighten(self, duration=None) -> None:
        """拧松（CH6 独立标定：1900us）。"""
        self._act_pulse("untighten", CHANNELS["tighten"], TIGHTEN_PULSE_OPEN, duration)

    def press(self, duration=None) -> None:
        """伺服升降-下压。

        注入 ``lift_drive``（X2P 伺服）时走真实伺服（down=下压）；
        否则保持占位/既有通道行为。
        """
        if self.lift_drive is not None:
            self._x2p_lift("press", "down", duration)
            return
        self._act("press", CHANNELS["bin_shallow"], "open", duration=duration)

    def lift(self, duration=None) -> None:
        """伺服升降-提升。

        注入 ``lift_drive``（X2P 伺服）时走真实伺服（up=提升）；
        否则保持占位/既有通道行为。
        """
        if self.lift_drive is not None:
            self._x2p_lift("lift", "up", duration)
            return
        self._act("lift", CHANNELS["bin_shallow"], "close", duration=duration)

    def _x2p_lift(self, name: str, direction: str, duration) -> None:
        """通过 X2P 伺服执行升降动作（含安全停止）。

        默认 ``duration`` 为 ``self.lift_duration``（秒），转速为
        ``self.lift_rpm``（r/min）；均可在品种参数/现场标定中调整。
        实机方向（forward=下压 or 提升）可能相反，用驱动器的
        ``forward_sign`` 配置翻转，不在此硬编码。
        """
        drive = self.lift_drive
        rpm = int(self.lift_rpm)
        secs = float(duration if duration else self.lift_duration)
        self.action_history.append(
            (name, {"direction": direction, "rpm": rpm,
                    "duration_s": secs, "x2p": True})
        )
        try:
            drive.run_speed(direction, rpm, duration_s=secs)
        finally:
            try:
                drive.stop()
            except Exception:  # noqa: BLE001 - 停机尽力而为
                pass

    def move_lift(self, direction: str, distance_cm: float,
                  duration_s: float | None = None,
                  tolerance_mm: float = X2P_POSITION_TOLERANCE_MM) -> object:
        """伺服升降按距离移动（编码器闭环，精确停在目标距离）。

        Parameters
        ----------
        direction : str
            ``up``/``down`` 为单程；``down_cycle`` 保存原点并往复下压，
            ``return`` 回到本轮保存的编码器原点；``extract_prepare`` 松夹时
            保存上方原点并下降，``extract`` 夹紧后沿往复下压的反向轨迹上提。
        distance_cm : float
            移动距离（厘米），必须 > 0。
        duration_s : float | None
            移动时长（秒）。None 时按 ``lift_rpm`` 与 5mm 导程自动计算。
            实际转速 = 距离/时长，超 ``max_rpm`` 会由 x2p 内部校验报错。
        tolerance_mm : float
            位置容差（毫米），默认 8.0（实机负载下误差实测 ~5.4mm，需放宽）。
        """
        if self.lift_drive is None:
            raise RuntimeError("X2P 伺服未注入，无法按距离移动")
        if not math.isfinite(distance_cm) or not distance_cm > 0:
            raise ValueError("distance_cm 必须大于 0")
        normalized_direction = str(direction).strip().lower()
        if normalized_direction not in {"up", "down", "down_cycle", "return",
                                        "extract_prepare", "extract"}:
            raise ValueError("direction 必须是 up/down/down_cycle/return/extract_prepare/extract")
        distance_mm = float(distance_cm) * 10.0
        if duration_s is None:
            rpm = max(1, int(self.lift_rpm))
            duration_s = distance_mm / (5.0 * rpm / 60.0)
        secs = float(duration_s)
        if not math.isfinite(secs) or secs <= 0:
            raise ValueError("duration_s 必须是有限正数")
        with self._lift_motion_lock:
            if self._stop_flag.is_set():
                raise RuntimeError("急停已锁定，拒绝升降")
            if normalized_direction == "down_cycle":
                if self._lift_cycle_origin is not None:
                    raise RuntimeError(
                        "上一次自动下压尚未完成绝对回程，拒绝覆盖原点"
                    )
                depths = _press_cycle_depths(distance_mm)
                limits = getattr(self.lift_drive.config, "limits", None)
                max_distance = float(getattr(limits, "max_distance_mm", 300.0))
                if distance_mm > max_distance:
                    raise ValueError("往复下压总行程超过伺服单段行程限制")
                counts = float(self.lift_drive.counts_per_mm)
                sign = int(getattr(self.lift_drive.config, "encoder_forward_sign", 1))
                if not math.isfinite(counts) or counts <= 0 or sign not in (-1, 1):
                    raise ValueError("编码器比例或方向无效")
                origin = int(self.lift_drive.read_position())
                self._lift_cycle_origin = origin
                logger.info(
                    "LIFT_CYCLE_ORIGIN_SAVED position=%d distance_mm=%.3f",
                    origin, distance_mm,
                )
                progress = 0.0
                leg = 0
                try:
                    for depth in depths:
                        if self._stop_flag.is_set():
                            raise RuntimeError("往复下压被急停中断")
                        target = origin - sign * round(depth * counts)
                        actual_mm = abs(target - self.lift_drive.read_position()) / counts
                        leg += 1
                        logger.info("LIFT_CYCLE_LEG leg=%d depth_mm=%.3f target=%d", leg, depth, target)
                        result = self.lift_drive.move_to_position(
                            target, max(actual_mm, 0.001) * secs / distance_mm,
                            tolerance_mm=tolerance_mm,
                        )
                        logger.info("LIFT_CYCLE_LEG_DONE leg=%d result=%s", leg, result)
                        if self._stop_flag.is_set():
                            raise RuntimeError("往复下压被急停中断")
                        progress = depth
                        if progress < distance_mm - 1e-9 and self._stop_flag.wait(PRESS_PAUSE_S):
                            raise RuntimeError("往复下压被急停中断")
                    logger.info("LIFT_CYCLE_COMPLETE origin=%d depth_mm=%.3f legs=%d", origin, progress, leg)
                    return result
                except Exception:
                    logger.exception("LIFT_CYCLE_FAILED origin=%d leg=%d; 原点保留，禁止自动重放", origin, leg)
                    try:
                        self.lift_drive.stop()
                    except Exception:
                        logger.exception("往复下压失败后的停机失败")
                    raise
            elif normalized_direction == "return":
                if self._lift_extraction_targets is not None:
                    raise RuntimeError("取管上提未完成，禁止使用下压回程")
                if self._lift_cycle_origin is None:
                    raise RuntimeError("没有已保存的下压起点，拒绝自动回程")
                counts_per_mm = float(self.lift_drive.counts_per_mm)
                encoder_up_sign = int(
                    getattr(self.lift_drive.config, "encoder_forward_sign", 1)
                )
                clearance_pulses = round(
                    float(X2P_RETURN_CLEARANCE_MM) * counts_per_mm
                )
                target = (
                    int(self._lift_cycle_origin)
                    - encoder_up_sign * clearance_pulses
                )
                logger.info(
                    "LIFT_CYCLE_RETURN origin=%d target=%d clearance_mm=%.3f "
                    "tolerance_mm=%.3f",
                    self._lift_cycle_origin, target, X2P_RETURN_CLEARANCE_MM,
                    tolerance_mm,
                )
                result = self.lift_drive.move_to_position(
                    target, secs, tolerance_mm=tolerance_mm
                )
                self._lift_cycle_origin = None
                self.action_history.append(
                    ("move_lift", {"direction": "return", "target_position": target,
                                   "clearance_mm": X2P_RETURN_CLEARANCE_MM,
                                   "duration_s": secs, "x2p": True})
                )
                return result
            elif normalized_direction in {"extract_prepare", "extract"}:
                counts = float(self.lift_drive.counts_per_mm)
                sign = int(getattr(self.lift_drive.config, "encoder_forward_sign", 1))
                if not math.isfinite(counts) or counts <= 0 or sign not in (-1, 1):
                    raise ValueError("编码器比例或方向无效")
                if normalized_direction == "extract_prepare":
                    if self._lift_cycle_origin is not None:
                        raise RuntimeError("上次升降尚未完成，拒绝覆盖原点")
                    limits = getattr(self.lift_drive.config, "limits", None)
                    if distance_mm > float(getattr(limits, "max_distance_mm", 300.0)):
                        raise ValueError("取管总行程超过伺服单段行程限制")
                    depths = _press_cycle_depths(distance_mm)
                    origin = int(self.lift_drive.read_position())
                    targets = (origin,) + tuple(
                        origin - sign * round(depth * counts) for depth in depths
                    )
                    self._lift_cycle_origin = origin
                    self._lift_extraction_targets = targets
                    self._lift_extraction_ready = False
                    path = (targets[-1],)
                else:
                    targets = self._lift_extraction_targets
                    if targets is None or not self._lift_extraction_ready:
                        raise RuntimeError("没有已完成的取管下降准备，拒绝自动上提或重放")
                    if abs(targets[-1] - targets[0]) != round(distance_mm * counts):
                        raise ValueError("取管上提距离必须与下降准备一致")
                    # A failed/uncertain ascent cannot be issued a second time.
                    self._lift_extraction_ready = False
                    path = tuple(reversed(targets[:-1]))
                try:
                    for leg, target in enumerate(path):
                        if self._stop_flag.is_set():
                            raise RuntimeError("取管升降被急停中断")
                        actual_mm = abs(target - self.lift_drive.read_position()) / counts
                        logger.info("LIFT_EXTRACTION_LEG phase=%s leg=%d target=%d",
                                    normalized_direction, leg + 1, target)
                        result = self.lift_drive.move_to_position(
                            target, max(actual_mm, 0.001) * secs / distance_mm,
                            tolerance_mm=tolerance_mm,
                        )
                        if self._stop_flag.is_set():
                            raise RuntimeError("取管升降被急停中断")
                        if leg + 1 < len(path) and self._stop_flag.wait(PRESS_PAUSE_S):
                            raise RuntimeError("取管升降被急停中断")
                    if normalized_direction == "extract_prepare":
                        self._lift_extraction_ready = True
                    else:
                        self._lift_cycle_origin = None
                        self._lift_extraction_targets = None
                    return result
                except Exception:
                    self._lift_extraction_ready = False
                    logger.exception("LIFT_EXTRACTION_FAILED; 原点保留，禁止自动重放")
                    try:
                        self.lift_drive.stop()
                    except Exception:
                        logger.exception("取管失败后的停机失败")
                    raise
            else:
                command_direction = normalized_direction

            self.action_history.append(
                ("move_lift", {"direction": normalized_direction,
                               "distance_mm": distance_mm,
                               "duration_s": secs, "x2p": True})
            )
            return self.lift_drive.move_distance(
                command_direction, distance_mm, secs, tolerance_mm=tolerance_mm
            )

    def fan(self, duration=None) -> None:
        """负压风机 CH7。"""
        self._act("fan", CHANNELS["fan"], "open", duration=duration)

    # -- 关闭 -------------------------------------------------------------
    def shutdown(self) -> None:
        """取消后台任务，所有机构保持1500us。幂等。"""
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._stop_flag.set()
            for timer in self._bin_timers.values():
                timer.cancel()
            self._bin_timers.clear()
            running = sorted(set(self._running) | set(CHANNELS.values()))
            self._running.clear()
        for ch in running:
            try:
                self._write_off(ch)
            except Exception:  # noqa: BLE001 - 通信故障不能保证物理停机
                logger.exception('Failed to hold CH%d neutral during shutdown', ch)
        self._stop_flag.set()


class MechanismController(_BaseMechanismController):
    """真实机构控制器：PCA9685 + I2C 锁 + 失败重试 + 急停 + 品种参数。

    ``pca9685`` 缺省时：mock_mode=True 用内部记录件，mock_mode=False 用真实
    :class:`PCA9685`（此时需先调用 ``open()`` 打开 /dev/i2c-2）。
    """

    def __init__(self, pca9685=None, mock_mode: bool = False, lift_drive=None) -> None:
        if pca9685 is None:
            pca9685 = _RecordingPCA9685() if mock_mode else PCA9685()
        super().__init__(pca9685=pca9685, mock_mode=mock_mode)
        #: 可选 X2P 伺服升降驱动器（USB-RS485）。提供时 press/lift 走
        #: 真实伺服；缺省为占位 no-op。调用方负责构造并注入：
        #: ``lift_drive`` 需暴露位置控制接口；兼容入口
        #: ``run_speed(direction, rpm, duration_s)`` 与 ``stop()`` 由
        #: dais516 x2p.MotionController 提供。
        self.lift_drive = lift_drive
        #: 升降默认转速（r/min）与时长（s），可现场标定覆盖。
        self.lift_rpm = 30
        self.lift_duration = 2.0
        self._clamp_timer: threading.Timer | None = None

    def _clamp_write(self, operation) -> None:
        """Retry one DRV8701E input write using the mechanism I2C policy."""
        for attempt in range(self.WRITE_ATTEMPTS):
            try:
                operation()
                return
            except Exception:
                if attempt == self.WRITE_ATTEMPTS - 1:
                    raise
                time.sleep(self.RETRY_INTERVAL)

    def _cancel_clamp_timer(self) -> None:
        timer = self._clamp_timer
        self._clamp_timer = None
        if timer is not None:
            timer.cancel()

    def _stop_clamp_locked(self) -> None:
        """Sleep the bridge while leaving EN's speed PWM running."""
        error = None
        try:
            self._clamp_write(lambda: self.pca9685.set_level(CLAMP_NS_CHANNEL, False))
        except Exception as exc:
            error = exc
        self._running.discard(CLAMP_EN_CHANNEL)
        if error is not None:
            raise RuntimeError("failed to stop DRV8701E clamp") from error

    def _finish_clamp(self, timer: threading.Timer) -> None:
        with self._lock:
            if self._clamp_timer is not timer or self._stop_flag.is_set():
                return
            self._clamp_timer = None
            try:
                self._stop_clamp_locked()
            except Exception:
                logger.exception("Failed to stop DRV8701E clamp after duration")
                self.emergency_stop()

    def _drive_clamp(self, name: str, *, ph_high: bool, duration) -> None:
        with self._lock:
            if self._stop_flag.is_set() or self._shutdown:
                raise RuntimeError("controller is in emergency-stop state")
            self._cancel_clamp_timer()
            try:
                self._stop_clamp_locked()
                self._clamp_write(lambda: self.pca9685.set_level(CLAMP_PH_CHANNEL, ph_high))
                duty = (
                    CLAMP_EN_DUTY_OPEN_PERCENT if ph_high
                    else CLAMP_EN_DUTY_CLOSE_PERCENT
                )
                self._clamp_write(lambda: self.pca9685.set_duty_cycle(CLAMP_EN_CHANNEL, duty))
                self._clamp_write(lambda: self.pca9685.set_level(CLAMP_NS_CHANNEL, True))
                time.sleep(0.001)  # DRV8701E nSLEEP wake-up time
            except Exception:
                try:
                    self._stop_clamp_locked()
                except Exception:
                    logger.exception("Failed to disable clamp after start error")
                raise
            self._running.add(CLAMP_EN_CHANNEL)
            self.action_history.append(
                (name, {"channel": CLAMP_EN_CHANNEL, "ph_high": ph_high,
                        "duty_percent": duty, "duration": duration})
            )
            if duration is not None and duration > 0:
                timer = threading.Timer(duration, lambda: self._finish_clamp(timer))
                timer.daemon = True
                self._clamp_timer = timer
                timer.start()

    def clamp(self, duration=None) -> None:
        """DRV8701E: PH low closes the clamp, EN drives at configured duty."""
        self._drive_clamp("clamp", ph_high=False, duration=duration)

    def unclamp(self, duration=None) -> None:
        """DRV8701E: PH high opens the clamp, EN drives at configured duty."""
        self._drive_clamp("unclamp", ph_high=True, duration=duration)

    def emergency_stop(self) -> None:
        self._stop_flag.set()
        with self._lock:
            self._cancel_clamp_timer()
            try:
                self._stop_clamp_locked()
            except Exception:
                logger.exception("Failed to disable clamp during emergency stop")
            self._running.discard(CLAMP_EN_CHANNEL)
        super().emergency_stop()

    def shutdown(self) -> None:
        with self._lock:
            if self._shutdown:
                return
            self._cancel_clamp_timer()
            try:
                self._stop_clamp_locked()
            except Exception:
                logger.exception("Failed to disable clamp during shutdown")
            self._running.discard(CLAMP_EN_CHANNEL)
        super().shutdown()

    def open(self) -> None:
        """打开底层 I2C 设备（PCA9685 支持显式 open）。"""
        open_ = getattr(self.pca9685, "open", None)
        if open_ is not None:
            open_()

    def init_escs(self, hold_s: float = 3.0) -> None:
        """初始化机构 CH0–7 的电调中位。"""
        channels = tuple(sorted(set(CHANNELS.values())))
        try:
            for ch in channels:
                self.set_pulse(ch, PULSE_STOP)
            if hold_s > 0:
                time.sleep(float(hold_s))
        except Exception:
            self.emergency_stop()
            raise
        self.action_history.append(
            ("init_escs", {"channels": list(channels),
                           "hold_s": hold_s})
        )

    def close(self) -> None:
        """关闭底层 I2C 设备与 X2P 升降伺服连接。"""
        self.shutdown()  # PCA9685在关闭文件描述符后仍自主输出中位PWM。
        close_ = getattr(self.pca9685, "close", None)
        if close_ is not None:
            close_()
        if self.lift_drive is not None:
            close_drive = getattr(self.lift_drive, "close", None)
            if close_drive is not None:
                close_drive()
            elif hasattr(self.lift_drive, "drive"):
                drive_close = getattr(self.lift_drive.drive, "close", None)
                if drive_close is not None:
                    drive_close()

    # -- 低层：真实 PCA9685 写脉宽 set_pwm(channel, pulse_us) -------------
    def _write_hw(self, channel: int, pulse_us: float) -> None:
        self.pca9685.set_pwm(channel, pulse_us)

    def _write_hw_off(self, channel: int) -> None:
        self.pca9685.channel_stop(channel)

    # -- 未接线通道占位（press/lift） -------------------------------------
    def _unwired(self, name: str, channel: int, action: str, duration) -> None:
        if ENABLE_UNWIRED_CHANNELS:
            self._act(name, channel, action, duration=duration)
        else:
            self.action_history.append(
                (name, {"channel": channel, "action": action,
                        "duration": duration, "placeholder": True})
            )

    def press(self, duration=None) -> None:
        """伺服升降-下压。

        注入 ``lift_drive``（X2P 伺服）时走真实伺服（down=下压）；
        否则保持未接通道占位。
        """
        if self.lift_drive is not None:
            self._x2p_lift("press", "down", duration)
            return
        self._unwired("press", CHANNELS["bin_shallow"], "open", duration)

    def lift(self, duration=None) -> None:
        """伺服升降-提升。

        注入 ``lift_drive``（X2P 伺服）时走真实伺服（up=提升）；
        否则保持未接通道占位。
        """
        if self.lift_drive is not None:
            self._x2p_lift("lift", "up", duration)
            return
        self._unwired("lift", CHANNELS["bin_shallow"], "close", duration)

    def fan(self, duration=None) -> None:
        """负压风机 CH7，按机构脉宽输出。"""
        self._act("fan", CHANNELS["fan"], "open", duration=duration)


class MockMechanismController(MechanismController):
    """机构控制器（mock 模式，兼容 test/conftest.py）。

    conftest 注入的 ``mock_pca9685`` 提供脉宽、固定电平和占空比接口，
    签名 ``set_pwm(channel, on, off)``，side_effect 把 off（脉宽 us）记录到
    ``register_history[channel]``。因此本类写脉宽统一调用
    ``pca9685.set_pwm(channel, 0, pulse_us)``（on=0, off=pulse_us）。
    夹爪动作继承 DRV8701E 的 PH/NS 逻辑，并记录固定的 EN PWM。

    所有动作同时记录到 ``action_history``（[(action_name, kwargs), ...]）。
    """

    def __init__(self, pca9685=None, mock_mode: bool = True) -> None:
        super().__init__(pca9685=pca9685, mock_mode=mock_mode)
        self.pca9685.set_level(CLAMP_NS_CHANNEL, False)
        self.pca9685.set_level(CLAMP_PH_CHANNEL, False)
        self.pca9685.set_duty_cycle(CLAMP_EN_CHANNEL, CLAMP_EN_DUTY_CLOSE_PERCENT)

    # -- 低层：conftest mock 签名 set_pwm(channel, on, off) ---------------
    def _write_hw(self, channel: int, pulse_us: float) -> None:
        if self.pca9685 is not None:
            self.pca9685.set_pwm(channel, 0, pulse_us)

    def _write_hw_off(self, channel: int) -> None:
        if self.pca9685 is not None:
            self.pca9685.set_pwm(channel, 0, PULSE_STOP)

    def press(self, duration=None) -> None:
        _BaseMechanismController.press(self, duration)

    def lift(self, duration=None) -> None:
        _BaseMechanismController.lift(self, duration)
