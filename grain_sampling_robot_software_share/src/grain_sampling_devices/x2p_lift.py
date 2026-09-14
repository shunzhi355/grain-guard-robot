"""X2P 伺服升降适配器 — 为 mechanism press/lift 提供真实伺服驱动。

使用 dais516 的 ``x2p`` 包（USB-RS485 + Modbus RTU）控制 X2P 伺服驱动器，
实现粮食扦样机的升降（press=下压 / lift=提升）。

设计要点
--------
* **懒加载 import**：``x2p`` 依赖 ``pyserial``，仅在板端（有 serial）真正
  调用时导入；Windows 开发机/无串口环境 import 本模块不失败。
* **可选构造**：串口不可用或未接驱动时抛 ``DeviceError``，调用方（如
  mechanism_node 的 ``main()``）可捕获并退回占位模式。
* **安全**：X2P 库自带完整安全流程（OFF 基线、使能核验、超时停止、
  取消使能确认）。垂直轴仍需硬件急停/限位/防坠（见 X2P使用说明.md）。
"""

from __future__ import annotations

import logging
import sys
from typing import Optional

from grain_sampling_devices.base_adapter import DeviceError
from utils.sampling_params import (
    X2P_DURATION_S,
    X2P_FORWARD_SIGN,
    X2P_PACKAGE_PATH as DEFAULT_X2P_PACKAGE_PATH,
    X2P_PORT,
    X2P_RPM,
    X2P_SLAVE,
)

logger = logging.getLogger(__name__)

#: x2p 包所在目录（dais516 仓库根）。默认空=依赖 sys.path；可填绝对路径。
X2P_PACKAGE_PATH: str = DEFAULT_X2P_PACKAGE_PATH


def _map_lift_direction(direction: str) -> str:
    """把升降语义方向映射到 x2p 驱动器方向名。

    语义：``up``=提升（→驱动器 forward）、``down``=下压（→驱动器 reverse）。
    兼容旧值 forward/reverse 直接透传。
    """
    mapping = {"up": "forward", "down": "reverse"}
    return mapping.get(str(direction).strip().lower(), str(direction))


def build_x2p_lift_drive(
    port: str = X2P_PORT,
    slave: int = X2P_SLAVE,
    *,
    rpm: int = X2P_RPM,
    duration_s: float = X2P_DURATION_S,
    forward_sign: int = X2P_FORWARD_SIGN,
    config_path: str = "",
) -> object:
    """构造 X2P 升降驱动器（MotionController 包装）。

    Parameters
    ----------
    port : str
        USB-RS485 串口设备路径。
    slave : int
        Modbus 从站地址（默认 2，与 x2p_config.json 一致）。
    rpm : int
        升降转速（r/min），默认 30。
    duration_s : float
        升降时长（秒），默认 2.0。
    forward_sign : int
        升降方向：1=正向，-1=反向翻转（实机方向不对时在
        utils.sampling_params 改 X2P_FORWARD_SIGN）。
    config_path : str
        可选的 x2p_config.json 路径；缺省用内置安全默认值。

    Returns
    -------
    object
        一个 ``LiftDrive`` 适配对象，暴露：
        ``run_speed(direction, rpm, duration_s)``、``stop()``、``close()``。

    Raises
    ------
    DeviceError
        串口不可用 / x2p 无法导入 / 配置非法。
    """
    # x2p is deployed separately on some industrial PCs.  Add its configured
    # parent directory immediately before the lazy import so normal imports
    # remain safe on development machines without that package.
    if X2P_PACKAGE_PATH and X2P_PACKAGE_PATH not in sys.path:
        sys.path.insert(0, X2P_PACKAGE_PATH)
    try:
        from x2p import ControllerConfig, MotionController, X2PDrive
    except ImportError as exc:  # pragma: no cover - 依赖缺失
        raise DeviceError(
            "无法导入 x2p 包（需要 pyserial 且 x2p 在 sys.path）。"
            "请确认板端已部署 dais516 且 pip install pyserial。"
        ) from exc

    try:
        if config_path:
            config = ControllerConfig.load(config_path)
        else:
            from x2p import SafetyLimits

            # frozen dataclass：构造时传入 limits（max_rpm 覆盖为 rpm 与默认的较大值）
            config = ControllerConfig(
                port=port,
                slave=slave,
                forward_sign=int(forward_sign),
                limits=SafetyLimits(max_rpm=max(30, int(rpm))),
            )
        drive = X2PDrive(config.port, config.slave)
        controller = MotionController(drive, config)
    except Exception as exc:  # noqa: BLE001 - 统一包装为设备错误
        raise DeviceError(f"X2P 升降驱动器初始化失败: {exc}") from exc

    logger.info(
        "X2P 升降就绪: port=%s slave=%d rpm=%d duration=%.1fs",
        config.port, config.slave, rpm, duration_s,
    )
    return _LiftDrive(controller, rpm=rpm, duration_s=duration_s)


class _LiftDrive:
    """把 X2P MotionController 包装成 mechanism 可用的升降驱动器。

    属性 ``rpm``/``duration_s`` 可现场标定覆盖；方向由 X2P 的
    ``forward_sign`` 配置翻转，不在此硬编码。
    """

    def __init__(self, controller: object, *, rpm: int, duration_s: float) -> None:
        self._controller = controller
        self.rpm = int(rpm)
        self.duration_s = float(duration_s)
        self.config = getattr(controller, "config", None)

    def run_speed(self, direction: str, rpm: Optional[int] = None,
                  duration_s: Optional[float] = None) -> object:
        """按方向运行速度模式（安全流程内置：OFF→使能→运行→停→OFF）。

        方向语义：``up``=提升（映射到驱动器 forward）、``down``=下压
        （映射到驱动器 reverse）。实机方向反了用 ``forward_sign`` 翻转，
        不在此硬编码。
        """
        drive_direction = _map_lift_direction(direction)
        return self._controller.run_speed(
            drive_direction,
            rpm=int(rpm if rpm is not None else self.rpm),
            duration_s=float(duration_s if duration_s is not None else self.duration_s),
        )

    def move_distance(self, direction: str, distance_mm: float,
                      duration_s: float, tolerance_mm: float = 8.0) -> object:
        """按距离移动（编码器闭环，精确停在目标距离）。

        方向语义同 :meth:`run_speed`：``up``=提升（→forward）、
        ``down``=下压（→reverse）。底层走 ``move_timed_distance``，
        用编码器反馈保证到达 ``distance_mm``（±``tolerance_mm``）。

        Parameters
        ----------
        direction : str
            ``up`` / ``down``（或 forward/reverse 直接透传）。
        distance_mm : float
            移动距离（毫米），必须 > 0。
        duration_s : float
            移动时长（秒）。过长/过短由 x2p 内部按 max_rpm 校验。
        tolerance_mm : float
            位置容差（毫米）。默认 8.0mm——带负载（夹紧/拧紧）时
            误差波动到 2~6mm，x2p 默认 0.2mm 过严，放宽到 8mm
            保证 25cm 行程稳定通过（扦样机升降 8mm 精度足够）。
        """
        drive_direction = _map_lift_direction(direction)
        move = getattr(self._controller, "move_timed_distance", None)
        if move is None:
            raise DeviceError(
                "x2p 控制器不支持 move_timed_distance（距离控制不可用）"
            )
        return move(drive_direction, float(distance_mm), float(duration_s),
                    tolerance_mm=float(tolerance_mm))

    def stop(self) -> None:
        """安全停止（速度归零 + 取消使能 + 确认 OFF）。"""
        self._controller.stop()

    def close(self) -> None:
        """关闭底层串口。"""
        close_drive = getattr(self._controller, "drive", None)
        if close_drive is not None:
            drive_close = getattr(close_drive, "close", None)
            if drive_close is not None:
                drive_close()

    @property
    def drive(self) -> object:
        return getattr(self._controller, "drive", None)
