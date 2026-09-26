"""ROS mechanism service node — ``/mechanism/*`` action services + ``set_grain``.

把 PCA9685 机构驱动（:mod:`grain_sampling_devices.mechanism_driver`）暴露为
13 个 ROS 服务：

======================  ==================================================
Service                  Action
======================  ==================================================
/mechanism/clamp         夹紧采样管（CH5 关）
/mechanism/unclamp       松开夹紧（CH5 开）
/mechanism/tighten       拧紧接头（CH6 关）
/mechanism/untighten     拧松接头（CH6 开）
/mechanism/press         伺服升降-下压（未接通道占位）
/mechanism/lift          伺服升降-提升（未接通道占位）
/mechanism/start_suction 负压风机启动（CH7 未接通道占位）
/mechanism/stop_suction  负压风机停止
/mechanism/convey        螺旋输送开启（CH0/1，按品种 convey_duration 自动回停）
/mechanism/open_bin      开仓（按节点配置深度，默认 mid）
/mechanism/close_bin     关仓
/mechanism/emergency_stop 置停止标志 + 立即停所有运行中通道（抢断动作线程）
/mechanism/set_grain     选择品种（更新 MechanismController 品种参数）
======================  ==================================================

设计决策
--------
* **线程化动作 + 真实结果**：每个服务回调把动作派发到独立 daemon 工作线程并
  等待其结束，因此响应携带最终 ``success``/``message``。动作线程绝不阻塞
  rospy 主循环——ROS1 每条服务连接各自运行在线程中，所以
  ``/mechanism/emergency_stop`` 总能通过共享停止标志抢断运行中的动作。
  若想改为"立即返回"（fire-and-return），构造时传 ``wait_for_action=False``。
* **失败重试后上报**：动作先正常执行 1 次，再重试 ``retry_attempts`` 次
  （默认 2，即共 3 次尝试）；仍失败则响应 ``success=False``。
* **急停抢断**：``/mechanism/emergency_stop`` 置节点停止标志并立刻调用
  ``MechanismController.emergency_stop()``（对运行中通道写停止脉宽）。
  急停后节点拒绝新动作，直到 ``/mechanism/set_grain`` 重新使能
  （调用 ``controller.reset()``，作为自然的重置入口）。
* **未接通道占位**：press/lift 为占位动作——驱动层记录而不写
  I2C（除非 ``mechanism_driver.ENABLE_UNWIRED_CHANNELS`` 为 True）。节点默认
  返回占位成功；``placeholder_success=False`` 时返回失败。
* **HAS_ROS 判断**：无 rospy 环境（Windows 开发机）下模块可正常 import——
  服务类型退化为模块内最小类（``Trigger``/``SetGrain``），``rospy`` 为 None。
  ROS 节点初始化延迟到 :meth:`MechanismNode.start`，绝不在模块顶层执行。

.. note::
   ``/mechanism/open_bin`` 目前是单一 Trigger 服务（打开配置深度，默认
   ``open_bin_depth="mid"``），与本仓库 ``ros_bridge.call_open_bin`` 目前调用
   的 ``/mechanism/open_bin/{depth}`` 命名不同——桥接侧接入（orchestrator
   集成任务）时需对齐，或本节点按需补充 per-depth 变体。
"""

from __future__ import annotations

import logging
import uuid
import os
import threading
import time
from typing import Callable, Optional

from grain_sampling_devices.mechanism_driver import (
    CHANNELS,
    ENABLE_UNWIRED_CHANNELS,
    MechanismController,
    MockMechanismController,
)
from grain_sampling_workflow.mechanism_config import get_grain_params
from utils.sampling_params import X2P_DURATION_S, X2P_PORT, X2P_RPM

logger = logging.getLogger(__name__)


def _device_identity(path: str) -> Optional[str]:
    """Return a stable identity for a serial device path.

    ``os.readlink`` only works for udev symlinks such as ``/dev/x2p_lift``;
    the industrial PC may be configured with a plain ``/dev/ttyUSB1`` path.
    Combining the resolved path with ``st_rdev`` handles both forms and also
    detects USB re-enumeration when the tty number changes.
    """
    try:
        realpath = os.path.realpath(path)
        stat_result = os.stat(path)
    except (OSError, TypeError, ValueError):
        return None
    return f"{realpath}:{stat_result.st_rdev}"

# ---------------------------------------------------------------------------
# 回退消息类型（无 rospy 环境，Windows 开发机 / 测试）
# ---------------------------------------------------------------------------


class _TriggerRequest:
    """``std_srvs/Trigger`` 请求（回退实现：无字段）。"""

    __slots__ = ()
    _type = "std_srvs/TriggerRequest"


class _TriggerResponse:
    """``std_srvs/Trigger`` 响应（回退实现：success + message）。"""

    __slots__ = ("success", "message")
    _type = "std_srvs/TriggerResponse"

    def __init__(self, success: bool = False, message: str = "") -> None:
        self.success = success
        self.message = message


class _FallbackTrigger:
    """无 rospy 时的 ``std_srvs/Trigger`` 等价类型。"""

    _type = "std_srvs/Trigger"
    _request_class = _TriggerRequest
    _response_class = _TriggerResponse


class _SetGrainRequest:
    """``mechanism_node/SetGrain`` 请求（回退实现：单个字符串字段 grain）。"""

    __slots__ = ("grain",)
    _type = "mechanism_node/SetGrainRequest"

    def __init__(self, grain: str = "") -> None:
        self.grain = grain


class _SetGrainResponse:
    """``mechanism_node/SetGrain`` 响应（回退实现：success + message）。"""

    __slots__ = ("success", "message")
    _type = "mechanism_node/SetGrainResponse"

    def __init__(self, success: bool = False, message: str = "") -> None:
        self.success = success
        self.message = message


class _FallbackSetGrain:
    """无 rospy 时的 ``mechanism_node/SetGrain`` 等价类型。"""

    _type = "mechanism_node/SetGrain"
    _request_class = _SetGrainRequest
    _response_class = _SetGrainResponse


class _MoveLiftRequest:
    """``mechanism_node/MoveLift`` 请求（回退实现：direction + distance_cm）。"""

    __slots__ = ("direction", "distance_cm")
    _type = "mechanism_node/MoveLiftRequest"

    def __init__(self, direction: str = "", distance_cm: float = 0.0) -> None:
        self.direction = direction
        self.distance_cm = distance_cm


class _MoveLiftResponse:
    """``mechanism_node/MoveLift`` 响应（回退实现：success + message）。"""

    __slots__ = ("success", "message")
    _type = "mechanism_node/MoveLiftResponse"

    def __init__(self, success: bool = False, message: str = "") -> None:
        self.success = success
        self.message = message


class _FallbackMoveLift:
    """无 rospy 时的 ``mechanism_node/MoveLift`` 等价类型。"""

    _type = "mechanism_node/MoveLift"
    _request_class = _MoveLiftRequest
    _response_class = _MoveLiftResponse


# ---------------------------------------------------------------------------
# ROS 可选导入（HAS_ROS 模式，参考 ros_bridge.py）
# ---------------------------------------------------------------------------

SET_GRAIN_SRV_TEXT = (
    "string grain\n"
    "---\n"
    "bool success\n"
    "string message\n"
)

try:
    import rospy  # type: ignore[import-untyped]
    from std_srvs.srv import Trigger  # type: ignore[import-untyped]

    HAS_ROS = True
    try:
        # roslib 从已安装的 mechanism_node ROS 包加载 SetGrain/MoveLift 服务类型。
        # 注意：第二参数是 reload_on_error（布尔），不是 srv 文本。
        from roslib.message import get_service_class

        _SetGrain = get_service_class("mechanism_node/SetGrain")
        SetGrain = _SetGrain if _SetGrain is not None else _FallbackSetGrain
        if _SetGrain is None:
            logger.warning(
                "mechanism_node/SetGrain 服务类型不可用（ROS 包未安装）— "
                "set_grain 服务将不可用，其余 12 个 Trigger 服务正常"
            )
        _MoveLift = get_service_class("mechanism_node/MoveLift")
        MoveLift = _MoveLift if _MoveLift is not None else _FallbackMoveLift
        if _MoveLift is None:
            logger.warning(
                "mechanism_node/MoveLift 服务类型不可用（ROS 包未安装）— "
                "move_lift 服务将不可用"
            )
    except Exception:  # noqa: BLE001 - 退化为模块内回退类
        logger.warning(
            "roslib.message.get_service_class failed — SetGrain/MoveLift "
            "falls back to the local mock class (wire serialization unavailable)"
        )
        SetGrain = _FallbackSetGrain
        MoveLift = _FallbackMoveLift
except ImportError:
    rospy = None
    Trigger = _FallbackTrigger
    SetGrain = _FallbackSetGrain
    MoveLift = _FallbackMoveLift
    HAS_ROS = False
    logger.info("rospy not available — MechanismNode will run without ROS services")


# ---------------------------------------------------------------------------
# 节点常量
# ---------------------------------------------------------------------------

#: 12 个动作服务名（/mechanism/<name>，全部 std_srvs/Trigger）
ACTION_SERVICES: tuple[str, ...] = (
    "clamp",
    "unclamp",
    "tighten",
    "untighten",
    "press",
    "lift",
    "start_suction",
    "stop_suction",
    "convey",
    "start_convey",
    "stop_convey",
    "open_bin",
    "close_bin",
    "emergency_stop",
)

#: 占位动作（未接通道：伺服升降）。驱动层已按 ENABLE_UNWIRED_CHANNELS
#: 决定是否真实写 I2C；这里决定节点向调用方返回成功还是失败。
PLACEHOLDER_ACTIONS: frozenset[str] = frozenset({"press", "lift"})

#: 动作名 -> MechanismController 方法（无参/带参统一走 **kwargs）
ACTION_FUNCS: dict[str, Callable] = {
    "clamp": lambda c, **kw: c.clamp(**kw),
    "unclamp": lambda c, **kw: c.unclamp(**kw),
    "tighten": lambda c, **kw: c.tighten(**kw),
    "untighten": lambda c, **kw: c.untighten(**kw),
    "press": lambda c, **kw: c.press(**kw),
    "lift": lambda c, **kw: c.lift(**kw),
    "start_suction": lambda c, **kw: c.fan(**kw),
    "stop_suction": lambda c, **kw: c.actuate(CHANNELS["fan"], "stop"),
    "convey": lambda c, **kw: c.convey(**kw),
    "start_convey": lambda c, **kw: c.convey(duration=None),
    "stop_convey": lambda c, **kw: c.convey(duration=None, direction=0),
    "hold_bin_open": lambda c, **kw: c.hold_bin_open(depth=kw["depth"]),
    "open_bin": lambda c, **kw: c.open_bin(**kw),
    "close_bin": lambda c, **kw: c.close_bin(**kw),
}

#: 动作名 -> 品种参数键（提供自动回停时长；None = 不注入时长）
ACTION_DURATION_PARAM: dict[str, Optional[str]] = {
    "clamp": "clamp_duration",
    "unclamp": "unclamp_duration",
    "tighten": "tighten_duration",
    "untighten": "untighten_duration",
    "press": None,
    "lift": None,
    "start_suction": None,
    "stop_suction": None,
    "convey": "convey_duration",
    "start_convey": None,
    "stop_convey": None,
    "hold_bin_open": None,
    "open_bin": "open_duration",
    "close_bin": None,
}

DEFAULT_RETRY_ATTEMPTS = 2   # 额外重试次数（共 1 + 2 = 3 次尝试）
DEFAULT_RETRY_INTERVAL = 0.1  # 重试间隔（秒）
DEFAULT_OPEN_BIN_DEPTH = "mid"

#: 开仓深度名（对应 CH2 浅 / CH3 中 / CH4 深）。除默认 /mechanism/open_bin
#: 外，还注册 /mechanism/open_bin/{depth} 变体，供 ros_bridge 按
#: depth_level 调用（0→shallow、1→mid、2→deep）。
OPEN_BIN_DEPTHS: tuple[str, ...] = ("shallow", "mid", "deep")

#: depth_level 索引 → 深度名（ros_bridge call_open_bin(depth_level) 用）。
BIN_DEPTH_ALIASES: dict[int, str] = {0: "shallow", 1: "mid", 2: "deep"}


class MechanismNode:
    """ROS 机制服务节点。

    参数
    ----
    controller:
        机构控制器实例（真实 ``MechanismController`` 或 ``MockMechanismController``）。
        缺省时在 Windows/开发环境安全创建 mock 控制器。
    retry_attempts:
        动作失败后的额外重试次数，默认 2（共 3 次尝试）。
    retry_interval:
        每次重试间隔（秒）。
    wait_for_action:
        True：回调等待动作线程结束，响应携带最终结果（默认，供状态机判断成败）；
        False：回调只启动线程立即返回 success=True（fire-and-return），
        最终结果记录到 :attr:`last_results`。
    open_bin_depth:
        ``/mechanism/open_bin`` 默认打开深度（shallow/mid/deep）。
    placeholder_success:
        占位动作（press/lift/start_suction）是否返回成功。
    """

    SERVICE_PREFIX = "/mechanism"

    def __init__(
        self,
        controller: Optional[object] = None,
        retry_attempts: int = DEFAULT_RETRY_ATTEMPTS,
        retry_interval: float = DEFAULT_RETRY_INTERVAL,
        wait_for_action: bool = True,
        open_bin_depth: str = DEFAULT_OPEN_BIN_DEPTH,
        placeholder_success: bool = True,
    ) -> None:
        self._controller = controller or MockMechanismController(mock_mode=True)
        self._retry_attempts = max(0, int(retry_attempts))
        self._retry_interval = float(retry_interval)
        self._wait_for_action = bool(wait_for_action)
        self._open_bin_depth = open_bin_depth
        self._placeholder_success = bool(placeholder_success)

        self._stop_flag = threading.Event()  # 节点级急停标志（抢断动作线程）
        self._lock = threading.Lock()
        self._action_threads: dict[str, threading.Thread] = {}
        #: action name -> (success, message)（最近一次执行结果，wait_for_action=False 时查询用）
        self.last_results: dict[str, tuple[bool, str]] = {}
        self._grain: Optional[str] = None
        self._services: list = []
        self._started = False
        #: X2P 串口重连回调（USB 断开重连时重建 lift_drive）；None 表示未启用
        self._x2p_reconnect: Optional[Callable[[], None]] = None
        #: 当前运行时选择的 X2P 设备路径（支持 /dev/ttyUSB* 或 udev 链接）
        self._x2p_port: str = X2P_PORT
        #: 上次检测到的 X2P 串口设备身份（解析路径 + st_rdev）
        self._last_lift_device: Optional[str] = None
        self._x2p_rpm: float = max(1.0, float(X2P_RPM))
        self._x2p_duration: float = max(0.0, float(X2P_DURATION_S))

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """初始化 ROS 节点（如尚未初始化）并注册全部服务。幂等。"""
        if self._started:
            return
        if rospy is None:
            logger.warning("rospy unavailable — cannot register /mechanism services")
            return
        if HAS_ROS:
            try:
                rospy.init_node("mechanism_node", anonymous=True, disable_signals=True)
            except rospy.ROSException:
                logger.debug("rospy.init_node already called — reusing existing node")
        self._register_services()
        self._started = True
        logger.info(
            "MechanismNode started: %d services registered",
            len(self._services),
        )

    def run(self) -> None:
        """start() 后进入 rospy.spin()。无 rospy 时仅告警返回。"""
        self.start()
        if HAS_ROS and rospy is not None:
            rospy.spin()
        else:
            logger.warning("rospy unavailable — spin skipped (mock mode)")

    def shutdown(self) -> None:
        """急停 + 关闭控制器。幂等。"""
        self._stop_flag.set()
        try:
            self._controller.emergency_stop()
        except Exception:  # noqa: BLE001 - 关闭尽力而为
            pass
        try:
            self._controller.shutdown()
        except Exception:  # noqa: BLE001 - 关闭尽力而为
            pass

    # ------------------------------------------------------------------
    # 服务注册
    # ------------------------------------------------------------------

    def _register_services(self) -> None:
        """注册 12 个 Trigger 动作服务 + 1 个 set_grain 服务。

        当 ``mechanism_node/SetGrain`` ROS 服务类型不可用（包未安装）时，
        set_grain 服务跳过注册（回退类无 ROS 序列化能力），其余服务正常。
        """
        for action in ACTION_SERVICES:
            name = f"{self.SERVICE_PREFIX}/{action}"
            handler = (
                self._handle_emergency_stop
                if action == "emergency_stop"
                else self._make_action_handler(action)
            )
            self._services.append(rospy.Service(name, Trigger, handler))
        self._services.append(
            rospy.Service(
                f"{self.SERVICE_PREFIX}/lift_health",
                Trigger,
                self._handle_lift_health,
            )
        )
        if SetGrain is not _FallbackSetGrain:
            self._services.append(
                rospy.Service(f"{self.SERVICE_PREFIX}/set_grain", SetGrain, self._handle_set_grain)
            )
        else:
            logger.warning(
                "跳过 /mechanism/set_grain 注册：SetGrain 服务类型不可用"
            )
        if MoveLift is not _FallbackMoveLift:
            self._services.append(
                rospy.Service(f"{self.SERVICE_PREFIX}/move_lift", MoveLift, self._handle_move_lift)
            )
        else:
            logger.warning(
                "跳过 /mechanism/move_lift 注册：MoveLift 服务类型不可用"
            )
        # open_bin/close_bin 深度变体：/mechanism/{action}/{shallow|mid|deep}
        # （ros_bridge 按 depth_level 索引调用；固定深度，不依赖节点默认）。
        for action in ("open_bin", "close_bin", "hold_bin_open"):
            for depth in OPEN_BIN_DEPTHS:
                self._services.append(
                    rospy.Service(
                        f"{self.SERVICE_PREFIX}/{action}/{depth}",
                        Trigger,
                        self._make_bin_depth_handler(action, depth),
                    )
                )

    def _make_action_handler(self, action: str) -> Callable:
        """为单个动作服务生成 rospy 回调（``req`` 为 TriggerRequest）。"""

        def _handler(req) -> object:
            return self._handle_action(action, req)

        return _handler

    def _make_bin_depth_handler(self, action: str, depth: str) -> Callable:
        """为 /mechanism/{action}/{depth} 变体生成回调，固定深度。"""

        def _handler(req) -> object:
            return self._handle_action(action, req, depth=depth)

        return _handler

    # ------------------------------------------------------------------
    # 动作执行（线程内，含重试）
    # ------------------------------------------------------------------

    def run_action(self, action: str, **kwargs) -> tuple[bool, str]:
        """执行单个动作，失败重试 ``retry_attempts`` 次后返回 (success, message)。

        在调用前/调用后检查急停标志——急停可抢断阻塞中的动作调用。
        """
        if action not in ACTION_FUNCS or action not in ACTION_DURATION_PARAM:
            return False, f"unknown action {action!r}"
        if self._stop_flag.is_set():
            return False, "mechanism in emergency-stop state; call /mechanism/set_grain to reset"

        if action in ("open_bin", "close_bin") and "depth" not in kwargs:
            kwargs["depth"] = self._open_bin_depth
        if "duration" not in kwargs and action != "stop_suction":
            kwargs["duration"] = self._duration_for(action)

        # X2P may be plugged in after the ROS node starts. Try to discover and
        # reconnect it before deciding whether press/lift is still a placeholder.
        if action in ("press", "lift"):
            self._ensure_lift_connected()

        placeholder = action in PLACEHOLDER_ACTIONS and not ENABLE_UNWIRED_CHANNELS
        # press/lift 走 X2P 伺服时不是占位（lift_drive 已注入）——否则消息会误导
        if placeholder and action in ("press", "lift") and (
            getattr(self._controller, "lift_drive", None) is not None
        ):
            placeholder = False
        if placeholder and not self._placeholder_success:
            return False, f"{action} is a placeholder (channel unwired); placeholder disabled"

        last_error: Optional[Exception] = None
        for attempt in range(1 + self._retry_attempts):
            if self._stop_flag.is_set():
                return False, "mechanism in emergency-stop state"
            try:
                ACTION_FUNCS[action](self._controller, **kwargs)
            except Exception as exc:  # noqa: BLE001 - 重试语义捕获一切底层错误
                last_error = exc
                if "emergency-stop" in str(exc):
                    return False, str(exc)
                if attempt < self._retry_attempts:
                    time.sleep(self._retry_interval)
                continue
            if self._stop_flag.is_set():
                return False, f"{action} preempted by emergency stop"
            note = " (placeholder: channel unwired)" if placeholder else ""
            return True, f"{action} ok{note}"

        return (
            False,
            f"{action} failed after {1 + self._retry_attempts} attempts: {last_error}",
        )

    def _duration_for(self, action: str) -> Optional[float]:
        """当前品种参数中的动作自动回停时长（无参动作返回 None）。"""
        param = ACTION_DURATION_PARAM[action]
        if param is None:
            return None
        return get_grain_params(self._grain or "")[param]

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------

    def _handle_action(self, action: str, req=None, **kwargs) -> object:
        """动作服务回调：动作在独立线程执行，响应携带最终结果。

        回调只负责启动 daemon 工作线程；动作本体在独立线程中运行（急停可通过
        共享停止标志抢断它）。``wait_for_action=True`` 时回调阻塞在
        ``thread.join()`` 直到线程完成，从而把最终 success/message 回填进响应
        （每个 rospy 服务连接各自运行在线程中，互不阻塞其他服务）；
        ``wait_for_action=False`` 时立即返回"已启动"。

        ``kwargs``（如 ``open_bin`` 的 ``depth``）透传给底层动作。
        """
        logger.info("ACTION_REQUEST action=%s kwargs=%s", action, kwargs)
        resp = Trigger._response_class(success=False, message="starting")
        thread = threading.Thread(
            target=self._execute, args=(action, resp), kwargs=kwargs, daemon=True
        )
        with self._lock:
            self._action_threads[action] = thread
        thread.start()
        if self._wait_for_action:
            thread.join()
        else:
            resp.success = True
            resp.message = "action started (async)"
        return resp

    def _execute(self, action: str, resp, **kwargs) -> None:
        """动作工作线程体：执行动作并回填响应 / 记录结果。"""
        logger.info("ACTION_BEGIN action=%s kwargs=%s", action, kwargs)
        try:
            success, message = self.run_action(action, **kwargs)
        except Exception as exc:  # noqa: BLE001 - service must return a result
            logger.exception("ACTION_EXCEPTION action=%s", action)
            success, message = False, f"{action} unhandled exception: {exc}"
        resp.success = success
        resp.message = message
        log = logger.info if success else logger.error
        log("ACTION_RESULT action=%s success=%s message=%s", action, success, message)
        with self._lock:
            self.last_results[action] = (success, message)
            self._action_threads.pop(action, None)

    def _handle_emergency_stop(self, req=None) -> object:
        """急停：置停止标志 + 立刻停所有运行中通道（抢断阻塞动作线程）。"""
        self._stop_flag.set()
        try:
            self._controller.emergency_stop()
        except Exception as exc:  # noqa: BLE001 - 急停上报失败但仍保持锁定
            return Trigger._response_class(
                success=False,
                message=f"emergency stop failed: {exc}",
            )
        return Trigger._response_class(success=True, message="emergency stop executed")

    def _handle_set_grain(self, req) -> object:
        """set_grain：更新当前品种参数；急停后同时作为重新使能入口。"""
        reset_msg = ""
        if self._stop_flag.is_set():
            self._stop_flag.clear()
            try:
                self._controller.reset()
            except Exception:  # noqa: BLE001 - reset 尽力而为
                pass
            reset_msg = " (emergency stop cleared)"
        try:
            ok = self._controller.set_grain(req.grain)
        except Exception as exc:  # noqa: BLE001
            return SetGrain._response_class(success=False, message=f"set_grain failed: {exc}")
        self._grain = req.grain
        if ok:
            return SetGrain._response_class(
                success=True, message=f"grain set to {req.grain!r}{reset_msg}"
            )
        return SetGrain._response_class(
            success=False,
            message=f"unknown grain {req.grain!r}; using default params{reset_msg}",
        )

    def set_x2p_reconnect(self, fn: Callable[[], None], port: Optional[str] = None) -> None:
        """注入 X2P 串口重连回调（USB 断开重连时自动重建 lift_drive）。

        同时记录当前设备身份，供 :meth:`_ensure_lift_connected` 检测
        ``/dev/ttyUSB*`` 重枚举或 udev 链接目标变化。
        """
        self._x2p_reconnect = fn
        # Keep the runtime-selected port (for example /dev/ttyUSB1 on the
        # industrial PC) instead of consulting the historical global default.
        self._x2p_port = port or X2P_PORT
        self._last_lift_device = _device_identity(self._x2p_port)

    def _lift_device_changed(self) -> bool:
        """检测 X2P 设备身份是否变化（USB 重枚举或设备暂时消失）。"""
        if self._x2p_reconnect is None:
            return False
        target = _device_identity(self._x2p_port)
        # 初始状态和当前都不存在时不触发重连；其余变化（消失、出现、
        # tty 编号变化或 udev 链接目标变化）都需要重建串口句柄。
        if getattr(self._controller, "lift_drive", None) is None and target is not None:
            return True
        if target is None and self._last_lift_device is None:
            return False
        return target != self._last_lift_device

    def _reconnect_lift_and_track(self) -> None:
        """重建 lift_drive 并更新记录的设备号；重建后立即安全停机。"""
        self._x2p_reconnect()
        self._last_lift_device = _device_identity(self._x2p_port)
        # 断连瞬间伺服可能仍在运行（保持最后速度命令），重建后立即停机，
        # 避免重试 move 时伺服跑飞。
        try:
            lift = getattr(self._controller, "lift_drive", None)
            if lift is not None:
                lift.stop()
                logger.info("X2P 重连后安全停机完成")
        except Exception:  # noqa: BLE001 - 停机尽力而为
            logger.exception("X2P 重连后安全停机失败（可能伺服已停）")

    def _ensure_lift_connected(self) -> None:
        """检测 X2P 串口是否断开重连（ttyUSB 设备号变化），变化则自动重建。

        USB 串口（FTDI）在连续高速运转时可能掉电重连，设备从 ttyUSB0 变为
        ttyUSB1；旧文件句柄失效，后续 move_lift 会串口 I/O 错误甚至伺服跑飞。
        每次 move_lift 前调用本方法，一旦检测到设备号变化就调用重连回调
        重建 lift_drive，避免伺服失控。
        """
        if self._lift_device_changed():
            logger.warning(
                "X2P 串口重连检测: 设备号 %s 已变化", self._last_lift_device
            )
            try:
                self._reconnect_lift_and_track()
                logger.info("X2P 串口重连完成，lift_drive 已重建")
            except Exception:  # noqa: BLE001 - 重连失败不阻塞动作
                logger.exception("X2P 串口重连失败")

    def _wait_lift_device_change(self, timeout_s: float = 3.0) -> bool:
        """等待 /dev/x2p_lift 设备号变化（USB 重枚举），最多 timeout_s 秒。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._lift_device_changed():
                return True
            time.sleep(0.2)
        return self._lift_device_changed()

    def _handle_move_lift(self, req) -> object:
        """move_lift：按距离移动伺服升降（编码器闭环，精确停在目标距离）。"""
        direction = str(req.direction).strip().lower()
        distance_cm = float(req.distance_cm)
        if direction not in ("up", "down", "down_cycle", "return"):
            return MoveLift._response_class(
                success=False,
                message=("方向必须是 up/down/down_cycle/return，"
                         f"得到 {req.direction!r}"),
            )
        if not distance_cm > 0:
            return MoveLift._response_class(
                success=False, message="distance_cm 必须大于 0"
            )
        # 时长按 X2P_RPM 与 5mm 导程精确计算。X2P_RPM 已是经过
        # SafetyLimits 校验的软件上限，不再额外乘 1.2 降速。
        rpm = max(1.0, float(self._x2p_rpm))
        duration_s = (distance_cm * 10.0) / (5.0 * rpm / 60.0)
        request_id = uuid.uuid4().hex[:12]
        logger.info("MOVE_REQUEST id=%s direction=%s distance_cm=%s duration_s=%s",
                    request_id, direction, distance_cm, duration_s)
        try:
            self._ensure_lift_connected()
            result = self._controller.move_lift(direction, distance_cm, duration_s)
        except Exception as exc:  # noqa: BLE001 - 上报失败
            # Completion is uncertain: never replay a relative stroke after USB
            # recovery or an encoder tolerance failure. Workflow will stop.
            logger.exception("MOVE_FAILED id=%s; automatic replay disabled", request_id)
            return MoveLift._response_class(
                success=False, message=f"move_lift id={request_id} failed (no replay): {exc}"
            )
        logger.info("MOVE_SUCCESS id=%s result=%s", request_id, result)
        return MoveLift._response_class(
            success=True,
            message=f"move_lift {direction} {distance_cm}cm ok: {result}",
        )

    def _handle_lift_health(self, req=None) -> object:
        """Read the lift encoder without moving it.

        A serial port opening successfully does not prove that the X2P drive is
        responding.  This service is deliberately read-only so the workflow can
        verify the complete UART/RS485/Modbus path before operating the clamp.
        """
        try:
            self._ensure_lift_connected()
            lift = getattr(self._controller, "lift_drive", None)
            if lift is None:
                raise RuntimeError(
                    f"X2P servo unavailable on {self._x2p_port}: no responding drive"
                )
            position = lift.read_position()
            check = getattr(lift, "health_check", None)
            readiness = check() if callable(check) else None
        except Exception as exc:  # noqa: BLE001 - return the hardware cause to UI
            logger.exception("X2P read-only health check failed")
            lift = getattr(self._controller, "lift_drive", None)
            if lift is not None:
                try:
                    lift.close()
                except Exception:  # noqa: BLE001 - cleanup is best effort
                    pass
                self._controller.lift_drive = None
            return Trigger._response_class(
                success=False,
                message=f"X2P communication health check failed: {exc}",
            )
        return Trigger._response_class(
            success=True,
            message=(
                "X2P communication and servo-enable healthy; "
                f"encoder_position={position}; readiness={readiness}"
            ),
        )


def main() -> None:
    """启动入口（板端运行：真实控制器；开发机：mock 模式）。

    可选参数（环境变量，便于 start.sh 注入；默认值取自 utils.sampling_params）：
    - ``X2P_PORT``：X2P 伺服串口（工控机启动脚本默认 /dev/ttyUSB1，
      也可使用 /dev/x2p_lift 稳定链接）；设空串禁用 X2P
    - ``X2P_SLAVE``：Modbus 从站地址（默认 2）
    - ``X2P_RPM``：升降转速 r/min（默认值见 sampling_params.py）
    - ``X2P_DURATION``：升降时长秒（默认 2.0）
    - ``X2P_FORWARD_SIGN``：升降方向 1/-1（默认 1）
    """
    import os

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s [%(threadName)s]: %(message)s")

    from utils.sampling_params import (
        X2P_DURATION_S,
        X2P_FORWARD_SIGN,
        X2P_PORT,
        X2P_RPM,
        X2P_SLAVE,
    )

    controller = MechanismController(mock_mode=not HAS_ROS)
    reconnect_lift = None  # X2P 串口重连回调（USB 断开重连后重建 lift_drive）
    x2p_port = os.environ.get("X2P_PORT", X2P_PORT)
    runtime_x2p_slave = int(os.environ.get("X2P_SLAVE", str(X2P_SLAVE)))
    runtime_x2p_rpm = max(1, int(os.environ.get("X2P_RPM", str(X2P_RPM))))
    runtime_x2p_duration = max(
        0.0, float(os.environ.get("X2P_DURATION", str(X2P_DURATION_S)))
    )
    runtime_x2p_forward_sign = int(
        os.environ.get("X2P_FORWARD_SIGN", str(X2P_FORWARD_SIGN))
    )
    if HAS_ROS:
        controller.open()
        # 电调 1500us 中位初始化（实机确认：电调需先收到中位信号才能
        # 响应控制；full-off/无信号上电后直接给动作脉宽不响应）。
        try:
            controller.init_escs(hold_s=3.0)
            logger.info("机构 CH0–7 电调 1500us 中位初始化完成")
        except Exception:  # 不对外发布一个电调尚未初始化成功的机构服务
            controller.close()
            logger.exception("电调初始化失败，停止机构节点启动")
            raise
        # 尝试接入 X2P 伺服升降（失败仅告警，退回占位）
        def _build_lift() -> object:
            from grain_sampling_devices.x2p_lift import build_x2p_lift_drive

            drive = build_x2p_lift_drive(
                port=x2p_port,
                slave=runtime_x2p_slave,
                rpm=runtime_x2p_rpm,
                duration_s=runtime_x2p_duration,
                forward_sign=runtime_x2p_forward_sign,
            )
            try:
                position = drive.read_position()
            except Exception:
                try:
                    drive.close()
                except Exception:  # noqa: BLE001 - cleanup is best effort
                    pass
                raise
            logger.info(
                "X2P communication probe passed: port=%s encoder_position=%s",
                x2p_port,
                position,
            )
            return drive

        def _reconnect_lift() -> None:
            old_drive = getattr(controller, "lift_drive", None)
            new_drive = _build_lift()
            controller.lift_drive = new_drive
            if old_drive is not None and old_drive is not new_drive:
                try:
                    old_drive.stop()
                except Exception:  # noqa: BLE001 - replacement is best effort
                    pass
                try:
                    old_drive.close()
                except Exception:  # noqa: BLE001 - replacement is best effort
                    pass
            logger.info("X2P 伺服升降已重连: port=%s", x2p_port)

        if x2p_port:
            try:
                controller.lift_drive = _build_lift()
                logger.info("X2P 伺服升降已启用: port=%s", x2p_port)
            except Exception as exc:  # noqa: BLE001 - 升降缺失不阻塞机构服务
                logger.warning("X2P 伺服升降未启用: %s", exc)
            # Keep the callback even when the first open fails. If the USB-RS485
            # adapter is inserted later, press/lift/move_lift can reconnect it.
            reconnect_lift = _reconnect_lift
    controller.lift_rpm = runtime_x2p_rpm
    controller.lift_duration = runtime_x2p_duration
    node = MechanismNode(controller=controller)
    node._x2p_rpm = float(runtime_x2p_rpm)
    node._x2p_duration = runtime_x2p_duration
    if reconnect_lift is not None and x2p_port:
        node.set_x2p_reconnect(reconnect_lift, port=x2p_port)
    try:
        node.run()
    finally:
        node.shutdown()
        if HAS_ROS:
            controller.close()


if __name__ == "__main__":
    main()
