"""RC 手动控制独立 ROS 节点 — 不再依赖 PyQt UI。

把 :mod:`grain_sampling_workflow.rc_control` 的 RC 手动控制逻辑包装为独立
ROS 节点，使遥控底盘控制在 UI 未运行时同样可用。节点以 10 Hz 采样 RC 接收机，
默认经 :class:`~grain_sampling_workflow.ros_bridge.SamplingBridge` 输出到
``/cmd_vel``；工控机设置 ``RC_OUTPUT_MODE=direct_tracks`` 后，手动模式直接
发送左右履带 UDP 指令（CH1→CH8、CH3→CH9）。节点同时发布 ``/rc_mode``。

设计决策
--------
* **完全复用现有类**：模式映射/消抖/通道死区全部委托给
  :class:`~grain_sampling_workflow.rc_control.RCControl`，本节点只负责
  生命周期、节拍与状态发布，不重实现任何逻辑。
* **HAS_ROS 判断**：无 rospy 环境（Windows 开发机）下模块可正常 import——
  ``create_rc_receiver`` 返回 ``MockRCReceiver``，``SamplingBridge`` 退化为
  stub，RCControl 仍按真实映射/消抖逻辑执行，便于测试与开发。
* **init_node 顺序**：先 ``rospy.init_node`` 再构造 ``SamplingBridge``
  （其内部再次 ``init_node`` 会抛 ``rospy.ROSException``，已由桥自身捕获）；
  节点侧再兜一层 try/except，保证构造失败时不会留下半初始化状态。
* **/rc_mode 状态**：每次 tick 发布 ``manual``/``auto``/空串（模式尚未
  经消抖确定时为 None），供下游监控遥控是否接管底盘。

.. note::
   部署到机器人板（orangepi，ROS noetic）后与 ``mechanism_node`` 一并启动：
   ``rosrun grain_sampling_workflow rc_node.py`` 或
   ``python3 -m grain_sampling_workflow.rc_node``。
"""

from __future__ import annotations

import logging
import os
import time
from typing import Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ROS 可选导入（HAS_ROS 模式，参考 mechanism_node.py / ros_bridge.py）
# ---------------------------------------------------------------------------

try:
    import rospy  # type: ignore[import-untyped]
    from std_msgs.msg import String  # type: ignore[import-untyped]

    HAS_ROS = True
except ImportError:
    rospy = None
    String = None
    HAS_ROS = False
    logger.info("rospy not available — rc_node will run in mock mode")

from grain_sampling_devices.rc_receiver import create_rc_receiver
from grain_sampling_workflow.rc_control import RCControl
from grain_sampling_workflow.rc_motor_bridge import RCTrackMotorBridge
from grain_sampling_workflow.ros_bridge import SamplingBridge

# ---------------------------------------------------------------------------
# 节点常量
# ---------------------------------------------------------------------------

#: 节点名（anonymous 后缀避免与 UI 内嵌桥冲突）。
NODE_NAME = "rc_control_node"

#: 模式状态话题（manual / auto / 空串表示尚未确定）。
RC_MODE_TOPIC = "/rc_mode"

#: 采样 / 发布频率（Hz），与 UI 的 QTimer 节拍保持一致。
TICK_HZ = 10.0


class RCControlNode:
    """把 RC 手动控制包装成独立 ROS 节点。

    只负责节点生命周期、10 Hz 节拍与 ``/rc_mode`` 状态发布；遥控映射与
    消抖逻辑完全委托给 :class:`RCControl`，底盘输出经由
    :class:`SamplingBridge` 不变。
    """

    def __init__(self) -> None:
        self._receiver = None
        self._bridge = None
        self._navigation_bridge: Optional[SamplingBridge] = None
        self._control: Optional[RCControl] = None
        self._mode_pub = None
        self._rate = None
        self._last_mode: Optional[str] = None

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """初始化 ROS 节点、接收机、桥与控制器。"""
        if HAS_ROS:
            try:
                rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)
            except rospy.ROSException:
                logger.debug("rospy.init_node already called — reusing existing node")
            self._mode_pub = rospy.Publisher(RC_MODE_TOPIC, String, queue_size=10)

        self._receiver = create_rc_receiver()

        # 在 init_node 之后构造桥；桥内部再次 init_node 会抛 ROSException
        # （已由桥自身捕获），这里再兜一层，构造失败则终止节点。
        try:
            self._navigation_bridge = SamplingBridge(node_name=NODE_NAME)
        except Exception:  # noqa: BLE001 - 记录后重新抛出，节点无法继续
            logger.exception("SamplingBridge init failed — rc_node aborting")
            raise

        output_mode = os.environ.get("RC_OUTPUT_MODE", "cmd_vel").strip().lower()
        if output_mode == "direct_tracks":
            self._bridge = RCTrackMotorBridge(
                self._navigation_bridge,
                host=os.environ.get("MOTOR_DRIVER_HOST", "127.0.0.1"),
                port=int(os.environ.get("MOTOR_DRIVER_PORT", "8765")),
            )
        elif output_mode == "cmd_vel":
            self._bridge = self._navigation_bridge
        else:
            raise ValueError("RC_OUTPUT_MODE must be cmd_vel or direct_tracks")

        self._control = RCControl(self._receiver, self._bridge)
        self._rate = rospy.Rate(TICK_HZ) if HAS_ROS else None
        logger.info(
            "RC control node started (HAS_ROS=%s, receiver=%s, output=%s)",
            HAS_ROS,
            type(self._receiver).__name__,
            output_mode,
        )

    @property
    def manual_active(self) -> bool:
        """True 当消抖后的模式为 manual（底盘被遥控接管）。"""
        return bool(self._control is not None and self._control.manual_active)

    def run(self) -> None:
        """10 Hz 采样循环，直至 ``rospy.is_shutdown()``（mock 模式无限循环）。"""
        if self._control is None:
            raise RuntimeError("call start() before run()")
        while True:
            if HAS_ROS and rospy.is_shutdown():
                break
            self._tick_once()
            if self._rate is not None:
                self._rate.sleep()
            else:
                time.sleep(1.0 / TICK_HZ)

    def shutdown(self) -> None:
        """停止底盘（发布零速度）并释放接收机。幂等。"""
        if self._bridge is not None:
            try:
                self._bridge.publish_cmd_vel(0.0, 0.0)
            except Exception:  # noqa: BLE001 - 关闭尽力而为
                logger.exception("publish zero cmd_vel failed during shutdown")
        if self._receiver is not None:
            try:
                self._receiver.stop()
            except Exception:  # noqa: BLE001 - 关闭尽力而为
                pass
        if isinstance(self._bridge, RCTrackMotorBridge):
            self._bridge.close()
        logger.info("RC control node shut down")

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------

    def _tick_once(self) -> None:
        """采样一次、发布 /rc_mode 并记录模式切换。"""
        mode = self._control.tick()
        self._publish_mode(mode)
        if mode != self._last_mode:
            logger.info("RC mode transition: %s -> %s", self._last_mode, mode)
            self._last_mode = mode

    def _publish_mode(self, mode: Optional[str]) -> None:
        """把当前模式发布到 ``/rc_mode``（None -> 空串）。"""
        if self._mode_pub is not None:
            self._mode_pub.publish(String(mode or ""))


def main() -> None:
    """独立节点入口：``python3 -m grain_sampling_workflow.rc_node``。"""
    node = RCControlNode()
    node.start()
    try:
        node.run()
    except KeyboardInterrupt:
        logger.info("RC control node interrupted (KeyboardInterrupt)")
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
