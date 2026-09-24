"""ROS bridge interface for the grain sampling workflow.

Dual-layer architecture:
  - Navigation (chassis team): Topic-based — publishes PoseStamped to
    ``/move_base_simple/goal``, waits for ``/waypoint_task_done``.
  - Mechanism (structure team): Service-based — ``std_srvs/Trigger`` calls
    (to be confirmed when the mechanism team's code is on the board).

In stub mode (no ``rospy``), all calls return ``True`` immediately.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

# ------------------------------------------------------------------
# ROS imports (optional)
# ------------------------------------------------------------------


class _SetGrainRequest:
    """``mechanism_node/SetGrain`` 请求（回退实现：单个字符串字段 grain）。

    与 mechanism_node.py 的回退类保持一致。
    """

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


try:
    import rospy  # type: ignore[import-untyped]
    from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist  # type: ignore[import-untyped]
    from nav_msgs.msg import Odometry  # type: ignore[import-untyped]
    from std_msgs.msg import Empty, String  # type: ignore[import-untyped]
    from std_srvs.srv import Trigger  # type: ignore[import-untyped]

    # Prefer catkin's generated Python classes.  The second argument of
    # roslib.message.get_service_class is ``reload_on_error`` (bool), not srv
    # source text; passing the definition there used to silently return None
    # when the UI was launched without a sourced mechanism workspace.
    try:
        from mechanism_node.srv import MoveLift as _GeneratedMoveLift
        from mechanism_node.srv import SetGrain as _GeneratedSetGrain

        SetGrain = _GeneratedSetGrain
        MoveLift = _GeneratedMoveLift
    except Exception:  # noqa: BLE001 - 退化为模块内回退类
        try:
            from roslib.message import get_service_class

            SetGrain = get_service_class("mechanism_node/SetGrain")
            MoveLift = get_service_class("mechanism_node/MoveLift")
        except Exception:  # noqa: BLE001
            SetGrain = None
            MoveLift = None
        if SetGrain is None or MoveLift is None:
            logger.warning(
                "mechanism_node generated service classes unavailable — "
                "SetGrain/MoveLift calls cannot use ROS wire serialization"
            )
            SetGrain = _FallbackSetGrain
            MoveLift = _FallbackMoveLift
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    logger.info("rospy not available — SamplingBridge will use stub mode")
    SetGrain = _FallbackSetGrain
    MoveLift = _FallbackMoveLift


# ------------------------------------------------------------------
# Bridge
# ------------------------------------------------------------------


class SamplingBridge:
    """Unified ROS bridge for chassis and mechanism teams.

    Provides a blocking-style API for the UI and state machine.
    Internally uses Topics for chassis navigation and Services for
    mechanism operations.
    """

    NAV_TIMEOUT_SEC: float = 120.0    # max wait for navigation completion
    SERVICE_TIMEOUT_SEC: float = 5.0  # max wait for a Trigger service

    def __init__(self, node_name: str = "sampling_bridge") -> None:
        self._node_name: str = node_name
        # Exact failure returned by the most recent service call.  The
        # orchestrator uses it as the UI stop reason instead of discarding the
        # actionable hardware message.
        self.last_error: str = ""

        # Navigation state (updated by topic callback)
        self._nav_completed: bool = False
        self._nav_success: bool = False

        # Odometry state (updated by /Odometry subscriber)
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_lock = threading.Lock()
        self._has_odom: bool = False

        if HAS_ROS:
            try:
                rospy.init_node(node_name, anonymous=True, disable_signals=True)
            except rospy.ROSException:
                logger.debug("rospy.init_node already called — reusing existing node")

            # Publishers (chassis topics)
            self._goal_pub = rospy.Publisher(
                "/move_base_simple/goal", PoseStamped, queue_size=10
            )
            self._cancel_pub = rospy.Publisher(
                "/cancel_goal", Empty, queue_size=10
            )
            self._cmd_vel_pub = rospy.Publisher(
                "/cmd_vel", Twist, queue_size=10
            )

            # Subscribers (chassis topics)
            rospy.Subscriber(
                "/waypoint_task_done", String, self._on_nav_done
            )
            # Also monitor goal_controller status for arrival detection
            rospy.Subscriber(
                "/goal_controller/status", String, self._on_status
            )
            # Odometry for position tracking
            rospy.Subscriber(
                "/Odometry", Odometry, self._on_odom
            )

            # Let subscribers register with ROS master
            rospy.sleep(0.5)

            # Start a spinner thread so callbacks fire in any thread
            self._spinner_running = True
            self._spinner_thread = threading.Thread(
                target=self._spinner_loop, daemon=True
            )
            self._spinner_thread.start()

    # ----------------------------------------------------------------
    # Public API: Navigation (chassis team — Topic-based)
    # ----------------------------------------------------------------

    def call_navigate(self, x: float, y: float) -> bool:
        """Navigate to *(x, y)*.

        Publishes a ``PoseStamped`` to ``/move_base_simple/goal`` and
        waits for ``/waypoint_task_done`` (or timeout).

        Returns ``True`` on success, ``False`` on timeout or failure.
        """
        if not HAS_ROS:
            logger.debug("[Stub] Navigate to (%.2f, %.2f) -> True", x, y)
            return True

        # Reset completion flags
        self._nav_completed = False
        self._nav_success = False

        # Build and publish goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "camera_init"
        goal.pose.position = Point(x, y, 0.0)
        goal.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self._goal_pub.publish(goal)
        logger.info("Navigation goal published: (%.2f, %.2f)", x, y)

        # Wait for completion signal or timeout
        deadline = time.monotonic() + self.NAV_TIMEOUT_SEC
        rate = rospy.Rate(10)  # 10 Hz spin for callback processing
        while not self._nav_completed and time.monotonic() < deadline:
            rate.sleep()

        if self._nav_completed:
            logger.info(
                "Navigation to (%.2f, %.2f) %s",
                x, y,
                "succeeded" if self._nav_success else "reported failure",
            )
            # Arrival precision check (≤20 cm)
            pos = self._get_current_position()
            if pos is not None:
                dist = ((pos[0] - x) ** 2 + (pos[1] - y) ** 2) ** 0.5
                if dist > 0.2:
                    logger.warning(
                        "Arrival precision: %.2f m from goal (threshold 0.2 m)",
                        dist,
                    )
            return self._nav_success
        else:
            logger.warning(
                "Navigation to (%.2f, %.2f) timed out (%.0f s)",
                x, y, self.NAV_TIMEOUT_SEC,
            )
            return False

    def record_start_position(self) -> Optional[tuple[float, float]]:
        """Record the current position as the task start point.

        Reads the latest ``/Odometry`` and returns the position.
        Returns ``None`` if odometry data is not yet available.
        """
        pos = self._get_current_position()
        if pos is not None:
            logger.info("Start position recorded: (%.2f, %.2f)", pos[0], pos[1])
        else:
            logger.info("Start position recorded (odometry not available)")
        return pos

    # ----------------------------------------------------------------
    # Public API: Mechanism (structure team — Service-based)
    # ----------------------------------------------------------------

    def call_start_suction(self) -> bool:
        """Start suction / negative-pressure fan.

        Calls ``/mechanism/start_suction`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/start_suction")

    def call_stop_suction(self) -> bool:
        """Stop the suction fan.

        Calls ``/mechanism/stop_suction`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/stop_suction")

    def call_convey(self) -> bool:
        """Start the screw conveyors (augers 1 & 2).

        Calls ``/mechanism/convey`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/convey")

    def call_open_bin(self, depth_level: int) -> bool:
        """Open the storage bin for the given *depth_level*.

        Calls ``/mechanism/open_bin/{depth}`` (``Trigger``), where
        ``depth`` is the mapped depth name: 0→shallow, 1→mid, 2→deep
        (mechanism_node registers these per-depth variants).
        """
        depth_name = {0: "shallow", 1: "mid", 2: "deep"}.get(int(depth_level), "mid")
        return self._call_trigger(f"/mechanism/open_bin/{depth_name}")

    def call_clamp(self) -> bool:
        """Clamp the sampling mechanism.

        Calls ``/mechanism/clamp`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/clamp")

    def call_lift_health(self) -> bool:
        """Verify X2P communication by reading the encoder without motion."""
        return self._call_trigger("/mechanism/lift_health")

    def call_unclamp(self) -> bool:
        """Release the clamp of the sampling mechanism.

        Calls ``/mechanism/unclamp`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/unclamp")

    def call_tighten(self) -> bool:
        """Tighten the sampling mechanism.

        Calls ``/mechanism/tighten`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/tighten")

    def call_untighten(self) -> bool:
        """Loosen / untighten the sampling mechanism.

        Calls ``/mechanism/untighten`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/untighten")

    def call_press(self) -> bool:
        """Press the sampling mechanism down.

        Calls ``/mechanism/press`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/press")

    def call_lift(self) -> bool:
        """Lift the sampling mechanism up.

        Calls ``/mechanism/lift`` (``Trigger``).
        """
        return self._call_trigger("/mechanism/lift")

    def call_move_lift(self, direction: str, distance_cm: float) -> bool:
        """Move the X2P servo lift by an exact distance (encoder closed-loop).

        Calls ``/mechanism/move_lift`` (``mechanism_node/MoveLift``, request
        carries ``direction`` ``up``/``down``/``down_cycle``/``return`` and
        ``distance_cm``). ``down_cycle`` captures the absolute encoder origin;
        its paired ``return`` stops below that origin by a safety clearance.
        This is
        the precise-distance control (encoding feedback, tolerance ~5mm),
        unlike :meth:`call_press`/``call_lift`` which are short speed-mode
        jogs.  Stub mode (no ROS) returns True.
        """
        if not HAS_ROS:
            self.last_error = ""
            logger.debug("[Stub] move_lift(%s, %.1fcm) -> True", direction, distance_cm)
            return True

        service_name = "/mechanism/move_lift"
        started = time.monotonic()
        logger.info(
            "SERVICE_CALL_BEGIN service=%s direction=%s distance_cm=%.3f node=%s "
            "master=%s thread=%s",
            service_name, direction, distance_cm, self._ros_node_name(),
            os.environ.get("ROS_MASTER_URI", "<unset>"), threading.current_thread().name,
        )
        try:
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT_SEC)
            logger.info("SERVICE_WAIT_OK service=%s elapsed=%.3fs", service_name, time.monotonic() - started)
        except Exception:
            self.last_error = f"{service_name}: service unavailable"
            logger.exception(
                "SERVICE_WAIT_FAILED service=%s timeout=%.1fs elapsed=%.3fs",
                service_name, self.SERVICE_TIMEOUT_SEC, time.monotonic() - started,
            )
            return False

        try:
            proxy = rospy.ServiceProxy(service_name, MoveLift)
            response = proxy(direction=direction, distance_cm=float(distance_cm))
        except Exception:
            self.last_error = f"{service_name}: service call exception"
            logger.exception(
                "SERVICE_CALL_EXCEPTION service=%s elapsed=%.3fs",
                service_name, time.monotonic() - started,
            )
            return False

        logger.info(
            "SERVICE_CALL_RESPONSE service=%s success=%s message=%r elapsed=%.3fs",
            service_name, response.success, response.message, time.monotonic() - started,
        )

        if not response.success:
            self.last_error = f"{service_name}: {response.message}"
            logger.warning(
                "Service %s returned failure: %s",
                service_name, response.message,
            )
            return False

        self.last_error = ""
        logger.info(
            "Service %s succeeded: %s", service_name, response.message
        )
        return True

    def call_close_bin(self, depth_level: int) -> bool:
        """Close the storage bin for the given *depth_level*.

        Calls ``/mechanism/close_bin/{depth}`` (``Trigger``), where ``depth``
        is the mapped depth name: 0→shallow, 1→mid, 2→deep (mirrors
        ``call_open_bin``).
        """
        depth_name = {0: "shallow", 1: "mid", 2: "deep"}.get(int(depth_level), "mid")
        return self._call_trigger(f"/mechanism/close_bin/{depth_name}")

    def call_set_grain(self, grain: str) -> bool:
        """Set the current grain variety for the mechanism.

        Calls ``/mechanism/set_grain`` (``mechanism_node/SetGrain``, request
        carries a single ``string grain`` field); stub mode (no ROS) always
        returns True for backward compatibility.
        """
        if not HAS_ROS:
            logger.debug("[Stub] set_grain(%s) -> True", grain)
            return True

        service_name = "/mechanism/set_grain"
        started = time.monotonic()
        logger.info(
            "SERVICE_CALL_BEGIN service=%s grain=%r node=%s master=%s thread=%s",
            service_name, grain, self._ros_node_name(),
            os.environ.get("ROS_MASTER_URI", "<unset>"), threading.current_thread().name,
        )
        try:
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT_SEC)
            logger.info("SERVICE_WAIT_OK service=%s elapsed=%.3fs", service_name, time.monotonic() - started)
        except Exception:
            logger.exception(
                "SERVICE_WAIT_FAILED service=%s timeout=%.1fs elapsed=%.3fs",
                service_name, self.SERVICE_TIMEOUT_SEC, time.monotonic() - started,
            )
            return False

        try:
            proxy = rospy.ServiceProxy(service_name, SetGrain)
            response = proxy(grain=grain)
        except Exception:
            logger.exception(
                "SERVICE_CALL_EXCEPTION service=%s elapsed=%.3fs",
                service_name, time.monotonic() - started,
            )
            return False

        logger.info(
            "SERVICE_CALL_RESPONSE service=%s success=%s message=%r elapsed=%.3fs",
            service_name, response.success, response.message, time.monotonic() - started,
        )

        if not response.success:
            logger.warning(
                "Service %s returned failure: %s",
                service_name, response.message,
            )
            return False

        logger.info("Service %s succeeded: %s", service_name, response.message)
        return True

    # ----------------------------------------------------------------
    # Public API: Emergency / Safety
    # ----------------------------------------------------------------

    def call_emergency_stop(self) -> bool:
        """Immediately halt all operations.

        Chassis: publishes ``Twist()`` (zero) to ``/cmd_vel`` and
        ``Empty`` to ``/cancel_goal``.
        Mechanism: also attempts ``/mechanism/emergency_stop`` if available.
        """
        if HAS_ROS:
            # Stop chassis motion immediately
            self._cmd_vel_pub.publish(Twist())
            self._cancel_pub.publish(Empty())
            logger.info("Emergency stop: chassis halted (cmd_vel=0, cancel_goal)")

        # Also try mechanism emergency stop
        mech_stopped = self._call_trigger("/mechanism/emergency_stop")
        return HAS_ROS or mech_stopped  # stub mode returns True

    def cancel_goal(self) -> bool:
        """Cancel the current navigation goal (chassis only).

        Publishes ``Empty`` to ``/cancel_goal`` so ``move_base`` aborts the
        active goal.  Unlike :meth:`call_emergency_stop` it never touches
        the mechanism or sampling machinery.  Stub mode (no ROS) always
        returns ``True``.
        """
        if HAS_ROS:
            self._cancel_pub.publish(Empty())
            logger.info("Navigation goal cancelled (/cancel_goal)")
        else:
            logger.debug("[Stub] cancel_goal -> True")
        return True

    def publish_cmd_vel(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> bool:
        """Publish a chassis velocity command to ``/cmd_vel``.

        Values are forwarded as-is; the caller is responsible for staying
        within the bridge limits (``cmd_vel_to_motor`` uses
        ``max_linear_mps=0.5`` and ``max_angular_rps=1.0``).
        """
        if HAS_ROS:
            twist = Twist()
            twist.linear.x = float(linear_mps)
            twist.angular.z = float(angular_rps)
            self._cmd_vel_pub.publish(twist)
            logger.info(
                "cmd_vel published: linear=%.3f m/s, angular=%.3f rad/s",
                twist.linear.x, twist.angular.z,
            )
            return True
        logger.debug(
            "[Stub] publish_cmd_vel(linear=%.3f, angular=%.3f)",
            linear_mps, angular_rps,
        )
        return True  # stub mode pretends success (matches bridge convention)

    # ----------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------

    def _on_nav_done(self, msg: String) -> None:
        """Callback for ``/waypoint_task_done``."""
        self._nav_completed = True
        self._nav_success = (msg.data == "arrive")
        logger.info("Received navigation done: data=%s", msg.data)

    # ----------------------------------------------------------------
    # Internal helpers
    # ----------------------------------------------------------------

    def _on_status(self, msg):
        if msg.data.startswith("ARRIVED"):
            self._nav_completed = True
            self._nav_success = True
            logger.info("Navigation arrived: %s", msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        """Callback for ``/Odometry`` — track current position."""
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            self._has_odom = True

    def _get_current_position(self) -> Optional[tuple[float, float]]:
        """Return the latest odometry position, or None if unavailable."""
        with self._odom_lock:
            if not self._has_odom:
                return None
            return (self._odom_x, self._odom_y)

    def _call_trigger(self, service_name: str) -> bool:
        """Call a ``std_srvs/Trigger`` service, return success."""
        if not HAS_ROS:
            self.last_error = ""
            logger.debug("[Stub] Trigger %s -> True", service_name)
            return True

        started = time.monotonic()
        logger.info(
            "SERVICE_CALL_BEGIN service=%s node=%s master=%s thread=%s",
            service_name, self._ros_node_name(),
            os.environ.get("ROS_MASTER_URI", "<unset>"), threading.current_thread().name,
        )
        try:
            rospy.wait_for_service(service_name, timeout=self.SERVICE_TIMEOUT_SEC)
            logger.info("SERVICE_WAIT_OK service=%s elapsed=%.3fs", service_name, time.monotonic() - started)
        except Exception:
            self.last_error = f"{service_name}: service unavailable"
            logger.exception(
                "SERVICE_WAIT_FAILED service=%s timeout=%.1fs elapsed=%.3fs",
                service_name, self.SERVICE_TIMEOUT_SEC, time.monotonic() - started,
            )
            return False

        try:
            proxy = rospy.ServiceProxy(service_name, Trigger)
            response = proxy()
        except Exception:
            self.last_error = f"{service_name}: service call exception"
            logger.exception(
                "SERVICE_CALL_EXCEPTION service=%s elapsed=%.3fs",
                service_name, time.monotonic() - started,
            )
            return False

        logger.info(
            "SERVICE_CALL_RESPONSE service=%s success=%s message=%r elapsed=%.3fs",
            service_name, response.success, response.message, time.monotonic() - started,
        )

        if not response.success:
            self.last_error = f"{service_name}: {response.message}"
            logger.warning(
                "Service %s returned failure: %s",
                service_name, response.message,
            )
            return False

        self.last_error = ""
        logger.info("Service %s succeeded", service_name)
        return True

    @staticmethod
    def _ros_node_name() -> str:
        """Return the current rospy node name without masking call errors."""
        try:
            return str(rospy.get_name())
        except Exception:
            return "<unavailable>"

    # ----------------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------------

    def shutdown(self) -> None:
        """Clean up the ROS node."""
        self._spinner_running = False
        if HAS_ROS:
            try:
                rospy.signal_shutdown("SamplingBridge shutdown")
            except Exception:
                pass

    def _spinner_loop(self) -> None:
        """Background loop that processes ROS callbacks."""
        while self._spinner_running and not rospy.is_shutdown():
            rospy.rostime.wallsleep(0.05)
