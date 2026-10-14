"""Mock robot node for grain sampling robot software.

Provides a simulated ROS node that publishes synthetic sensor data
for development and testing without physical hardware.

Entry point (via setup.py)::
    mock-robot=mock_robot.main:main
"""

from __future__ import annotations

import json
import math

import rospy

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance, TransformStamped
from geometry_msgs.msg import Quaternion, Point, Vector3
from tf2_msgs.msg import TFMessage


# Simulated motion parameters
CIRCLE_RADIUS = 3.0           # metres
ANGULAR_SPEED = 0.1           # rad/s
ODOMETRY_RATE = 0.1           # seconds (10 Hz)
MECHANISM_STATUS_RATE = 1.0   # seconds (1 Hz)

# Map parameters
MAP_WIDTH = 200               # cells
MAP_HEIGHT = 200              # cells
MAP_RESOLUTION = 0.05         # m/cell
MAP_ORIGIN_X = -5.0           # metres
MAP_ORIGIN_Y = -5.0           # metres


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    """Convert a yaw angle to a geometry_msgs/Quaternion."""
    half_yaw = yaw / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


def _make_header(frame_id: str, stamp: rospy.Time | None = None) -> Header:
    """Create a standard ROS Header for the given frame_id."""
    h = Header()
    h.frame_id = frame_id
    if stamp is not None:
        h.stamp = stamp
    return h


class MockRobot:
    """Simulated robot node for development and testing.

    Publishes synthetic sensor data on key ROS topics so that
    the grain sampling UI and workflow layers can be developed
    without a physical robot.
    """

    def __init__(self):
        try:
            rospy.init_node("mock_robot", anonymous=True, disable_signals=True)
        except rospy.exceptions.ROSException:
            pass  # already initialized by test fixture

        # ── Simulated state ──────────────────────────────────────────
        self._angle = 0.0
        self._prev_angle = 0.0

        # ── Publisher creation ───────────────────────────────────────
        # Odometry
        self._odom_pub = rospy.Publisher(
            "/odometry/filtered", Odometry, queue_size=10
        )

        # Map (latched so late joiners get the map)
        self._map_pub = rospy.Publisher(
            "/map", OccupancyGrid, queue_size=10, latch=True
        )

        # TF
        self._tf_pub = rospy.Publisher(
            "/tf", TFMessage, queue_size=100
        )

        # Mechanism status (latched)
        self._mech_pub = rospy.Publisher(
            "/mechanism/status", String, queue_size=10, latch=True
        )

        # ── Timers ────────────────────────────────────────────────────
        self._odom_timer = rospy.Timer(
            rospy.Duration(ODOMETRY_RATE), self._publish_odometry
        )
        self._mech_timer = rospy.Timer(
            rospy.Duration(MECHANISM_STATUS_RATE), self._publish_mechanism_status
        )

        # Publish the static map once on startup
        self._publish_map()

        # ── Last-message cache (for testing) ──────────────────────────
        self._last_odom: Odometry | None = None
        self._last_mechanism_status: String | None = None

        rospy.loginfo("MockRobot started — publishing synthetic data")

    # ── Odometry ──────────────────────────────────────────────────────

    def _publish_odometry(self, _event=None) -> None:
        """Compute and publish a synthetic odometry message.

        The robot follows a circular trajectory of radius CIRCLE_RADIUS
        at a constant angular speed.
        """
        now = rospy.Time.now()

        # Advance the simulation
        self._prev_angle = self._angle
        self._angle += ANGULAR_SPEED * ODOMETRY_RATE

        # Position on the circle
        x = CIRCLE_RADIUS * math.cos(self._angle)
        y = CIRCLE_RADIUS * math.sin(self._angle)

        # Orientation: direction of motion (tangent to circle)
        yaw = self._angle + math.pi / 2.0

        # Linear velocity (tangent direction * tangential speed)
        linear_speed = CIRCLE_RADIUS * ANGULAR_SPEED
        vx = -linear_speed * math.sin(self._angle)
        vy = linear_speed * math.cos(self._angle)

        # Build the Odometry message
        odom = Odometry()
        odom.header = _make_header("odom", now)
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(x=x, y=y, z=0.0)
        odom.pose.pose.orientation = _quaternion_from_yaw(yaw)

        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=ANGULAR_SPEED)

        self._odom_pub.publish(odom)
        self._last_odom = odom

        # Also publish the odom → base_link transform
        self._publish_tf(now, x, y, yaw)

    # ── TF ────────────────────────────────────────────────────────────

    def _publish_tf(self, stamp: rospy.Time, x: float, y: float, yaw: float) -> None:
        """Publish odom → base_link transform matching the odometry."""
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header = _make_header("odom", stamp)
        transform.child_frame_id = "base_link"
        transform.transform.translation = Vector3(x=x, y=y, z=0.0)
        transform.transform.rotation = _quaternion_from_yaw(yaw)
        tf_msg.transforms = [transform]
        self._tf_pub.publish(tf_msg)

    # ── Map ───────────────────────────────────────────────────────────

    def _publish_map(self) -> None:
        """Publish a static dummy occupancy grid.

        A 10 × 10 metre map with 0.05 m resolution, all cells marked
        as free (0). Origin placed at (-5, -5) so the circle trajectory
        is centred in the map.
        """
        grid = OccupancyGrid()
        grid.header = _make_header("map")

        info = MapMetaData()
        info.resolution = MAP_RESOLUTION
        info.width = MAP_WIDTH
        info.height = MAP_HEIGHT
        info.origin = Pose(position=Point(x=MAP_ORIGIN_X, y=MAP_ORIGIN_Y, z=0.0))

        grid.info = info
        grid.data = [0] * (MAP_WIDTH * MAP_HEIGHT)   # all free

        self._map_pub.publish(grid)
        rospy.loginfo(
            "Published dummy map: %d × %d @ %.2f m/cell",
            MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION,
        )

    # ── Mechanism status ──────────────────────────────────────────────

    def _publish_mechanism_status(self, _event=None) -> None:
        """Publish the current mechanism status as a JSON string."""
        status = {
            "state": "idle",
            "depth": 0.0,
            "pressure": 0.0,
            "bin_weights": [0, 0, 0],
        }
        msg = String()
        msg.data = json.dumps(status)
        self._mech_pub.publish(msg)
        self._last_mechanism_status = msg


def main() -> None:
    """Entry point for the mock robot ROS node.

    Initialises rospy, spins the MockRobot, and shuts down cleanly
    on SIGINT (Ctrl+C).
    """
    node = MockRobot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("MockRobot shutting down (KeyboardInterrupt)")


if __name__ == "__main__":
    main()
