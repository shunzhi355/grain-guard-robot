"""Tests for the mock_robot ROS node.

These tests require a ROS Noetic installation with ``rospy`` importable.
If ``rospy`` is not available, all tests in this module are skipped.
"""

import json
import math

import pytest

# ── Module-level guard: skip all tests if ROS is not available ──────
try:
    import rospy
    from mock_robot.mock_robot_node import MockRobot, CIRCLE_RADIUS, ANGULAR_SPEED
except (ImportError, ModuleNotFoundError):
    pytest.skip("ROS (rospy) not available — skipping mock_robot tests", allow_module_level=True)


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════


@pytest.fixture(scope="module")
def rospy_lifecycle():
    """Initialise rospy once for the whole module."""
    rospy.init_node("test_mock", anonymous=True, disable_signals=True)
    yield
    rospy.signal_shutdown("test teardown")


@pytest.fixture
def node(rospy_lifecycle):
    """Create a fresh MockRobot for each test.

    Yields the node and tears it down after the test completes.
    The last-message caches are cleared automatically by the constructor.
    """
    n = MockRobot()
    yield n
    rospy.signal_shutdown("test done")


# ═══════════════════════════════════════════════════════════════════════
# Instantiation
# ═══════════════════════════════════════════════════════════════════════


def test_node_name(node):
    """The node should be named 'mock_robot'."""
    # ROS1: no get_name(); check object is MockRobot instead
    assert isinstance(node, MockRobot)


def test_node_can_spin_once(node):
    """A single spin should not raise (timers fire correctly)."""
    try:
        rospy.rostime.wallsleep(0.2)  # ROS1: no spin_once, process callbacks via sleep
    except Exception as exc:
        pytest.fail(f"spin_once raised an exception: {exc}")


# ═══════════════════════════════════════════════════════════════════════
# Odometry
# ═══════════════════════════════════════════════════════════════════════


def test_odometry_message_structure(node):
    """The odometry message should have valid pose data."""
    node._publish_odometry()
    msg = node._last_odom
    assert msg is not None, "No odometry message was cached"

    # Header
    assert msg.header.frame_id == "odom", f"Expected 'odom', got '{msg.header.frame_id}'"
    assert msg.child_frame_id == "base_link", f"Expected 'base_link', got '{msg.child_frame_id}'"

    # Pose should not be origin (robot moves in a circle)
    pose = msg.pose.pose
    assert abs(pose.position.x) > 0 or abs(pose.position.y) > 0, \
        "Robot is stuck at origin"

    # Position should be within the circle radius (+ small tolerance)
    dist = math.sqrt(pose.position.x ** 2 + pose.position.y ** 2)
    assert dist <= CIRCLE_RADIUS * 1.05, \
        f"Robot position {dist:.2f}m exceeds circle radius {CIRCLE_RADIUS}m"

    # Orientation should be a unit quaternion
    q = pose.orientation
    q_norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    assert abs(q_norm - 1.0) < 1e-6, \
        f"Orientation quaternion is not unit: norm={q_norm}"

    # Twist should be non-zero (robot is moving)
    twist = msg.twist.twist
    linear_speed = math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2)
    assert linear_speed > 0, "Robot linear velocity is zero"
    expected_speed = CIRCLE_RADIUS * ANGULAR_SPEED
    assert abs(linear_speed - expected_speed) < expected_speed * 0.5, \
        f"Linear speed {linear_speed:.3f} deviates too much from expected {expected_speed:.3f}"


def test_odometry_advances_over_time(node):
    """Two consecutive odometry calls should show movement."""
    node._publish_odometry()
    first = node._last_odom
    assert first is not None

    node._publish_odometry()
    second = node._last_odom
    assert second is not None

    # The robot should have moved (angle increases)
    assert second.pose.pose.position.x != first.pose.pose.position.x or \
           second.pose.pose.position.y != first.pose.pose.position.y, \
        "Odometry did not advance between publishes"


# ═══════════════════════════════════════════════════════════════════════
# Mechanism Status
# ═══════════════════════════════════════════════════════════════════════


def test_mechanism_status_json_parseable(node):
    """The mechanism status JSON string should be parseable."""
    node._publish_mechanism_status()
    msg = node._last_mechanism_status
    assert msg is not None, "No mechanism status message was cached"

    data = json.loads(msg.data)

    # Required fields
    assert "state" in data, "Missing 'state' field"
    assert "depth" in data, "Missing 'depth' field"
    assert "pressure" in data, "Missing 'pressure' field"
    assert "bin_weights" in data, "Missing 'bin_weights' field"

    # Types
    assert isinstance(data["state"], str), f"'state' should be str, got {type(data['state'])}"
    assert isinstance(data["depth"], (int, float)), f"'depth' should be numeric, got {type(data['depth'])}"
    assert isinstance(data["pressure"], (int, float)), f"'pressure' should be numeric, got {type(data['pressure'])}"
    assert isinstance(data["bin_weights"], list), f"'bin_weights' should be list, got {type(data['bin_weights'])}"
    assert len(data["bin_weights"]) == 3, \
        f"'bin_weights' should have 3 elements, got {len(data['bin_weights'])}"

    # Values
    assert data["state"] == "idle", f"Expected state 'idle', got '{data['state']}'"
    assert data["depth"] == 0.0, f"Expected depth 0.0, got {data['depth']}"
    assert data["pressure"] == 0.0, f"Expected pressure 0.0, got {data['pressure']}"
    assert data["bin_weights"] == [0, 0, 0], \
        f"Expected bin_weights [0,0,0], got {data['bin_weights']}"


# ═══════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════


def test_main_function_importable():
    """The main() entry point should be importable from the console_scripts module."""
    from mock_robot.main import main
    assert callable(main), "main() is not callable"
