"""CameraNode — USB camera ROS node for grain sampling robot.

Publishes ``sensor_msgs/Image`` messages to ``/camera/image_raw`` and
provides ``/camera/start_stream`` / ``/camera/stop_stream`` services to
control publishing at run-time.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

try:
    import rospy
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
except (ImportError, ModuleNotFoundError) as exc:
    raise ImportError(
        "ROS (rospy) is required but not available. "
        "Install rospy from the ROS Noetic distribution."
    ) from exc

try:
    import cv2
except ImportError as exc:
    raise ImportError(
        "opencv-python (cv2) is required but not available. "
        "Install with: pip install opencv-python"
    ) from exc

try:
    from cv_bridge import CvBridge
except ImportError as exc:
    raise ImportError(
        "cv_bridge is required but not available. "
        "Install with: sudo apt install ros-noetic-cv-bridge"
    ) from exc

_logger = logging.getLogger("camera_node")


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

DEFAULT_IMAGE_TOPIC = "/camera/image_raw"
DEFAULT_START_SERVICE = "/camera/start_stream"
DEFAULT_STOP_SERVICE = "/camera/stop_stream"

DEFAULT_RESOLUTION = (1280, 720)
DEFAULT_FPS = 15
DEFAULT_DEVICE = 0

# Encoding used when converting OpenCV BGR frames to ROS Image messages.
DEFAULT_CV_ENCODING = "bgr8"

# How long to wait (seconds) before retrying after a camera disconnect.
RECONNECT_INTERVAL_S = 3.0


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────


class CameraNode:
    """ROS node that captures frames from a USB camera and publishes them.

    Parameters
    ----------
    node_name:
        ROS node name (default ``"camera_node"``).
    device:
        Camera device index (int) or path string like ``"/dev/video0"``.
        Default ``0``.
    resolution:
        Desired capture resolution as ``(width, height)``.
        Default ``(1280, 720)``.
    fps:
        Desired publishing frame-rate.
        Default ``15``.
    image_topic:
        ROS topic name for ``sensor_msgs/Image`` messages.
        Default ``"/camera/image_raw"``.
    start_service:
        ROS service name for ``Trigger`` start-stream service.
        Default ``"/camera/start_stream"``.
    stop_service:
        ROS service name for ``Trigger`` stop-stream service.
        Default ``"/camera/stop_stream"``.
    encoding:
        OpenCV → ROS encoding string (default ``"bgr8"``).
    """

    def __init__(
        self,
        node_name: str = "camera_node",
        device: int | str = DEFAULT_DEVICE,
        resolution: tuple[int, int] = DEFAULT_RESOLUTION,
        fps: float = DEFAULT_FPS,
        image_topic: str = DEFAULT_IMAGE_TOPIC,
        start_service: str = DEFAULT_START_SERVICE,
        stop_service: str = DEFAULT_STOP_SERVICE,
        encoding: str = DEFAULT_CV_ENCODING,
    ) -> None:
        # Note: rospy.init_node() must be called before creating this instance

        # ── Parameters ──────────────────────────────────────────────────
        self._device = device
        self._resolution = resolution
        self._fps = float(fps)
        self._encoding = encoding

        # ── CV bridge ───────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._cap: Optional[cv2.VideoCapture] = None

        # ── State ───────────────────────────────────────────────────────
        self._publishing: bool = False
        self._timer = None

        # ── Publisher ───────────────────────────────────────────────────
        self._publisher = rospy.Publisher(image_topic, Image, queue_size=10)

        # ── Services ────────────────────────────────────────────────────
        self._start_srv = rospy.Service(
            start_service, Trigger, self._start_stream_cb
        )
        self._stop_srv = rospy.Service(
            stop_service, Trigger, self._stop_stream_cb
        )

        rospy.loginfo(
            f"CameraNode initialised: device={device!r}, "
            f"resolution={resolution[0]}x{resolution[1]}, fps={fps}"
        )

    # ── Open / close helpers ────────────────────────────────────────────────

    def _open_camera(self) -> bool:
        """Open the USB camera and apply resolution settings.

        Returns ``True`` on success, ``False`` otherwise.
        """
        if self._cap is not None:
            self._close_camera()

        cap = cv2.VideoCapture(self._device)  # type: ignore[arg-type]
        if not cap.isOpened():
            rospy.logwarn(
                f"Failed to open camera device {self._device!r}"
            )
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._resolution[1])
        cap.set(cv2.CAP_PROP_FPS, self._fps)

        # Give the camera a moment to settle after setting properties.
        time.sleep(0.15)

        self._cap = cap
        rospy.loginfo(f"Camera opened: {self._device!r}")
        return True

    def _close_camera(self) -> None:
        """Release the OpenCV capture handle."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            rospy.loginfo("Camera released")

    # ── Timer callback ───────────────────────────────────────────────────────

    def _publish_frame(self, event=None) -> None:
        """Read one frame from the camera and publish it.

        If the frame cannot be read (camera disconnected), logging a
        warning and attempt to re-open the device.
        """
        if self._cap is None or not self._cap.isOpened():
            rospy.logwarn(
                "Camera not available — attempting reconnect ..."
            )
            if not self._open_camera():
                return  # will retry on the next timer tick

        ret, frame = self._cap.read()  # type: ignore[union-attr]

        if not ret or frame is None:
            rospy.logwarn(
                "Failed to read frame — camera may have disconnected"
            )
            self._close_camera()
            return

        try:
            img_msg = self._bridge.cv2_to_imgmsg(frame, encoding=self._encoding)
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera_link"
            self._publisher.publish(img_msg)
        except Exception as exc:
            rospy.logerr(f"cv_bridge conversion failed: {exc}")

    # ── Service callbacks ────────────────────────────────────────────────────

    def _start_stream_cb(self, request):
        """Handle ``/camera/start_stream`` service call."""
        from std_srvs.srv import TriggerResponse

        if self._publishing:
            return TriggerResponse(success=True, message="Stream is already running")

        if not self._open_camera():
            return TriggerResponse(
                success=False,
                message=f"Failed to open camera device {self._device!r}",
            )

        period = rospy.Duration(1.0 / self._fps)
        self._timer = rospy.Timer(period, self._publish_frame)
        self._publishing = True

        rospy.loginfo("Stream started")
        return TriggerResponse(success=True, message="Stream started")

    def _stop_stream_cb(self, request):
        """Handle ``/camera/stop_stream`` service call."""
        from std_srvs.srv import TriggerResponse

        if not self._publishing:
            return TriggerResponse(success=True, message="Stream is not running")

        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None

        self._close_camera()
        self._publishing = False

        rospy.loginfo("Stream stopped")
        return TriggerResponse(success=True, message="Stream stopped")

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Clean-up resources before node shutdown."""
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        self._close_camera()

    # ── Property accessors (for testing) ─────────────────────────────────────

    @property
    def is_publishing(self) -> bool:
        """Return ``True`` when the periodic frame publisher is active."""
        return self._publishing

    @property
    def cap(self) -> Optional[cv2.VideoCapture]:
        """Expose the internal VideoCapture handle (test-only)."""
        return self._cap


# ──────────────────────────────────────────────────────────────────────────────
# Entry-point
# ──────────────────────────────────────────────────────────────────────────────


def main(args=None) -> None:
    """Spin the camera_node standalone."""
    rospy.init_node("camera_node", anonymous=True)
    node = CameraNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
