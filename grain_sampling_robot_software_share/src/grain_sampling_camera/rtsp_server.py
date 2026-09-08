"""rtsp_server.py — RTSP streaming server for grain sampling robot camera.

Provides ``RTSPServer`` — a class that manages H264 RTSP streaming from a
V4L2 camera device using gstreamer, with an MJPEG HTTP fallback when
gstreamer is not available.
"""

from __future__ import annotations

import logging
import os
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Optional

_logger = logging.getLogger("rtsp_server")


# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

_GST_LAUNCH = "gst-launch-1.0"

DEFAULT_WIDTH: int = 1280
DEFAULT_HEIGHT: int = 720
DEFAULT_FRAMERATE: int = 15
DEFAULT_BITRATE: int = 2000  # kbps
DEFAULT_RTP_PORT: int = 5000
DEFAULT_RTCP_PORT: int = 5001

# Seconds to wait for subprocess initialisation before checking its status.
_STARTUP_GRACE_S: float = 0.5


# ──────────────────────────────────────────────────────────────────────────────
# RTSPServer
# ──────────────────────────────────────────────────────────────────────────────


class RTSPServer:
    """Manages an RTSP H264 stream from a V4L2 camera device.

    Uses ``gst-launch-1.0`` subprocess when gstreamer is available.
    Falls back to a simple MJPEG HTTP streaming server when gstreamer
    is not installed or when the gstreamer pipeline fails at start-up.

    Parameters
    ----------
    device:
        V4L2 device path (default ``"/dev/video0"``).
    port:
        RTSP / HTTP server port (default ``8554``).
    mount_point:
        Stream mount-point path (default ``"/camera"``).
    host:
        Hostname used when constructing the ``stream_url`` and
        ``http_url`` properties (default ``"localhost"``).
    width:
        Capture width in pixels (default ``1280``).
    height:
        Capture height in pixels (default ``720``).
    framerate:
        Frames per second (default ``15``).
    bitrate:
        H264 encoder bitrate in kbps when using gstreamer (default
        ``2000``).  Ignored for the MJPEG fallback.
    """

    def __init__(
        self,
        device: str = "/dev/video0",
        port: int = 8554,
        mount_point: str = "/camera",
        host: str = "localhost",
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        framerate: int = DEFAULT_FRAMERATE,
        bitrate: int = DEFAULT_BITRATE,
    ) -> None:
        self._device = device
        self._port = port
        self._mount_point = mount_point
        self._host = host
        self._width = width
        self._height = height
        self._framerate = framerate
        self._bitrate = bitrate

        self._process: Optional[subprocess.Popen[str]] = None
        self._http_server: Optional[HTTPServer] = None
        self._server_thread: Optional[threading.Thread] = None
        self._running: bool = False
        self._use_fallback: bool = not self._gstreamer_available()

        _logger.info(
            "RTSPServer initialised: device=%r, port=%d, mount_point=%r, "
            "mode=%s",
            device,
            port,
            mount_point,
            "MJPEG-fallback" if self._use_fallback else "gstreamer",
        )

    # ── URL properties ────────────────────────────────────────────────────────

    @property
    def stream_url(self) -> str:
        """Return ``rtsp://{host}:{port}{mount_point}``."""
        return f"rtsp://{self._host}:{self._port}{self._mount_point}"

    @property
    def http_url(self) -> str:
        """Return ``http://{host}:{port}{mount_point}`` (MJPEG fallback URL)."""
        return f"http://{self._host}:{self._port}{self._mount_point}"

    # ── GStreamer detection ───────────────────────────────────────────────────

    @staticmethod
    def _gstreamer_available() -> bool:
        """Return ``True`` when ``gst-launch-1.0`` is on ``PATH``."""
        return shutil.which(_GST_LAUNCH) is not None

    @property
    def is_gstreamer_mode(self) -> bool:
        """``True`` when the server will use gstreamer, ``False`` for MJPEG."""
        return not self._use_fallback

    # ── Start ─────────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Start the streaming server (gstreamer or MJPEG fallback).

        Returns
        -------
        bool
            ``True`` on success, ``False`` on failure.
        """
        if self._running:
            _logger.warning("Server is already running")
            return True

        if self._use_fallback:
            return self._start_mjpeg()
        else:
            return self._start_gstreamer()

    def _start_gstreamer(self) -> bool:
        """Launch the gstreamer pipeline as a subprocess.

        The pipeline captures from a V4L2 source, encodes to H264, and
        streams RTP via UDP.  An SDP file is written to ``/tmp`` so that
        RTSP clients can discover the stream parameters.

        If the process exits immediately, the method automatically falls
        back to the MJPEG HTTP server.
        """
        pipeline = (
            f"v4l2src device={self._device} ! "
            f"video/x-raw,width={self._width},height={self._height},"
            f"framerate={self._framerate}/1 ! "
            "videoconvert ! "
            f"x264enc tune=zerolatency bitrate={self._bitrate} "
            "speed-preset=ultrafast ! "
            "h264parse ! rtph264pay name=pay0 pt=96 ! "
            f"multiudpsink "
            f"clients=127.0.0.1:{DEFAULT_RTP_PORT},"
            f"127.0.0.1:{DEFAULT_RTCP_PORT} sync=false"
        )

        _logger.info("Launching gstreamer pipeline: %s", pipeline)

        try:
            self._process = subprocess.Popen(
                [_GST_LAUNCH, pipeline],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid if os.name == "posix" else None,
            )
        except (FileNotFoundError, OSError) as exc:
            _logger.error("Failed to launch gstreamer subprocess: %s", exc)
            self._use_fallback = True
            return self._start_mjpeg()

        # Allow the pipeline to initialise before checking its health.
        time.sleep(_STARTUP_GRACE_S)

        rc = self._process.poll()
        if rc is not None:
            _logger.error(
                "GStreamer process exited immediately (returncode=%d)", rc
            )
            self._process.communicate()  # reap the dead child
            self._process = None
            self._use_fallback = True
            _logger.warning("Falling back to MJPEG HTTP server")
            return self._start_mjpeg()

        # Write SDP for client discovery.
        self._write_sdp_file()

        self._running = True
        _logger.info("RTSP stream started via gstreamer")
        return True

    def _start_mjpeg(self) -> bool:
        """Start the MJPEG HTTP fallback server.

        Opens the camera via OpenCV and serves JPEG frames as a
        ``multipart/x-mixed-replace`` stream through Python's built-in
        ``http.server``.
        """
        try:
            import cv2  # noqa: PLC0415  —  local import avoids hard dep
        except ImportError as exc:
            _logger.error(
                "opencv-python (cv2) is required for MJPEG fallback: %s", exc
            )
            return False

        cap = cv2.VideoCapture(self._device)  # type: ignore[call-overload]
        if not cap.isOpened():
            _logger.error(
                "Failed to open camera device %r for MJPEG fallback",
                self._device,
            )
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._framerate)

        capture = cap
        server_port = self._port

        # ── Inner handler class ──────────────────────────────────────────
        class _MJPEGHandler(BaseHTTPRequestHandler):
            """Serve a multipart MJPEG stream from the captured OpenCV frames."""

            # pylint: disable=invalid-name
            def do_GET(self) -> None:  # noqa: N802
                if self.path in (self._mount_ok, "/"):
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=frame",
                    )
                    self.end_headers()
                    try:
                        while True:
                            ret, frame = capture.read()
                            if not ret:
                                break
                            _, jpeg = cv2.imencode(".jpg", frame)
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(
                                b"Content-Type: image/jpeg\r\n\r\n"
                            )
                            self.wfile.write(jpeg.tobytes())
                            self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(  # noqa: N802
                self, fmt: str, *args: object
            ) -> None:
                """Suppress access-log noise from the HTTP server."""
                pass  # pragma: no cover

        # Inject the mount point so the inner class can reference it.
        _MJPEGHandler._mount_ok = self._mount_point  # type: ignore[attr-defined]

        try:
            self._http_server = HTTPServer(
                ("0.0.0.0", server_port), _MJPEGHandler
            )
        except OSError as exc:
            _logger.error(
                "Cannot bind MJPEG HTTP server on port %d: %s",
                server_port,
                exc,
            )
            capture.release()
            return False

        self._server_thread = threading.Thread(
            target=self._http_server.serve_forever, daemon=True
        )
        self._server_thread.start()
        self._running = True
        _logger.info(
            "MJPEG HTTP fallback stream started on port %d", server_port
        )
        return True

    # ── Stop ──────────────────────────────────────────────────────────────────

    def stop(self) -> None:
        """Stop the streaming server and release all resources."""
        if not self._running:
            return

        _logger.info("Stopping streaming server ...")

        self._kill_gstreamer()
        self._stop_mjpeg()
        self._remove_sdp_file()

        self._running = False
        _logger.info("Streaming server stopped")

    def _kill_gstreamer(self) -> None:
        """Terminate the gstreamer subprocess gracefully, then forcefully."""
        if self._process is None:
            return

        proc = self._process
        try:
            if os.name == "posix":
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                proc.kill()
                proc.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        finally:
            self._process = None

    def _stop_mjpeg(self) -> None:
        """Shut down the MJPEG HTTP server and join its thread."""
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None

        if self._server_thread is not None:
            self._server_thread.join(timeout=3)
            self._server_thread = None

    # ── Status ────────────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        """Return ``True`` when the streaming server is considered active."""
        if not self._running:
            return False

        # GStreamer health-check.
        if self._process is not None:
            if self._process.poll() is not None:
                _logger.warning(
                    "GStreamer process died unexpectedly (returncode=%s)",
                    self._process.returncode,
                )
                self._running = False
                return False

        # MJPEG thread health-check (best-effort).
        if (
            self._http_server is not None
            and self._server_thread is not None
            and not self._server_thread.is_alive()
        ):
            _logger.warning("MJPEG server thread died unexpectedly")
            self._running = False
            return False

        return True

    # ── SDP helpers ───────────────────────────────────────────────────────────

    def _sdp_path(self) -> Path:
        """Return the filesystem path of the SDP descriptor file."""
        return Path(f"/tmp/rtsp_server_{self._port}.sdp")

    def _write_sdp_file(self) -> None:
        """Write an SDP file describing the RTP stream parameters."""
        sdp = (
            "v=0\r\n"
            "o=- 0 0 IN IP4 127.0.0.1\r\n"
            "s=Grain Sampling Camera Stream\r\n"
            "c=IN IP4 127.0.0.1\r\n"
            "t=0 0\r\n"
            f"m=video {DEFAULT_RTP_PORT} RTP/AVP 96\r\n"
            "a=rtpmap:96 H264/90000\r\n"
            "a=fmtp:96 packetization-mode=1\r\n"
        )
        self._sdp_path().write_text(sdp)
        _logger.debug("SDP file written to %s", self._sdp_path())

    def _remove_sdp_file(self) -> None:
        """Remove the SDP file if it exists."""
        path = self._sdp_path()
        try:
            path.unlink(missing_ok=True)
        except Exception:
            pass

    # ── Destructor ────────────────────────────────────────────────────────────

    def __del__(self) -> None:
        """Ensure resources are released on garbage collection."""
        try:
            self.stop()
        except Exception:
            pass
