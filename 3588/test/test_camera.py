"""test_camera.py — Unit tests for the RTSPServer class."""

from __future__ import annotations

import subprocess
import threading
from unittest import mock

import pytest

from grain_sampling_camera.rtsp_server import RTSPServer


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────


def _make_mock_popen(returncode: int | None = None) -> mock.MagicMock:
    """Build a ``subprocess.Popen`` mock whose ``poll()`` returns *returncode*.

    ``None`` means "process is still running".
    """
    proc = mock.MagicMock(spec=subprocess.Popen)
    proc.returncode = returncode
    proc.poll.return_value = returncode
    return proc


def _patch_gstreamer(which_result: str | None) -> mock._patch:
    """Patch ``shutil.which`` to simulate gstreamer presence/absence."""
    return mock.patch("shutil.which", return_value=which_result)


# ──────────────────────────────────────────────────────────────────────────────
# Instantiation & URL construction
# ──────────────────────────────────────────────────────────────────────────────


class TestInstantiation:
    """Tests for RTSPServer construction and URL properties."""

    def test_defaults(self) -> None:
        server = RTSPServer()
        assert server._device == "/dev/video0"
        assert server._port == 8554
        assert server._mount_point == "/camera"
        assert server._host == "localhost"
        assert server._width == 1280
        assert server._height == 720
        assert server._framerate == 15
        assert server._bitrate == 2000

    def test_custom_construction(self) -> None:
        server = RTSPServer(
            device="/dev/video2",
            port=9999,
            mount_point="/stream",
            host="192.168.1.100",
            width=640,
            height=480,
            framerate=30,
            bitrate=4000,
        )
        assert server._device == "/dev/video2"
        assert server._port == 9999
        assert server._mount_point == "/stream"
        assert server._host == "192.168.1.100"
        assert server._width == 640
        assert server._height == 480
        assert server._framerate == 30
        assert server._bitrate == 4000

    def test_stream_url_default(self) -> None:
        server = RTSPServer()
        assert server.stream_url == "rtsp://localhost:8554/camera"

    def test_stream_url_custom(self) -> None:
        server = RTSPServer(
            host="10.0.0.5", port=1234, mount_point="/video"
        )
        assert server.stream_url == "rtsp://10.0.0.5:1234/video"

    def test_http_url(self) -> None:
        server = RTSPServer(
            host="10.0.0.5", port=8080, mount_point="/mjpeg"
        )
        assert server.http_url == "http://10.0.0.5:8080/mjpeg"


# ──────────────────────────────────────────────────────────────────────────────
# Mode detection (gstreamer vs MJPEG fallback)
# ──────────────────────────────────────────────────────────────────────────────


class TestModeDetection:
    """Cover ``is_gstreamer_mode`` and the internal ``_use_fallback`` flag."""

    @mock.patch("shutil.which", return_value="/usr/bin/gst-launch-1.0")
    def test_gstreamer_detected(self, _mock_which: mock.MagicMock) -> None:
        server = RTSPServer()
        assert server.is_gstreamer_mode is True
        assert server._use_fallback is False

    @mock.patch("shutil.which", return_value=None)
    def test_gstreamer_not_detected(self, _mock_which: mock.MagicMock) -> None:
        server = RTSPServer()
        assert server.is_gstreamer_mode is False
        assert server._use_fallback is True


# ──────────────────────────────────────────────────────────────────────────────
# Lifecycle — gstreamer path
# ──────────────────────────────────────────────────────────────────────────────


class TestGStreamerLifecycle:
    """Tests for the start / stop lifecycle when gstreamer is available."""

    def test_start_success(self) -> None:
        mock_proc = _make_mock_popen(returncode=None)  # still alive
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", return_value=mock_proc
        ), mock.patch("time.sleep"), mock.patch.object(
            RTSPServer, "_write_sdp_file"
        ):
            server = RTSPServer()
            result = server.start()

        assert result is True
        assert server.is_running() is True

    def test_start_when_already_running(self) -> None:
        mock_proc = _make_mock_popen(returncode=None)
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", return_value=mock_proc
        ), mock.patch("time.sleep"), mock.patch.object(
            RTSPServer, "_write_sdp_file"
        ):
            server = RTSPServer()
            assert server.start() is True
            # Second call — should return True immediately, no new process.
            popen_count_before = subprocess.Popen.call_count  # type: ignore[attr-defined]
            result = server.start()
            assert result is True
            # Popen should not have been called a second time.
            assert subprocess.Popen.call_count == popen_count_before  # type: ignore[attr-defined]

    def test_start_gstreamer_exits_early_falls_back(self) -> None:
        """GStreamer exits immediately → fall back to MJPEG and fail gracefully."""
        mock_proc = _make_mock_popen(returncode=1)  # died immediately
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", return_value=mock_proc
        ), mock.patch("time.sleep"), mock.patch.object(
            RTSPServer, "_start_mjpeg", return_value=False
        ) as mock_mjpeg:
            server = RTSPServer()
            result = server.start()

        assert result is False
        mock_mjpeg.assert_called_once()

    def test_stop_terminates_process(self) -> None:
        mock_proc = _make_mock_popen(returncode=None)
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", return_value=mock_proc
        ), mock.patch("time.sleep"), mock.patch.object(
            RTSPServer, "_write_sdp_file"
        ):
            server = RTSPServer()
            server.start()
            server.stop()

        assert server.is_running() is False
        # The process should have been waited on.
        mock_proc.wait.assert_called()

    def test_subprocess_file_not_found_falls_back(self) -> None:
        """FileNotFoundError in Popen → MJPEG fallback."""
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", side_effect=FileNotFoundError
        ), mock.patch.object(
            RTSPServer, "_start_mjpeg", return_value=False
        ) as mock_mjpeg:
            server = RTSPServer()
            result = server.start()

        assert result is False
        mock_mjpeg.assert_called_once()


# ──────────────────────────────────────────────────────────────────────────────
# Lifecycle — MJPEG fallback path
# ──────────────────────────────────────────────────────────────────────────────


class TestMJPEGLifecycle:
    """Tests for the MJPEG HTTP fallback server lifecycle."""

    def test_start_mjpeg_success(self) -> None:
        mock_cv2 = mock.MagicMock()
        mock_cv2.VideoCapture.return_value.isOpened.return_value = True

        with _patch_gstreamer(None), mock.patch.dict(
            "sys.modules", {"cv2": mock_cv2}
        ), mock.patch(
            "grain_sampling_camera.rtsp_server.HTTPServer"
        ) as mock_http, mock.patch(
            "grain_sampling_camera.rtsp_server.threading.Thread"
        ) as mock_thread:
            mock_http_instance = mock.MagicMock()
            mock_http.return_value = mock_http_instance

            server = RTSPServer()
            result = server.start()

        assert result is True
        assert server.is_running() is True
        mock_thread.assert_called_once()
        mock_http.assert_called_once()

    def test_start_mjpeg_camera_fails(self) -> None:
        mock_cv2 = mock.MagicMock()
        mock_cv2.VideoCapture.return_value.isOpened.return_value = False

        with _patch_gstreamer(None), mock.patch.dict(
            "sys.modules", {"cv2": mock_cv2}
        ):
            server = RTSPServer()
            result = server.start()

        assert result is False
        assert server.is_running() is False

    def test_start_mjpeg_cv2_not_installed(self) -> None:
        with _patch_gstreamer(None), mock.patch.dict(
            "sys.modules", {"cv2": None}
        ):
            server = RTSPServer()
            # The inner import will raise ImportError; _start_mjpeg should
            # catch it and return False.
            result = server.start()

        assert result is False
        assert server.is_running() is False

    def test_stop_mjpeg_shutdown(self) -> None:
        mock_cv2 = mock.MagicMock()
        mock_cv2.VideoCapture.return_value.isOpened.return_value = True

        with _patch_gstreamer(None), mock.patch.dict(
            "sys.modules", {"cv2": mock_cv2}
        ), mock.patch(
            "grain_sampling_camera.rtsp_server.HTTPServer"
        ) as mock_http, mock.patch(
            "grain_sampling_camera.rtsp_server.threading.Thread"
        ):
            mock_http_instance = mock.MagicMock()
            mock_http.return_value = mock_http_instance

            server = RTSPServer()
            server.start()
            server.stop()

        assert server.is_running() is False
        mock_http_instance.shutdown.assert_called_once()
        mock_http_instance.server_close.assert_called_once()


# ──────────────────────────────────────────────────────────────────────────────
# Status / is_running edge cases
# ──────────────────────────────────────────────────────────────────────────────


class TestIsRunning:
    """Granular tests for the ``is_running`` method."""

    def test_not_running_initially(self) -> None:
        server = RTSPServer()
        assert server.is_running() is False

    def test_stopped_twice_is_safe(self) -> None:
        """Calling stop() when not running should be a no-op, not a crash."""
        server = RTSPServer()
        # First call on a never-started server — must not raise.
        server.stop()
        assert server.is_running() is False

    def test_gstreamer_dies_after_start(self) -> None:
        """is_running() should detect a dead gstreamer subprocess."""
        mock_proc = _make_mock_popen(returncode=None)
        with _patch_gstreamer("/usr/bin/gst-launch-1.0"), mock.patch(
            "subprocess.Popen", return_value=mock_proc
        ), mock.patch("time.sleep"), mock.patch.object(
            RTSPServer, "_write_sdp_file"
        ):
            server = RTSPServer()
            server.start()
            assert server.is_running() is True

            # Simulate process death.
            mock_proc.poll.return_value = 42
            mock_proc.returncode = 42
            assert server.is_running() is False


# ──────────────────────────────────────────────────────────────────────────────
# SDP helpers
# ──────────────────────────────────────────────────────────────────────────────


class TestSDPHelpers:
    """Tests for SDP file generation and cleanup."""

    def test_write_sdp_file(self, tmp_path: mock.MagicMock) -> None:
        server = RTSPServer()
        # Monkey-patch the path to use tmp_path.
        server._sdp_path = lambda: tmp_path / "test.sdp"  # type: ignore[assignment]
        server._write_sdp_file()
        content = (tmp_path / "test.sdp").read_text()
        assert "v=0" in content
        assert "H264" in content
        assert "m=video" in content

    def test_remove_sdp_file_missing_ok(self) -> None:
        """Removing a non-existent SDP file should not raise."""
        server = RTSPServer()
        # Point to a path that definitely does not exist.
        server._sdp_path = lambda: mock.MagicMock()  # type: ignore[assignment]
        server._remove_sdp_file()  # should not raise
