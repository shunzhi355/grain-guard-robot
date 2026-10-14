#!/usr/bin/env python3
"""RTMP streamer — push /dev/video0 camera frames to cloud RTMP relay via FFmpeg.

Pipeline::

    /dev/video0 ──→ FFmpeg(libx264) ──→ rtmp://124.220.41.27:41935/live/1

Config sources (priority)::

    1. Environment variables: RTMP_PUSH_URL, CAMERA_DEVICE
    2. ``AppConfig`` from ``src/utils/config.py``

Usage::

    python3 grain_sampling_camera/rtmp_streamer.py

Intended to be launched by ``scripts/start.sh``. Runs as a persistent daemon,
auto-restarting FFmpeg on crash with exponential backoff.
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger("rtmp_streamer")

# ── defaults ─────────────────────────────────────────────────────────────────

_DEFAULT_PUSH_URL = "rtmp://124.220.41.27:41935/live/1"
_DEFAULT_DEVICE = "/dev/video0"
_DEFAULT_SIZE = "640x480"
_DEFAULT_FPS = 15

# ── FFmpeg command builder ────────────────────────────────────────────────────


def _build_cmd(push_url: str, device: str, size: str, fps: int) -> list[str]:
    """Build the FFmpeg command line for v4l2 → H264 → RTMP."""
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel", "error",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", size,
        "-framerate", str(fps),
        "-i", device,
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-pix_fmt", "yuv420p",
        "-g", str(fps * 2),
        "-f", "flv",
        push_url,
    ]


# ── config resolution ────────────────────────────────────────────────────────


def _resolve_config() -> tuple[str, str, str, int]:
    """Resolve push URL and camera device.

    Precedence: env vars → ``AppConfig`` → hard-coded defaults.
    """
    push_url = os.environ.get("RTMP_PUSH_URL")
    device = os.environ.get("CAMERA_DEVICE")

    if not push_url or not device:
        try:
            # Add project src/ to path so AppConfig can be imported from
            # the project root directory.
            proj_root = Path(__file__).resolve().parents[2]
            src_dir = str(proj_root / "src")
            if src_dir not in sys.path:
                sys.path.insert(0, src_dir)
            from utils.config import AppConfig

            cfg = AppConfig()
            if not push_url:
                push_url = cfg.rtmp_push_url
            if not device:
                device = cfg.camera_device
        except Exception:
            pass

    return (
        push_url or _DEFAULT_PUSH_URL,
        device or _DEFAULT_DEVICE,
        _DEFAULT_SIZE,
        _DEFAULT_FPS,
    )


# ── RTMPStreamer ─────────────────────────────────────────────────────────────


class RTMPStreamer:
    """Manages a persistent FFmpeg subprocess for RTMP camera streaming.

    Start it once; it auto-restarts FFmpeg on unexpected exit with
    exponential backoff (max 60 s between attempts).
    """

    def __init__(self) -> None:
        push_url, device, size, fps = _resolve_config()
        self._push_url: str = push_url
        self._device: str = device
        self._size: str = size
        self._fps: int = fps
        self._proc: Optional[subprocess.Popen] = None
        self._running: bool = False
        self._restart_count: int = 0

    # ── public API ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the FFmpeg streamer loop (blocks until stopped)."""
        self._running = True
        logger.info("RTMP streamer starting: %s → %s (%s @ %d fps)",
                     self._device, self._push_url, self._size, self._fps)
        while self._running:
            self._launch_ffmpeg()
            self._watch_ffmpeg()

    def stop(self) -> None:
        """Gracefully stop streaming."""
        logger.info("RTMP streamer stopping...")
        self._running = False
        self._kill_ffmpeg()

    def is_running(self) -> bool:
        """Check if the FFmpeg subprocess is alive."""
        return self._proc is not None and self._proc.poll() is None

    # ── internals ─────────────────────────────────────────────────────────

    def _launch_ffmpeg(self) -> None:
        cmd = _build_cmd(self._push_url, self._device, self._size, self._fps)
        logger.debug("FFmpeg cmd: %s", " ".join(cmd))
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            logger.info("FFmpeg launched (pid=%d)", self._proc.pid)
            self._launch_time = time.time()
        except FileNotFoundError:
            logger.critical("ffmpeg binary not found — install ffmpeg")
            self._running = False
        except PermissionError:
            logger.critical("Cannot access %s — check device permissions", self._device)
            self._running = False

    def _watch_ffmpeg(self) -> None:
        """Block until FFmpeg exits or stop is requested, then handle restart."""
        if self._proc is None:
            return
        while self._proc.poll() is None and self._running:
            time.sleep(1)
        if not self._running:
            return
        returncode = self._proc.returncode
        stderr_tail = ""
        try:
            stderr_tail = self._proc.stderr.read().decode(errors="replace")[-200:]
        except Exception:
            pass
        logger.warning("FFmpeg exited (code=%d). stderr tail: %s", returncode, stderr_tail)
        # Reset backoff if the last run was stable (≥60 s)
        if hasattr(self, "_launch_time") and (time.time() - self._launch_time) >= 60:
            self._restart_count = 0
        self._backoff_restart()

    def _backoff_restart(self) -> None:
        max_delay = 60
        delays = [1, 2, 4, 8, 16, 30, max_delay]
        idx = min(self._restart_count, len(delays) - 1)
        delay = delays[idx]
        self._restart_count += 1
        logger.info("Restart #%d in %ds…", self._restart_count, delay)
        time.sleep(delay)

    def _kill_ffmpeg(self) -> None:
        if self._proc is None or self._proc.poll() is not None:
            return
        logger.info("Sending SIGTERM to FFmpeg (pid=%d)…", self._proc.pid)
        self._proc.terminate()
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            logger.warning("FFmpeg did not exit, sending SIGKILL")
            self._proc.kill()
            self._proc.wait()
        logger.info("FFmpeg stopped")


# ── main ──────────────────────────────────────────────────────────────────────


def _main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    )
    streamer = RTMPStreamer()
    # Graceful shutdown on SIGTERM / SIGINT
    signal.signal(signal.SIGTERM, lambda *_: streamer.stop())
    signal.signal(signal.SIGINT, lambda *_: streamer.stop())
    streamer.start()
    logger.info("RTMP streamer exited cleanly")


if __name__ == "__main__":
    _main()
