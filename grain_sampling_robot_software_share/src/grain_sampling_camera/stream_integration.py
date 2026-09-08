"""stream_integration.py — RTSP streaming management.

Orchestrates the RTSPServer from ``grain_sampling_camera`` for local
camera streaming.  Cloud connectivity for stream control is handled
separately via the HTTP cloud client.
"""

from __future__ import annotations

import logging
import socket
import time
from threading import Lock
from typing import Any, Dict, Optional

from grain_sampling_camera.rtsp_server import RTSPServer

_logger = logging.getLogger("stream_integration")


def _get_default_ip() -> str:
    """Return a best-guess local IP address used for the default RTSP URL.

    Falls back to ``"127.0.0.1"`` when no external interface is found.
    Uses a short timeout to avoid blocking on restricted networks.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(1.0)
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except (OSError, TimeoutError):
        return "127.0.0.1"


def _default_config() -> Dict[str, Any]:
    return {
        "host": "127.0.0.1",  # safe default; overridden by explicit config
        "port": 8554,
        "mount_point": "/camera",
        "device": "/dev/video0",
        "width": 1280,
        "height": 720,
        "framerate": 15,
        "bitrate": 2000,
    }


class StreamManager:
    """Orchestrates camera RTSP streaming.

    Wraps ``RTSPServer`` for local streaming.  Exposes a simple API:
    ``start_streaming``, ``stop_streaming``, ``get_stream_status``,
    and ``update_config``.

    Parameters
    ----------
    device_id:
        Unique robot identifier (used for logging / identification).
    config:
        Optional configuration dict overriding defaults (see
        ``_default_config`` for keys).
    rtsp_server:
        Optional pre-configured ``RTSPServer`` instance (allows
        injecting a mock for testing).
    """

    def __init__(
        self,
        device_id: str = "grain-robot-001",
        config: Optional[Dict[str, Any]] = None,
        rtsp_server: Optional[RTSPServer] = None,
    ) -> None:
        self._device_id = device_id
        self._lock = Lock()

        # Merge supplied config over defaults
        self._config = _default_config()
        if config:
            self._config.update(config)

        # RTSP server (accept injected mock)
        if rtsp_server is not None:
            self._rtsp = rtsp_server
        else:
            self._rtsp = RTSPServer(
                device=self._config.get("device", "/dev/video0"),
                port=int(self._config.get("port", 8554)),
                mount_point=self._config.get("mount_point", "/camera"),
                host=self._config.get("host", "localhost"),
                width=int(self._config.get("width", 1280)),
                height=int(self._config.get("height", 720)),
                framerate=int(self._config.get("framerate", 15)),
                bitrate=int(self._config.get("bitrate", 2000)),
            )

        self._streaming: bool = False
        self._stream_start_time: float = 0.0

    # ── Public API ────────────────────────────────────────────────────

    @property
    def rtsp_url(self) -> str:
        """Return the RTSP URL for the current configuration."""
        host = self._config.get("host", "localhost")
        port = self._config.get("port", 8554)
        mount = self._config.get("mount_point", "/camera")
        return f"rtsp://{host}:{port}{mount}"

    def start_streaming(self, rtsp_url: Optional[str] = None) -> bool:
        """Start the RTSP server.

        Parameters
        ----------
        rtsp_url:
            Optional override URL.  If provided, the ``host``,
            ``port``, and ``mount_point`` are parsed from it and the
            internal config is updated.

        Returns
        -------
        bool
            ``True`` if the stream was started, ``False`` otherwise.
        """
        with self._lock:
            if self._streaming:
                _logger.warning("Streaming is already active")
                return True

            if rtsp_url is not None:
                self._parse_url(rtsp_url)

            ok = self._rtsp.start()
            if ok:
                self._streaming = True
                self._stream_start_time = time.time()
                _logger.info("Streaming started: %s", self.rtsp_url)
            else:
                _logger.error("Failed to start RTSP server")
            return ok

    def stop_streaming(self) -> None:
        """Stop the RTSP server."""
        with self._lock:
            if not self._streaming:
                return
            self._rtsp.stop()
            self._streaming = False
            self._stream_start_time = 0.0
            _logger.info("Streaming stopped")

    def get_stream_status(self) -> Dict[str, Any]:
        """Return a dict with current stream status info.

        Fields:
            - ``streaming`` (bool)
            - ``rtsp_url`` (str)
            - ``duration_s`` (float) — elapsed time or 0
            - ``config`` (dict) — current configuration snapshot
        """
        with self._lock:
            elapsed = 0.0
            if self._streaming and self._stream_start_time > 0:
                elapsed = time.time() - self._stream_start_time
            return {
                "streaming": self._streaming,
                "rtsp_url": self.rtsp_url,
                "duration_s": round(elapsed, 1),
                "config": dict(self._config),
            }

    def update_config(self, config: Dict[str, Any]) -> None:
        """Update the internal RTSP / camera configuration from a cloud dict.

        Does *not* restart an active stream — the caller should stop and
        restart if the config change requires it.
        """
        with self._lock:
            self._config.update(config)
            _logger.info("Stream config updated: %s", config)

    # ── URL parsing ───────────────────────────────────────────────────

    def _parse_url(self, url: str) -> None:
        """Parse an RTSP URL and update host, port, mount_point in config."""
        # Format: rtsp://host:port/path
        if not url.startswith("rtsp://"):
            raise ValueError(f"Invalid RTSP URL: {url!r}")
        rest = url[7:]  # strip "rtsp://"
        # Split host:port from path
        if "/" in rest:
            host_part, path_part = rest.split("/", 1)
        else:
            host_part = rest
            path_part = ""
        if ":" in host_part:
            host, port_str = host_part.rsplit(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                raise ValueError(f"Invalid port in URL: {url!r}") from None
        else:
            host = host_part
            port = 8554
        self._config["host"] = host
        self._config["port"] = port
        self._config["mount_point"] = f"/{path_part}" if path_part else "/camera"
