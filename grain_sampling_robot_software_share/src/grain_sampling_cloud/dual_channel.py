"""
Dual-channel communication manager.

Manages two HTTP cloud clients — a primary (5G) channel and a fallback
(WiFi) channel. Automatically detects network availability, switches
channels on failure, and reverts when the primary recovers.  Emits
events on channel switches.
"""

import logging
import subprocess
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

from .http_client import CloudHttpClient

logger = logging.getLogger(__name__)


class Channel(Enum):
    """Identifies which communication channel is currently active."""

    PRIMARY = "primary"  # 5G
    FALLBACK = "fallback"  # WiFi


class ChannelSwitchReason(Enum):
    """Reason for a channel switch."""

    PRIMARY_CONNECTED = "primary_connected"
    PRIMARY_DISCONNECTED = "primary_disconnected"
    PRIMARY_RECOVERED = "primary_recovered"
    FALLBACK_ACTIVATED = "fallback_activated"
    MANUAL_SWITCH = "manual_switch"


@dataclass
class ChannelSwitchEvent:
    """Payload for channel switch signals."""

    previous: Channel
    current: Channel
    reason: ChannelSwitchReason
    timestamp: float = field(default_factory=time.time)


class DualChannelManager:
    """Manages a primary (5G) and a fallback (WiFi) HTTP cloud channel.

    The manager:
        - Keeps both clients configured but only one active at a time.
        - Monitors network availability for each channel.
        - Switches to fallback when the primary fails.
        - Switches back to primary when it recovers.
        - Forwards ``post()`` and ``get()`` requests to the active client.

    Configuration via ``config`` dict:
        ``primary``: Config dict for the primary ``CloudHttpClient`` (5G).
        ``fallback``: Config dict for the fallback ``CloudHttpClient`` (WiFi).
        ``network_check``: Dict with:
            - ``enabled`` (bool, default ``True``)
            - ``primary_ping`` (str, default ``"8.8.8.8"``) — host to ping for primary
            - ``fallback_ping`` (str, default ``"192.168.1.1"``) — host to ping for fallback
            - ``interval`` (float, default ``5.0``) — seconds between checks
            - ``timeout`` (int, default ``2``) — ping timeout in seconds
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self._config = config or {}

        # Build client configs
        primary_cfg = dict(self._config.get("primary", {}))
        fallback_cfg = dict(self._config.get("fallback", {}))

        self._primary_client = CloudHttpClient(primary_cfg)
        self._fallback_client = CloudHttpClient(fallback_cfg)

        # Network check config
        net_check = self._config.get("network_check", {})
        self._network_check_enabled = net_check.get("enabled", True)
        self._primary_ping_host = net_check.get("primary_ping", "8.8.8.8")
        self._fallback_ping_host = net_check.get("fallback_ping", "192.168.1.1")
        self._check_interval = float(net_check.get("interval", 5.0))
        self._ping_timeout = int(net_check.get("timeout", 2))

        # State
        self._active_channel = Channel.PRIMARY
        self._lock = threading.Lock()
        self._running = threading.Event()

        # Signal callbacks
        self._on_channel_switch_callbacks: List[Callable[[ChannelSwitchEvent], None]] = []

        # Monitor thread
        self._monitor_thread: Optional[threading.Thread] = None

        # Track last known network state
        self._primary_network_up = True
        self._fallback_network_up = True

    # ------------------------------------------------------------------
    # Public API — delegates to the active client
    # ------------------------------------------------------------------

    def start_monitoring(self) -> None:
        """Start channel monitoring (network health checks).

        Unlike MQTT, HTTP clients do not maintain persistent connections,
        so no explicit ``connect()`` is needed.  Network monitoring is
        started so the manager can detect failures and switch channels.
        """
        self._running.set()
        self._active_channel = Channel.PRIMARY
        logger.info("Dual-channel monitoring started (primary: 5G)")

        # Start background monitor
        if (
            self._network_check_enabled
            and (self._monitor_thread is None or not self._monitor_thread.is_alive())
        ):
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop, daemon=True, name="channel-monitor"
            )
            self._monitor_thread.start()

    def stop_monitoring(self) -> None:
        """Stop channel monitoring."""
        self._running.clear()
        logger.info("Dual-channel monitoring stopped")

    def post(self, path: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Send an HTTP POST via the currently active channel.

        Args:
            path: API path (e.g. ``/api/tasks``).
            data: JSON-serialisable request body.

        Returns:
            Parsed JSON response.

        Raises:
            CloudConnectionError: If all retries fail.
            CloudAuthError: On 401/403.
            CloudProtocolError: On unexpected response format.
        """
        active = self.get_active_channel()
        client = self._get_client(active)
        return client.post(path, data)

    def get(self, path: str, params: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
        """Send an HTTP GET via the currently active channel.

        Args:
            path: API path.
            params: Optional query-string parameters.

        Returns:
            Parsed JSON response.
        """
        active = self.get_active_channel()
        client = self._get_client(active)
        return client.get(path, params)

    def is_connected(self) -> bool:
        """Return whether the currently active channel is reachable."""
        client = self._get_client(self._active_channel)
        return client.is_connected()

    def get_active_channel(self) -> Channel:
        """Return the currently active channel."""
        return self._active_channel

    # ------------------------------------------------------------------
    # Signal registration
    # ------------------------------------------------------------------

    def on_channel_switch(
        self, callback: Callable[[ChannelSwitchEvent], None]
    ) -> None:
        """Register a callback invoked when the active channel changes.

        The callback receives a ``ChannelSwitchEvent`` instance with
        ``previous``, ``current``, ``reason``, and ``timestamp`` fields.
        """
        self._on_channel_switch_callbacks.append(callback)

    # ------------------------------------------------------------------
    # Internal: channel clients
    # ------------------------------------------------------------------

    def _get_client(self, channel: Channel) -> CloudHttpClient:
        """Return the HTTP client for the given channel."""
        if channel == Channel.PRIMARY:
            return self._primary_client
        return self._fallback_client

    # ------------------------------------------------------------------
    # Internal: channel switching
    # ------------------------------------------------------------------

    def _activate_fallback(self, reason: ChannelSwitchReason) -> None:
        """Switch the active channel to fallback."""
        with self._lock:
            if self._active_channel == Channel.FALLBACK:
                return
            previous = self._active_channel
            self._active_channel = Channel.FALLBACK

        logger.warning("Switched to FALLBACK (WiFi) channel — reason: %s", reason.value)
        self._fire_channel_switch(previous, Channel.FALLBACK, reason)

    def _activate_primary(self, reason: ChannelSwitchReason) -> None:
        """Switch the active channel back to primary."""
        with self._lock:
            if self._active_channel == Channel.PRIMARY:
                return
            previous = self._active_channel
            self._active_channel = Channel.PRIMARY

        logger.info("Switched back to PRIMARY (5G) channel — reason: %s", reason.value)
        self._fire_channel_switch(previous, Channel.PRIMARY, reason)

    def _fire_channel_switch(
        self,
        previous: Channel,
        current: Channel,
        reason: ChannelSwitchReason,
    ) -> None:
        """Notify all registered channel-switch callbacks."""
        event = ChannelSwitchEvent(previous=previous, current=current, reason=reason)
        for cb in self._on_channel_switch_callbacks:
            try:
                cb(event)
            except Exception as exc:
                logger.error("on_channel_switch callback error: %s", exc)

    def switch_to_channel(self, channel: Channel) -> None:
        """Manually switch to a specific channel.

        Args:
            channel: The channel to activate (``Channel.PRIMARY`` or ``Channel.FALLBACK``).
        """
        if channel == Channel.PRIMARY:
            self._activate_primary(ChannelSwitchReason.MANUAL_SWITCH)
        else:
            self._activate_fallback(ChannelSwitchReason.MANUAL_SWITCH)

    # ------------------------------------------------------------------
    # Internal: network monitoring
    # ------------------------------------------------------------------

    def _monitor_loop(self) -> None:
        """Background thread: periodically checks network availability."""
        while self._running.is_set():
            self._check_network()
            self._running.wait(self._check_interval)

    def _check_network(self) -> None:
        """Check network connectivity for primary and fallback channels."""
        primary_up = self._ping_host(self._primary_ping_host)
        fallback_up = self._ping_host(self._fallback_ping_host)

        self._primary_network_up = primary_up
        self._fallback_network_up = fallback_up

        current = self._active_channel

        if current == Channel.PRIMARY and not primary_up:
            logger.warning(
                "Primary network (%s) unreachable, switching to fallback",
                self._primary_ping_host,
            )
            self._activate_fallback(ChannelSwitchReason.PRIMARY_DISCONNECTED)

        elif current == Channel.FALLBACK and primary_up:
            logger.info(
                "Primary network (%s) recovered, switching back",
                self._primary_ping_host,
            )
            self._activate_primary(ChannelSwitchReason.PRIMARY_RECOVERED)

    def _ping_host(self, host: str) -> bool:
        """Check if a host is reachable via ICMP ping.

        Uses the platform-appropriate ping command. Returns ``True``
        if the host responds.

        Args:
            host: Hostname or IP address to ping.

        Returns:
            ``True`` if the host is reachable, ``False`` otherwise.
        """
        try:
            # Windows ping uses -n, Linux/Mac use -c
            cmd = ["ping", "-n", "1", "-w", str(self._ping_timeout * 1000), host]
            result = subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=self._ping_timeout + 1,
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError, OSError) as exc:
            logger.debug("Ping to %s failed: %s", host, exc)
            return False

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "DualChannelManager":
        return self

    def __exit__(self, *args: Any) -> None:
        self.stop_monitoring()
