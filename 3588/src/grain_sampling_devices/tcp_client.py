"""Raw TCP socket client for device communication.

Provides a thread-safe, auto-reconnecting TCP transport layer used by
every device adapter.  All socket I/O is wrapped so that higher-level
adapters never deal with raw ``socket`` calls directly.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Optional

from grain_sampling_devices.base_adapter import (
    DeviceConnectionError,
    DeviceError,
    DeviceTimeoutError,
)

logger = logging.getLogger(__name__)

# ── Constants ───────────────────────────────────────────────────────────

_DEFAULT_BUF_SIZE = 4096
_RECONNECT_DELAY = 0.1  # seconds between reconnect attempts


# ── TCPClient ───────────────────────────────────────────────────────────


class TCPClient:
    """Thread-safe TCP client with automatic reconnection.

    Parameters
    ----------
    host : str
        IPv4 address of the target device.
    port : int
        TCP port number.
    timeout : float
        Socket timeout in seconds (applied to connect, send, recv).
    retry_count : int
        Maximum number of automatic reconnection attempts when a send
        fails because of a broken pipe or reset connection.
    """

    def __init__(
        self,
        host: str,
        port: int,
        timeout: float = 5.0,
        retry_count: int = 3,
    ) -> None:
        self._host = host
        self._port = port
        self._timeout = timeout
        self._retry_count = retry_count

        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()

    # ── Public API ──────────────────────────────────────────────────

    def connect(self) -> None:
        """Open a TCP connection to the configured host:port.

        Raises
        ------
        DeviceTimeoutError
            If the connect operation exceeds *timeout*.
        DeviceConnectionError
            For any other socket-level failure.
        """
        with self._lock:
            self._disconnect_inner()

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self._timeout)
                sock.connect((self._host, self._port))
                self._socket = sock
                logger.info("TCP connected to %s:%d", self._host, self._port)
            except socket.timeout as exc:
                raise DeviceTimeoutError(
                    f"Connection to {self._host}:{self._port} timed out "
                    f"after {self._timeout}s"
                ) from exc
            except OSError as exc:
                raise DeviceConnectionError(
                    f"Failed to connect to {self._host}:{self._port}: {exc}"
                ) from exc

    def send(self, data: bytes) -> bytes:
        """Send binary data and return the raw response.

        Automatically attempts up to *retry_count* reconnections when
        the socket is broken.  Each send is guarded by :class:`threading.Lock`.

        Parameters
        ----------
        data : bytes
            Raw payload to transmit.

        Returns
        -------
        bytes
            Response received from the device.

        Raises
        ------
        DeviceTimeoutError
            If any I/O operation exceeds the configured timeout.
        DeviceConnectionError
            If the connection is lost and cannot be re-established.
        """
        last_exception: Optional[Exception] = None

        for attempt in range(self._retry_count + 1):
            with self._lock:
                try:
                    if self._socket is None:
                        self._connect_inner()
                    else:
                        self._socket.settimeout(self._timeout)

                    self._socket.sendall(data)
                    response = self._receive_response()
                    return response

                except (BrokenPipeError, ConnectionResetError, OSError) as exc:
                    last_exception = exc
                    logger.warning(
                        "Send attempt %d/%d failed: %s",
                        attempt + 1,
                        self._retry_count + 1,
                        exc,
                    )
                    self._disconnect_inner()
                    if attempt < self._retry_count:
                        time.sleep(_RECONNECT_DELAY)
                        try:
                            self._connect_inner()
                        except DeviceError as reconnect_exc:
                            last_exception = reconnect_exc

                except socket.timeout as exc:
                    last_exception = exc
                    if attempt < self._retry_count:
                        logger.warning(
                            "Send timed out (attempt %d/%d)",
                            attempt + 1,
                            self._retry_count + 1,
                        )
                    else:
                        break

        # ── All retries exhausted ──────────────────────────────────
        if isinstance(last_exception, DeviceError):
            raise last_exception
        if isinstance(last_exception, socket.timeout):
            raise DeviceTimeoutError(
                f"Send to {self._host}:{self._port} timed out after "
                f"{self._retry_count + 1} attempts"
            ) from last_exception
        raise DeviceConnectionError(
            f"Send to {self._host}:{self._port} failed after "
            f"{self._retry_count + 1} attempts: {last_exception}"
        ) from last_exception

    def disconnect(self) -> None:
        """Gracefully close the TCP connection."""
        with self._lock:
            self._disconnect_inner()

    def is_connected(self) -> bool:
        """Return ``True`` when the socket is open and writable."""
        return self._socket is not None

    # ── Properties ──────────────────────────────────────────────────

    @property
    def host(self) -> str:
        """IPv4 address of the device."""
        return self._host

    @property
    def port(self) -> int:
        """TCP port number."""
        return self._port

    @property
    def timeout(self) -> float:
        """Current socket timeout in seconds."""
        return self._timeout

    @timeout.setter
    def timeout(self, value: float) -> None:
        with self._lock:
            self._timeout = value
            if self._socket is not None:
                self._socket.settimeout(value)

    # ── Internal helpers ────────────────────────────────────────────

    def _connect_inner(self) -> None:
        """Connect without acquiring the lock (caller must hold it)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout)
            sock.connect((self._host, self._port))
            self._socket = sock
        except socket.timeout as exc:
            raise DeviceTimeoutError(
                f"Connection to {self._host}:{self._port} timed out"
            ) from exc
        except OSError as exc:
            raise DeviceConnectionError(
                f"Failed to connect to {self._host}:{self._port}: {exc}"
            ) from exc

    def _disconnect_inner(self) -> None:
        """Disconnect without acquiring the lock (caller must hold it)."""
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            finally:
                self._socket.close()
                self._socket = None

    def _receive_response(self) -> bytes:
        """Read all available data until the peer closes or times out.

        Because TCP is a stream protocol, this method keeps reading
        chunks until a ``socket.timeout`` signals that no more data is
        immediately available.  For production protocols that define
        explicit message boundaries (length-prefix, delimiter, …) this
        method should be overridden or replaced.
        """
        chunks: list[bytes] = []
        try:
            while True:
                chunk = self._socket.recv(_DEFAULT_BUF_SIZE)
                if not chunk:
                    break  # peer closed connection
                chunks.append(chunk)
                # After the first chunk, switch to a short inter-byte
                # timeout so we can detect end-of-message quickly.
                self._socket.settimeout(0.1)
        except socket.timeout:
            # Expected: no more data within the timeout window.
            pass

        return b"".join(chunks) if chunks else b""
