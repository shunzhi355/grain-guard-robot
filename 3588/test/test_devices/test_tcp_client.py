"""Tests for :mod:`grain_sampling_devices.tcp_client`.

All tests use :mod:`unittest.mock` — no actual network activity is
performed.
"""

import socket
from unittest import mock

import pytest

from grain_sampling_devices.base_adapter import (
    DeviceConnectionError,
    DeviceTimeoutError,
)
from grain_sampling_devices.tcp_client import TCPClient


# ── Helpers ─────────────────────────────────────────────────────────────


def _make_mock_socket() -> mock.MagicMock:
    """Return a :class:`MagicMock` that quacks like a connected socket."""
    sock = mock.MagicMock(spec=socket.socket)
    sock.recv.return_value = b""
    return sock


# ═════════════════════════════════════════════════════════════════════════
# Instantiation
# ═════════════════════════════════════════════════════════════════════════


class TestInstantiation:
    """Verify that TCPClient stores configuration correctly."""

    def test_default_values(self) -> None:
        client = TCPClient(host="192.168.1.100", port=5001)
        assert client.host == "192.168.1.100"
        assert client.port == 5001
        assert client.timeout == 5.0
        assert not client.is_connected()

    def test_custom_timeout_and_retries(self) -> None:
        client = TCPClient(
            host="10.0.0.1", port=9999, timeout=2.5, retry_count=5
        )
        assert client.timeout == 2.5
        assert client.host == "10.0.0.1"
        assert client.port == 9999

    def test_timeout_setter(self) -> None:
        client = TCPClient(host="192.168.1.1", port=5000, timeout=1.0)
        client.timeout = 8.0
        assert client.timeout == 8.0

    def test_timeout_setter_while_connected(self) -> None:
        """Changing timeout on a connected socket should call settimeout."""
        client = TCPClient(host="192.168.1.1", port=5000)
        fake = _make_mock_socket()
        with mock.patch("socket.socket", return_value=fake):
            client.connect()

        client.timeout = 3.0
        fake.settimeout.assert_called_with(3.0)


# ═════════════════════════════════════════════════════════════════════════
# Connect / Disconnect lifecycle
# ═════════════════════════════════════════════════════════════════════════


class TestConnectionLifecycle:

    def test_connect_creates_socket(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()

        with mock.patch("socket.socket", return_value=fake):
            client.connect()

        fake.settimeout.assert_called_once_with(5.0)
        fake.connect.assert_called_once_with(("1.2.3.4", 9999))
        assert client.is_connected()

    def test_disconnect_closes_socket(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()

        with mock.patch("socket.socket", return_value=fake):
            client.connect()

        client.disconnect()
        fake.shutdown.assert_called_once_with(socket.SHUT_RDWR)
        fake.close.assert_called_once()
        assert not client.is_connected()

    def test_disconnect_idempotent(self) -> None:
        """Calling disconnect when not connected should not raise."""
        client = TCPClient(host="1.2.3.4", port=9999)
        client.disconnect()  # should not raise
        client.disconnect()  # twice should be fine

    def test_connect_closes_previous_socket(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake1 = _make_mock_socket()
        fake2 = _make_mock_socket()

        with mock.patch("socket.socket", side_effect=[fake1, fake2]):
            client.connect()
            client.connect()

        # First socket must have been shut down
        fake1.shutdown.assert_called_once()
        fake1.close.assert_called_once()
        assert client.is_connected()

    def test_connect_timeout_raises_device_timeout(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999, timeout=0.01)
        fake = _make_mock_socket()
        fake.connect.side_effect = socket.timeout("timed out")

        with mock.patch("socket.socket", return_value=fake):
            with pytest.raises(DeviceTimeoutError, match="timed out"):
                client.connect()

        assert not client.is_connected()

    def test_connect_os_error_raises_connection_error(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.connect.side_effect = OSError("refused")

        with mock.patch("socket.socket", return_value=fake):
            with pytest.raises(DeviceConnectionError, match="refused"):
                client.connect()

    def test_disconnect_handles_shutdown_error(self) -> None:
        """Shutdown may fail on a half-closed socket; we must survive."""
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.shutdown.side_effect = OSError("already closed")

        with mock.patch("socket.socket", return_value=fake):
            client.connect()

        client.disconnect()  # should not raise
        fake.close.assert_called_once()


# ═════════════════════════════════════════════════════════════════════════
# Send / Receive
# ═════════════════════════════════════════════════════════════════════════


class TestSendReceive:

    def test_send_sends_all_data(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.side_effect = [b"OK\r\n", socket.timeout()]

        with mock.patch("socket.socket", return_value=fake):
            client.connect()
            resp = client.send(b"PING\n")

        fake.sendall.assert_called_once_with(b"PING\n")
        assert resp == b"OK\r\n"

    def test_send_receives_multiple_chunks(self) -> None:
        """Partial reads must be accumulated into a single response."""
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.side_effect = [b"HEL", b"LO ", b"WORLD", socket.timeout()]

        with mock.patch("socket.socket", return_value=fake):
            client.connect()
            resp = client.send(b"GREET\n")

        assert resp == b"HELLO WORLD"

    def test_send_empty_response(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.side_effect = socket.timeout()  # no data at all

        with mock.patch("socket.socket", return_value=fake):
            client.connect()
            resp = client.send(b"PING\n")

        assert resp == b""

    def test_send_peer_closed(self) -> None:
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.return_value = b""  # peer closed gracefully

        with mock.patch("socket.socket", return_value=fake):
            client.connect()
            resp = client.send(b"PING\n")

        assert resp == b""

    def test_send_auto_connects_when_disconnected(self) -> None:
        """If the client is not connected, send() calls connect first."""
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.side_effect = [b"ACK", socket.timeout()]

        with mock.patch("socket.socket", return_value=fake):
            resp = client.send(b"CMD")

        fake.connect.assert_called_once_with(("1.2.3.4", 9999))
        assert resp == b"ACK"


# ═════════════════════════════════════════════════════════════════════════
# Reconnection logic
# ═════════════════════════════════════════════════════════════════════════


class TestReconnection:

    def test_retry_on_broken_pipe(self) -> None:
        """After a broken pipe, the client should reconnect and retry."""
        client = TCPClient(
            host="1.2.3.4", port=9999, retry_count=2, timeout=1.0
        )

        # First socket: sendall raises BrokenPipeError
        sock1 = _make_mock_socket()
        sock1.sendall.side_effect = BrokenPipeError("broken")

        # Second socket: works fine
        sock2 = _make_mock_socket()
        sock2.recv.side_effect = [b"OK", socket.timeout()]

        with mock.patch("socket.socket", side_effect=[sock1, sock2]):
            resp = client.send(b"DATA")

        assert resp == b"OK"
        sock1.close.assert_called()
        sock2.sendall.assert_called_once_with(b"DATA")

    def test_retry_exhausted_raises_connection_error(self) -> None:
        """When all retries fail the client must raise."""
        client = TCPClient(
            host="1.2.3.4", port=9999, retry_count=1, timeout=1.0
        )

        sock = _make_mock_socket()
        sock.sendall.side_effect = BrokenPipeError("broken")

        with mock.patch("socket.socket", return_value=sock):
            with pytest.raises(DeviceConnectionError, match="failed after 2"):
                client.send(b"DATA")

    def test_retry_on_connection_reset(self) -> None:
        client = TCPClient(
            host="1.2.3.4", port=9999, retry_count=1, timeout=1.0
        )

        sock1 = _make_mock_socket()
        sock1.sendall.side_effect = ConnectionResetError("reset")

        sock2 = _make_mock_socket()
        sock2.recv.side_effect = [b"DATA", socket.timeout()]

        with mock.patch("socket.socket", side_effect=[sock1, sock2]):
            resp = client.send(b"REQ")

        assert resp == b"DATA"

    def test_timeout_exhausted_raises_device_timeout(self) -> None:
        client = TCPClient(
            host="1.2.3.4", port=9999, retry_count=1, timeout=0.1
        )
        fake = _make_mock_socket()
        fake.sendall.side_effect = socket.timeout("timed out")  # always times out

        with mock.patch("socket.socket", return_value=fake):
            with pytest.raises(DeviceTimeoutError, match="timed out after 2"):
                client.send(b"DATA")


# ═════════════════════════════════════════════════════════════════════════
# Thread safety (basic)
# ═════════════════════════════════════════════════════════════════════════


class TestThreadSafety:

    def test_send_is_serialised_by_lock(self) -> None:
        """send must acquire the internal lock before doing any I/O."""
        client = TCPClient(host="1.2.3.4", port=9999)
        fake = _make_mock_socket()
        fake.recv.side_effect = [b"OK", socket.timeout()]

        # Replace the raw _thread.lock with a MagicMock so we can
        # observe acquire/release via __enter__ / __exit__.
        mock_lock = mock.MagicMock()
        client._lock = mock_lock

        with mock.patch("socket.socket", return_value=fake):
            client.connect()
            client.send(b"PING")

        # send() uses "with self._lock:" — verify __enter__ was called.
        mock_lock.__enter__.assert_called()
        mock_lock.__exit__.assert_called()
