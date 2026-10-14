"""Exercise persistent navigation IPC on an isolated socket pair, never hardware."""
import json
import socket
import threading
import time
from unittest.mock import Mock
import pytest
from grain_sampling_local.navigation_worker import NavigationSocketClient


def pair(monkeypatch, *, timeout=.12):
    left, right = socket.socketpair()
    factory = Mock(return_value=left)
    monkeypatch.setattr(socket, "socket", factory)
    # Inject an already connected pair: no path connects to production.
    client = NavigationSocketClient("TEST_ONLY", timeout_s=timeout)
    left.settimeout(timeout)
    client.conn, client.stream = left, left.makefile("rb")
    return client, right, factory


def test_many_navigation_updates_use_one_connection_and_exact_sequence(monkeypatch):
    client, peer, factory = pair(monkeypatch)
    calls = []
    def serve():
        with peer, peer.makefile("rb") as stream:
            for line in stream:
                calls.append(json.loads(line))
                peer.sendall(b'{"ok":true}\n')
    thread = threading.Thread(target=serve, daemon=True);thread.start()
    try:
        for i in range(100):
            assert client.request("nav_inputs", token="test", sequence=i+1, inputs={})["ok"]
        assert len(calls) == 100
        assert [c["sequence"] for c in calls] == list(range(1, 101))
        factory.assert_not_called()
    finally:
        client.close();thread.join(1)
    assert not thread.is_alive()


def test_stalled_command_discards_connection_without_retry(monkeypatch):
    client, peer, _ = pair(monkeypatch, timeout=.03)
    start = time.monotonic()
    try:
        with pytest.raises(TimeoutError):
            client.request("nav_velocity", token="test", sequence=1,
                           goal_id="test", forward=.5, turn=0)
        assert time.monotonic() - start < .2
        assert client.conn is None and client.stream is None
        peer.settimeout(.05)
        raw=peer.recv(4096)
        assert raw.count(b"\n") == 1  # no retransmit / rearm
        assert peer.recv(4096) == b""
    finally:
        client.close();peer.close()


def test_manual_rejection_is_propagated_and_never_rearmed(monkeypatch):
    client, peer, _ = pair(monkeypatch)
    peer.sendall(b'{"ok":false,"error":"manual override; explicit new goal required"}\n')
    try:
        with pytest.raises(RuntimeError, match="manual override"):
            client.request("nav_velocity", sequence=1)
        assert client.conn is None
        assert b"rearm" not in peer.recv(4096)
    finally:
        client.close();peer.close()


def test_navigation_socket_cannot_send_ui_or_raw_hardware_requests(monkeypatch):
    client, peer, _=pair(monkeypatch)
    try:
        with pytest.raises(ValueError, match="navigation-only"):
            client.request("clear_estop")
    finally:
        client.close();peer.close()
