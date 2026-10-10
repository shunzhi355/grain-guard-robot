"""Linux integration check: real local socket, strictly simulated hardware."""
import importlib.util
import os
import socket
import stat
import tempfile
import threading
from pathlib import Path

import pytest

from grain_sampling_bench.runner import wait_for
from grain_sampling_local.config import LocalConfig
from grain_sampling_workflow.robot_bridge import RobotClient

pytestmark = pytest.mark.skipif(os.name != "posix", reason="Linux AF_UNIX runtime")
ROOT = Path(__file__).resolve().parents[2]


def test_actual_unix_socket_status_and_stop(monkeypatch):
    import serial
    def forbidden(*args, **kwargs):
        raise AssertionError("simulation opened physical serial port")
    monkeypatch.setattr(serial, "Serial", forbidden)
    real_socket = socket.socket
    class LocalOnlySocket(real_socket):
        def __init__(self, family=socket.AF_INET, *args, **kwargs):
            if family != socket.AF_UNIX:
                raise AssertionError("local robot opened a network listener")
            super().__init__(family, *args, **kwargs)
    monkeypatch.setattr(socket, "socket", LocalOnlySocket)
    spec = importlib.util.spec_from_file_location("ipc_entry", ROOT / "LENOVO/scripts/local_robot.py")
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    with tempfile.TemporaryDirectory(prefix="grain-local-ipc-") as directory:
        config = LocalConfig(runtime_dir=directory, suction_policy="external")
        service, peers = entry.create_service(config, simulate=True)
        errors = []
        def serve():
            try:
                service.serve()
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        try:
            wait_for(lambda: config.socket_path.exists(), timeout=3)
            assert stat.S_IMODE(config.socket_path.stat().st_mode) == 0o600
            client = RobotClient(str(config.socket_path))
            status = client.request("status", client_role="ui")
            assert status["runtime"] == "lenovo-local"
            assert status["simulation"] is True
            client.request("mechanism", name="set_grain", grain="稻谷")
            client.request("mechanism", name="start_convey")
            assert 2 in peers[0].active
            client.request("estop")
            assert not peers[0].active
            assert client.request("status")["safety_latched"]
        finally:
            service.running.clear()
            worker.join(timeout=3)
        assert not worker.is_alive()
        assert not config.socket_path.exists()
        assert not errors
