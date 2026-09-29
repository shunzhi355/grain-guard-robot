"""Physicochemical tester adapter (TCP / JSON stub).

⚠️ **STUB IMPLEMENTATION** — This adapter assumes a simple JSON-over-TCP
protocol.  The real wire protocol must be provided by the equipment
supplier's API documentation.  Replace every ``# STUB:`` marker with the
actual specification when the API docs become available.
"""

import json
import logging
from typing import Any, Dict, Optional

from grain_sampling_devices.base_adapter import (
    BaseDeviceAdapter,
    DeviceConnectionError,
    DeviceError,
    DeviceTimeoutError,
)
from grain_sampling_devices.tcp_client import TCPClient

logger = logging.getLogger(__name__)

_DEFAULT_PORT = 5002


class PhysicochemicalAdapter(BaseDeviceAdapter):
    """Adapter for a physicochemical grain-testing instrument.

    Communicates via Ethernet / TCP.  The stub protocol wraps each
    command in a JSON object sent as UTF-8 bytes over a raw TCP socket.

    Parameters
    ----------
    host : str
        IPv4 address of the physicochemical tester.
    port : int, optional
        TCP port.  Defaults to *5002*.
    """

    def __init__(self, host: str, port: int = _DEFAULT_PORT) -> None:
        self._host = host
        self._port = port
        self._client = TCPClient(host=host, port=port, timeout=10.0, retry_count=3)

        # STUB: Replace with real device identifiers after discovery.
        self._device_name = "Physicochemical-Tester"
        self._firmware_version = "unknown"

    # ── BaseDeviceAdapter properties ────────────────────────────────

    @property
    def device_name(self) -> str:
        return self._device_name

    @property
    def device_type(self) -> str:
        return "physicochemical"

    @property
    def ip_address(self) -> str:
        return self._host

    @property
    def port(self) -> int:
        return self._port

    # ── Connection lifecycle ────────────────────────────────────────

    def connect(self) -> bool:
        try:
            self._client.connect()
            return True
        except (DeviceConnectionError, DeviceTimeoutError):
            return False

    def disconnect(self) -> None:
        self._client.disconnect()

    def is_connected(self) -> bool:
        return self._client.is_connected()

    # ── Command I/O ─────────────────────────────────────────────────

    def send_command(self, cmd: bytes) -> bytes:
        """Send a raw binary command and return the raw response."""
        return self._client.send(cmd)

    # ── Status & info ───────────────────────────────────────────────

    def get_device_info(self) -> Dict[str, Any]:
        """Return device metadata.

        ⚠️ STUB: Returns hard-coded identifiers.  Replace with a real
        discovery / handshake command once the API docs specify one.
        """
        return {
            "device_name": self._device_name,
            "device_type": self.device_type,
            "ip_address": self._host,
            "port": self._port,
            "firmware_version": self._firmware_version,
            # STUB: Replace when API docs available
            "serial_number": "unknown",
        }

    def get_status(self) -> str:
        """Determine whether the device is reachable.

        ⚠️ STUB: Uses a simple ping command.  The real status-check
        command (and its response format) must come from the API docs.
        """
        if not self.is_connected():
            return "offline"
        try:
            # STUB: Replace "ping" command with the real status query.
            self.send_command(b'{"cmd":"ping"}\n')
            return "online"
        except (DeviceError, OSError):
            return "error"

    # ── Physicochemical-specific operations ─────────────────────────

    def start_test(self, test_type: str, params: Optional[Dict[str, Any]] = None) -> bool:
        """Begin a physicochemical test of the specified type.

        Parameters
        ----------
        test_type : str
            Identifier for the test to run, e.g. ``"moisture"``,
            ``"protein"``, ``"fat"``.
        params : dict, optional
            Additional test parameters.

        Returns
        -------
        bool
            ``True`` if the device accepted the command.

        ⚠️ STUB: Sends ``{"cmd": "test", "test_type": ..., "params": {...}}``
        as JSON.  Replace with the real protocol format when API docs
        are available.
        """
        # STUB: Replace command format with actual protocol.
        payload: Dict[str, Any] = {
            "cmd": "test",
            "test_type": test_type,
        }
        if params:
            payload["params"] = params

        # STUB: Replace newline delimiter with actual message delimiter.
        cmd_bytes = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            response = self.send_command(cmd_bytes)
            # STUB: Replace response parsing with actual protocol logic.
            resp_data: Dict[str, Any] = json.loads(response.decode("utf-8").strip())
            return resp_data.get("status") == "accepted"
        except (json.JSONDecodeError, UnicodeDecodeError):
            # STUB: Fallback — raw bytes comparison.  Remove when
            # protocol is formalised.
            return b"accepted" in response
        except DeviceError:
            return False

    def get_result(self) -> Dict[str, Any]:
        """Retrieve the latest test results.

        Returns
        -------
        dict
            Test result data.  Returns an error dict on failure.

        ⚠️ STUB: Sends ``{"cmd": "get_result"}`` and parses the JSON
        response.  Replace with real protocol.
        """
        # STUB: Replace command and response format.
        payload = {"cmd": "get_result"}
        cmd_bytes = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        try:
            response = self.send_command(cmd_bytes)
            # STUB: Replace response parsing with actual protocol logic.
            return json.loads(response.decode("utf-8").strip())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            logger.warning("Failed to decode physicochemical result: %s", exc)
            return {"status": "error", "message": str(exc)}
        except DeviceError as exc:
            logger.warning("Physicochemical result retrieval failed: %s", exc)
            return {"status": "error", "message": str(exc)}
