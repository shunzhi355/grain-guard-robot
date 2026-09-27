"""Modbus RTU transport regression tests for X2P field failures."""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType

try:  # pragma: no cover - depends on the developer environment
    import serial  # noqa: F401
except ImportError:  # pragma: no cover
    sys.modules["serial"] = ModuleType("serial")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from x2p.errors import CommunicationError  # noqa: E402
from x2p.protocol import ModbusRTUClient, add_crc  # noqa: E402


class _ScriptedSerial:
    def __init__(self, responses: list[bytes]):
        self.responses = list(responses)
        self.writes: list[bytes] = []

    def reset_input_buffer(self):
        return None

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        return None

    def read(self, _size):
        return self.responses.pop(0)

    def close(self):
        return None


def test_read_retries_transient_x2p_exception_03():
    port = _ScriptedSerial(
        [
            add_crc(bytes((2, 0x83, 0x03))),
            add_crc(bytes((2, 0x03, 0x02, 0x12, 0x34))),
        ]
    )
    client = ModbusRTUClient(
        slave=2, serial_port=port, min_request_interval_s=0
    )

    assert client.read_registers(0x2000) == [0x1234]
    assert len(port.writes) == 2


def test_safe_config_write_retries_transient_x2p_exception_03():
    port = _ScriptedSerial(
        [
            add_crc(bytes((2, 0x86, 0x03))),
            add_crc(bytes((2, 0x06, 0x04, 0x0F, 0x00, 0x00))),
        ]
    )
    client = ModbusRTUClient(
        slave=2, serial_port=port, min_request_interval_s=0
    )

    client.write_register(0x040F, 0)
    assert len(port.writes) == 2


def test_write_does_not_retry_non_transient_modbus_exception():
    port = _ScriptedSerial([add_crc(bytes((2, 0x86, 0x02)))])
    client = ModbusRTUClient(
        slave=2, serial_port=port, min_request_interval_s=0
    )

    try:
        client.write_register(0x0600, 1)
    except CommunicationError as exc:
        assert "0x02" in str(exc)
    else:  # pragma: no cover - assertion branch
        raise AssertionError("写异常不应被吞掉")
    assert len(port.writes) == 1


def test_position_execute_write_is_never_replayed_after_bad_response():
    # Pn701=1 executes motion immediately in mode 7.  If its acknowledgement
    # is damaged, replaying the request could start a second move.
    port = _ScriptedSerial([b"\x02\x06\x07\x01\x00\x01\x00\x00"])
    client = ModbusRTUClient(
        slave=2, serial_port=port, min_request_interval_s=0
    )

    try:
        client.write_register(0x0701, 1)
    except CommunicationError:
        pass
    else:  # pragma: no cover - assertion branch
        raise AssertionError("损坏应答必须报错")
    assert len(port.writes) == 1


def test_position_execute_write_is_not_replayed_after_exception_03():
    port = _ScriptedSerial([add_crc(bytes((2, 0x86, 0x03)))])
    client = ModbusRTUClient(
        slave=2, serial_port=port, min_request_interval_s=0
    )

    try:
        client.write_register(0x0701, 1)
    except CommunicationError as exc:
        assert "0x03" in str(exc)
    else:  # pragma: no cover - assertion branch
        raise AssertionError("位置执行命令异常必须报错")
    assert len(port.writes) == 1
