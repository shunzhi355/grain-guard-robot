"""Mechanism replies share UART RX with unsolicited RC mode STATUS."""
from concurrent.futures import ThreadPoolExecutor
import struct
import time

import pytest

from grain_sampling_devices import chassis_protocol as wire
from grain_sampling_devices import mechanism_protocol as mechanism
from grain_sampling_devices.chassis_serial import (
    ChassisSerial, MechanismReplyError, MechanismTimeoutError,
)
from grain_sampling_interhost.chassis_controller import ChassisController


class Port:
    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True

    def inject(self, frame):
        self.rx.extend(frame)


def wait_for_write(port):
    deadline = time.monotonic() + 1
    while not port.writes and time.monotonic() < deadline:
        time.sleep(0.001)
    assert port.writes
    return wire.Parser().feed(port.writes[0])[0]


def reply(request, result=0, *, session=None, sequence=None, command=None, device=None):
    cmd, dev = request.payload
    payload = mechanism.REPLY_STRUCT.pack(
        request.session if session is None else session,
        request.sequence if sequence is None else sequence,
        cmd if command is None else command,
        dev if device is None else device,
        result,
    )
    return wire.encode(mechanism.REPLY_TYPE, 0, 1, payload)


def status():
    payload = bytearray(wire.STATUS_STRUCT.size)
    struct.pack_into("<I", payload, 0, 42)
    payload[13:15] = b"\x02\x04"  # auto, RC valid
    return wire.encode(wire.STATUS, 0, 1, payload)


def test_status_and_reply_interleave_without_stealing_each_other():
    port = Port()
    link = ChassisSerial(port)
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(link.mechanism_command, mechanism.START,
                              mechanism.BIN_SHALLOW, confirm=True)
        request = wait_for_write(port)
        port.inject(status() + reply(request, session=request.session + 1))
        assert link.poll_mode() == "auto"
        assert not pending.done()
        port.inject(reply(request, sequence=request.sequence + 1))
        port.inject(reply(request, device=mechanism.BIN_MID))
        link.poll_mode()
        assert not pending.done()
        packet = reply(request)
        port.inject(packet[:9])
        link.poll_mode()
        assert not pending.done()
        port.inject(packet[9:])
        link.poll_mode()
        assert pending.result(timeout=1) == request.sequence
    assert len(port.writes) == 1
    assert link.mode_telemetry.mode() == "auto"


def test_mcu_rejection_is_failure_without_retransmission():
    port = Port()
    link = ChassisSerial(port)
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(link.mechanism_command, mechanism.START,
                              mechanism.CONVEY, confirm=True)
        request = wait_for_write(port)
        port.inject(reply(request, result=6))
        link.poll_mode()
        with pytest.raises(MechanismReplyError, match="rejected"):
            pending.result(timeout=1)
    assert len(port.writes) == 1


def test_missing_reply_retries_same_frame_before_failing(monkeypatch):
    from grain_sampling_interhost import chassis_controller as module
    monkeypatch.setattr(module, "MECHANISM_RETRY_WINDOW_S", 0.9)
    port = Port()
    controller = ChassisController()
    controller.link = ChassisSerial(port)
    controller.mode = "auto"
    with pytest.raises(MechanismReplyError, match="timeout"):
        controller.mechanism_command(mechanism.START, mechanism.CONVEY)
    frames = wire.Parser().feed(b"".join(port.writes))
    starts = [f for f in frames if f.kind == mechanism.RELIABLE_FRAME_TYPE]
    assert len(starts) >= 2
    assert all((f.session, f.sequence, f.payload) ==
               (starts[0].session, starts[0].sequence, b"\x01\x02") for f in starts)
    assert frames[-1].kind == mechanism.FRAME_TYPE
    assert frames[-1].payload == b"\x03\x00"


def test_same_request_can_be_resent_after_lost_reply():
    port = Port()
    link = ChassisSerial(port)
    with pytest.raises(MechanismTimeoutError):
        link.mechanism_command(mechanism.START, mechanism.CONVEY,
                               confirm=True, timeout=0.03)
    original = wire.Parser().feed(port.writes[0])[0]
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(link.mechanism_command, mechanism.START,
                              mechanism.CONVEY, confirm=True, sequence=original.sequence)
        deadline = time.monotonic() + 1
        while len(port.writes) < 2 and time.monotonic() < deadline:
            time.sleep(0.001)
        assert port.writes[0] == port.writes[1]
        port.inject(reply(original))
        link.poll_mode()
        assert pending.result(timeout=1) == original.sequence
