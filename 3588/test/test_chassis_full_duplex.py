"""Exercise independent UART directions through the production controller."""
from concurrent.futures import ThreadPoolExecutor
import struct
import threading
import time

import pytest

from grain_sampling_devices import chassis_protocol as wire
from grain_sampling_devices.chassis_serial import ChassisSerial, ChassisError
from grain_sampling_interhost.chassis_controller import ChassisController


def report(mode=2, flags=4, sequence=1):
    payload = bytearray(wire.STATUS_STRUCT.size)
    struct.pack_into("<I", payload, 0, 42)
    payload[13:15] = bytes((mode, flags))
    return wire.encode(wire.STATUS, 0, sequence, payload)


def eventually(predicate):
    deadline = time.monotonic() + 1
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert predicate()


class DuplexPort:
    """No command replies. Only explicitly injected periodic reports reach RX."""
    def __init__(self):
        self.rx = bytearray()
        self.writes = []
        self.closed = False
        self.rx_lock = threading.Lock()

    @property
    def in_waiting(self):
        with self.rx_lock:
            return len(self.rx)

    def inject(self, packet):
        with self.rx_lock:
            self.rx.extend(packet)

    def read(self, size):
        with self.rx_lock:
            packet = bytes(self.rx[:size])
            del self.rx[:size]
            return packet

    def write(self, packet):
        self.writes.append(packet)
        return len(packet)

    def close(self):
        self.closed = True

    def frames(self):
        return wire.Parser().feed(b"".join(self.writes))


def connected_controller():
    port = DuplexPort()
    link = ChassisSerial(port)
    port.inject(report())
    assert link.poll_mode() == "auto"
    controller = ChassisController()
    controller.link = link
    controller.mode = "auto"
    return controller, port


def test_pending_rx_does_not_block_mechanism_tx():
    controller, port = connected_controller()
    entered, release = threading.Event(), threading.Event()
    read = port.read

    def delayed_read(size):
        entered.set()
        assert release.wait(2)
        return read(size)

    port.read = delayed_read
    port.inject(report(sequence=2))
    controller.start()
    try:
        assert entered.wait(1)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(controller.mechanism_command, 1, 6)
            try:
                future.result(timeout=0.3)
                assert [(f.kind, f.payload) for f in port.frames()] == [(0x36, b"\x01\x06")]
            finally:
                release.set()
    finally:
        release.set()
        controller.close()
    assert not controller.worker.is_alive() and not controller.receiver.is_alive()


def test_pending_tx_does_not_block_ch8_rx_and_manual_blocks_next_start():
    controller, port = connected_controller()
    entered, release = threading.Event(), threading.Event()
    write = port.write

    def delayed_write(packet):
        frame = wire.Parser().feed(packet)[0]
        if frame.kind == 0x36 and frame.payload == b"\x01\x06":
            entered.set()
            assert release.wait(2)
        return write(packet)

    port.write = delayed_write
    controller.start()
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(controller.mechanism_command, 1, 6)
            try:
                assert entered.wait(1)
                port.inject(report(mode=1, sequence=2))
                eventually(lambda: controller.link.mode_telemetry.mode() == "manual")
                assert not future.done()
            finally:
                release.set()
            future.result(timeout=1)
        eventually(lambda: controller.status()["rc_mode"] == "manual")
        with pytest.raises(RuntimeError, match="auto mode"):
            controller.mechanism_command(1, 2)
    finally:
        release.set()
        controller.close()


def test_reset_returns_without_reply_and_only_new_status_can_relatch():
    controller, port = connected_controller()
    port.inject(report(flags=6, sequence=2))
    controller.link.poll_mode()
    controller.estop_latched = True
    # No receiver or worker running: any confirmation wait would fail.
    controller.clear_estop()
    assert not controller.estop_latched and controller.epoch is None
    assert [f.payload for f in port.frames()] == [bytes((wire.AUTO_STOP,)), bytes((wire.CLEAR_ESTOP,))]
    controller.start()
    try:
        time.sleep(0.1)
        assert not controller.estop_latched  # Cached pre-reset report ignored.
        port.inject(report(flags=6, sequence=3))
        eventually(lambda: controller.estop_latched)
        assert controller.status()["execution_confirmed"] is False
    finally:
        controller.close()


@pytest.mark.parametrize("failure", ["short", "io"])
def test_reset_write_failure_keeps_latch_and_disconnects(failure):
    controller, port = connected_controller()
    write = port.write

    def failed_reset(packet):
        frame = wire.Parser().feed(packet)[0]
        if frame.payload == bytes((wire.CLEAR_ESTOP,)):
            if failure == "io":
                raise OSError("serial disconnected")
            return 1
        return write(packet)

    port.write = failed_reset
    controller.estop_latched = True
    with pytest.raises((ChassisError, OSError)):
        controller.clear_estop()
    assert controller.estop_latched and controller.link is None and port.closed


def test_receive_io_error_reconnects_without_replaying_mechanism_start():
    controller, old = connected_controller()
    controller.mechanism_command(1, 2)
    replacement = DuplexPort()
    controller.serial_factory = lambda: replacement

    def failed_read(size):
        raise OSError("USB disconnected")

    old.read = failed_read
    old.inject(report(sequence=2))
    controller.start()
    try:
        eventually(lambda: controller.link is not None and controller.link.serial is replacement)
        assert old.closed and controller.epoch is None
        assert [(f.kind, f.payload) for f in replacement.frames()] == [
            (wire.STREAM_CONTROL, bytes((wire.AUTO_STOP,))), (0x36, b"\x03\x00")]
    finally:
        controller.close()


def test_late_read_failure_from_old_handle_does_not_disconnect_replacement():
    controller, old = connected_controller()
    entered, release = threading.Event(), threading.Event()
    replacement = DuplexPort()

    def closed_handle_read(size):
        entered.set()
        assert release.wait(2)
        # A concurrent close can leave pyserial's POSIX fd as None.
        raise TypeError("file descriptor is None")

    old.read = closed_handle_read
    old.inject(report(sequence=2))
    controller.start()
    try:
        assert entered.wait(1)
        with controller.lock:
            controller._disconnect_locked()
            controller.link = ChassisSerial(replacement)
        replacement.inject(report(mode=1))
        release.set()
        eventually(lambda: controller.status()["rc_mode"] == "manual")
        assert controller.receiver.is_alive()
        assert controller.link.serial is replacement and not replacement.closed
    finally:
        release.set()
        controller.close()


def test_failed_stop_write_cannot_create_new_motion_authorization():
    controller, port = connected_controller()
    port.write = lambda packet: 1
    with pytest.raises(RuntimeError, match="stop write failed"):
        controller.arm("goal-1")
    assert controller.link is None and controller.epoch is None
