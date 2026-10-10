"""Full-duplex UART: one RX parser dispatches mode reports and mechanism replies."""
import secrets
import struct
import threading
import time
from contextlib import nullcontext

from . import chassis_protocol as p
from . import mechanism_protocol as mechanism
from .chassis_telemetry import ModeTelemetry

DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0"


class ChassisError(RuntimeError):
    pass


class MechanismReplyError(ChassisError):
    pass


class MechanismTimeoutError(MechanismReplyError):
    pass


class MechanismDisconnectedError(MechanismReplyError):
    pass


class MechanismRejectedError(MechanismReplyError):
    pass


class ChassisSerial:
    def __init__(self, serial_port, clock=time.monotonic, *, session=None, sequence=0):
        self.serial = serial_port
        self.session = session if session is not None else secrets.randbelow(0xffffffff) + 1
        self.sequence = sequence
        # Only writers serialize; RX never takes this lock.
        self._tx_lock = threading.Lock()
        self.mode_telemetry = ModeTelemetry(clock)
        self._parser = p.Parser()
        self._reply_cv = threading.Condition()
        self._pending_replies = {}
        self.clock = clock

    def poll_mode(self):
        """Only UART reader: dispatch STATUS and mechanism replies."""
        available = min(self.serial.in_waiting, 4096)
        if available > 0:
            for frame in self._parser.feed(self.serial.read(available)):
                self.mode_telemetry.accept(frame)
                if frame.kind != mechanism.REPLY_TYPE or len(frame.payload) != mechanism.REPLY_STRUCT.size:
                    continue
                session, sequence, command, device, result = mechanism.REPLY_STRUCT.unpack(frame.payload)
                if session != self.session:
                    continue
                with self._reply_cv:
                    pending = self._pending_replies.get(sequence)
                    if pending is not None and tuple(pending[:2]) == (command, device):
                        pending[2] = result
                        self._reply_cv.notify_all()
        return self.mode_telemetry.mode()

    def _write_locked(self, kind, payload):
        self.sequence = (self.sequence + 1) & 0xffffffff
        frame = p.encode(kind, self.session, self.sequence, payload)
        if self.serial.write(frame) != len(frame):
            raise ChassisError("short serial write")
        return self.sequence

    def send(self, kind, payload=b""):
        """Write a one-way chassis frame."""
        with self._tx_lock:
            return self._write_locked(kind, payload)

    def reserve_sequence(self):
        """Allocate once so transport retries can reuse the exact request ID."""
        with self._tx_lock:
            self.sequence = (self.sequence + 1) & 0xffffffff
            return self.sequence

    def stream_effort(self, forward, turn):
        if not all(isinstance(v, int) and -1000 <= v <= 1000 for v in (forward, turn)):
            raise ValueError("effort must be an integer in [-1000, 1000]")
        return self.send(p.STREAM_EFFORT, struct.pack("<hh", forward, turn))

    def stream_control(self, kind):
        if kind not in (p.AUTO_STOP, p.ESTOP, p.CLEAR_ESTOP, p.RECOVER):
            raise ValueError("unsupported stream control")
        return self.send(p.STREAM_CONTROL, bytes((kind,)))

    def mechanism_command(self, command, device, *, confirm=False, timeout=0.4,
                          sequence=None, write_guard=None):
        """Send once; caller can resend the same sequence after a lost reply."""
        payload = mechanism.command_payload(command, device)
        if not confirm:
            with (write_guard if write_guard is not None else nullcontext()):
                return self.send(mechanism.FRAME_TYPE, payload)
        # Optional supervisor guard covers only the write, never the ACK wait.
        # This orders a checked action before/after ESTOP without blocking RX.
        with (write_guard if write_guard is not None else nullcontext()), self._tx_lock:
            if sequence is None:
                self.sequence = (self.sequence + 1) & 0xffffffff
                sequence = self.sequence
            with self._reply_cv:
                self._pending_replies[sequence] = [command, device, None]
            try:
                frame = p.encode(mechanism.RELIABLE_FRAME_TYPE, self.session,
                                 sequence, payload)
                if self.serial.write(frame) != len(frame):
                    raise ChassisError("short serial write")
            except Exception:
                with self._reply_cv:
                    self._pending_replies.pop(sequence, None)
                raise
        deadline = self.clock() + timeout
        with self._reply_cv:
            try:
                while True:
                    pending = self._pending_replies[sequence]
                    if isinstance(pending[2], Exception):
                        raise pending[2]
                    if pending[2] is not None:
                        if pending[2] != 0:
                            raise MechanismRejectedError(
                                f"STM32 rejected mechanism command={command} device={device} reason={pending[2]}"
                            )
                        return sequence
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise MechanismTimeoutError(
                            f"STM32 mechanism reply timeout command={command} device={device} sequence={sequence}"
                        )
                    self._reply_cv.wait(remaining)
            finally:
                self._pending_replies.pop(sequence, None)

    def fail_pending(self, reason="serial disconnected"):
        with self._reply_cv:
            for pending in self._pending_replies.values():
                pending[2] = MechanismDisconnectedError(reason)
            self._reply_cv.notify_all()

    def close(self):
        self.fail_pending("serial closed")
        try:
            self.stream_control(p.AUTO_STOP)
        finally:
            self.serial.close()
