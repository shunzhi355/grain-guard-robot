"""Single-owner synchronous serial transport; callers serialize access with a lock."""
import secrets
import struct
import time

from . import chassis_protocol as p
from . import mechanism_protocol as mechanism
from .chassis_telemetry import ModeTelemetry

DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0"


class ChassisError(RuntimeError):
    pass


class ChassisSerial:
    def __init__(self, serial_port, timeout=0.12, clock=time.monotonic):
        self.serial = serial_port
        self.timeout = timeout
        self.clock = clock
        self.parser = p.Parser()
        self.session = secrets.randbelow(0xffffffff) + 1
        self.sequence = 0
        self.epoch = None
        self.boot = None
        self.status = None
        self.status_time = float("-inf")
        self.mode_telemetry = ModeTelemetry(clock)

    def poll_mode(self):
        """Read only buffered bytes, without HELLO, ACK waits or motion effects."""
        available = min(self.serial.in_waiting, 4096)
        if available > 0:
            self.mode_telemetry.feed(self.serial.read(available))
        return self.mode_telemetry.mode()

    def send(self, kind, payload=b""):
        self.sequence = (self.sequence + 1) & 0xffffffff
        frame = p.encode(kind, self.session, self.sequence, payload)
        if self.serial.write(frame) != len(frame):
            raise ChassisError("short serial write")
        return self.sequence

    def receive(self):
        frames = self.parser.feed(self.serial.read(max(1, min(self.serial.in_waiting, 4096))))
        accepted = []
        for frame in frames:
            if frame.session != self.session:
                continue
            if frame.kind == p.STATUS:
                if len(frame.payload) != 52:
                    continue
                status = p.decode_status(frame.payload)
                if self.boot is not None and status["boot"] != self.boot:
                    raise ChassisError("MCU restarted")
                self.status = status
                self.epoch = status["epoch"]
                self.status_time = self.clock()
            accepted.append(frame)
        return accepted

    def request(self, kind, payload=b""):
        sequence = self.send(kind, payload)
        deadline = self.clock() + self.timeout
        while self.clock() < deadline:
            for frame in self.receive():
                if kind == p.HELLO and frame.kind == p.HELLO_ACK and len(frame.payload) == 9:
                    self.boot, self.epoch, enabled = struct.unpack("<IIB", frame.payload)
                    if not enabled:
                        raise ChassisError("MCU PWM disabled")
                    return
                if frame.kind not in (p.ACK, p.NACK) or len(frame.payload) != 10:
                    continue
                request, reason, reply_sequence, epoch = struct.unpack("<BBII", frame.payload)
                if request != kind or reply_sequence != sequence:
                    continue
                self.epoch = epoch
                if frame.kind == p.NACK or reason:
                    raise ChassisError("MCU rejected request %d: reason %d" % (kind, reason))
                return
        raise ChassisError("MCU request %d timed out" % kind)

    def token_request(self, kind):
        if self.epoch is None:
            raise ChassisError("handshake required")
        self.request(kind, struct.pack("<I", self.epoch))

    def effort(self, forward, turn):
        """Stream a setpoint; a successful write is not execution confirmation."""
        if self.epoch is None:
            raise ChassisError("handshake required")
        return self.send(p.SET_EFFORT, struct.pack("<Ihh", self.epoch, forward, turn))

    def renew_session(self):
        """A fresh HELLO revokes previous motion before a new arm request."""
        self.session = self.session % 0xffffffff + 1
        self.sequence = 0
        self.epoch = None
        self.status = None
        self.status_time = float("-inf")
        self.request(p.HELLO)

    def stream_effort(self, forward, turn):
        if not all(isinstance(v, int) and -1000 <= v <= 1000 for v in (forward, turn)):
            raise ValueError("effort must be an integer in [-1000, 1000]")
        return self.send(p.STREAM_EFFORT, struct.pack("<hh", forward, turn))

    def stream_control(self, kind):
        if kind not in (p.AUTO_STOP, p.ESTOP, p.CLEAR_ESTOP, p.RECOVER):
            raise ValueError("unsupported stream control")
        return self.send(p.STREAM_CONTROL, bytes((kind,)))

    def mechanism_command(self, command, device):
        """Send one 18-byte action frame, sharing chassis session and sequence.

        Firmware has no mechanism ACK. Never wait, retry START, or send PWM/time.
        The owner must hold the same lock used for chassis writes.
        """
        return self.send(mechanism.FRAME_TYPE, mechanism.command_payload(command, device))

    def close(self):
        try:
            self.stream_control(p.AUTO_STOP)
        finally:
            self.serial.close()
