"""Full-duplex UART: send commands once and passively receive mode reports."""
import secrets
import struct
import threading
import time

from . import chassis_protocol as p
from . import mechanism_protocol as mechanism
from .chassis_telemetry import ModeTelemetry

DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB3N91KX-if00-port0"


class ChassisError(RuntimeError):
    pass


class ChassisSerial:
    def __init__(self, serial_port, clock=time.monotonic):
        self.serial = serial_port
        self.session = secrets.randbelow(0xffffffff) + 1
        self.sequence = 0
        # Only writers serialize; RX never takes this lock.
        self._tx_lock = threading.Lock()
        self.mode_telemetry = ModeTelemetry(clock)

    def poll_mode(self):
        """Read only buffered bytes, without HELLO, ACK waits or motion effects."""
        available = min(self.serial.in_waiting, 4096)
        if available > 0:
            self.mode_telemetry.feed(self.serial.read(available))
        return self.mode_telemetry.mode()

    def send(self, kind, payload=b""):
        """A successful write means sent, never execution confirmed."""
        with self._tx_lock:
            self.sequence = (self.sequence + 1) & 0xffffffff
            frame = p.encode(kind, self.session, self.sequence, payload)
            if self.serial.write(frame) != len(frame):
                raise ChassisError("short serial write")
            return self.sequence

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
        The TX lock keeps mechanism and chassis frames from interleaving.
        """
        return self.send(mechanism.FRAME_TYPE, mechanism.command_payload(command, device))

    def close(self):
        try:
            self.stream_control(p.AUTO_STOP)
        finally:
            self.serial.close()
