"""Passive MCU reports, independent of command sessions and acknowledgements."""
import threading
import time

from . import chassis_protocol as p


class ModeTelemetry:
    TIMEOUT = 1.0

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.parser = p.Parser()
        self._lock = threading.Lock()
        self.status = None
        self.sequence = None
        self.received_at = float("-inf")

    def feed(self, data):
        for frame in self.parser.feed(data):
            if frame.kind != p.STATUS or len(frame.payload) != p.STATUS_STRUCT.size:
                continue
            status = p.decode_status(frame.payload)
            # Unsolicited reports may have session=0, including in manual mode.
            # Only this passive STATUS path ignores the command session token.
            with self._lock:
                if self.status is not None and status["boot"] == self.status["boot"]:
                    delta = (frame.sequence - self.sequence) & 0xffffffff
                    if not 0 < delta < 0x80000000:
                        continue  # Replays cannot keep an old displayed mode alive.
                self.status = status
                self.sequence = frame.sequence
                self.received_at = self.clock()

    def snapshot(self):
        """Return one coherent report/time pair; no UART I/O or TX lock."""
        with self._lock:
            return self.status, self.received_at

    def mode(self):
        status, received_at = self.snapshot()
        if status is None or self.clock() - received_at >= self.TIMEOUT:
            return ""
        if not status["flags"] & 4 or status["rc_age_ms"] >= 300:
            return ""
        return {1: "manual", 2: "auto"}.get(status["mode"], "")
