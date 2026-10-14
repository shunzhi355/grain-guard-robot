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
        self.reports_seen = 0
        self.reports_accepted = 0
        self.last_seen = None
        self.last_seen_at = float("-inf")
        self.rejected_reports = []

    def feed(self, data):
        for frame in self.parser.feed(data):
            self.accept(frame)

    def accept(self, frame):
        if frame.kind != p.STATUS or len(frame.payload) != p.STATUS_STRUCT.size:
            return
        status = p.decode_status(frame.payload)
        # Unsolicited reports may have session=0, including in manual mode.
        # Only this passive STATUS path ignores the command session token.
        with self._lock:
            self.reports_seen += 1
            self.last_seen_at = self.clock()
            self.last_seen = {"session": frame.session, "sequence": frame.sequence,
                              "boot": status["boot"], "uptime_ms": status["uptime_ms"],
                              "mode": status["mode"], "flags": status["flags"],
                              "faults": status["faults"], "rc_age_ms": status["rc_age_ms"]}
            if self.status is not None and status["boot"] == self.status["boot"]:
                delta = (frame.sequence - self.sequence) & 0xffffffff
                if not 0 < delta < 0x80000000:
                    self.rejected_reports.append({**self.last_seen,
                        "reason": "duplicate_or_out_of_order_sequence",
                        "accepted_sequence": self.sequence, "delta": delta})
                    self.rejected_reports = self.rejected_reports[-8:]
                    return  # Replays cannot keep an old displayed mode alive.
            self.reports_accepted += 1
            self.status = status
            self.sequence = frame.sequence
            self.received_at = self.clock()

    def snapshot(self):
        """Return one coherent report/time pair; no UART I/O or TX lock."""
        with self._lock:
            return self.status, self.received_at

    def diagnostics(self):
        """Distinguish wire silence from rejected reports without refreshing health."""
        with self._lock:
            now = self.clock()
            return {"reports_seen": self.reports_seen,
                    "reports_accepted": self.reports_accepted,
                    "report_age_ms": None if self.status is None else round((now - self.received_at) * 1000),
                    "wire_report_age_ms": None if self.last_seen is None else round((now - self.last_seen_at) * 1000),
                    "last_wire_report": dict(self.last_seen) if self.last_seen else None,
                    "rejected_reports": [dict(v) for v in self.rejected_reports]}

    def mode(self):
        status, received_at = self.snapshot()
        if status is None or self.clock() - received_at >= self.TIMEOUT:
            return ""
        if not status["flags"] & 4 or status["rc_age_ms"] >= 300:
            return ""
        return {1: "manual", 2: "auto"}.get(status["mode"], "")
