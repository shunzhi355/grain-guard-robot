"""Wire format shared with dipan/MDK-ARM/chassis/chassis.c (protocol V1)."""
import binascii
import struct
from dataclasses import dataclass

HELLO, HELLO_ACK, AUTO_ARM, AUTO_DISARM, SET_EFFORT, AUTO_STOP, ESTOP, CLEAR_ESTOP, HEARTBEAT, STATUS, ACK, NACK, RECOVER = range(1, 14)
STREAM_EFFORT, STREAM_CONTROL = 14, 15
STATUS_STRUCT = struct.Struct("<IIIBBBBIIIHHHhhHHIIBB")
STATUS_KEYS = ("boot", "uptime_ms", "epoch", "state", "mode", "flags", "faults",
               "motion_sequence", "rc_age_ms", "motion_age_ms", "ch1", "ch3", "ch8",
               "left", "right", "pwm_left", "pwm_right", "crc_errors", "rejects",
               "pwm_enabled", "reserved")


@dataclass
class Frame:
    kind: int
    session: int
    sequence: int
    payload: bytes


def encode(kind, session, sequence, payload=b""):
    if len(payload) > 128:
        raise ValueError("payload exceeds MCU buffer")
    body = struct.pack("<BBHII", 1, kind, len(payload), session, sequence) + payload
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xffff))


def decode_status(payload):
    return dict(zip(STATUS_KEYS, STATUS_STRUCT.unpack(payload)))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bytes_received = 0
        self.crc_errors = 0
        self.header_errors = 0
        self.frames_received = 0
        self.last_crc_failure = None

    def feed(self, data):
        self.bytes_received += len(data)
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b"\xa5\x5a":
                del self.buffer[0]
                continue
            if len(self.buffer) < 6:
                break
            version, kind, size = struct.unpack_from("<BBH", self.buffer, 2)
            if version != 1 or size > 128:
                self.header_errors += 1
                del self.buffer[0]
                continue
            total = size + 16
            if len(self.buffer) < total:
                break
            crc = struct.unpack_from("<H", self.buffer, total - 2)[0]
            if binascii.crc_hqx(self.buffer[2:total-2], 0xffff) != crc:
                self.crc_errors += 1
                self.last_crc_failure = {"kind": kind, "payload_size": size,
                    "received_crc": crc,
                    "computed_crc": binascii.crc_hqx(self.buffer[2:total-2], 0xffff),
                    "frame_hex": bytes(self.buffer[:total]).hex()}
                del self.buffer[0]
                continue
            session, sequence = struct.unpack_from("<II", self.buffer, 6)
            self.frames_received += 1
            frames.append(Frame(kind, session, sequence, bytes(self.buffer[14:total-2])))
            del self.buffer[:total]
        return frames
