"""GRICP v1 wire format shared by the 3588 server and Lenovo client.

This module has no ROS or hardware dependency.  The protocol is specified in
docs/LENOVO_LPA3588_COMM_PROTOCOL.md.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import socket
import struct
import time
from dataclasses import dataclass
from enum import IntEnum

MAGIC = b"GLC1"
VERSION = (1, 0)
HEADER = struct.Struct("!4sBBHHHIIIQ")
MOTION = struct.Struct("!IiiiHBB")
STOP = struct.Struct("!II")
HEADER_SIZE = 32
AUTH_TAG_SIZE = 16
AUTH_TAG_FLAG = 1 << 5
TCP_MAX_PAYLOAD = 1024 * 1024
UDP_MAX_FRAME = 256


class MessageType(IntEnum):
    HELLO = 0x0001
    HELLO_ACK = 0x0002
    HEARTBEAT = 0x0003
    ERROR_RESPONSE = 0x0004
    NAV_GOAL_REQUEST = 0x0100
    NAV_GOAL_RESPONSE = 0x0101
    NAV_CANCEL = 0x0102
    NAV_STATUS = 0x0103
    NAV_RESULT = 0x0104
    MOTION_ARM_REQUEST = 0x0110
    MOTION_ARM_RESPONSE = 0x0111
    MOTION_COMMAND = 0x0120
    MOTION_STOP = 0x0121
    MOTION_EVENT = 0x0122
    SLAM_COMMAND = 0x0200
    SLAM_RESPONSE = 0x0201
    SLAM_STATUS = 0x0202
    MAP_LIST_REQUEST = 0x0210
    MAP_LIST_RESPONSE = 0x0211
    ROBOT_STATUS = 0x0300
    SAFETY_EVENT = 0x0301
    POSE_STATUS = 0x0310
    OBSTACLE_STATUS = 0x0320


class ProtocolError(ValueError):
    """A frame failed validation; the caller must not act on it."""


@dataclass(frozen=True)
class Frame:
    kind: MessageType
    session_id: int
    sequence: int
    timestamp_ms: int
    payload: bytes
    flags: int = 0

    def json(self) -> dict:
        try:
            value = json.loads(self.payload.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise ProtocolError("invalid JSON") from exc
        if not isinstance(value, dict):
            raise ProtocolError("JSON payload must be an object")
        return value


def crc32c(data: bytes) -> int:
    """CRC-32C/Castagnoli, reflected polynomial 0x82F63B78."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def json_payload(value: dict) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def encode(kind: MessageType, session_id: int, sequence: int, payload: bytes = b"",
           *, timestamp_ms: int | None = None, flags: int = 0,
           key: bytes | None = None) -> bytes:
    if not isinstance(payload, bytes) or len(payload) > TCP_MAX_PAYLOAD:
        raise ProtocolError("invalid payload length")
    if not 0 <= session_id <= 0xFFFFFFFF or not 0 <= sequence <= 0xFFFFFFFF:
        raise ProtocolError("invalid session or sequence")
    if key is not None:
        if len(key) != 32:
            raise ProtocolError("UDP key must contain 32 bytes")
        flags |= AUTH_TAG_FLAG
    elif flags & AUTH_TAG_FLAG:
        raise ProtocolError("AUTH_TAG flag requires a key")
    stamp = int(time.time() * 1000) if timestamp_ms is None else timestamp_ms
    header = HEADER.pack(MAGIC, *VERSION, int(kind), flags, HEADER_SIZE,
                         len(payload), session_id, sequence, stamp)
    body = header + payload
    body += struct.pack("!I", crc32c(body[4:]))
    if key is not None:
        body += hmac.new(key, body, hashlib.sha256).digest()[:AUTH_TAG_SIZE]
    return body


def decode(raw: bytes, *, key: bytes | None = None, udp: bool = False) -> Frame:
    if len(raw) < HEADER_SIZE + 4:
        raise ProtocolError("short frame")
    magic, major, minor, kind, flags, header_len, length, session, seq, stamp = HEADER.unpack_from(raw)
    if magic != MAGIC or (major, minor) != VERSION or header_len != HEADER_SIZE:
        raise ProtocolError("invalid header")
    if flags & ~0x3F:
        raise ProtocolError("reserved flags set")
    authenticated = bool(flags & AUTH_TAG_FLAG)
    if udp != authenticated or (udp and (key is None or len(key) != 32)):
        raise ProtocolError("invalid authentication mode")
    if length > (UDP_MAX_FRAME if udp else TCP_MAX_PAYLOAD):
        raise ProtocolError("payload too large")
    expected = HEADER_SIZE + length + 4 + (AUTH_TAG_SIZE if udp else 0)
    if len(raw) != expected or (udp and len(raw) > UDP_MAX_FRAME):
        raise ProtocolError("frame length mismatch")
    received_crc = struct.unpack_from("!I", raw, HEADER_SIZE + length)[0]
    if crc32c(raw[4:HEADER_SIZE + length]) != received_crc:
        raise ProtocolError("CRC mismatch")
    if udp:
        assert key is not None
        tag = hmac.new(key, raw[:-AUTH_TAG_SIZE], hashlib.sha256).digest()[:AUTH_TAG_SIZE]
        if not hmac.compare_digest(raw[-AUTH_TAG_SIZE:], tag):
            raise ProtocolError("HMAC mismatch")
    try:
        message_type = MessageType(kind)
    except ValueError as exc:
        raise ProtocolError("unknown message type") from exc
    if udp and message_type not in (MessageType.MOTION_COMMAND, MessageType.MOTION_STOP):
        raise ProtocolError("message type not allowed on UDP")
    return Frame(message_type, session, seq, stamp, raw[HEADER_SIZE:HEADER_SIZE + length], flags)


def recv_exact(sock: socket.socket, length: int) -> bytes:
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            raise ConnectionError("peer closed connection")
        data.extend(chunk)
    return bytes(data)


def recv_tcp(sock: socket.socket) -> Frame:
    header = recv_exact(sock, HEADER_SIZE)
    try:
        magic, major, minor, _, flags, header_len, length, _, _, _ = HEADER.unpack(header)
    except struct.error as exc:
        raise ProtocolError("bad TCP header") from exc
    if magic != MAGIC or (major, minor) != VERSION or header_len != HEADER_SIZE or length > TCP_MAX_PAYLOAD or flags & AUTH_TAG_FLAG:
        raise ProtocolError("bad TCP header")
    return decode(header + recv_exact(sock, length + 4))


def decode_motion(payload: bytes) -> tuple[int, int, int, int, int]:
    if len(payload) != MOTION.size:
        raise ProtocolError("bad motion payload length")
    epoch, vx, vy, wz, validity, source, reserved = MOTION.unpack(payload)
    if not epoch or not -300 <= vx <= 300 or vy != 0 or not -800 <= wz <= 800:
        raise ProtocolError("motion outside chassis limits")
    if not 50 <= validity <= 150 or source != 1 or reserved != 0:
        raise ProtocolError("invalid motion metadata")
    return epoch, vx, vy, wz, validity
