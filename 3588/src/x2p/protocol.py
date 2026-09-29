"""Small Modbus RTU transport with X2P word-order helpers."""

from __future__ import annotations

import logging
import os
import struct
import threading
import time
from typing import Protocol

import serial

from .errors import CommunicationError

logger = logging.getLogger(__name__)

# RS485 收发调试日志：设环境变量 X2P_DEBUG=1 启用（打印 TX/RX 字节 + CRC 结果），
# 用于诊断偶发超时/CRC 损坏/EIO 掉线等通信问题。
if os.environ.get("X2P_DEBUG"):
    _dbg = logging.StreamHandler()
    _dbg.setFormatter(
        logging.Formatter("%(asctime)s %(name)s %(levelname)s: %(message)s")
    )
    logger.addHandler(_dbg)
    logger.setLevel(logging.DEBUG)
    logger.propagate = False


class SerialPort(Protocol):
    def reset_input_buffer(self) -> None: ...
    def write(self, data: bytes) -> int | None: ...
    def flush(self) -> None: ...
    def read(self, size: int) -> bytes: ...
    def close(self) -> None: ...


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def add_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16(payload))


def signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


class ModbusRTUClient:
    """Modbus client：对安全可重放交易重试 1 次，抗超时/CRC 损坏。

    普通配置写入可重试；Pn701 在内部位置模式 7 下是立即执行命令，
    因此非零 Pn701 写入绝不自动重放，避免丢应答时发生二次运动。
    """

    def __init__(
        self,
        port: str | None = None,
        slave: int = 2,
        timeout: float = 1.0,  # USB-RS485 occasional slow ack (2026-09, was 0.5)
        *,
        serial_port: SerialPort | None = None,
        min_request_interval_s: float = 0.05,
    ):
        if not 1 <= slave <= 247:
            raise ValueError("slave必须在1..247之间")
        self.slave = slave
        if serial_port is not None:
            self.serial = serial_port
        elif port is not None:
            self.serial = serial.Serial(
                port=port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
                write_timeout=timeout,
            )
        else:
            raise ValueError("port和serial_port至少提供一个")
        if min_request_interval_s < 0:
            raise ValueError("min_request_interval_s不能小于0")
        # X2P at 9600 baud becomes unreliable when monitor reads are sent
        # back-to-back.  Serialize callers and leave a quiet interval between
        # complete RTU transactions; this also prevents two ROS callbacks from
        # interleaving frames on the same RS485 adapter.
        self.min_request_interval_s = float(min_request_interval_s)
        self._last_exchange_end = 0.0
        self._io_lock = threading.Lock()

    def __enter__(self) -> "ModbusRTUClient":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        self.serial.close()

    def _exchange(self, function: int, body: bytes, response_length: int) -> bytes:
        with self._io_lock:
            return self._exchange_locked(function, body, response_length)

    def _exchange_locked(
        self, function: int, body: bytes, response_length: int
    ) -> bytes:
        request = add_crc(bytes((self.slave, function)) + body)
        logger.debug("[TX] %s", request.hex(" "))
        # 读/写均重试 1 次，且 CRC 校验失败也纳入重试（RS485 偶发损坏）。
        # 实测 3 次联调各撞一次：超时(写)/CRC(写)/CRC(读)，需重试兜底。
        position_execute_write = (
            function == 0x06
            and len(body) >= 4
            and int.from_bytes(body[:2], "big") == 0x0701
            and int.from_bytes(body[2:4], "big") != 0
        )
        attempts = 1 if position_execute_write else 2
        response = b""
        crc_ok = False
        for attempt in range(attempts):
            remaining = (
                self.min_request_interval_s
                - (time.monotonic() - self._last_exchange_end)
            )
            if remaining > 0:
                time.sleep(remaining)
            try:
                self.serial.reset_input_buffer()
                self.serial.write(request)
                self.serial.flush()
                response = self.serial.read(response_length)
            except OSError as exc:
                # EIO 等串口/USB 掉线错误：记录 TX 上下文后原样抛出
                logger.error(
                    "[IO错误 attempt %d/%d] %r (TX=%s)",
                    attempt + 1,
                    attempts,
                    exc,
                    request.hex(" "),
                )
                raise
            finally:
                self._last_exchange_end = time.monotonic()
            crc_ok = (
                len(response) >= 5
                and crc16(response[:-2]) == int.from_bytes(response[-2:], "little")
            )
            logger.debug(
                "[RX %d/%d] %s crc_ok=%s",
                attempt + 1,
                attempts,
                response.hex(" "),
                crc_ok,
            )
            # During sustained reciprocation the X2P occasionally returns
            # exception 0x03 for an otherwise valid monitor read or idempotent
            # configuration write.  Retry those transactions after a quiet
            # interval.  Pn701=non-zero executes motion immediately and is
            # explicitly excluded above (attempts=1), so motion is never
            # triggered twice by this recovery path.
            transient_busy_exception = (
                crc_ok
                and len(response) >= 5
                and response[1] == (function | 0x80)
                and response[2] == 0x03
                and not position_execute_write
            )
            if transient_busy_exception and attempt + 1 < attempts:
                logger.warning(
                    "X2P交易暂时返回0x03，延时后重试"
                )
                time.sleep(0.10)
                continue
            if crc_ok:
                break
            if attempt + 1 < attempts:
                time.sleep(0.05)
        if len(response) < 5:
            raise CommunicationError(
                f"通信超时或应答过短: {response.hex(' ')}"
            )
        if not crc_ok:
            raise CommunicationError(f"CRC错误: {response.hex(' ')}")
        if response[0] != self.slave:
            raise CommunicationError(f"从机地址错误: {response[0]}")
        if response[1] == (function | 0x80):
            raise CommunicationError(f"Modbus异常响应: 0x{response[2]:02X}")
        if response[1] != function:
            raise CommunicationError(f"功能码错误: 0x{response[1]:02X}")
        if len(response) != response_length:
            raise CommunicationError(
                f"应答长度错误: 期望{response_length}，实际{len(response)}"
            )
        return response

    def diagnostic(self, value: int = 0x51A3) -> None:
        body = struct.pack(">HH", 0, value)
        response = self._exchange(0x08, body, 8)
        if response[2:-2] != body:
            raise CommunicationError("0x08线路诊断回显不一致")

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        if not 1 <= count <= 125:
            raise ValueError("读取寄存器数量必须在1..125之间")
        response = self._exchange(
            0x03, struct.pack(">HH", address, count), 5 + count * 2
        )
        if response[2] != count * 2:
            raise CommunicationError("读取寄存器的字节数不匹配")
        return list(struct.unpack(f">{count}H", response[3:-2]))

    def write_register(self, address: int, value: int) -> None:
        body = struct.pack(">HH", address, value & 0xFFFF)
        response = self._exchange(0x06, body, 8)
        if response[2:-2] != body:
            raise CommunicationError("写单寄存器回显不一致")

    def write_registers(self, address: int, values: list[int]) -> None:
        if not 1 <= len(values) <= 8:
            raise ValueError("一次只能写1至8个寄存器")
        encoded = struct.pack(f">{len(values)}H", *(v & 0xFFFF for v in values))
        body = struct.pack(">HHB", address, len(values), len(encoded)) + encoded
        response = self._exchange(0x10, body, 8)
        if response[2:-2] != struct.pack(">HH", address, len(values)):
            raise CommunicationError("写多寄存器回显不一致")

    def read_signed32(self, address: int) -> int:
        low, high = self.read_registers(address, 2)
        value = (high << 16) | low
        return value - 0x100000000 if value & 0x80000000 else value

    def write_signed32(self, address: int, value: int) -> None:
        encoded = value & 0xFFFFFFFF
        self.write_registers(address, [encoded & 0xFFFF, encoded >> 16])
