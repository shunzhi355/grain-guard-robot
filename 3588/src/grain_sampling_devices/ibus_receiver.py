"""FlySky i-BUS receiver input over a USB TTL serial adapter.

The Linux receiver factory selects this USB serial backend by default.
It implements the existing read/start/stop API and preserves channel mapping,
debouncing and the stale-signal timeout used by RCControl.

The receiver channels used by this project are mapped as follows::

    i-BUS CH3 -> CH1  (logical throttle: forward / reverse)
    i-BUS CH1 -> CH3  (logical steering: left / right)
    i-BUS CH8 -> CH5  (manual / automatic mode switch)

Typical i-BUS frames are 32 bytes, start with ``0x20 0x40``, contain fourteen
little-endian channel values, and end with a little-endian checksum.  Values
are normally pulse widths in microseconds (roughly 1000~2000 us).
"""

from __future__ import annotations

import logging
import threading
from typing import Dict, Mapping, Optional, Protocol

from .rc_receiver import _BaseRCReceiver

logger = logging.getLogger(__name__)

IBUS_BAUDRATE = 115200
IBUS_FRAME_LENGTH = 32
IBUS_HEADER = b"\x20\x40"
IBUS_CHANNEL_COUNT = 14


class SerialLike(Protocol):
    """Small serial interface used by the reader and by unit tests."""

    def read(self, size: int = 1) -> bytes: ...

    def close(self) -> None: ...


def parse_ibus_frame(frame: bytes) -> Optional[Dict[int, int]]:
    """Validate one i-BUS frame and return 1-based channel values.

    ``None`` is returned for an incomplete, malformed, or checksum-invalid
    frame.  Keeping parsing as a pure function makes it possible to verify
    protocol handling without hardware.
    """
    if len(frame) != IBUS_FRAME_LENGTH or frame[:2] != IBUS_HEADER:
        return None

    expected = (0xFFFF - (sum(frame[:-2]) & 0xFFFF)) & 0xFFFF
    received = int.from_bytes(frame[-2:], byteorder="little")
    if received != expected:
        return None

    return {
        channel: int.from_bytes(
            frame[2 + (channel - 1) * 2 : 4 + (channel - 1) * 2],
            byteorder="little",
        )
        for channel in range(1, IBUS_CHANNEL_COUNT + 1)
    }


def build_ibus_frame(values: Mapping[int, int]) -> bytes:
    """Build a valid frame for tests or serial replay tools.

    Missing channels are filled with the neutral value ``1500``.  This helper
    is not used by the receiver itself and does not transmit anything.
    """
    payload = bytearray(IBUS_HEADER)
    for channel in range(1, IBUS_CHANNEL_COUNT + 1):
        value = int(values.get(channel, 1500))
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"i-BUS channel {channel} value out of range: {value}")
        payload.extend(value.to_bytes(2, byteorder="little"))
    checksum = (0xFFFF - (sum(payload) & 0xFFFF)) & 0xFFFF
    payload.extend(checksum.to_bytes(2, byteorder="little"))
    return bytes(payload)


class IBusRCReceiver(_BaseRCReceiver):
    """Read the project's three RC channels from an i-BUS serial stream.

    Parameters are deliberately compatible with a USB TTL adapter.  The
    ``serial_port`` argument is useful for tests and for applications that
    already opened the device.  When it is omitted, ``pyserial`` is imported
    lazily and opens ``port``.

    Set ``RC_RECEIVER_BACKEND=ibus`` to require this backend, or leave the
    backend as ``auto`` to try i-BUS first and retain the legacy GPIO fallbacks.
    """

    # Keep the historical logical keys consumed by RCControl while mapping
    # them to the transmitter's physical i-BUS channels.
    DEFAULT_CHANNEL_MAP: Mapping[str, int] = {"CH1": 3, "CH3": 1, "CH5": 8}

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = IBUS_BAUDRATE,
        timeout: float = 0.05,
        *,
        serial_port: Optional[SerialLike] = None,
        channel_map: Optional[Mapping[str, int]] = None,
        auto_start: bool = True,
        **kwargs: object,
    ) -> None:
        super().__init__(**kwargs)
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.channel_map = dict(channel_map or self.DEFAULT_CHANNEL_MAP)
        if set(self.channel_map) != set(self.CHANNELS):
            raise ValueError(f"channel_map must contain exactly {self.CHANNELS}")
        if any(not 1 <= int(ch) <= IBUS_CHANNEL_COUNT for ch in self.channel_map.values()):
            raise ValueError(f"i-BUS channels must be in 1..{IBUS_CHANNEL_COUNT}")

        self._serial: Optional[SerialLike] = serial_port
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._available = serial_port is not None
        self._buffer = bytearray()

        if self._serial is None:
            self._open_serial()
        if auto_start:
            self.start()

    @property
    def is_available(self) -> bool:
        """Whether the serial device was opened successfully."""
        return self._available

    def _open_serial(self) -> None:
        try:
            import serial  # type: ignore[import-untyped]

            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
        except ImportError as exc:
            raise RuntimeError("IBusRCReceiver requires pyserial") from exc
        except Exception as exc:  # noqa: BLE001 - preserve a clear device error
            logger.error("failed to open i-BUS serial port %s: %s", self.port, exc)
            self._serial = None
            self._available = False
            raise
        self._available = True
        logger.info("i-BUS serial receiver opened on %s", self.port)

    def start(self) -> None:
        """Start the background serial reader (idempotent)."""
        if not self._available or self._serial is None:
            logger.warning("i-BUS receiver unavailable - not starting")
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name="ibus-rc-receiver",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the reader and close the serial device."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.timeout * 4.0))
            self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - shutdown is best effort
                logger.debug("failed to close i-BUS serial port", exc_info=True)
        self._available = False
        self._serial = None
        self._buffer.clear()
        self.reset()

    close = stop

    def _read_loop(self) -> None:
        assert self._serial is not None
        while not self._stop_event.is_set():
            try:
                chunk = self._serial.read(64)
            except (OSError, IOError) as exc:
                logger.error("i-BUS serial read failed: %s", exc)
                self._available = False
                self._buffer.clear()
                self.reset()
                try:
                    self._serial.close()
                finally:
                    self._serial = None
                return
            if chunk:
                self._buffer.extend(chunk)
                self._consume_frames()

    def _consume_frames(self) -> None:
        """Extract valid frames while tolerating dropped or extra bytes."""
        while True:
            start = self._buffer.find(IBUS_HEADER)
            if start < 0:
                # Preserve a possible first header byte split across reads.
                if self._buffer[-1:] == IBUS_HEADER[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return
            if start:
                del self._buffer[:start]
            if len(self._buffer) < IBUS_FRAME_LENGTH:
                return

            frame = bytes(self._buffer[:IBUS_FRAME_LENGTH])
            values = parse_ibus_frame(frame)
            if values is None:
                # Move one byte forward and search for the next possible frame.
                del self._buffer[:1]
                continue
            del self._buffer[:IBUS_FRAME_LENGTH]
            if any(not self._valid_min <= values[channel] <= self._valid_max
                   for channel in self.channel_map.values()):
                continue  # Drop the complete control sample, not just one axis.
            for name, channel in self.channel_map.items():
                self._push_sample(name, float(values[channel]))


__all__ = [
    "IBUS_BAUDRATE",
    "IBUS_FRAME_LENGTH",
    "IBusRCReceiver",
    "build_ibus_frame",
    "parse_ibus_frame",
]
