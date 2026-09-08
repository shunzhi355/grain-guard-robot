"""Abstract base class and exceptions for external device adapters.

Defines the interface that every external-device adapter must implement.
Concrete adapters (biochemical analyser, physicochemical tester, etc.)
inherit from :class:`BaseDeviceAdapter` and handle protocol-specific details.
"""

from abc import ABC, abstractmethod
from typing import Optional


# ── Custom exception hierarchy ──────────────────────────────────────────


class DeviceError(Exception):
    """Base class for all device-level errors."""

    def __init__(self, message: str, device_name: Optional[str] = None) -> None:
        super().__init__(message)
        self.device_name = device_name


class DeviceConnectionError(DeviceError):
    """Raised when a network or socket-level connection fails."""


class DeviceTimeoutError(DeviceError):
    """Raised when a device operation times out."""


# ── Abstract adapter ────────────────────────────────────────────────────


class BaseDeviceAdapter(ABC):
    """Abstract interface for all external testing-device adapters.

    Each subclass represents one physical testing instrument connected
    via Ethernet / TCP.  The adapter owns a :class:`TCPClient` (or other
    transport) instance and translates high-level method calls into the
    device-specific wire protocol.

    Required properties
    -------------------
    ``device_name``
        Human-readable name, e.g. ``"BioChem-2000"``.
    ``device_type``
        Category string, e.g. ``"biochemical"`` or ``"physicochemical"``.
    ``ip_address``
        IPv4 address of the instrument.
    ``port``
        TCP port number.

    Required methods
    ----------------
    Each subclass **must** implement the abstract methods below.
    """

    # ── Properties (must be provided by subclasses) ─────────────────

    @property
    @abstractmethod
    def device_name(self) -> str:
        """Human-readable device identifier."""

    @property
    @abstractmethod
    def device_type(self) -> str:
        """Category label, e.g. ``"biochemical"``."""

    @property
    @abstractmethod
    def ip_address(self) -> str:
        """IPv4 address of the physical instrument."""

    @property
    @abstractmethod
    def port(self) -> int:
        """TCP port the instrument listens on."""

    # ── Abstract interface methods ──────────────────────────────────

    @abstractmethod
    def connect(self) -> bool:
        """Establish a TCP connection to the device.

        Returns
        -------
        bool
            ``True`` when the connection was opened successfully.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Gracefully close the TCP connection."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return ``True`` when the transport is connected and usable."""

    @abstractmethod
    def send_command(self, cmd: bytes) -> bytes:
        """Send a raw binary command and return the raw response.

        Parameters
        ----------
        cmd : bytes
            Command payload (protocol-specific).

        Returns
        -------
        bytes
            Raw response from the device.
        """

    @abstractmethod
    def get_device_info(self) -> dict:
        """Return device metadata as a dictionary.

        Typical keys: ``"device_name"``, ``"device_type"``,
        ``"firmware_version"``, ``"serial_number"``.
        """

    @abstractmethod
    def get_status(self) -> str:
        """Return the current operational status of the device.

        Returns
        -------
        str
            One of ``"online"``, ``"offline"``, or ``"error"``.
        """
