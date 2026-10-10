"""External device adapter framework for biochemical and physicochemical
testing equipment communicating via Ethernet / TCP.

Public API
----------
- :class:`BaseDeviceAdapter` — abstract base class for all device adapters
- :class:`TCPClient` — thread-safe TCP transport with auto-reconnect
- :class:`BiochemicalAdapter` — stub adapter for biochemical analysers
- :class:`PhysicochemicalAdapter` — stub adapter for physicochemical testers
- :class:`RCReceiver` — RC receiver GPIO pulse-width reader (CH1/CH3/CH5)
- :class:`MockRCReceiver` — mock RC receiver for Windows / tests
- :func:`create_rc_receiver` — pick the real or mock receiver automatically

Exceptions
----------
- :class:`DeviceError`
- :class:`DeviceConnectionError`
- :class:`DeviceTimeoutError`
"""

from grain_sampling_devices.base_adapter import (
    BaseDeviceAdapter,
    DeviceConnectionError,
    DeviceError,
    DeviceTimeoutError,
)
from grain_sampling_devices.tcp_client import TCPClient
from grain_sampling_devices.biochemical_adapter import BiochemicalAdapter
from grain_sampling_devices.physicochemical_adapter import PhysicochemicalAdapter
from grain_sampling_devices.rc_receiver import (
    MockRCReceiver,
    RCReceiver,
    create_rc_receiver,
)

__all__ = [
    "BaseDeviceAdapter",
    "BiochemicalAdapter",
    "DeviceConnectionError",
    "DeviceError",
    "DeviceTimeoutError",
    "MockRCReceiver",
    "PhysicochemicalAdapter",
    "RCReceiver",
    "TCPClient",
    "create_rc_receiver",
]
