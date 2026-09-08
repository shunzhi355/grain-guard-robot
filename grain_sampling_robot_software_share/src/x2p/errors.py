"""Domain-specific exceptions used by the controller."""


class X2PError(RuntimeError):
    """Base error for expected drive/controller failures."""


class CommunicationError(X2PError):
    """The Modbus transport or response was invalid."""


class ConfigurationError(X2PError):
    """Drive or local configuration is incompatible with the command."""


class SafetyInterlockError(X2PError):
    """A safety precondition was not met."""


class MotionTimeoutError(X2PError):
    """The drive did not reach the requested state before the deadline."""


class PositionNotReachedError(MotionTimeoutError):
    """An experimental position command did not reach its target."""
