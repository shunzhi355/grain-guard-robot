"""Direct manual-RC bridge to the chassis motor daemon.

Legacy UDP control uses independent left/right track commands. Production
manual control now runs on STM32 and does not use PCA9685 outputs.
"""

from __future__ import annotations

import logging
import socket
from typing import Optional

from utils.sampling_params import RC_MAX_ANGULAR_RPS, RC_MAX_LINEAR_MPS


logger = logging.getLogger(__name__)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def rc_cmd_vel_to_tracks(
    linear_mps: float,
    angular_rps: float,
    *,
    max_linear_mps: float = RC_MAX_LINEAR_MPS,
    max_angular_rps: float = RC_MAX_ANGULAR_RPS,
) -> tuple[float, float]:
    """Convert RCControl output to independent left/right track commands.

    ``RCControl`` represents CH1 as linear velocity and defines positive
    angular velocity for a *low* CH3 pulse.  Negating the latter restores the
    direct channel direction used by the verified oscilloscope test.
    """
    if max_linear_mps <= 0.0 or max_angular_rps <= 0.0:
        raise ValueError("RC speed limits must be > 0")
    left = _clamp(float(linear_mps) / max_linear_mps, -1.0, 1.0)
    right = _clamp(-float(angular_rps) / max_angular_rps, -1.0, 1.0)
    return left, right


class RCTrackMotorBridge:
    """SamplingBridge-compatible adapter for independent manual tracks."""

    def __init__(
        self,
        navigation_bridge,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        sock: Optional[socket.socket] = None,
    ) -> None:
        self._navigation_bridge = navigation_bridge
        self._sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._address = (str(host), int(port))
        self._closed = False

    def _send(self, command: str) -> None:
        if self._closed:
            raise RuntimeError("RC motor bridge is closed")
        self._sock.sendto(command.encode("ascii"), self._address)

    def cancel_goal(self) -> bool:
        """Cancel navigation and neutralize tracks before manual takeover."""
        result = bool(self._navigation_bridge.cancel_goal())
        self._send("stop")
        return result

    def publish_cmd_vel(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> bool:
        """Send independent CH1/CH3 values as ``lr LEFT RIGHT`` over UDP."""
        left, right = rc_cmd_vel_to_tracks(linear_mps, angular_rps)
        self._send(f"lr {left:.4f} {right:.4f}")
        logger.debug(
            "manual RC direct tracks: CH1/left=%.4f CH3/right=%.4f",
            left,
            right,
        )
        return True

    def close(self) -> None:
        if self._closed:
            return
        try:
            self._send("stop")
        except OSError:
            pass
        finally:
            self._closed = True
            self._sock.close()


__all__ = ["RCTrackMotorBridge", "rc_cmd_vel_to_tracks"]
