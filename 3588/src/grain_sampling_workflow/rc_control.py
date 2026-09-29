"""RC manual control — map receiver sticks to ``/cmd_vel``.

Reads the three PWM channels of :class:`~grain_sampling_devices.rc_receiver.RCReceiver`
and converts them into manual chassis control::

    CH5  two-position mode switch  (board-measured 2026-09-03)
            manual band (700~1450 us) -> operator drives the chassis
            auto   band (1550~2300 us) -> chassis handed back to navigation
    CH1 logical throttle (physical i-BUS CH3), forward / reverse
            pulse > 1750 us -> forward  (linear.x > 0)
            pulse < 1350 us -> reverse  (linear.x < 0)
    CH3 logical steering (physical i-BUS CH1), left / right
            pulse < 1350 us -> LEFT  (angular.z > 0)
            pulse > 1750 us -> RIGHT (angular.z < 0)

The CH5 mode switch is debounced: a band change only takes effect after
``debounce_samples`` consecutive samples (default 5), so transient
switch-bounce readings near the 1450~1550 us dead zone never trigger a
mode change.

Only the chassis is controlled — mechanism / sampling machinery is never
touched, and no independent emergency stop is issued (taking manual
control cancels any active navigation goal, which halts the robot).

In mock mode (no ROS) every action is still recorded on the instance
(``cancel_goal_calls`` and ``cmd_vel_history``), so tests can assert the
exact action sequence without a ROS master.
"""

from __future__ import annotations

import logging
from typing import Mapping, Optional

from utils.sampling_params import (
    RC_DEADBAND_HIGH,
    RC_DEADBAND_LOW,
    RC_DEBOUNCE_SAMPLES,
    RC_MAX_ANGULAR_RPS,
    RC_MAX_LINEAR_MPS,
    RC_MODE_RANGES,
    RC_STICK_CENTER,
)

logger = logging.getLogger(__name__)

#: Full-scale stick travel used by the mapping formulas, in us
#: (one half of a 1000~2000 us channel).
STICK_FULL_SCALE_US = 500.0


class RCControl:
    """Map a 3-channel RC receiver to manual chassis control.

    Parameters
    ----------
    receiver :
        An RC receiver with a ``read()`` API (``MockRCReceiver`` or
        ``RCReceiver``).  ``read()`` may return a ``{channel: us}`` dict
        (no argument) or a single ``float``/``None`` per channel.
    bridge :
        A ``SamplingBridge``-like object exposing ``cancel_goal()`` and
        ``publish_cmd_vel(linear_mps, angular_rps)``.
    mode_ranges : Mapping[str, tuple[float, float]], optional
        CH5 pulse-width band (inclusive) per mode; defaults to
        ``{"manual": (700, 1450), "auto": (1550, 2300)}``.
    debounce_samples : int
        Consecutive samples required before a mode change takes effect
        (default 5).
    stick_center : float
        Centre pulse width of the sticks (default 1450 us).
    deadband_low / deadband_high : float
        Stick deadband (default 1350~1750 us): inside it the axis maps
        to 0.
    max_linear_mps : float
        Full-scale forward speed (default 0.3 m/s, matches
        ``cmd_vel_to_motor``).
    max_angular_rps : float
        Full-scale turning rate (default 0.8 rad/s, matches
        ``cmd_vel_to_motor``).
    """

    MODE_MANUAL = "manual"
    MODE_AUTO = "auto"

    #: Board-measured CH5 two-position bands (us), inclusive — 2026-09-03.
    DEFAULT_MODE_RANGES: Mapping[str, tuple[float, float]] = RC_MODE_RANGES

    def __init__(
        self,
        receiver,
        bridge,
        *,
        mode_ranges: Optional[Mapping[str, tuple[float, float]]] = None,
        debounce_samples: int = RC_DEBOUNCE_SAMPLES,
        stick_center: float = RC_STICK_CENTER,
        deadband_low: float = RC_DEADBAND_LOW,
        deadband_high: float = RC_DEADBAND_HIGH,
        max_linear_mps: float = RC_MAX_LINEAR_MPS,
        max_angular_rps: float = RC_MAX_ANGULAR_RPS,
    ) -> None:
        self._receiver = receiver
        self._bridge = bridge
        self._mode_ranges: Mapping[str, tuple[float, float]] = dict(
            mode_ranges or self.DEFAULT_MODE_RANGES
        )
        self._debounce_samples = max(1, int(debounce_samples))
        self._stick_center = float(stick_center)
        self._deadband_low = float(deadband_low)
        self._deadband_high = float(deadband_high)
        self._max_linear_mps = float(max_linear_mps)
        self._max_angular_rps = float(max_angular_rps)
        if self._max_linear_mps <= 0.0 or self._max_angular_rps <= 0.0:
            raise ValueError("max_linear_mps and max_angular_rps must be > 0")

        #: Current debounced mode; ``None`` until ``debounce_samples``
        #: consecutive samples establish it.
        self.mode: Optional[str] = None
        #: Number of ``bridge.cancel_goal()`` calls (mock-mode audit trail).
        self.cancel_goal_calls: int = 0
        #: ``(linear.x, angular.z)`` for every manual-mode tick
        #: (mock-mode audit trail).
        self.cmd_vel_history: list[tuple[float, float]] = []
        self._streak: int = 0

    # ── Public API ────────────────────────────────────────────────────

    def tick(self) -> Optional[str]:
        """Sample the receiver once and apply mode / stick mapping.

        Returns the current (debounced) mode after the tick.
        """
        values = self._receiver.read()
        if isinstance(values, dict):
            ch1 = values.get("CH1")
            ch3 = values.get("CH3")
            ch5 = values.get("CH5")
        else:
            ch1 = self._receiver.read("CH1")
            ch3 = self._receiver.read("CH3")
            ch5 = self._receiver.read("CH5")

        self._update_mode(ch5)

        # Auto mode (or no mode yet): hands-off — never drive the chassis.
        if self.mode != self.MODE_MANUAL:
            return self.mode

        linear = self._map_linear(ch1)
        angular = self._map_angular(ch3)
        self.cmd_vel_history.append((linear, angular))
        try:
            self._bridge.publish_cmd_vel(linear, angular)
        except Exception:  # noqa: BLE001 - control loop must never die
            logger.exception("RC publish_cmd_vel failed")
        return self.mode

    @property
    def manual_active(self) -> bool:
        """True when the debounced mode is manual (chassis taken over)."""
        return self.mode == self.MODE_MANUAL

    @property
    def last_cmd_vel(self) -> Optional[tuple[float, float]]:
        """Most recent ``(linear.x, angular.z)``, or ``None`` if never."""
        return self.cmd_vel_history[-1] if self.cmd_vel_history else None

    # ── Mode debounce ─────────────────────────────────────────────────

    def _update_mode(self, ch5: Optional[float]) -> None:
        """Debounce the CH5 switch; cancel the nav goal on entering manual.

        A reading inside either configured band counts towards that mode;
        a transition value (or stale ``None``) resets the streak so a mode
        change only takes effect after ``debounce_samples`` clean samples.
        """
        band = self._classify_mode(ch5)
        if band is None:
            self._streak = 0
            return
        if band == self.mode:
            self._streak = 0
            return
        self._streak += 1
        if self._streak >= self._debounce_samples:
            self._streak = 0
            self._switch_mode(band)

    def _classify_mode(self, pulse: Optional[float]) -> Optional[str]:
        """Return the CH5 mode band for *pulse*, or ``None`` if it is a
        transition value (in no configured band)."""
        if pulse is None:
            return None
        for mode_name, (low, high) in self._mode_ranges.items():
            if low <= pulse <= high:
                return mode_name
        return None

    def _switch_mode(self, new_mode: str) -> None:
        old_mode = self.mode
        self.mode = new_mode
        if new_mode == self.MODE_MANUAL:
            # Taking manual control halts any active navigation goal.
            try:
                self._bridge.cancel_goal()
            except Exception:  # noqa: BLE001 - control loop must never die
                logger.exception("RC cancel_goal failed")
            self.cancel_goal_calls += 1
        logger.info("RC mode switched %s -> %s", old_mode, new_mode)

    # ── Stick mapping ─────────────────────────────────────────────────

    def _map_stick(self, pulse: Optional[float]) -> float:
        """Continuous signed effort measured from the deadband edge."""
        if pulse is None or self._deadband_low <= pulse <= self._deadband_high:
            return 0.0
        if pulse > self._deadband_high:
            span = self._stick_center + STICK_FULL_SCALE_US - self._deadband_high
            return min(1.0, (pulse - self._deadband_high) / span)
        span = self._deadband_low - (self._stick_center - STICK_FULL_SCALE_US)
        return -min(1.0, (self._deadband_low - pulse) / span)

    def _map_linear(self, pulse: Optional[float]) -> float:
        """CH1 -> ``linear.x`` (m/s): deadband 0, large=forward."""
        return self._map_stick(pulse) * self._max_linear_mps

    def _map_angular(self, pulse: Optional[float]) -> float:
        """CH3 -> ``angular.z`` (rad/s): deadband 0, small=LEFT, large=RIGHT.

        Note this is inverted versus the usual convention: ``angular.z > 0``
        (left turn) corresponds to a small pulse, ``angular.z < 0`` (right
        turn) to a large pulse — confirmed on the board on 2026-08-16.
        """
        return -self._map_stick(pulse) * self._max_angular_rps


__all__ = ["RCControl"]
