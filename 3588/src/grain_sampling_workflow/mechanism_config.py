"""Grain mechanism configuration — per-grain actuation parameters.

Pure configuration module for the grain sampling robot's execution
mechanisms (PCA9685-driven actuators): hopper open/close, clamp/unclamp,
tighten/untighten, and throttle control.

No I/O, no ROS, no third-party dependencies.

All tunable values live in :mod:`utils.sampling_params` (single source of
truth); this module re-exports them and provides ``get_grain_params``.
"""

from __future__ import annotations

from utils.sampling_params import (
    DEFAULT_GRAIN_PARAMS,
    GRAIN_MECHANISM_CONFIG,
)

__all__ = ["DEFAULT_GRAIN_PARAMS", "GRAIN_MECHANISM_CONFIG", "get_grain_params"]


def get_grain_params(name: str) -> dict[str, float]:
    """Return the full mechanism parameter dict for a grain type.

    Unknown grain names fall back to ``DEFAULT_GRAIN_PARAMS`` so that the
    caller always receives a complete, usable dict.
    """
    return GRAIN_MECHANISM_CONFIG.get(name, DEFAULT_GRAIN_PARAMS)
