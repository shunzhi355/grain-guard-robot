"""Unit tests for grain mechanism configuration (mechanism_config.py).

Pure data-module tests — no I/O, no ROS, no hardware.
"""

from __future__ import annotations

import pytest

from grain_sampling_workflow.mechanism_config import (
    DEFAULT_GRAIN_PARAMS,
    GRAIN_MECHANISM_CONFIG,
    get_grain_params,
)

# Every grain's parameter dict must contain all of these fields.
REQUIRED_KEYS = {
    "sampling_duration",
    "convey_duration",
    "open_duration",
    "close_duration",
    "clamp_duration",
    "unclamp_duration",
    "tighten_duration",
    "untighten_duration",
    "throttle_open",
    "throttle_close",
    "stop_value",
}

EXPECTED_SPECIFIC = {
    "稻谷": {"sampling_duration": 120.0, "convey_duration": 120.0},
    "玉米": {"sampling_duration": 180.0, "convey_duration": 180.0},
    "黄豆": {"sampling_duration": 90.0, "convey_duration": 90.0},
}


# ── Field completeness ───────────────────────────────────────────────────


def test_grains_registered():
    assert set(GRAIN_MECHANISM_CONFIG) == {"稻谷", "玉米", "黄豆"}


@pytest.mark.parametrize("grain", ["稻谷", "玉米", "黄豆"])
def test_grain_params_have_complete_fields(grain):
    params = GRAIN_MECHANISM_CONFIG[grain]
    assert REQUIRED_KEYS <= set(params)
    # Every numeric field must be a real positive number.
    for key in REQUIRED_KEYS:
        value = params[key]
        assert isinstance(value, (int, float)), f"{grain}.{key} not numeric"
        assert value > 0, f"{grain}.{key} must be positive"


def test_default_params_have_complete_fields():
    assert REQUIRED_KEYS <= set(DEFAULT_GRAIN_PARAMS)


# ── get_grain_params: known grains ──────────────────────────────────────


@pytest.mark.parametrize("grain", ["稻谷", "玉米", "黄豆"])
def test_get_grain_params_returns_full_dict(grain):
    params = get_grain_params(grain)
    assert REQUIRED_KEYS <= set(params)
    assert params is GRAIN_MECHANISM_CONFIG[grain]


@pytest.mark.parametrize("grain", ["稻谷", "玉米", "黄豆"])
def test_get_grain_params_specific_values(grain):
    params = get_grain_params(grain)
    expected = EXPECTED_SPECIFIC[grain]
    assert params["sampling_duration"] == expected["sampling_duration"]
    assert params["convey_duration"] == expected["convey_duration"]
    # Non-grain-specific fields use the default values.
    assert params["open_duration"] == 5.0
    assert params["close_duration"] == 3.0
    assert params["clamp_duration"] == 2.0
    assert params["unclamp_duration"] == 5.0
    assert params["tighten_duration"] == 10.0
    assert params["untighten_duration"] == 3.0
    assert params["throttle_open"] == 1200.0
    assert params["throttle_close"] == 1400.0
    assert params["stop_value"] == 1500.0


# ── get_grain_params: unknown grain → default fallback ──────────────────


@pytest.mark.parametrize("unknown", ["小麦", "燕麦", "", "Rice", None])
def test_get_grain_params_unknown_returns_default(unknown):
    params = get_grain_params(unknown)
    assert params == DEFAULT_GRAIN_PARAMS
    assert REQUIRED_KEYS <= set(params)
