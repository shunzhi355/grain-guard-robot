"""机构驱动模块测试（mechanism_driver.py）。

覆盖：通道映射/脉宽标定、PCA9685 纯函数换算、MockMechanismController 的
三态控制、急停、失败重试、duration 自动回停、品种参数，以及 conftest
``mock_pca9685`` 兼容性。全程 mock，不触碰真实 I2C。
"""

from __future__ import annotations

import time

import pytest

from grain_sampling_devices.mechanism_driver import (
    BIN_CLOSE_PULSE,
    BIN_OPEN_PULSE,
    CHANNELS,
    PULSE_CLOSE,
    PULSE_OPEN,
    PULSE_STOP,
    frequency_to_prescale,
    pulse_us_to_counts,
)

# ── 通道映射与脉宽标定 ─────────────────────────────────────────────────


def test_channel_mapping():
    assert CHANNELS == {
        "convey_1": 0,
        "convey_2": 1,
        "bin_shallow": 2,
        "bin_mid": 3,
        "bin_deep": 4,
        "clamp": 5,
        "tighten": 6,
        "fan": 7,
    }
    # 0~7 每个通道恰好映射一次，无重复/缺号
    assert sorted(CHANNELS.values()) == list(range(8))
    assert (PULSE_OPEN, PULSE_CLOSE, PULSE_STOP) == (1200, 1900, 1500)


# ── PCA9685 纯函数换算 ─────────────────────────────────────────────────


def test_frequency_to_prescale_math():
    # prescale = round(osc/(4096×freq))-1；用当前校准振荡器频率计算
    from grain_sampling_devices import mechanism_driver as md
    expect = int(round(md.OSCILLATOR_HZ / (4096 * 50.0) - 1.0))
    assert frequency_to_prescale(50.0) == expect
    with pytest.raises(ValueError):
        frequency_to_prescale(0.0)


def test_pulse_us_to_counts_math():
    # 50Hz 周期 20000us，4096 计数/周期
    assert pulse_us_to_counts(1300) == 266
    assert pulse_us_to_counts(1500) == 307
    assert pulse_us_to_counts(1900) == 389
    with pytest.raises(ValueError):
        pulse_us_to_counts(0)
    with pytest.raises(ValueError):
        pulse_us_to_counts(20000)


# ── Mock 三态控制（conftest mock_pca9685 兼容） ────────────────────────


def test_actuate_open_close_stop_writes_pulse(mock_mechanism):
    mock_mechanism.actuate(0, "open")
    mock_mechanism.actuate(0, "close")
    mock_mechanism.actuate(0, "stop")

    assert mock_mechanism.pca9685.register_history[0] == [1200, 1900, 1500.0]
    actions = [name for name, _ in mock_mechanism.action_history]
    assert actions == [
        "set_pulse", "actuate",
        "set_pulse", "actuate",
        "actuate",
    ]


def test_actuate_unknown_action_raises(mock_mechanism):
    with pytest.raises(ValueError, match="unknown action"):
        mock_mechanism.actuate(0, "sideways")


# ── 急停 ───────────────────────────────────────────────────────────────


def test_emergency_stop_stops_running_channels(mock_mechanism):
    mock_mechanism.actuate(0, "open", duration=60)
    mock_mechanism.actuate(2, "close", duration=60)
    assert mock_mechanism._running == {0, 2}

    mock_mechanism.emergency_stop()

    assert mock_mechanism._running == set()
    assert mock_mechanism.pca9685.register_history[0] == [1200, 1500.0]
    assert mock_mechanism.pca9685.register_history[2] == [1900, 1500.0]
    for channel in CHANNELS.values():
        assert mock_mechanism.pca9685.register_history[channel][-1] == 1500.0
    # 急停后拒绝新动作
    with pytest.raises(RuntimeError, match="emergency-stop"):
        mock_mechanism.actuate(0, "open")


# ── 失败重试 ───────────────────────────────────────────────────────────


def test_write_failure_retries_then_raises(mock_mechanism):
    calls = {"n": 0}

    def _failing(channel, on, off):
        calls["n"] += 1
        raise OSError("simulated i2c error")

    mock_mechanism.pca9685.set_pwm.side_effect = _failing

    with pytest.raises(RuntimeError, match="after 3 attempts"):
        mock_mechanism.actuate(0, "open")
    # 1 次正常 + 2 次重试
    assert calls["n"] == 3


# ── duration 自动回停 ──────────────────────────────────────────────────


def test_duration_auto_stop(mock_mechanism):
    mock_mechanism.actuate(0, "open", duration=0.05)
    assert 0 in mock_mechanism._running

    # 轮询等待后台线程断电释放（避免固定 sleep 的时序抖动）
    deadline = time.monotonic() + 3.0
    while (
        mock_mechanism.pca9685.register_history.get(0) != [1200, 1500.0]
        and time.monotonic() < deadline
    ):
        time.sleep(0.02)

    assert mock_mechanism.pca9685.register_history[0] == [1200, 1500.0]
    assert 0 not in mock_mechanism._running


# ── 品种参数 ───────────────────────────────────────────────────────────


def test_set_grain_overrides_throttle(mock_mechanism):
    # 先污染默认值，验证 set_grain 会覆盖
    mock_mechanism.throttle_open = 999
    mock_mechanism.throttle_close = 1001
    mock_mechanism.stop_value = 1002

    assert mock_mechanism.set_grain("稻谷") is True
    assert mock_mechanism.current_grain == "稻谷"
    assert mock_mechanism.throttle_open == 1200
    assert mock_mechanism.throttle_close == 1400
    assert mock_mechanism.stop_value == 1500

    # set_grain 后的脉宽写入使用品种参数
    mock_mechanism.actuate(0, "open")
    assert mock_mechanism.pca9685.register_history[0] == [1200]


def test_set_grain_unknown_falls_back(mock_mechanism):
    assert mock_mechanism.set_grain("unknown") is False
    assert mock_mechanism.current_grain == "unknown"
    # 未知品种回退默认标定值
    assert mock_mechanism.throttle_open == 1200
    assert mock_mechanism.throttle_close == 1400
    assert mock_mechanism.stop_value == 1500


# ── 语义动作（conftest 兼容） ──────────────────────────────────────────


def test_semantic_actions_write_expected_channels(mock_mechanism):
    mock_mechanism.convey()
    mock_mechanism.open_bin(depth="deep")
    mock_mechanism.close_bin(depth="deep")
    mock_mechanism.clamp()
    mock_mechanism.unclamp()
    mock_mechanism.tighten()
    mock_mechanism.untighten()
    mock_mechanism.press()
    mock_mechanism.lift()
    mock_mechanism.fan()

    assert mock_mechanism.pca9685.register_history[0] == [1200]         # convey → CH0 开
    assert mock_mechanism.pca9685.register_history[4] == [BIN_OPEN_PULSE, BIN_CLOSE_PULSE]
    assert 5 not in mock_mechanism.pca9685.register_history  # CH5 仅保持旧电调中位
    assert mock_mechanism.pca9685.level_history[8][-2:] == [False, True]
    assert mock_mechanism.pca9685.duty_history[9] == [9.5, 9.5, 6.0]
    assert mock_mechanism.pca9685.level_history[10][-1] is True
    assert mock_mechanism.pca9685.register_history[6] == [1300, 1900]   # tighten 关/拧紧 / untighten 开/拧松 (CH6 独立)
    assert mock_mechanism.pca9685.register_history[2] == [1200, 1900]   # press 开 / lift 关
    assert mock_mechanism.pca9685.register_history[7] == [1200]         # fan → CH7 开


def test_close_bin_uses_depth_channel(mock_mechanism):
    """close_bin 按 depth 关对应仓，而非固定 CH3。"""
    mock_mechanism.close_bin(depth="shallow")
    mock_mechanism.close_bin(depth="mid")
    mock_mechanism.close_bin(depth="deep")
    assert mock_mechanism.pca9685.register_history[2] == [BIN_CLOSE_PULSE]
    assert mock_mechanism.pca9685.register_history[3] == [BIN_CLOSE_PULSE]
    assert mock_mechanism.pca9685.register_history[4] == [BIN_CLOSE_PULSE]


def test_close_bin_unknown_depth_raises(mock_mechanism):
    with pytest.raises(ValueError, match="unknown bin depth"):
        mock_mechanism.close_bin(depth="sideways")


def test_convey_close_uses_off(mock_mechanism):
    """输送停料（direction<=0）回到1500us并保持PWM，而非写关脉宽。"""
    mock_mechanism.convey(direction=0)
    assert mock_mechanism.pca9685.register_history[0] == [1500.0]
    assert mock_mechanism.pca9685.register_history[1] == [1500.0]


def test_open_bin_duration_auto_closes_then_turns_output_off(
    mock_mechanism, monkeypatch
):
    """自动关仓在关仓时长结束后回1500us，持续输出PWM。"""
    monkeypatch.setattr(
        "grain_sampling_devices.mechanism_driver.get_grain_params",
        lambda _grain: {"close_duration": 0.05},
    )
    mock_mechanism.open_bin(depth="deep", duration=0.05)
    assert mock_mechanism.pca9685.register_history[4] == [BIN_OPEN_PULSE]

    deadline = time.monotonic() + 3.0
    while (
        mock_mechanism.pca9685.register_history.get(4)
        != [BIN_OPEN_PULSE, BIN_CLOSE_PULSE, 1500.0]
        and time.monotonic() < deadline
    ):
        time.sleep(0.02)

    assert mock_mechanism.pca9685.register_history[4] == [
        BIN_OPEN_PULSE,
        BIN_CLOSE_PULSE,
        1500.0,
    ]
    assert 4 not in mock_mechanism._running
    assert 4 not in mock_mechanism._bin_timers


def test_emergency_stop_cancels_pending_bin_timer(mock_mechanism):
    """急停后旧定时器不得再次写关仓 PWM。"""
    mock_mechanism.open_bin(depth="deep", duration=0.05)
    mock_mechanism.emergency_stop()
    time.sleep(0.1)

    assert mock_mechanism.pca9685.register_history[4] == [BIN_OPEN_PULSE, 1500.0]
    assert mock_mechanism._bin_timers == {}


def test_explicit_close_cancels_pending_auto_close(mock_mechanism):
    """显式关仓取消旧定时器，避免稍后重复上电。"""
    mock_mechanism.open_bin(depth="deep", duration=0.05)
    mock_mechanism.close_bin(depth="deep", duration=0.02)
    time.sleep(0.1)

    assert mock_mechanism.pca9685.register_history[4] == [
        BIN_OPEN_PULSE,
        BIN_CLOSE_PULSE,
        1500.0,
    ]


def test_shutdown_stops_running_channels(mock_mechanism):
    mock_mechanism.actuate(1, "open", duration=60)
    assert mock_mechanism._running == {1}

    mock_mechanism.shutdown()

    assert mock_mechanism._running == set()
    assert mock_mechanism.pca9685.register_history[1] == [1200, 1500.0]
