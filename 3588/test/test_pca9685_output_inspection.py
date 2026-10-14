"""Register-level checks for the live, read-only PCA9685 inspection tool."""

from contextlib import nullcontext

from grain_sampling_devices.mechanism_driver import (
    LED0_ON_L,
    MechanismController,
    PCA9685,
)
from scripts.inspect_pca9685_outputs import decode_channel


def test_decoder_distinguishes_direction_sleep_and_legacy_pwm():
    assert decode_channel((0, 0, 0, 0x10), 0x04, 50) == "LOW"
    assert decode_channel((0, 0x10, 0, 0), 0x04, 50) == "HIGH"
    assert decode_channel((0, 0x10, 0, 0x10), 0x04, 50) == "LOW"
    assert decode_channel((0, 0, 0xF6, 0), 0x04, 50) == "PWM 6.01% (1201 us high)"
    assert decode_channel((0, 0, 0x33, 0x01), 0x04, 50) == "PWM 7.50% (1499 us high)"


def test_clamp_direction_writes_do_not_change_three_bin_registers(monkeypatch):
    pca = PCA9685()
    registers = {}
    monkeypatch.setattr(pca, "_transaction", nullcontext)
    monkeypatch.setattr(pca, "write_register", lambda address, value: registers.__setitem__(address, value))
    controller = MechanismController(pca9685=pca, mock_mode=True)
    try:
        controller.hold_bin_open("mid")
        bins_before = {
            address: value for address, value in registers.items()
            if LED0_ON_L + 2 * 4 <= address < LED0_ON_L + 5 * 4
        }
        assert len(bins_before) == 12

        controller.clamp(duration=60)
        controller.unclamp(duration=60)

        bins_after = {address: registers[address] for address in bins_before}
        assert bins_after == bins_before
        ph = LED0_ON_L + 8 * 4
        en = LED0_ON_L + 9 * 4
        ns = LED0_ON_L + 10 * 4
        assert tuple(registers[ph + offset] for offset in range(4)) == (0, 0x10, 0, 0)
        assert tuple(registers[en + offset] for offset in range(4)) == (0, 0, 0xF6, 0)
        assert tuple(registers[ns + offset] for offset in range(4)) == (0, 0x10, 0, 0)
    finally:
        controller.shutdown()
