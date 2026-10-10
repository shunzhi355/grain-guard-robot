from pathlib import Path

import pytest

from grain_sampling_devices.tp_i2c import validate_tp_bus


@pytest.fixture(autouse=True)
def explicit_fixture_alias(monkeypatch):
    monkeypatch.setenv('PCA9685_DT_ALIAS', 'i2c4')


def setup_tree(tmp_path, status=b'okay\0'):
    dt = tmp_path / 'dt'
    (dt / 'aliases').mkdir(parents=True)
    (dt / 'compatible').write_bytes(b'neardi,lpb3588-linux-f0,\0')
    (dt / 'aliases/i2c4').write_bytes(b'/i2c@feac0000\0')
    target = dt / 'i2c@feac0000'
    target.mkdir()
    (target / 'status').write_bytes(status)
    return dt, target


def test_disabled_refuses_even_wrong_bus(tmp_path):
    dt, _ = setup_tree(tmp_path, b'disabled\0')
    with pytest.raises(RuntimeError, match='disabled'):
        validate_tp_bus('/dev/i2c-2', dt, tmp_path / 'sys')


def test_wrong_or_missing_adapter_refused(tmp_path):
    dt, _ = setup_tree(tmp_path)
    with pytest.raises(RuntimeError, match='refusing I2C writes'):
        validate_tp_bus('/dev/i2c-2', dt, tmp_path / 'sys')


def test_development_without_dt(tmp_path):
    validate_tp_bus('/dev/i2c-4', tmp_path / 'absent')


def test_default_is_commissioned_i2c2(tmp_path, monkeypatch):
    monkeypatch.delenv('PCA9685_DT_ALIAS')
    dt, _ = setup_tree(tmp_path)
    with pytest.raises(RuntimeError, match='no commissioned alias i2c2'):
        validate_tp_bus('/dev/i2c-4', dt, tmp_path / 'sys')


def test_correct_identity_accepts_renumbered_adapter(tmp_path):
    dt, target = setup_tree(tmp_path)
    adapter = tmp_path / 'sys/i2c-7/device'
    adapter.mkdir(parents=True)
    try:
        (adapter / 'of_node').symlink_to(target, target_is_directory=True)
    except OSError:
        pytest.skip('Symlink privilege unavailable')
    validate_tp_bus('/dev/i2c-7', dt, tmp_path / 'sys')
