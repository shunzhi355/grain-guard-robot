"""Validate the commissioned controller, not the connector label in a manual.

On the installed board PCA9685 disappeared/reappeared on I2C2 when its logic
power was removed/restored. I2C4 never acknowledged it. No slave writes here.
"""
import os
from pathlib import Path


def validate_tp_bus(device, dt_root='/proc/device-tree', sys_root='/sys/class/i2c-adapter', expected_alias=None):
    dt = Path(dt_root)
    compatible = dt / 'compatible'
    if not compatible.exists():
        return  # Development hosts have no Rockchip device tree.
    if b'neardi,lpb3588' not in compatible.read_bytes():
        return
    expected_alias = expected_alias or os.environ.get('PCA9685_DT_ALIAS', 'i2c2')
    if expected_alias not in {f'i2c{i}' for i in range(9)}:
        raise RuntimeError('Invalid PCA9685_DT_ALIAS')
    alias = dt / 'aliases' / expected_alias
    if not alias.exists():
        raise RuntimeError(f'Device tree has no commissioned alias {expected_alias}')
    target = dt / alias.read_bytes().rstrip(b'\0').decode().lstrip('/')
    status = target / 'status'
    if status.exists() and status.read_bytes().rstrip(b'\0') not in (b'okay', b'ok'):
        raise RuntimeError(f'{expected_alias} is disabled; verify pinmux before enabling; no automatic bus fallback')
    adapter = Path(sys_root) / Path(device).resolve().name
    candidates = [adapter / 'device/of_node', adapter / 'of_node']
    if not any(p.exists() and p.resolve() == target.resolve() for p in candidates):
        raise RuntimeError(f'{device} is not the commissioned {expected_alias} controller ({target.name}); refusing I2C writes')


if __name__ == '__main__':
    import sys
    validate_tp_bus(sys.argv[1] if len(sys.argv) > 1 else '/dev/i2c-2')
    print('TP I2C controller validation passed (no slave writes performed)')
