import sys
from pathlib import Path

# Add the src directory to sys.path so that tests can import packages
# under src/ without needing a pip install -e .
SRC_DIR = Path(__file__).resolve().parent.parent / "src"
PROJECT_ROOT = SRC_DIR.parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import pytest
from unittest.mock import MagicMock

from grain_sampling_devices.mechanism_driver import MockMechanismController


@pytest.fixture
def mock_pca9685():
    """Mock PCA9685 寄存器对象，记录 channel→脉宽 写入历史。

    通过 ``set_pwm(channel, on, off)`` 模拟寄存器写入，
    每次写入按通道追加到 ``register_history``：
    ``register_history[channel] -> [脉冲宽度(off), ...]``（按写入顺序）。

    用法：测试中可断言 ``pca.register_history[0] == [1300, 1650]``，
    或直接使用 ``pca.set_pwm.assert_called_with(...)``。
    """
    pca = MagicMock(spec=["set_pwm", "channel_off"])
    pca.register_history = {}

    def _record_pwm(channel, on, off):
        pca.register_history.setdefault(channel, []).append(off)

    def _record_off(channel):
        pca.register_history.setdefault(channel, []).append("OFF")

    pca.set_pwm.side_effect = _record_pwm
    pca.channel_off.side_effect = _record_off
    return pca


@pytest.fixture
def mock_mechanism(mock_pca9685):
    """返回 MockMechanismController 实例（mock 模式）。

    内部自动注入 ``mock_pca9685``，所有脉宽写入可通过
    ``controller.pca9685.register_history`` 或
    ``controller.action_history`` 断言。

    测试结束后自动调用 ``shutdown()`` 清理。
    """
    controller = MockMechanismController(pca9685=mock_pca9685, mock_mode=True)
    yield controller
    controller.shutdown()
