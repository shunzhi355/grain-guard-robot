"""STM32-only action adapter. No PCA9685, GPIO or second UART owner."""
from __future__ import annotations

import math
import threading
import time

from grain_sampling_devices import mechanism_protocol as mcu
from grain_sampling_workflow.mechanism_config import get_grain_params


def command_plan(action: str, depth: str | None = None) -> tuple:
    """Match the UART side of 3588 MechanismRuntime, including both bin opens."""
    if action in ("open_bin", "close_bin", "hold_bin_open"):
        if depth not in mcu.BIN_DEVICES:
            raise ValueError("invalid bin depth")
        device = mcu.BIN_DEVICES[depth]
        if action == "close_bin":
            return ((mcu.STOP, device),)
        commands = [(mcu.START, device), (mcu.START, device)]
        if action == "hold_bin_open":
            commands.extend((mcu.STOP, other) for other in mcu.BIN_DEVICES.values()
                            if other != device)
        return tuple(commands)
    plans = {
        "tighten": ((mcu.START, mcu.TIGHTEN),),
        # Intentionally preserve the existing UART flow, not a new MCU opcode.
        "untighten": ((mcu.STOP, mcu.TIGHTEN),),
        "clamp": ((mcu.START, mcu.CLAMP),),
        "unclamp": ((mcu.START, mcu.UNCLAMP),),
        "open_bin_default": ((mcu.START, mcu.BIN_MID), (mcu.START, mcu.BIN_MID)),
        "convey": ((mcu.START, mcu.CONVEY),),
        "start_convey": ((mcu.START, mcu.CONVEY),),
        "stop_convey": ((mcu.STOP, mcu.CONVEY),),
        "close_all_bins": tuple((mcu.STOP, dev) for dev in mcu.BIN_DEVICES.values()),
        "stop_all": ((mcu.STOP_ALL, 0),),
    }
    if action in ("start_suction", "stop_suction"):
        raise NotImplementedError(f"{action}: existing 3588 flow has no STM32 frame")
    if action not in plans:
        raise ValueError(f"unknown STM32 action: {action}")
    return plans[action]


class SerialMechanisms:
    """Bench adapter; timed convey is blocking and cancellable, never a stale timer.

    ACK means command accepted, not mechanically completed. Mechanism dwell
    times other than timed conveying remain the caller/workflow's responsibility.
    """

    def __init__(self, send, check, *, grain=""):
        self.send = send
        self.check = check
        self.grain = grain
        self._action_lock = threading.Lock()

    def execute(self, action, *, depth=None, duration_s=None):
        commands = command_plan(action, depth)
        if duration_s is not None and action != "convey":
            raise ValueError("duration_s is only valid for timed convey")
        if action == "convey":
            duration_s = (get_grain_params(self.grain)["convey_duration"]
                          if duration_s is None else duration_s)
            if (isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
                    or not math.isfinite(duration_s) or not 0 < duration_s <= 120):
                raise ValueError("convey duration must be finite and in (0, 120] seconds")
        with self._action_lock:
            if action != "stop_all":
                self.check()
            sent = [self.send(cmd, dev) for cmd, dev in commands]
            if action == "convey":
                try:
                    deadline = time.monotonic() + duration_s
                    while time.monotonic() < deadline:
                        self.check()
                        time.sleep(min(0.02, max(0, deadline - time.monotonic())))
                finally:
                    sent.append(self.send(mcu.STOP, mcu.CONVEY))
            return sent
