"""scripts/x2p_enable_probe.py 的回归测试。

该脚本必须在**没有 x2p 包、src/ 版本任意**的板端也能跑，所以这里用一个
内存 Modbus 从站替身，只验证：正常判读、撤销强制输入/使能、以及不 import x2p。
"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path
from types import ModuleType

ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT / "scripts" / "x2p_enable_probe.py"


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _make_memory(*, force_mirrors_inputs: bool, fn000_works: bool) -> dict:
    return {
        0x0400: 1,  # Pn400: DI1 = SRV-ON
        0x0401: 11,  # Pn401: DI2 = CTRG
        0x0402: 0,
        0x0403: 0,
        0x040F: 0,  # Pn415
        0x2020: 0,  # Un032
        0x203A: 0,  # Un058
        0x2000: 0,
        0x2064: 0,
        0x3E00: 3,
        0x3F00: 0,  # Fn000
        "_force_mirrors_inputs": force_mirrors_inputs,
        "_fn000_works": fn000_works,
    }


class _FakeSerial:
    def __init__(self, memory: dict):
        self.memory = memory
        self.buffer = b""

    def close(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def flush(self) -> None:
        pass

    def write(self, data: bytes) -> int:
        self.buffer += data
        return len(data)

    def read(self, size: int) -> bytes:
        request, self.buffer = self.buffer, b""
        slave, function = request[0], request[1]
        body = request[2:-2]
        if function == 0x03:
            address = int.from_bytes(body[0:2], "big")
            count = int.from_bytes(body[2:4], "big")
            values = [
                self.memory.get(address + i, 0) if isinstance(
                    self.memory.get(address + i, 0), int
                ) else 0
                for i in range(count)
            ]
            payload = bytes((slave, function, count * 2)) + b"".join(
                value.to_bytes(2, "big") for value in values
            )
        else:
            address = int.from_bytes(body[0:2], "big")
            value = int.from_bytes(body[2:4], "big")
            self.memory[address] = value
            if address == 0x040F and self.memory["_force_mirrors_inputs"]:
                self.memory[0x2020] = value
            if address == 0x3F00 and self.memory["_fn000_works"] and value == 1:
                self.memory[0x203A] = 1
            payload = bytes((slave, function)) + body
        return (payload + struct.pack("<H", _crc16(payload)))[:size]


class _FakeSerialModule(ModuleType):
    EIGHTBITS = 8
    PARITY_NONE = "N"
    STOPBITS_ONE = 1

    def __init__(self, name: str, memory: dict):
        super().__init__(name)
        self._memory = memory

    def Serial(self, **_kwargs) -> _FakeSerial:  # noqa: N802 - pyserial 命名
        return _FakeSerial(self._memory)


def _run_probe(*, force_mirrors_inputs: bool, fn000_works: bool) -> tuple[str, dict]:
    memory = _make_memory(
        force_mirrors_inputs=force_mirrors_inputs, fn000_works=fn000_works
    )
    saved = sys.modules.get("serial")
    saved_sleep = time.sleep
    sys.modules["serial"] = _FakeSerialModule("serial", memory)
    # 假串口是同步应答的，等待只用来给真实驱动器时间，测试里没必要真睡。
    time.sleep = lambda _seconds: None
    captured: list[str] = []
    real_stdout = sys.stdout
    try:
        sys.stdout = type("Out", (), {"write": lambda _s, text: captured.append(text) or len(text), "flush": lambda _s: None})()
        source = PROBE.read_text(encoding="utf-8")
        exec(compile(source, str(PROBE), "exec"), {"__name__": "__main__"})
    finally:
        sys.stdout = real_stdout
        time.sleep = saved_sleep
        if saved is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = saved
    return "".join(captured), memory


def test_probe_does_not_import_the_project_x2p_package():
    source = PROBE.read_text(encoding="utf-8")
    assert "import x2p" not in source
    assert "from x2p" not in source


def test_probe_reports_drive_refusing_enable_and_restores_registers():
    output, memory = _run_probe(force_mirrors_inputs=True, fn000_works=False)
    assert "[对照] 强制 DI4 后 Un032 bit3 由0变1" in output
    assert "[C]" in output
    assert "驱动器自己拒绝上电" in output
    assert memory[0x040F] == 0
    assert memory[0x3F00] == 0


def test_probe_reports_force_channel_not_sticking():
    output, memory = _run_probe(force_mirrors_inputs=False, fn000_works=False)
    assert "Pn415 不是本机" in output or "写入被驱动器忽略" in output
    assert "[B]" in output
    assert memory[0x040F] == 0


def test_probe_reports_fn000_internal_enable_available():
    output, memory = _run_probe(force_mirrors_inputs=True, fn000_works=True)
    assert "内部使能可用" in output
    assert memory[0x040F] == 0
    assert memory[0x3F00] == 0
