"""X2P 使能链路诊断脚本（scripts/x2p_enable_diag.py）的逻辑测试。

只测纯函数与只读分支，不打开串口、不写任何寄存器。
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

try:  # pragma: no cover - depends on the developer environment
    import serial  # noqa: F401
except ImportError:  # pragma: no cover
    sys.modules["serial"] = ModuleType("serial")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))


def _load_diag():
    path = ROOT / "scripts" / "x2p_enable_diag.py"
    spec = importlib.util.spec_from_file_location("x2p_enable_diag", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


diag = _load_diag()


def _snapshot(*, forced=0x01, inputs=0x00, enabled=0, fn_enabled=0):
    return (
        {
            "P400_DI1功能": 1,
            "P415_强制输入": forced,
            "Un032_DI状态": inputs,
            "Un058_伺服使能": enabled,
        },
        {"Un058_伺服使能": fn_enabled},
    )


def test_interpret_flags_forced_input_not_sticking():
    after_force, after_fn = _snapshot(forced=0x00)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("强制写入没有生效" in line for line in conclusions)


def test_interpret_flags_di1_not_configured_as_srv_on():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x01)
    after_force["P400_DI1功能"] = 5
    conclusions = diag.interpret(after_force, after_fn)
    assert any("Pn400=5" in line for line in conclusions)


def test_interpret_flags_drive_not_seeing_di():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x00)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("DI 公共端" in line for line in conclusions)


def test_interpret_flags_drive_refusing_enable():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x01, enabled=0)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("主动拒绝使能" in line for line in conclusions)
    assert any("不在 Modbus 控制方式" in line for line in conclusions)


def test_interpret_reports_success_and_fn000_alternative():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x01, enabled=0)
    after_fn["Un058_伺服使能"] = 1
    conclusions = diag.interpret(after_force, after_fn)
    assert any("Fn000 内部使能" in line for line in conclusions)

    after_force, after_fn = _snapshot(forced=0x01, inputs=0x01, enabled=1)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("使能链路是通的" in line for line in conclusions)


def test_write_requires_explicit_confirm():
    args = diag.build_parser().parse_args([])
    assert args.confirm == ""
    args = diag.build_parser().parse_args(["--confirm", "ENABLE"])
    assert args.confirm == "ENABLE"


def _with_di_functions(values: dict, *functions: int) -> dict:
    """把 DI1..DI4 功能分配补进快照。"""
    for di, function in enumerate(functions, start=1):
        values[f"Pn{400 + di - 1}_DI{di}功能"] = function
    return values


def test_interpret_flags_srv_on_missing_entirely():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x00)
    _with_di_functions(after_force, 5, 11, 0, 0)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("没有任何 DI 配成 SRV-ON" in line for line in conclusions)


def test_interpret_flags_srv_on_on_a_different_di():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x00)
    after_force["P400_DI1功能"] = 0
    _with_di_functions(after_force, 0, 1, 0, 0)
    conclusions = diag.interpret(after_force, after_fn)
    assert any("SRV-ON 实际配在 DI2" in line for line in conclusions)


def test_interpret_stays_quiet_when_di1_is_the_srv_on_pin():
    after_force, after_fn = _snapshot(forced=0x01, inputs=0x01, enabled=0)
    _with_di_functions(after_force, 1, 11, 0, 0)
    conclusions = diag.interpret(after_force, after_fn)
    assert not any("配成 SRV-ON" in line for line in conclusions)
    assert any("主动拒绝使能" in line for line in conclusions)


def test_read_full_snapshot_survives_individual_read_failures():
    class FlakyDrive:
        def read_registers(self, address, count=1):
            if address == 0x0401:
                raise RuntimeError("boom")
            if address == 0x2020:
                raise RuntimeError("timeout")
            return [0]

    values = diag.read_full_snapshot(FlakyDrive())
    assert values["P400_DI1功能"] == 0
    assert values["Un032_DI状态"] == "读取失败(timeout)"
    assert values["Pn401_DI2功能"] == "读取失败(boom)"
    assert values["Pn402_DI3功能"] == 0
    assert values["Pn604_写入策略"] == 0


def test_read_full_snapshot_works_without_read_enable_chain():
    """板端旧版 src/x2p/ 没有 read_enable_chain()，诊断脚本不能依赖它。"""

    class OldBoardDrive:
        def read_registers(self, address, count=1):
            return [{0x0400: 1, 0x2020: 1}.get(address, 0)]

    assert not hasattr(OldBoardDrive(), "read_enable_chain")
    values = diag.read_full_snapshot(OldBoardDrive())
    assert values["P400_DI1功能"] == 1
    assert values["Un032_DI状态"] == 1
    assert values["Un058_伺服使能"] == 0


def test_watch_option_defaults_to_single_sample():
    args = diag.build_parser().parse_args([])
    assert args.watch == 0.0
    args = diag.build_parser().parse_args(["--watch", "6"])
    assert args.watch == 6.0
