#!/usr/bin/env python3
"""Configure X2P for internal position mode and auto tuning mode 2.

This utility intentionally does not enable the servo or command motion.  It
writes the persistent parameter set, verifies readback, then asks the operator
to power-cycle the drive before any movement test.

Parameters written by default:
    Pn001 = 0       position control mode
    Pn002 = 2       auto tuning mode 2
    Pn003 = 14      starting rigidity
    Pn008 = 131072  command pulses per motor revolution (17-bit encoder)
    Pn321 = 1       internal multi-position command source
    Pn700 = 7       Pn701-selected segment, execute immediately
    Pn701 = 0       idle until a motion command selects Pr1
    Pn400 = 1       DI1 = SRV-ON
    Pn401 = 11      DI2 = CTRG

For a 23-bit encoder, pass ``--command-pulses-per-rev 8388608``.  The chosen
value must equal ``ControllerConfig.encoder_counts_per_motor_rev``; otherwise
the motion layer refuses to use Pn706 so that a position is never scaled by the
wrong ratio.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
for _extra in (ROOT_DIR / "src", ROOT_DIR):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))

FALLBACK_PORT = "/dev/ttyS0"
FALLBACK_SLAVE = 2
MAX_COMMAND_PULSES = 8_388_608


def load_defaults() -> tuple[str, int]:
    port, slave = FALLBACK_PORT, FALLBACK_SLAVE
    try:
        import sampling_params as params
    except Exception:  # noqa: BLE001 - standalone utility keeps working
        return port, slave
    port = getattr(params, "X2P_PORT", port) or port
    try:
        slave = int(getattr(params, "X2P_SLAVE", slave))
    except (TypeError, ValueError):
        slave = FALLBACK_SLAVE
    return str(port), slave


def build_parser() -> argparse.ArgumentParser:
    port, slave = load_defaults()
    parser = argparse.ArgumentParser(
        description=(
            "配置X2P为位置模式+自动调整模式2；默认不使能、不运动、不触发辨识"
        )
    )
    parser.add_argument("--port", default=port, help=f"串口，默认 {port}")
    parser.add_argument(
        "--slave", type=int, default=slave, help=f"Modbus从站，默认 {slave}"
    )
    parser.add_argument(
        "--rigidity",
        type=int,
        default=14,
        help="Pn003刚性等级1..31，默认14",
    )
    parser.add_argument(
        "--command-pulses-per-rev",
        type=int,
        default=131_072,
        help="Pn008电机每转指令脉冲数；17bit=131072，23bit=8388608",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="将参数保存到驱动器EEPROM；现场常规配置必须加此项",
    )
    parser.add_argument(
        "--no-di-config",
        action="store_true",
        help="不写Pn400/Pn401；默认写DI1=SRV-ON、DI2=CTRG",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="确认已停机、机构安全且允许写参数",
    )
    return parser


def _read_u16(drive: object, address: int) -> int:
    return int(drive.read_registers(address)[0])


def _read_parameter(drive: object, address: int) -> int:
    """Read a parameter using its Modbus storage width.

    Pn008 is a 32-bit value stored low word first; all parameters this
    configuration utility otherwise touches are single 16-bit registers.
    """
    if address == Register.COMMAND_PULSES_PER_REV:
        return int(drive.read_signed32(address))
    return _read_u16(drive, address)


def _safe_read_u16(drive: object, address: int) -> int | str:
    try:
        return _read_u16(drive, address)
    except Exception as exc:  # noqa: BLE001 - monitor may be absent
        return f"读取失败({exc})"


def _verify_or_raise(
    drive: object, checks: list[tuple[str, int, int]]
) -> None:
    failures: list[str] = []
    for label, address, expected in checks:
        actual = _read_parameter(drive, address)
        print(f"  {label:<28} = {actual}")
        if actual != expected:
            failures.append(f"{label}: 期望{expected}，实际{actual}")
    if failures:
        raise RuntimeError("参数回读校验失败: " + "; ".join(failures))


def run(args: argparse.Namespace) -> int:
    if not 1 <= args.rigidity <= 31:
        print("错误：--rigidity必须在1..31之间", file=sys.stderr)
        return 2
    if not 1 <= args.command_pulses_per_rev <= MAX_COMMAND_PULSES:
        print(
            "错误：--command-pulses-per-rev必须在1..8388608之间",
            file=sys.stderr,
        )
        return 2

    from x2p import X2PDrive
    from x2p.protocol import signed16
    from x2p.registers import Register

    drive = X2PDrive(args.port, args.slave)
    try:
        print(f"端口={args.port} 从站={args.slave}")
        drive.diagnostic()
        print("Modbus线路诊断：通过")

        current = {
            "Pn001控制模式": _read_u16(drive, Register.CONTROL_MODE),
            "Pn002调整模式": _read_u16(drive, Register.TUNING_MODE),
            "Pn003刚性": _read_u16(drive, Register.RIGIDITY),
            "Pn004负载惯量比": _read_u16(drive, Register.LOAD_INERTIA),
            "Pn008每转指令脉冲": drive.read_signed32(
                Register.COMMAND_PULSES_PER_REV
            ),
            "Pn321位置指令来源": _read_u16(
                drive, Register.POSITION_SOURCE
            ),
            "Pn400DI1功能": _read_u16(drive, Register.DI1_FUNCTION),
            "Pn401DI2功能": _read_u16(drive, Register.DI2_FUNCTION),
            "Pn415强制输入": _read_u16(
                drive, Register.FORCE_DIGITAL_INPUTS
            ),
            "Un032DI状态": _read_u16(
                drive, Register.DIGITAL_INPUT_STATUS
            ),
            "STATUS运行状态": _read_u16(drive, Register.STATUS),
            "Un085惯量比": _safe_read_u16(
                drive, Register.INERTIA_MONITOR
            ),
        }
        print("--- 当前值 ---")
        for label, value in current.items():
            print(f"  {label:<28} = {value}")

        status = current["STATUS运行状态"]
        speed = signed16(_read_u16(drive, Register.ACTUAL_SPEED))
        if status == 2 or abs(speed) > 1:
            raise RuntimeError(
                "驱动器当前已使能或电机未静止，拒绝修改参数："
                f"STATUS={status}, Un000={speed} r/min"
            )

        planned = {
            "Pn001位置模式": 0,
            "Pn002自动调整模式2": 2,
            "Pn003刚性": args.rigidity,
            "Pn008每转指令脉冲": args.command_pulses_per_rev,
            "Pn321内部多段位置": 1,
            "Pn700选择Pn701段并立即执行": 7,
            "Pn701空闲": 0,
        }
        if not args.no_di_config:
            planned["Pn400DI1=SRV-ON"] = 1
            planned["Pn401DI2=CTRG"] = 11

        print("--- 计划写入 ---")
        for label, value in planned.items():
            print(f"  {label:<28} = {value}")
        if not args.save:
            print("警告：未指定--save，断电后参数可能恢复。")
        if not args.yes:
            print("只读检查完成；加--yes后才写参数。")
            return 2

        # Pn604=0 enables parameter writes.  Pn605=0 requests EEPROM save;
        # without --save use Pn605=1 so this configuration remains volatile.
        drive.write_register(Register.MODBUS_NO_SAVE, 0)
        drive.write_register(
            Register.MODBUS_SAVE_POLICY, 0 if args.save else 1
        )
        drive.write_register(Register.CONTROL_MODE, 0)
        drive.write_register(Register.TUNING_MODE, 2)
        drive.write_register(Register.RIGIDITY, args.rigidity)
        drive.write_signed32(
            Register.COMMAND_PULSES_PER_REV,
            args.command_pulses_per_rev,
        )
        drive.write_register(Register.POSITION_SOURCE, 1)
        drive.write_register(Register.POSITION_MODE, 7)
        drive.write_register(Register.POSITION_SEGMENT, 0)
        if not args.no_di_config:
            drive.write_register(Register.DI1_FUNCTION, 1)
            drive.write_register(Register.DI2_FUNCTION, 11)

        checks = [
            ("Pn001控制模式", Register.CONTROL_MODE, 0),
            ("Pn002调整模式", Register.TUNING_MODE, 2),
            ("Pn003刚性", Register.RIGIDITY, args.rigidity),
            (
                "Pn008每转指令脉冲",
                Register.COMMAND_PULSES_PER_REV,
                args.command_pulses_per_rev,
            ),
            ("Pn321位置指令来源", Register.POSITION_SOURCE, 1),
            ("Pn700内部位置模式", Register.POSITION_MODE, 7),
            ("Pn701当前段", Register.POSITION_SEGMENT, 0),
        ]
        if not args.no_di_config:
            checks.extend(
                [
                    ("Pn400DI1功能", Register.DI1_FUNCTION, 1),
                    ("Pn401DI2功能", Register.DI2_FUNCTION, 11),
                ]
            )
        print("--- 写入后回读 ---")
        _verify_or_raise(drive, checks)
        print(
            "  Un085惯量比(0.01倍)          = "
            f"{_safe_read_u16(drive, Register.INERTIA_MONITOR)}"
        )
        print("参数写入完成；程序未使能电机、未运动、未触发惯量辨识。")
        print("Pn001/Pn002/Pn008必须断电重新上电后才按新配置运行。")
        print(
            "上电后先运行 scripts/x2p_enable_diag.py 检查STATUS，"
            "再做低速小行程位置测试。"
        )
        return 0
    finally:
        drive.close()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except Exception as exc:  # noqa: BLE001 - operator-facing utility
        print(f"配置失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
