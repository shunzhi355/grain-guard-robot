#!/usr/bin/env python3
"""X2P 升降伺服 SRV-ON 使能链路诊断（直连串口，只写使能相关寄存器）。

什么时候用
----------
``scripts/manual_lift_adjust.py`` 报 **伺服使能超时：Un058 仍为 0** 时，
不要反复重试移动命令。先用本程序定位断在使能链的哪一段：

    DI1 功能(Pn400) → 强制输入(Pn415) → 驱动器看到的 DI(Un032) → 使能(Un058)

本程序会先只读快照，再依次尝试两个使能途径：先强制 DI1（Pn415 bit0=1，
与正式程序 `servo_on()` 用的是同一条路），再释放 DI1 并改试 Fn000=1。
每次尝试后立刻回读寄存器；最后**无论成功失败都撤销强制输入并取消使能**
（try/finally 保证）。

安全须知（务必先读）
--------------------
* 写 Pn415 可能让伺服**真实上电**。运行前确认：
  - 机构周围无人，升降轴不会撞到限位或工具；
  - 手放在物理急停上；
  - 已经停掉占用串口的服务：``sudo systemctl stop grain-sampling``。
* 本程序不会让电机转动（只写使能寄存器、不写转速/位置命令），
  但伺服上电后负载/重力可能让机构轻微动作。
* 诊断结束务必看到最后一行提示"已撤销强制输入并取消使能"。

用法
----
::

    cd "/home/neardi/project/grain guard robot/grain-guard-robot/grain_sampling_robot_software_share"
    export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
    sudo systemctl stop grain-sampling
    python3 scripts/x2p_enable_diag.py
    python3 scripts/x2p_enable_diag.py --port /dev/ttyS0 --slave 2

把整段输出（含每个寄存器值）发给维护人员即可定位问题。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
for _extra in (ROOT_DIR / "src", ROOT_DIR):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))


FALLBACK_PORT = "/dev/ttyS0"
FALLBACK_SLAVE = 2


def load_defaults() -> tuple[str, int]:
    """从 sampling_params.py 读取现场默认串口/从站，缺失时用内置值。"""
    port, slave = FALLBACK_PORT, FALLBACK_SLAVE
    try:
        import sampling_params as params
    except Exception:  # noqa: BLE001 - 拿不到就继续用内置值
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
        description="X2P SRV-ON 使能链路诊断（会真实写使能寄存器）"
    )
    parser.add_argument("--port", default=port, help=f"串口，默认 {port}")
    parser.add_argument(
        "--slave", type=int, default=slave, help=f"Modbus 从站，默认 {slave}"
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="每次写寄存器后等待回读的时间（秒），默认 0.5",
    )
    parser.add_argument(
        "--watch",
        type=float,
        default=0.0,
        help=(
            "强制 DI1 后持续观察使能状态的秒数（默认0=只按 --settle 采一次）。"
            "用于区分“根本不上电”和“上电比程序等得慢”"
        ),
    )
    parser.add_argument(
        "--confirm",
        default="",
        help="确认现场安全后传 ENABLE，程序才真正写使能寄存器",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只做只读快照，不写任何寄存器",
    )
    parser.add_argument(
        "--control",
        action="store_true",
        help=(
            "额外做对照试验：把一个空闲 DI 强制为有效，确认 Pn415 强制通道"
            "本身能不能改变 Un032。用于区分“驱动器拒绝使能”和“强制输入根本没生效”"
        ),
    )
    return parser


def _fmt(value: object) -> str:
    if isinstance(value, int) and not isinstance(value, bool):
        return f"{value} (0x{value & 0xFFFF:04X})"
    return str(value)


def print_snapshot(title: str, values: dict[str, object]) -> None:
    print(f"\n--- {title} ---")
    for key, value in values.items():
        print(f"  {key:<16} = {_fmt(value)}")


def spare_di_number(values: dict[str, object]) -> int | None:
    """挑一个没有分配功能的 DI 做强制输入对照试验。

    返回 1..4 中的编号（优先靠后的 DI），全部都被占用时返回 None。
    """
    free: list[int] = []
    for di in range(1, 5):
        function = values.get(f"Pn{400 + di - 1}_DI{di}功能")
        if isinstance(function, int) and function == 0:
            free.append(di)
    return free[-1] if free else None


def forced_bit_changed(
    baseline: dict[str, object],
    after: dict[str, object],
    di_number: int,
) -> bool:
    """对照试验判据：强制某个 DI 后，Un032 对应位是否真的从0变1。"""
    from x2p.registers import digital_input_bit

    bit = digital_input_bit(di_number)
    before = baseline.get("Un032_DI状态")
    now = after.get("Un032_DI状态")
    if not isinstance(before, int) or not isinstance(now, int):
        return False
    return not before & bit and bool(now & bit)


def read_full_snapshot(drive: object) -> dict[str, object]:
    """使能链路快照 + DI1..DI4 功能分配 + Modbus 写入策略。

    自带使能链路读取，不依赖 ``X2PDrive.read_enable_chain()``：板端可能仍是
    旧版 ``src/x2p/``，没有那个方法。除链路六项外还多读 DI1..DI4 的功能分配
    与 Modbus 写入策略(Pn604/Pn605)，用来回答两个现场高频问题：SRV-ON 究竟
    配在哪个 DI、参数写入策略是不是被改过。单点读失败只记录文本，不中断诊断。
    """
    from x2p.registers import Register, digital_input_function_register

    values: dict[str, object] = {}
    for label, address in (
        ("P400_DI1功能", Register.DI1_FUNCTION),
        ("P415_强制输入", Register.FORCE_DIGITAL_INPUTS),
        ("Un032_DI状态", Register.DIGITAL_INPUT_STATUS),
        ("Un058_伺服使能", Register.SERVO_ENABLE_STATUS),
        ("STATUS_0x3E00", Register.STATUS),
        ("Un100_故障码", Register.LAST_FAULT_CODE),
        ("Un000_转速", Register.ACTUAL_SPEED),
    ):
        try:
            values[label] = drive.read_registers(address)[0]
        except Exception as exc:  # noqa: BLE001 - 单点失败不中断诊断
            values[label] = f"读取失败({exc})"
    for di in range(1, 5):
        label = f"Pn{400 + di - 1}_DI{di}功能"
        try:
            values[label] = drive.read_registers(
                digital_input_function_register(di)
            )[0]
        except Exception as exc:  # noqa: BLE001 - 单点失败不中断诊断
            values[label] = f"读取失败({exc})"
    for label, address in (
        ("Pn604_写入策略", Register.MODBUS_NO_SAVE),
        ("Pn605_保存策略", Register.MODBUS_SAVE_POLICY),
    ):
        try:
            values[label] = drive.read_registers(address)[0]
        except Exception as exc:  # noqa: BLE001
            values[label] = f"读取失败({exc})"
    return values


def interpret(after_force: dict[str, object], after_fn: dict[str, object]) -> list[str]:
    """把两次尝试后的快照翻译成结论列表。"""
    conclusions: list[str] = []
    function = after_force.get("P400_DI1功能")
    forced = after_force.get("P415_强制输入")
    inputs = after_force.get("Un032_DI状态")
    enabled = after_force.get("Un058_伺服使能")
    fn_enabled = after_fn.get("Un058_伺服使能")

    di_functions = [
        after_force.get(f"Pn{400 + di - 1}_DI{di}功能") for di in range(1, 5)
    ]
    if all(isinstance(value, int) for value in di_functions):
        srv_on_dis = [
            di for di, value in enumerate(di_functions, start=1) if value == 1
        ]
        if not srv_on_dis:
            conclusions.append(
                "Pn400..Pn403 里没有任何 DI 配成 SRV-ON(功能1)："
                "驱动器不会接受软件使能；先把某个 DI 功能改成 1 再重新上电"
            )
        elif 1 not in srv_on_dis:
            listed = "、".join(f"DI{di}" for di in srv_on_dis)
            conclusions.append(
                f"SRV-ON 实际配在 {listed}，程序强制的却是 DI1："
                "把接线或配置对齐后重试"
            )

    if isinstance(function, int) and function != 1:
        conclusions.append(
            f"Pn400={function}：DI1 没有配置成 SRV-ON(1)，"
            "强制 DI1 当然不会使能；先把 Pn400 改成 1 并重新上电"
        )
    elif isinstance(forced, int) and not forced & 0x01:
        conclusions.append(
            "Pn415 回读第0位为0：强制写入没有生效，"
            "怀疑 Modbus 写进了缓存/驱动器不支持该参数"
        )
    elif isinstance(inputs, int) and not inputs & 0x01:
        conclusions.append(
            "Pn415 已置位但 Un032 第0位为0：驱动器没有把 DI1 认作有效输入，"
            "优先查 DI 公共端(COM)、24V 供电和端子接线"
        )
    elif isinstance(enabled, int) and enabled == 0:
        conclusions.append(
            "DI1 已有效但 Un058 仍为0：驱动器主动拒绝使能，"
            "优先看驱动面板 E 码/报警、主电源(动力电)是否上电、外部急停"
        )
    else:
        conclusions.append("强制 DI1 后 Un058 已置位：使能链路是通的")

    if isinstance(fn_enabled, int) and fn_enabled != 0 and (
        not isinstance(enabled, int) or enabled == 0
    ):
        conclusions.append(
            "Fn000=1 能让 Un058 置位：应把 servo_on() 改成走 Fn000 内部使能"
        )
    elif isinstance(fn_enabled, int) and fn_enabled == 0 and (
        isinstance(enabled, int) and enabled == 0
    ):
        conclusions.append(
            "Fn000=1 也不生效：问题在驱动器上电/保护状态，不在 Modbus 控制方式"
        )
    return conclusions


def run(args: argparse.Namespace) -> int:
    try:
        import serial  # noqa: F401 - 仅用于给出缺依赖的明确提示
    except ImportError:
        print(
            "错误：未安装 pyserial。板端执行 python3 -m pip install pyserial",
            file=sys.stderr,
        )
        return 1

    from x2p import X2PDrive
    from x2p.registers import Register, digital_input_bit

    write_enabled = bool(args.confirm == "ENABLE") and not args.dry_run
    if not write_enabled:
        print("=" * 68)
        print("只读模式：本次不会写任何寄存器。")
        print("确认现场安全后，加 --confirm ENABLE 才能执行写使能诊断。")
        print("=" * 68)

    drive = X2PDrive(args.port, args.slave)
    try:
        print(f"端口={args.port} 从站={args.slave}")
        try:
            drive.diagnostic()
            print("Modbus 0x08 线路诊断：通过")
        except Exception as exc:  # noqa: BLE001 - 诊断失败也继续读快照
            print(f"Modbus 0x08 线路诊断：失败({exc})")

        baseline = read_full_snapshot(drive)
        print_snapshot("步骤1 只读基线", baseline)
        if not write_enabled:
            return 0

        settle = max(0.0, float(args.settle))
        watch = max(0.0, float(args.watch))
        print(f"\n步骤2 写 Fn000=0 + 强制 DI1（Pn415 bit0=1），等待 {settle:.1f}s")
        drive.write_register(Register.INTERNAL_SERVO_ON, 0)
        drive.force_digital_inputs(digital_input_bit(1))
        time.sleep(settle)
        if watch:
            print(f"持续观察使能状态 {watch:.1f}s（每 0.5s 采一次 Un058/Un032）：")
            deadline = time.monotonic() + watch
            while time.monotonic() < deadline:
                print(
                    f"  Un058={drive.read_registers(Register.SERVO_ENABLE_STATUS)[0]}"
                    f" Un032={drive.read_registers(Register.DIGITAL_INPUT_STATUS)[0]}"
                    f" Pn415={drive.read_registers(Register.FORCE_DIGITAL_INPUTS)[0]}"
                )
                time.sleep(0.5)
        after_force = read_full_snapshot(drive)
        print_snapshot("步骤2 结果", after_force)

        print(
            f"\n步骤3 先撤销强制输入(Pn415=0)，再改试 Fn000=1（内部使能），"
            f"等待 {settle:.1f}s"
        )
        drive.force_digital_inputs(0)
        drive.write_register(Register.INTERNAL_SERVO_ON, 1)
        time.sleep(settle)
        after_fn = read_full_snapshot(drive)
        print_snapshot("步骤3 结果", after_fn)

        control_result: tuple[int, dict[str, object]] | None = None
        if args.control:
            spare = spare_di_number(after_fn)
            if spare is None:
                print(
                    "\n步骤4 对照试验：跳过，DI1..DI4 都分配了功能，"
                    "没有空闲端子可用来做对照"
                )
            else:
                bit = digital_input_bit(spare)
                print(
                    f"\n步骤4 对照试验：Fn000=0，改强制 DI{spare}"
                    f"（Pn415 bit{spare - 1}，该端子功能为0），等待 {settle:.1f}s"
                )
                drive.write_register(Register.INTERNAL_SERVO_ON, 0)
                drive.force_digital_inputs(bit)
                time.sleep(settle)
                after_control = read_full_snapshot(drive)
                print_snapshot(f"步骤4 结果（强制 DI{spare}）", after_control)
                control_result = (spare, after_control)

        print("\n--- 判读 ---")
        for line in interpret(after_force, after_fn):
            print(f"  * {line}")
        if control_result is not None:
            spare, after_control = control_result
            if forced_bit_changed(
                baseline, after_control, spare
            ):
                print(
                    f"  * [对照] 强制 DI{spare} 后 Un032 bit{spare - 1} 由0变1："
                    "Pn415 强制通道本身是通的，DI1 那条路的问题是真的"
                )
            else:
                print(
                    f"  * [对照] 强制 DI{spare} 后 Un032 bit{spare - 1} 没有变化："
                    "Pn415 很可能不是本驱动器的强制输入寄存器（或写入被忽略），"
                    "软件使能这条路整体可疑；先用硬件方式把 DI1 接 24V+COM 复核"
                )
        return 0
    finally:
        if write_enabled:
            try:
                drive.force_digital_inputs(0)
                drive.write_register(Register.INTERNAL_SERVO_ON, 0)
                print("\n[安全] 已撤销强制输入并取消使能(Fn000=0, Pn415=0)。")
            except Exception as exc:  # noqa: BLE001 - 必须告知用户撤销失败
                print(
                    f"\n[警告] 撤销强制输入/使能失败：{exc}；"
                    "请立即用物理急停或断动力电处理！",
                    file=sys.stderr,
                )
        try:
            drive.close()
        except Exception:  # noqa: BLE001
            pass


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.settle < 0:
        print("错误：--settle 不能为负", file=sys.stderr)
        return 2
    try:
        return run(args)
    except Exception as exc:  # noqa: BLE001 - 现场工具统一报错退出
        print(f"诊断失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
