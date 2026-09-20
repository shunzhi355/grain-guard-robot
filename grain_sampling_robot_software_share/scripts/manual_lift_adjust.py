#!/usr/bin/env python3
"""X2P 升降伺服 —— 手动调整初始位置（交互式，直连串口，不依赖 ROS）。

为什么需要这个程序
------------------
X2P 伺服没有断电保持的多圈绝对编码器：断电重新上电后编码器计数从零开始，
驱动器并不知道升降机构此刻停在行程的哪个位置。所以每次上电后，如果机构
不在物理最高点，就需要人工把它移回去 —— 本程序就是这个人工工具。

特点
----
* 直接经 RS485（Modbus RTU）操作 X2P 伺服，不依赖 ROS，也不依赖
  ``/mechanism/move_lift`` 服务；机构节点没起来也能用。
* 操作员反复输入“方向 + 距离”，程序用编码器闭环做相对移动；每次移动前
  打印当前编码器位置，移动后打印实际位移和结果。
* 输入 ``q`` 退出。

用法
----
::

    cd "/home/neardi/project/grain guard robot/grain-guard-robot/grain_sampling_robot_software_share"
    export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
    python3 scripts/manual_lift_adjust.py             # 交互模式，默认 /dev/ttyS0
    python3 scripts/manual_lift_adjust.py up 0.5      # 单次：从当前位置上升 0.5 cm
    python3 scripts/manual_lift_adjust.py --status    # 只读位置，不移动

交互模式命令::

    up 0.5      从当前位置上升 0.5 cm（也支持 上 / 上升 / 提升）
    down 1.2    从当前位置下降 1.2 cm（也支持 下 / 下降 / 下压）
    s           只读打印编码器位置
    m           把当前位置记为“物理最高点”（只在本次运行内有效）
    clear       清除上面记录的参考点
    q           退出

安全须知
--------
* 系统当前没有机械原点回零，也没有全行程软限位。
* 单次移动距离默认最多 20 cm，可用 ``--max-cm`` 或环境变量
  ``MANUAL_LIFT_MAX_CM`` 修改。
* 首次调试请用 0.5～2 cm 的小距离，并留在急停按钮旁边。
* 用 ``m`` 记录物理最高点并给出 ``--travel-mm`` 后，程序会在本次运行内
  拒绝超出总行程的移动；退出程序后该参考点失效（编码器本身不保持）。
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
# 板端一般已经 export PYTHONPATH="$PWD/src"；这里再兜底一次，便于直接运行。
for _extra in (ROOT_DIR / "src", ROOT_DIR):
    if str(_extra) not in sys.path:
        sys.path.insert(0, str(_extra))


#: 内置兜底默认值；板端会被 sampling_params.py 覆盖。
_FALLBACK_PARAMS: dict[str, object] = {
    "port": "/dev/ttyS0",
    "slave": 2,
    "rpm": 200,
    "forward_sign": 1,
    "tolerance_mm": 2.0,
    "screw_lead_mm": 5.0,
    "motor_revs_per_screw_rev": 1.0,
    "encoder_counts_per_motor_rev": 131_072,
}

#: 操作员输入的方向词 → 驱动器方向名（与 x2p_lift._map_lift_direction 一致）。
_DIRECTION_ALIASES: dict[str, str] = {
    "up": "forward",
    "上": "forward",
    "上升": "forward",
    "提升": "forward",
    "forward": "forward",
    "fwd": "forward",
    "+": "forward",
    "正向": "forward",
    "正转": "forward",
    "down": "reverse",
    "下": "reverse",
    "下降": "reverse",
    "下压": "reverse",
    "reverse": "reverse",
    "rev": "reverse",
    "-": "reverse",
    "反向": "reverse",
    "反转": "reverse",
}

_DIRECTION_TEXT: dict[str, str] = {"forward": "上升", "reverse": "下降"}

QUIT_COMMANDS = {"q", "quit", "exit", "退出", "结束"}
STATUS_COMMANDS = {"s", "status", "状态"}
MARK_COMMANDS = {"m", "mark", "原点", "最高点"}
CLEAR_COMMANDS = {"clear", "清除", "取消原点"}


def load_defaults() -> dict[str, object]:
    """从 sampling_params.py 读取现场标定默认值，缺失时退回内置值。"""
    defaults = dict(_FALLBACK_PARAMS)
    try:
        import sampling_params as params
    except Exception:  # noqa: BLE001 - 拿不到就继续用内置值
        return defaults
    mapping = {
        "port": "X2P_PORT",
        "slave": "X2P_SLAVE",
        "rpm": "X2P_RPM",
        "forward_sign": "X2P_FORWARD_SIGN",
        "tolerance_mm": "X2P_POSITION_TOLERANCE_MM",
    }
    for key, attribute in mapping.items():
        value = getattr(params, attribute, None)
        if value is not None:
            defaults[key] = value
    return defaults


def parse_direction(text: object) -> str:
    """把操作员输入的方向词翻译成驱动器方向 forward / reverse。"""
    key = str(text).strip().lower()
    try:
        return _DIRECTION_ALIASES[key]
    except KeyError:
        raise ValueError(
            f"无法识别的方向 {text!r}；请输入 up/上/上升/提升 "
            "或 down/下/下降/下压"
        ) from None


def parse_distance_cm(text: object) -> float:
    """把操作员输入的距离文本解析成大于 0 的 cm 数值。"""
    try:
        value = float(str(text).strip())
    except (TypeError, ValueError):
        raise ValueError(f"距离 {text!r} 不是有效数字，单位是 cm") from None
    if not math.isfinite(value) or value <= 0:
        raise ValueError("距离必须是大于 0 的有限数字，单位是 cm")
    return value


def max_distance_cm(environ: dict[str, str] | None = None) -> float:
    """单次移动上限（cm）；与 scripts/move_lift.py 使用同一环境变量。"""
    source = os.environ if environ is None else environ
    raw = source.get("MANUAL_LIFT_MAX_CM", "20")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise ValueError("MANUAL_LIFT_MAX_CM 必须是数字") from None
    if not math.isfinite(value) or value <= 0:
        raise ValueError("MANUAL_LIFT_MAX_CM 必须大于 0")
    return value


def duration_for_move(
    distance_mm: float,
    rpm: float,
    *,
    screw_lead_mm: float = 5.0,
    motor_revs_per_screw_rev: float = 1.0,
    margin: float = 1.2,
) -> float:
    """按转速和导程算出移动时长，留 20% 余量避免低速接近段超速。

    与 mechanism_node._handle_move_lift 的算法一致，保证手动程序与自动
    流程使用相同的速度和接近段行为。
    """
    if not distance_mm > 0:
        raise ValueError("distance_mm必须大于0")
    if not rpm > 0:
        raise ValueError("rpm必须大于0")
    if screw_lead_mm <= 0 or motor_revs_per_screw_rev <= 0:
        raise ValueError("丝杆导程和传动比必须大于0")
    motor_revolutions = distance_mm / screw_lead_mm * motor_revs_per_screw_rev
    return motor_revolutions * 60.0 / rpm * margin


def resolve_tolerance_mm(requested_mm: float, distance_mm: float) -> float:
    """保证停止容差小于移动距离（move_timed_distance 的硬性要求）。"""
    if not requested_mm > 0:
        raise ValueError("位置容差必须大于 0")
    if requested_mm < distance_mm:
        return requested_mm
    return max(1e-3, distance_mm * 0.5)


def projected_up_offset_mm(
    *,
    direction: str,
    distance_mm: float,
    origin_count: int,
    current_count: int,
    counts_per_mm: float,
    encoder_forward_sign: int = 1,
) -> float:
    """移动后相对“最高点”的位移：0=在最高点，负值=在最高点下方。"""
    up_offset_mm = (
        (current_count - origin_count)
        * int(encoder_forward_sign)
        / float(counts_per_mm)
    )
    step_mm = distance_mm if direction == "forward" else -distance_mm
    return up_offset_mm + step_mm


@dataclass(frozen=True)
class ManualAdjustOptions:
    """本程序的有效参数（命令行 / 环境变量 / sampling_params 合并结果）。"""

    rpm: int
    max_cm: float
    tolerance_mm: float
    screw_lead_mm: float = 5.0
    motor_revs_per_screw_rev: float = 1.0
    encoder_counts_per_motor_rev: int = 131_072
    travel_mm: float = 0.0
    encoder_forward_sign: int = 1


class ManualLiftAdjuster:
    """交互式相对移动控制器；驱动通过 duck-type 注入，便于单元测试。"""

    def __init__(
        self,
        controller: object,
        options: ManualAdjustOptions,
        *,
        output: Callable[[str], None] = print,
        input_fn: Callable[[str], str] = input,
    ):
        self.controller = controller
        self.options = options
        self.output = output
        self.input_fn = input_fn
        #: 操作员用 m 命令记录的“物理最高点”编码器计数（仅本次运行有效）。
        self.origin_count: int | None = None

    # ── 基础读取 ──────────────────────────────────────────────
    def counts_per_mm(self) -> float:
        """编码器计数/mm；优先用控制器配置，缺失时按丝杆参数换算。"""
        config = getattr(self.controller, "config", None)
        configured = getattr(config, "pulses_per_mm", None)
        if configured:
            return float(configured)
        lead = float(
            getattr(config, "screw_lead_mm", None) or self.options.screw_lead_mm
        )
        ratio = float(
            getattr(config, "motor_revs_per_screw_rev", None)
            or self.options.motor_revs_per_screw_rev
        )
        counts = float(
            getattr(config, "encoder_counts_per_motor_rev", None)
            or self.options.encoder_counts_per_motor_rev
        )
        return counts * ratio / lead

    def encoder_sign(self) -> int:
        config = getattr(self.controller, "config", None)
        sign = getattr(config, "encoder_forward_sign", None)
        if sign in (-1, 1):
            return int(sign)
        return int(self.options.encoder_forward_sign)

    def read_position(self) -> int | None:
        try:
            return int(self.controller.read_encoder_position())
        except Exception as exc:  # noqa: BLE001 - 读位置失败不致命
            self.output(f"警告：读取编码器位置失败：{exc}")
            return None

    def distance_to_origin_mm(self, current_position: int) -> float:
        """距记录的最高点还有多少 mm（正值=还在最高点下方）。"""
        if self.origin_count is None:
            raise ValueError("尚未记录物理最高点")
        up_offset_mm = (
            (int(current_position) - int(self.origin_count))
            * self.encoder_sign()
            / self.counts_per_mm()
        )
        return -up_offset_mm

    def describe_state(self) -> str:
        position = self.read_position()
        if position is None:
            return "当前状态：编码器位置读取失败（检查串口与驱动器供电）"
        text = f"当前编码器位置 {position}"
        if self.origin_count is None:
            text += "；尚未记录物理最高点（交互命令 m）"
        else:
            text += f"；距记录的最高点 {self.distance_to_origin_mm(position):.3f} mm"
        return text

    def best_effort_stop(self) -> None:
        try:
            self.controller.stop(verify_off=False)
        except Exception as exc:  # noqa: BLE001 - 紧急停机尽力而为
            self.output(f"紧急停止命令未完全确认：{exc}")

    # ── 移动 ──────────────────────────────────────────────────
    def _check_travel(
        self, direction: str, distance_mm: float, start_position: int | None
    ) -> None:
        """有参考点时拒绝越界移动；没有参考点时不做行程判断。"""
        if self.origin_count is None or start_position is None:
            return
        projected_mm = projected_up_offset_mm(
            direction=direction,
            distance_mm=distance_mm,
            origin_count=int(self.origin_count),
            current_count=int(start_position),
            counts_per_mm=self.counts_per_mm(),
            encoder_forward_sign=self.encoder_sign(),
        )
        if projected_mm > 0:
            raise ValueError(
                "本次移动会越过记录的最高点"
                f"（超出 {projected_mm:.3f} mm）；请先用 m 重新记录最高点"
            )
        if self.options.travel_mm > 0 and -projected_mm > self.options.travel_mm:
            raise ValueError(
                f"本次移动后距最高点 {-projected_mm:.3f} mm，超过总行程 "
                f"{self.options.travel_mm:g} mm；请分多次移动"
            )

    def _report_remaining(self, current_position: int | None) -> None:
        if self.origin_count is None or current_position is None:
            return
        text = (
            f"距记录的最高点 {self.distance_to_origin_mm(current_position):.3f} mm"
        )
        if self.options.travel_mm > 0:
            text += f"（总行程 {self.options.travel_mm:g} mm）"
        self.output(text)

    def execute(self, direction_text: object, distance_text: object) -> dict:
        """校验并执行一次相对移动；输入非法或超限时抛 ValueError。"""
        direction = parse_direction(direction_text)
        distance_cm = parse_distance_cm(distance_text)
        options = self.options
        if distance_cm > options.max_cm:
            raise ValueError(
                f"单次距离 {distance_cm:g} cm 超过安全上限 "
                f"{options.max_cm:g} cm；需要更长距离请分多次移动，"
                "或显式提高 --max-cm"
            )
        distance_mm = distance_cm * 10.0
        tolerance_mm = resolve_tolerance_mm(options.tolerance_mm, distance_mm)
        duration_s = duration_for_move(
            distance_mm,
            options.rpm,
            screw_lead_mm=options.screw_lead_mm,
            motor_revs_per_screw_rev=options.motor_revs_per_screw_rev,
        )
        start_position = self.read_position()
        self._check_travel(direction, distance_mm, start_position)
        if tolerance_mm != options.tolerance_mm:
            self.output(
                f"提示：本次移动距离较小，位置容差自动收紧为 {tolerance_mm:.4f} mm"
            )
        self.output(
            f"准备执行：{_DIRECTION_TEXT[direction]} {distance_cm:g} cm"
            f"（驱动器方向 {direction}，时长 {duration_s:.2f} s，"
            f"容差 {tolerance_mm:g} mm）；本程序不会自动寻找机械零点。"
        )
        result = self.controller.move_timed_distance(
            direction, distance_mm, duration_s, tolerance_mm=tolerance_mm
        )
        final_position = self.read_position()
        up_delta_mm: float | None = None
        if start_position is not None and final_position is not None:
            up_delta_mm = (
                (final_position - start_position)
                * self.encoder_sign()
                / self.counts_per_mm()
            )
            self.output(
                f"实际位移 {up_delta_mm:+.3f} mm（目标 {distance_mm:g} mm；"
                f"编码器 {start_position} → {final_position}）"
            )
            signed_target_mm = (
                distance_mm if direction == "forward" else -distance_mm
            )
            if abs(up_delta_mm) + tolerance_mm < distance_mm:
                self.output("警告：实际位移明显小于目标，请检查是否卡阻或方向错误。")
            elif up_delta_mm * signed_target_mm < 0:
                self.output(
                    "警告：编码器位移方向与指令相反，"
                    "请检查 forward_sign 与接线方向。"
                )
        self._report_remaining(final_position)
        self.output("执行完成。")
        return {
            "direction": direction,
            "distance_mm": distance_mm,
            "duration_s": duration_s,
            "tolerance_mm": tolerance_mm,
            "start_position": start_position,
            "final_position": final_position,
            "up_delta_mm": up_delta_mm,
            "result": result,
        }

    # ── 参考点 ────────────────────────────────────────────────
    def mark_origin(self) -> None:
        position = self.read_position()
        if position is None:
            self.output("无法记录最高点：读取编码器位置失败。")
            return
        self.origin_count = position
        self.output(
            f"已把当前编码器位置 {position} 记为物理最高点（只在本次运行内有效）。"
        )
        if self.options.travel_mm > 0:
            self.output(
                f"本次运行会拒绝超出总行程 {self.options.travel_mm:g} mm 的移动。"
            )
        else:
            self.output("提示：未指定 --travel-mm，因此不会做行程软限位。")

    def clear_origin(self) -> None:
        if self.origin_count is None:
            self.output("当前没有记录最高点。")
            return
        self.origin_count = None
        self.output("已清除记录的物理最高点。")

    # ── 交互循环 ──────────────────────────────────────────────
    def print_banner(self) -> None:
        options = self.options
        self.output("=" * 72)
        self.output("X2P 升降伺服 —— 手动调整初始位置")
        self.output("=" * 72)
        self.output(
            "警告：X2P 伺服没有断电保持的多圈编码器，上电后驱动器不知道"
            "机构停在行程的哪个位置。"
        )
        self.output(
            "警告：本程序只做相对移动，不会自动回机械零点；系统当前也没有"
            "全行程软限位。"
        )
        self.output(
            f"本次运行：转速 {options.rpm} r/min，单次距离上限 "
            f"{options.max_cm:g} cm，位置容差 {options.tolerance_mm:g} mm"
        )
        if options.travel_mm > 0:
            self.output(
                f"行程软限位：{options.travel_mm:g} mm"
                "（需要先用 m 记录物理最高点）"
            )
        else:
            self.output(
                "行程软限位：未启用"
                "（可加 --travel-mm 指定总行程，再用 m 记录最高点）"
            )
        self.output(self.describe_state())
        self.output("首次调试请用 0.5～2 cm 的小距离，并留在急停按钮旁边。")
        self.output("=" * 72)

    def repl(self) -> int:
        self.print_banner()
        while True:
            try:
                line = self.input_fn(
                    "请输入命令（up/down + 距离cm；s 查看，m 记最高点，q 退出）> "
                )
            except EOFError:
                self.output("输入结束，退出。")
                return 0
            command = str(line).strip()
            if not command:
                continue
            tokens = command.split()
            head = tokens[0].strip().lower()
            if head in QUIT_COMMANDS:
                self.output("退出手动调整程序。")
                return 0
            if head in STATUS_COMMANDS:
                self.output(self.describe_state())
                continue
            if head in MARK_COMMANDS:
                self.mark_origin()
                continue
            if head in CLEAR_COMMANDS:
                self.clear_origin()
                continue
            try:
                if len(tokens) == 1:
                    distance_text = self.input_fn("请输入本次移动距离（cm）> ")
                    if not str(distance_text).strip():
                        self.output("已取消本次移动。")
                        continue
                elif len(tokens) == 2:
                    distance_text = tokens[1]
                else:
                    raise ValueError("命令格式是“方向 距离”，例如 up 0.5")
                self.execute(tokens[0], distance_text)
            except ValueError as exc:
                self.output(f"拒绝执行：{exc}")
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001 - 单次失败不退出循环
                self.output(f"执行失败：{exc}")
                self.best_effort_stop()
                self.output("本次未完成，可重新输入；反复失败请先用 s 查看状态。")


def build_parser(defaults: dict) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="X2P 升降伺服手动调整初始位置（交互式，直连串口，不依赖 ROS）",
    )
    parser.add_argument(
        "direction",
        nargs="?",
        help="单次模式：up/down（也支持 上/下、上升/下压）",
    )
    parser.add_argument("distance_cm", nargs="?", help="单次模式：移动距离 cm")
    parser.add_argument(
        "--port",
        default=None,
        help=f"X2P 伺服串口，默认 {defaults['port']}（可用 X2P_PORT 覆盖）",
    )
    parser.add_argument(
        "--slave",
        type=int,
        default=None,
        help=f"Modbus 从站地址，默认 {defaults['slave']}",
    )
    parser.add_argument(
        "--rpm",
        type=int,
        default=None,
        help=f"升降转速 r/min，默认 {defaults['rpm']}（可用 X2P_RPM 覆盖）",
    )
    parser.add_argument(
        "--tolerance-mm",
        type=float,
        default=None,
        help=f"停止位置容差 mm，默认 {defaults['tolerance_mm']}",
    )
    parser.add_argument(
        "--max-cm",
        type=float,
        default=None,
        help="单次最大距离 cm，默认读环境变量 MANUAL_LIFT_MAX_CM（20）",
    )
    parser.add_argument(
        "--travel-mm",
        type=float,
        default=0.0,
        help="总行程 mm；>0 时配合交互命令 m 做本次运行内的软限位",
    )
    parser.add_argument(
        "--screw-lead-mm",
        type=float,
        default=None,
        help=f"丝杆导程 mm，默认 {defaults['screw_lead_mm']}",
    )
    parser.add_argument(
        "--motor-revs-per-screw-rev",
        type=float,
        default=None,
        help="电机转数 / 丝杆转数，默认 1.0",
    )
    parser.add_argument(
        "--forward-sign",
        type=int,
        default=None,
        choices=[1, -1],
        help=f"升降方向翻转，默认 {defaults['forward_sign']}",
    )
    parser.add_argument(
        "--status",
        action="store_true",
        help="只读打印当前编码器位置后退出（不移动）",
    )
    return parser


def resolve_options(args: argparse.Namespace, defaults: dict) -> ManualAdjustOptions:
    """合并命令行、环境变量与 sampling_params 默认值，并校验范围。"""
    rpm = (
        args.rpm
        if args.rpm is not None
        else int(os.environ.get("X2P_RPM", str(defaults["rpm"])))
    )
    forward_sign = (
        args.forward_sign
        if args.forward_sign is not None
        else int(os.environ.get("X2P_FORWARD_SIGN", str(defaults["forward_sign"])))
    )
    tolerance_mm = (
        args.tolerance_mm
        if args.tolerance_mm is not None
        else float(defaults["tolerance_mm"])
    )
    max_cm = args.max_cm if args.max_cm is not None else max_distance_cm()
    screw_lead_mm = (
        args.screw_lead_mm
        if args.screw_lead_mm is not None
        else float(defaults["screw_lead_mm"])
    )
    motor_revs_per_screw_rev = (
        args.motor_revs_per_screw_rev
        if args.motor_revs_per_screw_rev is not None
        else float(defaults["motor_revs_per_screw_rev"])
    )
    if not rpm > 0:
        raise ValueError("--rpm 必须大于 0")
    if not max_cm > 0:
        raise ValueError("--max-cm 必须大于 0")
    if not tolerance_mm > 0:
        raise ValueError("--tolerance-mm 必须大于 0")
    if args.travel_mm < 0:
        raise ValueError("--travel-mm 不能为负")
    if forward_sign not in (-1, 1):
        raise ValueError("--forward-sign 只能是 1 或 -1")
    if screw_lead_mm <= 0 or motor_revs_per_screw_rev <= 0:
        raise ValueError("丝杆导程和传动比必须大于 0")
    return ManualAdjustOptions(
        rpm=int(rpm),
        max_cm=float(max_cm),
        tolerance_mm=float(tolerance_mm),
        screw_lead_mm=float(screw_lead_mm),
        motor_revs_per_screw_rev=float(motor_revs_per_screw_rev),
        encoder_counts_per_motor_rev=int(
            defaults["encoder_counts_per_motor_rev"]
        ),
        travel_mm=float(args.travel_mm),
        encoder_forward_sign=int(forward_sign),
    )


def build_config(
    options: ManualAdjustOptions,
    *,
    port: str,
    slave: int,
    forward_sign: int,
    config_cls: object,
    limits_cls: object,
) -> object:
    """构造 ControllerConfig；max_rpm/max_distance_mm 覆盖为手动程序的要求。"""
    return config_cls(
        port=port,
        slave=slave,
        screw_lead_mm=options.screw_lead_mm,
        motor_revs_per_screw_rev=options.motor_revs_per_screw_rev,
        forward_sign=int(forward_sign),
        limits=limits_cls(
            max_rpm=max(30, options.rpm),
            max_distance_mm=max(300.0, options.max_cm * 10.0),
        ),
    )


def main(argv: list[str] | None = None) -> int:
    defaults = load_defaults()
    args = build_parser(defaults).parse_args(argv)
    port = args.port or os.environ.get("X2P_PORT") or str(defaults["port"])
    slave = (
        args.slave
        if args.slave is not None
        else int(os.environ.get("X2P_SLAVE", str(defaults["slave"])))
    )
    try:
        options = resolve_options(args, defaults)
    except (TypeError, ValueError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    try:
        from x2p import ControllerConfig, MotionController, SafetyLimits, X2PDrive
    except ImportError as exc:
        print(
            f"错误：无法导入 x2p 包（{exc}）。请确认板端已安装 pyserial，"
            '并用 PYTHONPATH="$PWD/src" 运行本脚本。',
            file=sys.stderr,
        )
        return 1

    forward_sign = (
        args.forward_sign
        if args.forward_sign is not None
        else int(os.environ.get("X2P_FORWARD_SIGN", str(defaults["forward_sign"])))
    )
    drive = None
    try:
        drive = X2PDrive(port, slave)
        config = build_config(
            options,
            port=port,
            slave=slave,
            forward_sign=forward_sign,
            config_cls=ControllerConfig,
            limits_cls=SafetyLimits,
        )
        controller = MotionController(
            drive, config, output=lambda message: print(f"[伺服] {message}")
        )
    except Exception as exc:  # noqa: BLE001 - 初始化失败统一报错退出
        if drive is not None:
            try:
                drive.close()
            except Exception:  # noqa: BLE001
                pass
        print(f"错误：打开或初始化 X2P 失败：{exc}", file=sys.stderr)
        return 1

    adjuster = ManualLiftAdjuster(controller, options)

    def _emergency(_signum: int, _frame: object) -> None:
        adjuster.best_effort_stop()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _emergency)
    signal.signal(signal.SIGTERM, _emergency)

    try:
        if args.status:
            print(adjuster.describe_state())
            return 0
        if args.direction is not None or args.distance_cm is not None:
            if args.direction is None or args.distance_cm is None:
                print(
                    "错误：单次模式要同时给出方向和距离，例如 up 0.5",
                    file=sys.stderr,
                )
                return 2
            try:
                adjuster.execute(args.direction, args.distance_cm)
            except ValueError as exc:
                print(f"拒绝执行：{exc}", file=sys.stderr)
                return 2
            except Exception as exc:  # noqa: BLE001 - 失败后尽力停机
                adjuster.best_effort_stop()
                print(f"执行失败：{exc}", file=sys.stderr)
                return 1
            return 0
        return adjuster.repl()
    except KeyboardInterrupt:
        print("已中断，并已尝试安全停机。", file=sys.stderr)
        return 130
    except EOFError:
        print("输入结束，退出。")
        return 0
    finally:
        try:
            drive.close()
        except Exception:  # noqa: BLE001 - 关闭失败不影响退出码
            pass


if __name__ == "__main__":
    raise SystemExit(main())
