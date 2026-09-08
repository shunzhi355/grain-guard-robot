"""Command-line interface for safe X2P inspection and motion."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict

import serial

from .drive import X2PDrive
from .errors import X2PError
from .models import ControllerConfig
from .motion import MotionController
from .protocol import signed16
from .registers import Register, digital_input_bit


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X2P USB-RS485安全调试工具")
    parser.add_argument(
        "--config", help="JSON配置文件；默认使用内置安全限制"
    )
    parser.add_argument("--port", help="例如 /dev/ttyUSB0；覆盖配置文件")
    parser.add_argument(
        "--slave", type=int, help="覆盖配置文件中的从站地址"
    )
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser(
        "probe", help="通信诊断及只读状态（不驱动电机）"
    )
    commands.add_parser("status", help="读取驱动器状态")

    feedback = commands.add_parser(
        "feedback", help="读取编码器位置和位置偏差"
    )
    feedback.add_argument("--seconds", type=float, default=0)
    feedback.add_argument("--interval", type=float, default=0.2)

    speed = commands.add_parser(
        "speed", help="已验证：按方向、转速和时间运行"
    )
    speed.add_argument("--direction", required=True, choices=["forward", "reverse"])
    speed.add_argument("--rpm", type=int, required=True)
    speed.add_argument("--seconds", type=float, required=True)
    speed.add_argument("--confirm", required=True, choices=["RUN"])

    timed = commands.add_parser(
        "move-timed",
        help="速度模式加编码器反馈：按厘米距离和时间移动",
    )
    timed.add_argument(
        "--direction", required=True, choices=["forward", "reverse"]
    )
    timed.add_argument("--distance-cm", type=float, required=True)
    timed.add_argument("--seconds", type=float, required=True)
    timed.add_argument("--tolerance-mm", type=float)
    timed.add_argument("--confirm", required=True, choices=["MOVE"])

    pulses = commands.add_parser(
        "experimental-move-pulses",
        help="未通过实机验证：内部位置段按脉冲移动",
    )
    pulses.add_argument("--direction", required=True, choices=["forward", "reverse"])
    pulses.add_argument("--pulses", type=int, required=True)
    pulses.add_argument("--rpm", type=int, required=True)
    pulses.add_argument(
        "--confirm", required=True, choices=["EXPERIMENTAL-MOVE"]
    )

    distance = commands.add_parser(
        "experimental-move-distance",
        help="未通过实机验证：内部位置段按毫米移动",
    )
    distance.add_argument("--direction", required=True, choices=["forward", "reverse"])
    distance.add_argument("--distance-mm", type=float, required=True)
    distance.add_argument("--rpm", type=int, required=True)
    distance.add_argument(
        "--confirm", required=True, choices=["EXPERIMENTAL-MOVE"]
    )

    commands.add_parser("position-status", help="只读查看内部位置参数")
    commands.add_parser(
        "hybrid-status", help="只读检查Pn001=3及C-MODE端子配置"
    )
    commands.add_parser("stop", help="速度归零、取消软件使能并确认OFF")
    return parser


def _status(drive: X2PDrive) -> dict[str, int]:
    return {
        "status": drive.read_registers(Register.STATUS)[0],
        "speed_rpm": signed16(drive.read_registers(Register.ACTUAL_SPEED)[0]),
        "servo_enabled": drive.read_registers(Register.SERVO_ENABLE_STATUS)[0],
        "control_mode_pn001": drive.read_registers(Register.CONTROL_MODE)[0],
        "speed_source_pn300": drive.read_registers(Register.SPEED_SOURCE)[0],
    }


def _position_status(drive: X2PDrive) -> dict[str, int]:
    return {
        "control_mode_pn001": drive.read_registers(Register.CONTROL_MODE)[0],
        "position_source_pn321": drive.read_registers(Register.POSITION_SOURCE)[0],
        "position_mode_pn700": drive.read_registers(Register.POSITION_MODE)[0],
        "position_segment_pn701": drive.read_registers(Register.POSITION_SEGMENT)[0],
        "pr1_pulses_pn706": drive.read_signed32(Register.PR1_PULSES),
        "pr1_speed_pn708": drive.read_registers(Register.PR1_SPEED)[0],
    }


def _hybrid_status(drive: X2PDrive, mode_di: int) -> dict[str, int | bool]:
    inputs = drive.read_digital_inputs()
    return {
        "control_mode_pn001": drive.read_registers(Register.CONTROL_MODE)[0],
        "cmode_di": mode_di,
        "cmode_function": drive.read_digital_input_function(mode_di),
        "position_trigger_di2_function": (
            drive.read_digital_input_function(2)
        ),
        "cmode_active": bool(inputs & digital_input_bit(mode_di)),
        "digital_inputs_un032": inputs,
        "actual_speed_rpm": signed16(
            drive.read_registers(Register.ACTUAL_SPEED)[0]
        ),
    }


def _best_effort_stop(controller: MotionController | None) -> None:
    if controller is None:
        return
    try:
        controller.stop(verify_off=False)
    except Exception as exc:
        print(f"紧急停止命令未完全确认: {exc}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    drive: X2PDrive | None = None
    controller: MotionController | None = None
    try:
        config = ControllerConfig.load(args.config)
        port = args.port or config.port
        slave = args.slave if args.slave is not None else config.slave
        drive = X2PDrive(port, slave)
        controller = MotionController(drive, config)

        def emergency_stop(_signum: int, _frame: object) -> None:
            _best_effort_stop(controller)
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, emergency_stop)
        signal.signal(signal.SIGTERM, emergency_stop)

        if args.command in {"probe", "status"}:
            drive.diagnostic()
            print(json.dumps(_status(drive), ensure_ascii=False))
        elif args.command == "feedback":
            if not 0 <= args.seconds <= 60:
                raise ValueError("feedback --seconds必须在0..60之间")
            if not 0.05 <= args.interval <= 5:
                raise ValueError("feedback --interval必须在0.05..5之间")
            drive.diagnostic()
            deadline = time.monotonic() + args.seconds
            while True:
                print(
                    json.dumps(
                        drive.read_position_feedback(), ensure_ascii=False
                    )
                )
                if args.seconds == 0 or time.monotonic() >= deadline:
                    break
                time.sleep(args.interval)
        elif args.command == "speed":
            result = controller.run_speed(args.direction, args.rpm, args.seconds)
            print(json.dumps(asdict(result), ensure_ascii=False))
        elif args.command == "move-timed":
            result = controller.move_timed_distance(
                args.direction,
                args.distance_cm * 10,
                args.seconds,
                tolerance_mm=args.tolerance_mm,
            )
            print(json.dumps(asdict(result), ensure_ascii=False))
        elif args.command == "experimental-move-pulses":
            result = controller.experimental_move_pulses(
                args.direction,
                args.pulses,
                args.rpm,
                allow_experimental=True,
            )
            print(json.dumps(asdict(result), ensure_ascii=False))
        elif args.command == "experimental-move-distance":
            result = controller.experimental_move_distance(
                args.direction,
                args.distance_mm,
                args.rpm,
                allow_experimental=True,
            )
            print(json.dumps(asdict(result), ensure_ascii=False))
        elif args.command == "position-status":
            drive.diagnostic()
            print(json.dumps(_position_status(drive), ensure_ascii=False))
        elif args.command == "hybrid-status":
            drive.diagnostic()
            print(
                json.dumps(
                    _hybrid_status(drive, config.hybrid_mode_di),
                    ensure_ascii=False,
                )
            )
        elif args.command == "stop":
            controller.stop(verify_off=True)
            print("已发送速度0、取消软件使能并确认OFF。")
        return 0
    except KeyboardInterrupt:
        print("已中断，并尝试安全停机。", file=sys.stderr)
        return 130
    except (X2PError, ValueError, serial.SerialException) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1
    finally:
        if drive is not None:
            drive.close()
