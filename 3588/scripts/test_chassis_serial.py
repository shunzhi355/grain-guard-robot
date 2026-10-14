#!/usr/bin/env python3
"""Independent PC USB-TTL -> STM32 chassis test. No ROS installation required."""
import argparse
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))
from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices.chassis_serial import ChassisError, ChassisSerial, DEFAULT_SERIAL_PORT

DIRECTIONS = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}


def send_stop(link):
    link.stream_control(p.AUTO_STOP)
    print("停车指令已发送，不等待执行完成反馈。", flush=True)


def move(link, direction, effort, seconds, clock=time.monotonic, sleep=time.sleep):
    send_stop(link)
    forward, turn = (round(v * effort * 1000) for v in DIRECTIONS[direction])
    started = clock()
    next_report = started
    print("开始 %s：力度=%.3f，持续=%.2fs，20Hz发送；Ctrl+C停车。" %
          (direction, effort, seconds), flush=True)
    while clock() - started < seconds:
        tick = clock()
        link.stream_effort(forward, turn)
        if clock() >= next_report:
            print("已发送运动指令；执行状态未知。", flush=True)
            next_report = clock() + 0.5
        sleep(max(0.0, 0.05 - (clock() - tick)))
    send_stop(link)
    print("发送结束：运动指令按20Hz发送，已发送停车指令；不代表单片机执行或实际运动已验证。",
          flush=True)


def bounded_number(low, high):
    def parse(value):
        number = float(value)
        if not math.isfinite(number) or not low <= number <= high:
            raise argparse.ArgumentTypeError("数值必须在 %s～%s 之间" % (low, high))
        return number
    return parse


def parser():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("command", nargs="?", default="stop", choices=[
        "forward", "backward", "left", "right", "stop", "estop", "recover", "clear-estop"])
    args.add_argument("--port", default=os.getenv("CHASSIS_SERIAL_PORT", DEFAULT_SERIAL_PORT),
                      help="USB-TTL设备路径（默认CHASSIS_SERIAL_PORT或现场FT232固定by-id路径）")
    args.add_argument("--effort", type=bounded_number(0.01, 1.0), default=0.2,
                      help="归一化力度，0.01～1.0，默认0.2；不是m/s")
    args.add_argument("--seconds", type=bounded_number(0.3, 10.0), default=1.0,
                      help="单次运动时间0.3～10秒，默认1秒")
    return args


def main(argv=None):
    args = parser().parse_args(argv)
    link = None
    result = 0
    old_term = signal.getsignal(signal.SIGTERM)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        import serial
        print("串口=%s，115200 8N1。请先停止 chassis_node，避免串口争用。" % args.port, flush=True)
        options = {"exclusive": True} if os.name == "posix" else {}
        port = serial.Serial(args.port, 115200, timeout=0.01, write_timeout=0.05, **options)
        link = ChassisSerial(port)
        port.reset_input_buffer()
        print("单向控制：不握手、不接收状态；需要单向协议固件。", flush=True)
        if args.command in DIRECTIONS:
            move(link, args.command, args.effort, args.seconds)
        elif args.command == "stop":
            send_stop(link)
        else:
            kind = {"estop": p.ESTOP, "recover": p.RECOVER, "clear-estop": p.CLEAR_ESTOP}[args.command]
            link.stream_control(kind)
            print("指令已发送，执行未确认。", flush=True)
    except KeyboardInterrupt:
        print("已中断，正在停车。", flush=True)
        result = 130
    except (OSError, ChassisError, ImportError) as exc:
        print("FAIL：%s" % exc, file=sys.stderr, flush=True)
        result = 1
    finally:
        if link is not None:
            try:
                link.stream_control(p.AUTO_STOP)
            except (OSError, ChassisError) as exc:
                print("停车指令发送失败：%s；单片机运动看门狗为300ms。" % exc, file=sys.stderr)
                result = result or 1
            finally:
                try:
                    link.close()
                except (OSError, ChassisError) as exc:
                    print("串口关闭异常：%s" % exc, file=sys.stderr)
                    result = result or 1
        signal.signal(signal.SIGTERM, old_term)
    return result


if __name__ == "__main__":
    sys.exit(main())
