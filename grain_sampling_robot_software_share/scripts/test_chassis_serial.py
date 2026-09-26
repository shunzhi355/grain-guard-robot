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

STATES = {0: "上电安全", 1: "手动", 2: "自动未使能", 3: "自动使能", 4: "故障停止", 5: "急停锁存"}
DIRECTIONS = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}


def show_status(status):
    print("状态=%s mode=%d faults=0x%02x RC年龄=%dms CH1/3/8=%d/%d/%d "
          "力度L/R=%d/%d PWM=%d/%dus motion_seq=%d" % (
              STATES.get(status["state"], "未知"), status["mode"], status["faults"],
              status["rc_age_ms"], status["ch1"], status["ch3"], status["ch8"],
              status["left"], status["right"], status["pwm_left"], status["pwm_right"],
              status["motion_sequence"]), flush=True)


def wait_status(link, predicate=lambda s: True, timeout=0.8, clock=time.monotonic):
    """Require a newly received STATUS, never a cached pre-command result."""
    started = clock()
    while clock() - started < timeout:
        link.receive()
        if link.status is not None and link.status_time >= started and predicate(link.status):
            return link.status
    raise ChassisError("未收到符合条件的新状态反馈；检查串口、固件及遥控模式")


def check_auto(status, centered=False):
    if status["faults"]:
        raise ChassisError("底盘故障 0x%02x；RC故障可在恢复遥控并回中后显式执行 recover" % status["faults"])
    if status["flags"] & 2:
        raise ChassisError("急停已锁存；确认可恢复后显式执行 clear-estop")
    if not status["flags"] & 4 or status["rc_age_ms"] >= 300:
        raise ChassisError("遥控信号无效：遥控器必须开机并连接 STM32 USART1")
    if status["mode"] != 2 or status["state"] not in (2, 3):
        raise ChassisError("请将 CH8 切至自动档（>=1550），保持至少500ms")
    if not status["pwm_enabled"]:
        raise ChassisError("单片机未启用PWM")
    if centered and not all(1450 <= status[key] <= 1550 for key in ("ch1", "ch3")):
        raise ChassisError("使能前须将 CH1、CH3 摇杆回中（1450～1550）")


def send_stop(link):
    link.send(p.AUTO_STOP)
    print("停车指令已发送，不等待执行完成反馈。", flush=True)


def move(link, direction, effort, seconds, clock=time.monotonic, sleep=time.sleep):
    status = wait_status(link, clock=clock)
    show_status(status)
    check_auto(status, centered=True)
    # HELLO already revoked the previous automatic control session.
    link.token_request(p.AUTO_ARM)
    forward, turn = (round(v * effort * 1000) for v in DIRECTIONS[direction])
    started = clock()
    next_report = started
    print("开始 %s：力度=%.3f，持续=%.2fs，20Hz发送；Ctrl+C停车。" %
          (direction, effort, seconds), flush=True)
    while clock() - started < seconds:
        tick = clock()
        link.receive()
        if clock() - link.status_time > 0.2:
            raise ChassisError("底盘状态超过200ms未更新，终止运动")
        check_auto(link.status)
        link.effort(forward, turn)
        if clock() >= next_report:
            show_status(link.status)
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
    args.add_argument("command", nargs="?", default="status", choices=[
        "status", "forward", "backward", "left", "right", "stop", "estop", "recover", "clear-estop"])
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
        link.request(p.HELLO)
        print("握手成功：boot=%d session=%d epoch=%d" % (link.boot, link.session, link.epoch), flush=True)
        if args.command in DIRECTIONS:
            move(link, args.command, args.effort, args.seconds)
        elif args.command == "status":
            show_status(wait_status(link))
            print("PASS：双向串口通信正常（本次未请求运动）。")
        elif args.command == "stop":
            send_stop(link)
        else:
            wait_status(link)  # Synchronize token after RC/mode changes.
            kind = {"estop": p.ESTOP, "recover": p.RECOVER, "clear-estop": p.CLEAR_ESTOP}[args.command]
            if kind == p.ESTOP:
                link.request(kind)
            else:
                link.token_request(kind)
            show_status(wait_status(link))
            print("ACK：%s 已被单片机接受；不会自动开始运动。" % args.command)
    except KeyboardInterrupt:
        print("已中断，正在停车。", flush=True)
        result = 130
    except (OSError, ChassisError, ImportError) as exc:
        print("FAIL：%s" % exc, file=sys.stderr, flush=True)
        result = 1
    finally:
        if link is not None:
            try:
                link.send(p.AUTO_STOP)
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
