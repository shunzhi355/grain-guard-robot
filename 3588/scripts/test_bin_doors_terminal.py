#!/usr/bin/env python3
"""Interactive terminal for isolated testing of the three bin doors.

Commands:
  1  shallow bin (CH2): open 5 s, close 3 s, then output off
  2  middle bin  (CH3): open 5 s, close 3 s, then output off
  3  deep bin    (CH4): open 5 s, close 3 s, then output off
  a  run 1, 2 and 3 in sequence
  q  safely turn all outputs off and quit
"""

from __future__ import annotations

import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))


OPEN_HOLD_S = 5.2
CLOSE_HOLD_S = 3.2


@dataclass(frozen=True)
class BinDoor:
    key: str
    label: str
    channel: int


BINS = {
    "1": BinDoor("shallow", "浅仓", 2),
    "2": BinDoor("mid", "中仓", 3),
    "3": BinDoor("deep", "深仓", 4),
}


class DoorTestError(RuntimeError):
    """A service call failed; subsequent hardware actions must stop."""


class BinDoorTerminal:
    def __init__(self) -> None:
        self.rospy = None
        self.trigger_type = None
        self.set_grain_type = None
        self.connected = False
        self.failed = False
        self._interrupted = False

    @staticmethod
    def _print_menu() -> None:
        print(
            "\n三仓门独立测试（正式机构服务）\n"
            "  1 = 浅仓 CH2：开5秒 → 关3秒 → 断电\n"
            "  2 = 中仓 CH3：开5秒 → 关3秒 → 断电\n"
            "  3 = 深仓 CH4：开5秒 → 关3秒 → 断电\n"
            "  a = 依次测试 1、2、3\n"
            "  q = 全部输出断电并退出\n",
            flush=True,
        )

    def connect(self) -> None:
        try:
            import rospy
            from mechanism_node.srv import SetGrain
            from std_srvs.srv import Trigger
        except ImportError as exc:
            raise DoorTestError(
                "ROS 环境未加载，请通过 scripts/run_bin_door_terminal.sh 启动"
            ) from exc

        self.rospy = rospy
        self.trigger_type = Trigger
        self.set_grain_type = SetGrain
        rospy.init_node("bin_door_terminal", anonymous=True, disable_signals=True)

        required = ["/mechanism/set_grain", "/mechanism/emergency_stop"]
        for door in BINS.values():
            required.extend(
                (
                    f"/mechanism/open_bin/{door.key}",
                    f"/mechanism/close_bin/{door.key}",
                )
            )
        for service in required:
            try:
                rospy.wait_for_service(service, timeout=10.0)
            except rospy.ROSException as exc:
                raise DoorTestError(f"服务不可用：{service}") from exc

        self.connected = True
        self._set_grain("稻谷")
        print("[就绪] 三个仓门服务均可用。", flush=True)

    def _trigger(self, service: str, label: str) -> None:
        assert self.rospy is not None and self.trigger_type is not None
        try:
            response = self.rospy.ServiceProxy(service, self.trigger_type)()
        except Exception as exc:
            raise DoorTestError(f"{label}调用异常：{exc}") from exc
        print(
            f"[{'成功' if response.success else '失败'}] {label}：{response.message}",
            flush=True,
        )
        if not response.success:
            raise DoorTestError(f"{label}失败：{response.message}")

    def _set_grain(self, grain: str) -> None:
        assert self.rospy is not None and self.set_grain_type is not None
        response = self.rospy.ServiceProxy(
            "/mechanism/set_grain", self.set_grain_type
        )(grain=grain)
        if not response.success:
            raise DoorTestError(f"机构重新使能失败：{response.message}")

    def _wait(self, seconds: float, label: str) -> None:
        deadline = time.monotonic() + seconds
        last_shown: Optional[int] = None
        while time.monotonic() < deadline:
            if self._interrupted:
                raise KeyboardInterrupt
            remaining = max(0, int(deadline - time.monotonic() + 0.999))
            if remaining != last_shown:
                print(f"  {label}：剩余 {remaining} 秒", flush=True)
                last_shown = remaining
            time.sleep(min(0.1, deadline - time.monotonic()))

    def run_door(self, command: str) -> None:
        door = BINS[command]
        print(f"\n开始测试 {door.label} CH{door.channel}，人员请远离仓门。", flush=True)
        self._trigger(f"/mechanism/open_bin/{door.key}", f"打开{door.label}")
        self._wait(OPEN_HOLD_S, f"{door.label}保持打开/等待自动关门")
        self._trigger(f"/mechanism/close_bin/{door.key}", f"关闭{door.label}")
        self._wait(CLOSE_HOLD_S, f"{door.label}关闭并等待输出断电")
        print(f"[完成] {door.label} CH{door.channel} 开关测试完成。", flush=True)

    def emergency_stop(self, *, clear_after: bool) -> None:
        if not self.connected or self.rospy is None or self.trigger_type is None:
            return
        try:
            response = self.rospy.ServiceProxy(
                "/mechanism/emergency_stop", self.trigger_type
            )()
            print(f"[全部断电] {response.message}", flush=True)
            if clear_after and response.success:
                self._set_grain("稻谷")
                print("[就绪] 软件急停标志已清除，机构输出仍保持断电。", flush=True)
        except Exception as exc:
            print(f"[警告] 全部断电调用失败：{exc}", file=sys.stderr, flush=True)

    def run(self, input_fn: Callable[[str], str] = input) -> int:
        self.connect()
        self._print_menu()
        while True:
            try:
                command = input_fn("请输入命令 [1/2/3/a/q]：").strip().lower()
            except EOFError:
                command = "q"
            if command == "q":
                self.emergency_stop(clear_after=True)
                print("测试终端已安全退出。", flush=True)
                return 0
            if command == "a":
                for item in ("1", "2", "3"):
                    self.run_door(item)
                continue
            if command in BINS:
                self.run_door(command)
                continue
            print("无效命令，只能输入 1、2、3、a 或 q。", flush=True)


def main() -> int:
    terminal = BinDoorTerminal()

    def request_stop(_signum, _frame) -> None:
        terminal._interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGHUP, request_stop)
    try:
        return terminal.run()
    except KeyboardInterrupt:
        terminal.emergency_stop(clear_after=True)
        print("\n测试已中断并安全断电。", flush=True)
        return 130
    except DoorTestError as exc:
        terminal.failed = True
        terminal.emergency_stop(clear_after=False)
        print(f"\n测试失败：{exc}", file=sys.stderr, flush=True)
        return 1
    except Exception as exc:
        terminal.failed = True
        terminal.emergency_stop(clear_after=False)
        print(f"\n未预期异常：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
