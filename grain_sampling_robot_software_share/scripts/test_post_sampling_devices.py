#!/usr/bin/env python3
"""后半程机构实机专项测试。

直接调用正式 mechanism_node 服务，不创建 UI 任务，不发布导航目标，
不调用夹紧、拧管或升降服务。测试顺序：

1. 排废粮（正式 /mechanism/start_suction + stop_suction）；
2. 螺旋运粮（正式 /mechanism/convey，短时测试后急停）；
3. 浅、中、深三个仓门逐一打开和关闭。

任一阶段失败、Ctrl+C 或人工取消都会尽力调用机构急停。
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src"))

from sampling_params import (  # noqa: E402
    CHANNELS,
    ENABLE_UNWIRED_CHANNELS,
    GRAIN_MECHANISM_CONFIG,
    SUPPORTED_GRAINS,
)


BINS = (
    ("shallow", "浅仓", CHANNELS["bin_shallow"]),
    ("mid", "中仓", CHANNELS["bin_mid"]),
    ("deep", "深仓", CHANNELS["bin_deep"]),
)


class TestFailure(RuntimeError):
    pass


@dataclass(frozen=True)
class Options:
    dry_run: bool
    assume_yes: bool
    grain: str
    waste_seconds: float
    convey_seconds: float
    bin_hold_seconds: float
    service_timeout: float
    skip_waste: bool


class PostSamplingDeviceTest:
    def __init__(self, options: Options) -> None:
        self.options = options
        self.rospy = None
        self.trigger_type = None
        self.set_grain_type = None
        self.ros_ready = False

    @staticmethod
    def _banner(text: str) -> None:
        print(f"\n{'=' * 12} {text} {'=' * 12}", flush=True)

    def _confirm(self, text: str) -> None:
        if self.options.dry_run or self.options.assume_yes:
            prefix = "预演" if self.options.dry_run else "自动确认"
            print(f"[{prefix}] {text}", flush=True)
            return
        try:
            answer = input(f"\n{text}\n输入 YES 继续，其他内容中止：").strip()
        except EOFError as exc:
            raise KeyboardInterrupt("终端输入已关闭") from exc
        if answer != "YES":
            raise KeyboardInterrupt("用户取消")

    def _wait(self, seconds: float, label: str) -> None:
        print(f"[计时] {label}: {seconds:.1f} 秒", flush=True)
        if not self.options.dry_run:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                time.sleep(min(0.2, deadline - time.monotonic()))

    def connect(self) -> None:
        self._banner("环境与服务预检")
        print("已跳过：导航、插管、夹紧、拧管、伺服升降", flush=True)
        print(
            f"通道：输送 CH{CHANNELS['convey_1']}+CH{CHANNELS['convey_2']}；"
            f"三仓 CH{CHANNELS['bin_shallow']}/CH{CHANNELS['bin_mid']}/"
            f"CH{CHANNELS['bin_deep']}；风机 CH{CHANNELS['fan']}",
            flush=True,
        )
        if not ENABLE_UNWIRED_CHANNELS:
            print(
                "[重要] 当前 ENABLE_UNWIRED_CHANNELS=False：CH7 风机为占位，"
                "排废粮服务不会驱动实物。",
                flush=True,
            )
        if self.options.dry_run:
            print("[预演] 等待正式 /mechanism/* 服务", flush=True)
            return
        try:
            import rospy
            from mechanism_node.srv import SetGrain
            from std_srvs.srv import Trigger
        except ImportError as exc:
            raise TestFailure(
                "未加载 ROS 工作空间，请用 scripts/run_post_sampling_test.sh 启动"
            ) from exc
        self.rospy = rospy
        self.trigger_type = Trigger
        self.set_grain_type = SetGrain
        rospy.init_node("post_sampling_device_test", anonymous=True, disable_signals=True)
        required = [
            "/mechanism/set_grain",
            "/mechanism/start_suction",
            "/mechanism/stop_suction",
            "/mechanism/convey",
            "/mechanism/emergency_stop",
        ]
        for name, _, _ in BINS:
            required.extend(
                (f"/mechanism/open_bin/{name}", f"/mechanism/close_bin/{name}")
            )
        for service in required:
            try:
                rospy.wait_for_service(service, timeout=self.options.service_timeout)
            except rospy.ROSException as exc:
                raise TestFailure(f"服务不可用: {service}") from exc
        self.ros_ready = True
        print("[OK] 正式机构服务已就绪", flush=True)

    def _trigger(self, service: str, label: str) -> None:
        if self.options.dry_run:
            print(f"[预演] {label}: {service}", flush=True)
            return
        assert self.rospy is not None and self.trigger_type is not None
        try:
            response = self.rospy.ServiceProxy(service, self.trigger_type)()
        except Exception as exc:
            raise TestFailure(f"{label}调用异常: {exc}") from exc
        print(f"[{'OK' if response.success else 'FAIL'}] {label}: {response.message}", flush=True)
        if not response.success:
            raise TestFailure(f"{label}失败: {response.message}")

    def _set_grain(self, label: str = "设置粮食品种/清除急停") -> None:
        if self.options.dry_run:
            print(f"[预演] {label}: grain={self.options.grain}", flush=True)
            return
        assert self.rospy is not None and self.set_grain_type is not None
        try:
            response = self.rospy.ServiceProxy(
                "/mechanism/set_grain", self.set_grain_type
            )(grain=self.options.grain)
        except Exception as exc:
            raise TestFailure(f"{label}调用异常: {exc}") from exc
        print(f"[{'OK' if response.success else 'FAIL'}] {label}: {response.message}", flush=True)
        if not response.success:
            raise TestFailure(f"{label}失败: {response.message}")

    def emergency_stop(self, reason: str) -> bool:
        print(f"[急停] {reason}", flush=True)
        if self.options.dry_run:
            print("[预演] /mechanism/emergency_stop", flush=True)
            return True
        if not self.ros_ready:
            return False
        try:
            assert self.rospy is not None and self.trigger_type is not None
            response = self.rospy.ServiceProxy(
                "/mechanism/emergency_stop", self.trigger_type
            )()
            print(f"[急停响应] {response.success}: {response.message}", flush=True)
            return bool(response.success)
        except Exception as exc:
            print(f"[警告] 急停服务调用失败: {exc}", flush=True)
            return False

    def test_waste(self) -> None:
        self._banner("排废粮装置")
        if self.options.skip_waste:
            print("[跳过] --skip-waste", flush=True)
            return
        if not ENABLE_UNWIRED_CHANNELS:
            print(
                "[阻塞] 正式配置将 CH7 标记为未接线，本阶段只能验证 ROS，"
                "不能验证实物排废粮。不会冒充硬件测试成功。",
                flush=True,
            )
        self._confirm("确认废粮出口通畅、人员远离，准备启动排废粮。")
        self._trigger("/mechanism/start_suction", "启动排废粮")
        self._wait(self.options.waste_seconds, "排废粮观察")
        self._trigger("/mechanism/stop_suction", "停止排废粮")

    def test_conveyor(self) -> None:
        self._banner("螺旋运粮装置")
        production = GRAIN_MECHANISM_CONFIG[self.options.grain]["convey_duration"]
        print(
            f"本次点动 {self.options.convey_seconds:.1f}s；"
            f"正式品种参数为 {production:.1f}s。启动路径与正式流程相同。",
            flush=True,
        )
        self._confirm("确认 CH0/CH1 螺旋输送机周围无人、无异物。")
        self._trigger("/mechanism/convey", "启动 CH0+CH1 螺旋运粮")
        self._wait(self.options.convey_seconds, "运粮观察")
        if not self.emergency_stop("运粮点动到时，停止 CH0/CH1"):
            raise TestFailure("运粮点动后急停失败")
        self._set_grain("运粮停止后重新使能")

    def test_bins(self) -> None:
        self._banner("浅/中/深三仓存储装置")
        for name, label, channel in BINS:
            self._confirm(
                f"确认 {label}(CH{channel}) 周围安全，准备单独测试该仓门。"
            )
            self._trigger(f"/mechanism/open_bin/{name}", f"打开{label} CH{channel}")
            self._wait(self.options.bin_hold_seconds, f"{label}开门观察")
            self._trigger(f"/mechanism/close_bin/{name}", f"关闭{label} CH{channel}")
            print(f"[OK] {label}完成独立开/关测试", flush=True)

    def run(self) -> None:
        self.connect()
        self._set_grain()
        self.test_waste()
        self.test_conveyor()
        self.test_bins()
        self._banner("专项测试完成")
        print("未调用导航、插管、夹紧、拧管或升降动作。", flush=True)


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("不能小于 0")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="只打印流程，不调用硬件")
    parser.add_argument("--yes", action="store_true", help="跳过人工确认（实机不推荐）")
    parser.add_argument("--grain", choices=SUPPORTED_GRAINS, default="稻谷")
    parser.add_argument("--waste-seconds", type=nonnegative_float, default=5.0)
    parser.add_argument("--convey-seconds", type=nonnegative_float, default=5.0)
    parser.add_argument("--bin-hold-seconds", type=nonnegative_float, default=2.0)
    parser.add_argument("--service-timeout", type=positive_float, default=20.0)
    parser.add_argument("--skip-waste", action="store_true", help="跳过当前未接线的 CH7 排废粮")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    flow = PostSamplingDeviceTest(
        Options(
            dry_run=args.dry_run,
            assume_yes=args.yes,
            grain=args.grain,
            waste_seconds=args.waste_seconds,
            convey_seconds=args.convey_seconds,
            bin_hold_seconds=args.bin_hold_seconds,
            service_timeout=args.service_timeout,
            skip_waste=args.skip_waste,
        )
    )
    try:
        flow.run()
    except KeyboardInterrupt:
        flow.emergency_stop("用户中断或取消")
        print("测试已中止。", file=sys.stderr, flush=True)
        return 130
    except TestFailure as exc:
        flow.emergency_stop(f"专项测试失败: {exc}")
        print(f"测试失败：{exc}", file=sys.stderr, flush=True)
        return 1
    except Exception as exc:
        flow.emergency_stop(f"未预期异常: {exc}")
        print(f"测试异常：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
