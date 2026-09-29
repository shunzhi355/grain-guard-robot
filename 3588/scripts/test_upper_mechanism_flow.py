#!/usr/bin/env python3
"""上层机构实机流程测试：启动服务，模拟跳过建图/跑点，从人工插管后开始测试。

默认为交互模式，每个可能产生机械运动的阶段都需要人工确认。

板端用法（先加载 ROS 消息工作空间）：

  source /home/neardi/mechanism_ws/devel/setup.bash
  cd "/home/neardi/project/grain guard robot/grain-guard-robot/grain_sampling_robot_software_share"

  # 只看流程，不启动服务、不操作机构
  python3 scripts/test_upper_mechanism_flow.py --dry-run

  # 启动 grain-sampling 并交互式测试
  python3 scripts/test_upper_mechanism_flow.py --grain 稻谷 --bin shallow

安全说明：
  * Ctrl+C、服务失败或动作失败时，尽力调用 /mechanism/emergency_stop。
  * 输送服务本身会按品种参数运行 90~180 秒；本测试为短时点动，
    到 --convey-seconds 后用急停停止，随后重新 set_grain 解锁。
  * 当前代码中负压风机 CH7 仍为未接线占位；服务返回成功不代表风机已实际启动。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Optional


SERVICE_NAME = "grain-sampling"
SUPPORTED_GRAINS = ("稻谷", "玉米", "黄豆")
BIN_NAMES = ("shallow", "mid", "deep")


class FlowError(RuntimeError):
    """流程必须立即中止的错误。"""


@dataclass
class Options:
    dry_run: bool
    start_service: bool
    assume_yes: bool
    grain: str
    bin_name: str
    sections: int
    cycles_per_section: int
    down_cm: float
    up_cm: float
    pause_seconds: float
    waste_seconds: float
    sampling_seconds: float
    convey_seconds: float
    service_timeout: float
    release_pipe_at_end: bool


class UpperMechanismTest:
    """顺序执行上层机构测试，并统一处理急停。"""

    def __init__(self, options: Options) -> None:
        self.options = options
        self.rospy = None
        self.trigger_type = None
        self.move_lift_type = None
        self.set_grain_type = None
        self.ros_ready = False

    @staticmethod
    def _banner(text: str) -> None:
        print(f"\n{'=' * 12} {text} {'=' * 12}", flush=True)

    def _confirm(self, text: str) -> None:
        if self.options.dry_run:
            print(f"[预演/人工确认] {text}", flush=True)
            return
        if self.options.assume_yes:
            print(f"[自动确认] {text}", flush=True)
            return
        try:
            answer = input(f"\n{text}\n输入 YES 继续，其他内容中止：").strip()
        except EOFError as exc:
            raise KeyboardInterrupt("终端输入已关闭") from exc
        if answer != "YES":
            raise KeyboardInterrupt("用户取消")

    def _sleep(self, seconds: float, reason: str) -> None:
        print(f"[等待] {reason}: {seconds:.1f} 秒", flush=True)
        if not self.options.dry_run and seconds > 0:
            time.sleep(seconds)

    def start_system_service(self) -> None:
        self._banner("1. 启动项目服务")
        if not self.options.start_service:
            print("[跳过] --no-start-service：假定 grain-sampling 已启动", flush=True)
            return
        if self.options.dry_run:
            print("[预演] sudo systemctl start grain-sampling", flush=True)
            return
        print("[EXEC] sudo systemctl start grain-sampling", flush=True)
        try:
            subprocess.run(
                ["sudo", "systemctl", "start", SERVICE_NAME],
                check=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise FlowError(f"启动 {SERVICE_NAME} 失败: {exc}") from exc

    def fake_mapping_and_navigation(self) -> None:
        self._banner("2. 建图与跑点（模拟）")
        print("[FAKE OK] 建图命令已模拟成功，未启动雷达/建图节点", flush=True)
        print("[FAKE OK] 跑点命令已模拟成功，未发布导航目标、底盘不动", flush=True)

    def connect_ros(self) -> None:
        self._banner("3. 连接上层机构服务")
        if self.options.dry_run:
            print("[预演] 等待 /mechanism/* 服务", flush=True)
            return
        try:
            import rospy
            from mechanism_node.srv import MoveLift, SetGrain
            from std_srvs.srv import Trigger
        except ImportError as exc:
            raise FlowError(
                "无法导入 ROS 服务类型。请先执行: "
                "source /home/neardi/mechanism_ws/devel/setup.bash"
            ) from exc

        self.rospy = rospy
        self.trigger_type = Trigger
        self.move_lift_type = MoveLift
        self.set_grain_type = SetGrain
        rospy.init_node("upper_mechanism_flow_test", anonymous=True, disable_signals=True)

        required = [
            "/mechanism/set_grain",
            "/mechanism/move_lift",
            "/mechanism/clamp",
            "/mechanism/unclamp",
            "/mechanism/tighten",
            "/mechanism/start_suction",
            "/mechanism/stop_suction",
            "/mechanism/convey",
            f"/mechanism/open_bin/{self.options.bin_name}",
            f"/mechanism/close_bin/{self.options.bin_name}",
            "/mechanism/emergency_stop",
        ]
        if self.options.release_pipe_at_end and self.options.sections > 1:
            required.append("/mechanism/untighten")
        for name in required:
            print(f"[等待服务] {name}", flush=True)
            try:
                rospy.wait_for_service(name, timeout=self.options.service_timeout)
            except rospy.ROSException as exc:
                raise FlowError(f"服务不可用: {name}") from exc
        self.ros_ready = True
        print("[OK] 上层机构服务已就绪", flush=True)

    def _trigger(self, service: str, label: str) -> None:
        if self.options.dry_run:
            print(f"[预演] {label}: rosservice call {service}", flush=True)
            return
        assert self.rospy is not None and self.trigger_type is not None
        try:
            response = self.rospy.ServiceProxy(service, self.trigger_type)()
        except Exception as exc:
            raise FlowError(f"{label}调用异常: {exc}") from exc
        print(f"[{'OK' if response.success else 'FAIL'}] {label}: {response.message}", flush=True)
        if not response.success:
            raise FlowError(f"{label}失败: {response.message}")

    def _set_grain(self, reason: str = "设置粮食品种") -> None:
        if self.options.dry_run:
            print(f"[预演] {reason}: /mechanism/set_grain grain={self.options.grain}", flush=True)
            return
        assert self.rospy is not None and self.set_grain_type is not None
        try:
            response = self.rospy.ServiceProxy(
                "/mechanism/set_grain", self.set_grain_type
            )(grain=self.options.grain)
        except Exception as exc:
            raise FlowError(f"{reason}调用异常: {exc}") from exc
        print(f"[{'OK' if response.success else 'FAIL'}] {reason}: {response.message}", flush=True)
        if not response.success:
            raise FlowError(f"{reason}失败: {response.message}")

    def _move_lift(self, direction: str, distance_cm: float) -> None:
        direction_cn = "下压" if direction == "down" else "上升"
        if self.options.dry_run:
            print(
                f"[预演] 伺服{direction_cn}: /mechanism/move_lift "
                f"direction={direction} distance_cm={distance_cm:.1f}",
                flush=True,
            )
            return
        assert self.rospy is not None and self.move_lift_type is not None
        try:
            response = self.rospy.ServiceProxy(
                "/mechanism/move_lift", self.move_lift_type
            )(direction=direction, distance_cm=float(distance_cm))
        except Exception as exc:
            raise FlowError(f"伺服{direction_cn}调用异常: {exc}") from exc
        print(
            f"[{'OK' if response.success else 'FAIL'}] 伺服{direction_cn} "
            f"{distance_cm:.1f} cm: {response.message}",
            flush=True,
        )
        if not response.success:
            raise FlowError(f"伺服{direction_cn}失败: {response.message}")

    def emergency_stop(self, reason: str) -> bool:
        print(f"\n[急停] {reason}", flush=True)
        if self.options.dry_run:
            print("[预演] rosservice call /mechanism/emergency_stop", flush=True)
            return True
        if not self.ros_ready or self.rospy is None or self.trigger_type is None:
            print("[警告] ROS 尚未连接，无法从本脚本调用急停", flush=True)
            return False
        try:
            response = self.rospy.ServiceProxy(
                "/mechanism/emergency_stop", self.trigger_type
            )()
            print(f"[急停响应] success={response.success} {response.message}", flush=True)
            return bool(response.success)
        except Exception as exc:
            print(f"[警告] 急停服务调用失败: {exc}", flush=True)
            return False

    def _test_pressing(self) -> None:
        self._banner("4. 人工插管与下管")
        self._confirm(
            "确认急停可用、人员已远离夹具/伺服机构，"
            "且第 1 节管已人工插入正确位置。"
        )
        self._set_grain()

        for section in range(1, self.options.sections + 1):
            if section > 1:
                self._confirm(f"请插入第 {section} 节管，对准接头后确认。")
                self._trigger("/mechanism/tighten", f"第 {section} 节管拧紧")
                self._trigger("/mechanism/unclamp", "加管后松开夹具")

            self._trigger("/mechanism/clamp", f"第 {section} 节管夹紧")
            for cycle in range(1, self.options.cycles_per_section + 1):
                print(
                    f"\n[下管] 第 {section}/{self.options.sections} 节，"
                    f"往复 {cycle}/{self.options.cycles_per_section}",
                    flush=True,
                )
                self._move_lift("down", self.options.down_cm)
                self._sleep(self.options.pause_seconds, "伺服换向间停")
                self._trigger("/mechanism/unclamp", "松开夹具")
                self._move_lift("up", self.options.up_cm)
                self._sleep(self.options.pause_seconds, "伺服换向间停")
                self._trigger("/mechanism/clamp", "再次夹紧")

    def _test_suction(self) -> None:
        self._banner("5. 排废粮与正式抽粮")
        print("[重要] 当前风机 CH7 在代码中仍是未接线占位通道。", flush=True)
        print("[重要] start_suction 返回成功只能证明 ROS 流程成功。", flush=True)
        self._confirm("确认抽粮管路和废粮出口安全，准备测试排废粮。")
        self._trigger("/mechanism/start_suction", "启动风机（排废粮）")
        self._sleep(self.options.waste_seconds, "排废粮测试")
        self._trigger("/mechanism/start_suction", "进入正式抽粮")
        self._sleep(self.options.sampling_seconds, "正式抽粮测试")
        self._trigger("/mechanism/stop_suction", "停止风机")

    def _test_conveyor_and_bin(self) -> None:
        self._banner("6. 螺旋输送与粮仓")
        self._confirm(
            f"确认输送机周围无人无异物，准备点动 "
            f"{self.options.convey_seconds:.1f} 秒。"
        )
        self._trigger("/mechanism/convey", "启动 CH0/CH1 螺旋输送")
        self._sleep(self.options.convey_seconds, "螺旋输送点动")

        # convey 没有单独的 stop 服务，必须急停才能在品种默认时长前停止。
        if not self.emergency_stop("输送点动计时结束，停止 CH0/CH1"):
            raise FlowError("输送点动后急停失败，禁止继续粮仓测试")
        self._set_grain("输送急停后重新使能机构")

        self._confirm(f"准备打开 {self.options.bin_name} 粮仓，确认粮仓下方安全。")
        self._trigger(
            f"/mechanism/open_bin/{self.options.bin_name}",
            f"打开 {self.options.bin_name} 粮仓",
        )
        self._confirm("观察开仓动作后确认收粮完成，继续将关闭粮仓。")
        self._trigger(
            f"/mechanism/close_bin/{self.options.bin_name}",
            f"关闭 {self.options.bin_name} 粮仓",
        )

    def _finish_pipe(self) -> None:
        self._banner("7. 结束状态")
        if not self.options.release_pipe_at_end:
            print(
                "[安全保持] 默认不松管：夹具保持夹紧，防止管件下落。",
                flush=True,
            )
            print(
                "如需测试拧松并取管，重新运行时增加 --release-pipe-at-end。",
                flush=True,
            )
            return
        self._confirm(
            "即将拧松并松开夹具。必须已人工托住/固定管件，确保管件不会下落。"
        )
        if self.options.sections > 1:
            self._trigger("/mechanism/untighten", "拧松管路接头")
        self._trigger("/mechanism/unclamp", "松开夹具，准备人工取管")

    def run(self) -> None:
        self.start_system_service()
        self.fake_mapping_and_navigation()
        self.connect_ros()
        self._test_pressing()
        self._test_suction()
        self._test_conveyor_and_bin()
        self._finish_pipe()
        self._banner("测试完成")
        print("上层机构流程已全部执行完成。grain-sampling 服务保持运行。", flush=True)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("不能小于 0")
    return parsed


def lift_distance(value: str) -> float:
    parsed = float(value)
    if parsed < 2.0:
        raise argparse.ArgumentTypeError(
            "实机编码器容差约 1.5 cm，测试位移必须至少 2.0 cm"
        )
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="启动项目服务，模拟跳过建图/跑点，测试插管后的上层机构流程。"
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印流程，不启动服务、不调用硬件")
    parser.add_argument("--no-start-service", action="store_true", help="不执行 systemctl start，使用已启动的服务")
    parser.add_argument("--yes", action="store_true", help="跳过所有人工确认（实机不推荐）")
    parser.add_argument("--grain", choices=SUPPORTED_GRAINS, default="稻谷", help="粮食品种")
    parser.add_argument("--bin", dest="bin_name", choices=BIN_NAMES, default="shallow", help="测试的粮仓")
    parser.add_argument("--sections", type=positive_int, default=1, help="测试管节数；大于 1 时会测试拧紧动作")
    parser.add_argument("--cycles-per-section", type=positive_int, default=1, help="每节管执行的下/上往复次数")
    parser.add_argument("--down-cm", type=lift_distance, default=3.0, help="每次下压厘米数（>=2）")
    parser.add_argument("--up-cm", type=lift_distance, default=2.0, help="每次上升厘米数（>=2）")
    parser.add_argument("--pause-seconds", type=nonnegative_float, default=1.0, help="伺服换向间停秒数")
    parser.add_argument("--waste-seconds", type=nonnegative_float, default=3.0, help="排废粮测试秒数")
    parser.add_argument("--sampling-seconds", type=nonnegative_float, default=5.0, help="抽粮测试秒数")
    parser.add_argument("--convey-seconds", type=nonnegative_float, default=5.0, help="螺旋输送点动秒数，后续使用急停结束")
    parser.add_argument("--service-timeout", type=positive_int, default=30, help="等待每个 ROS 服务的超时秒数")
    parser.add_argument(
        "--release-pipe-at-end",
        action="store_true",
        help="结束时在人工确认后拧松/松开夹具；默认保持夹紧防止掉管",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.down_cm <= args.up_cm:
        print("参数错误：--down-cm 必须大于 --up-cm，否则没有净下压位移。", file=sys.stderr)
        return 2
    options = Options(
        dry_run=args.dry_run,
        start_service=not args.no_start_service,
        assume_yes=args.yes,
        grain=args.grain,
        bin_name=args.bin_name,
        sections=args.sections,
        cycles_per_section=args.cycles_per_section,
        down_cm=args.down_cm,
        up_cm=args.up_cm,
        pause_seconds=args.pause_seconds,
        waste_seconds=args.waste_seconds,
        sampling_seconds=args.sampling_seconds,
        convey_seconds=args.convey_seconds,
        service_timeout=float(args.service_timeout),
        release_pipe_at_end=args.release_pipe_at_end,
    )
    flow = UpperMechanismTest(options)
    try:
        flow.run()
    except KeyboardInterrupt:
        flow.emergency_stop("用户中断或取消")
        print("测试已中止。", file=sys.stderr, flush=True)
        return 130
    except FlowError as exc:
        flow.emergency_stop(f"流程失败: {exc}")
        print(f"测试失败：{exc}", file=sys.stderr, flush=True)
        return 1
    except Exception as exc:  # 未预期异常也必须先停机构
        flow.emergency_stop(f"未预期异常: {exc}")
        print(f"测试异常：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
