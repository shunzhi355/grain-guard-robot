"""X2P 升降伺服手动调整程序测试（scripts/manual_lift_adjust.py）。

全程 mock，不触碰真实串口：
- 方向/距离输入解析与拒绝逻辑
- 单次移动的安全上限、时长公式、容差自动收紧
- 用 m 记录最高点后的行程软限位
- 交互循环（含失败后 continue）
- 直连串口路径下的 OFF 基线（Un058 + 实际转速），覆盖 status=3 的固件行为
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

# pyserial 是板端依赖；本机 Windows 单测通过 stub 提供（同 test_x2p_motion_safety.py）。
try:  # pragma: no cover - depends on the developer environment
    import serial  # noqa: F401
except ImportError:  # pragma: no cover
    sys.modules["serial"] = ModuleType("serial")

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))


def _load_manual_adjust():
    """按路径加载 scripts/manual_lift_adjust.py（scripts 不是包）。"""
    path = ROOT / "scripts" / "manual_lift_adjust.py"
    spec = importlib.util.spec_from_file_location("manual_lift_adjust", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


manual = _load_manual_adjust()

from x2p.errors import SafetyInterlockError  # noqa: E402
from x2p.models import SafetyLimits  # noqa: E402
from x2p.motion import MotionController, MotionState  # noqa: E402
from x2p.registers import DriveStatus, Register  # noqa: E402


# ── 桩对象 ────────────────────────────────────────────────────────────


class _FakeConfig:
    """manual_lift_adjust 需要的 ControllerConfig 子集。"""

    def __init__(
        self,
        *,
        pulses_per_mm=None,
        screw_lead_mm=5.0,
        motor_revs_per_screw_rev=1.0,
        encoder_counts_per_motor_rev=131_072,
        encoder_forward_sign=1,
    ):
        self.pulses_per_mm = pulses_per_mm
        self.screw_lead_mm = screw_lead_mm
        self.motor_revs_per_screw_rev = motor_revs_per_screw_rev
        self.encoder_counts_per_motor_rev = encoder_counts_per_motor_rev
        self.encoder_forward_sign = encoder_forward_sign


class _FakeLiftController:
    """MotionController 的最小鸭子类型，记录每次移动参数。"""

    def __init__(self, config=None, *, positions=None):
        self.config = config or _FakeConfig()
        self.moves: list[tuple] = []
        self.stop_calls: list[bool] = []
        self.positions = list(positions or [])
        self.read_failures = 0
        self.move_error: Exception | None = None

    def read_encoder_position(self):
        if self.positions:
            return self.positions.pop(0)
        return 0

    def move_timed_distance(
        self, direction, distance_mm, duration_s, *, tolerance_mm=None
    ):
        self.moves.append((direction, distance_mm, duration_s, tolerance_mm))
        if self.move_error is not None:
            raise self.move_error
        return object()

    def stop(self, *, verify_off=True):
        self.stop_calls.append(verify_off)


def _scripted_input(answers, prompts=None):
    """按顺序返回输入；用完后抛 EOFError（模拟 Ctrl-D）。

    ``prompts`` 非空时，把每次 input 的提示文字记录进去，便于断言
    “距离缺失时会追问”这类交互行为。
    """
    queue = list(answers)

    def _read(prompt=""):
        if prompts is not None:
            prompts.append(prompt)
        if not queue:
            raise EOFError
        return queue.pop(0)

    return _read


def _make_adjuster(
    *,
    controller=None,
    positions=None,
    collected=None,
    answers=None,
    prompts=None,
    **option_overrides,
):
    """构造 ManualLiftAdjuster；默认把输出收集到 list。"""
    controller = controller or _FakeLiftController(positions=positions)
    params = {
        "rpm": 200,
        "max_cm": 20.0,
        "tolerance_mm": 2.0,
    }
    params.update(option_overrides)
    options = manual.ManualAdjustOptions(**params)
    lines = collected if collected is not None else []
    adjuster = manual.ManualLiftAdjuster(
        controller,
        options,
        output=lines.append,
        input_fn=_scripted_input(answers or [], prompts),
    )
    return adjuster, controller, lines


def _joined(lines):
    return "\n".join(str(line) for line in lines)


# ── 方向解析 ──────────────────────────────────────────────────────────


def test_direction_aliases_map_both_ways():
    for text in ("up", "UP", "上", "上升", "提升", "forward", "fwd", "+", "正向", " 正转 "):
        assert manual.parse_direction(text) == "forward", text
    for text in ("down", "DOWN", "下", "下降", "下压", "reverse", "rev", "-", "反向", "反转"):
        assert manual.parse_direction(text) == "reverse", text


def test_direction_matches_x2p_lift_mapping():
    """与 grain_sampling_devices.x2p_lift._map_lift_direction 保持同一语义。"""
    from grain_sampling_devices.x2p_lift import _map_lift_direction

    assert _map_lift_direction("up") == manual.parse_direction("up")
    assert _map_lift_direction("down") == manual.parse_direction("down")


def test_direction_rejects_unknown_value():
    with pytest.raises(ValueError, match="无法识别的方向"):
        manual.parse_direction("sideways")


# ── 距离解析 ──────────────────────────────────────────────────────────


def test_distance_accepts_positive_numbers():
    assert manual.parse_distance_cm("0.5") == 0.5
    assert manual.parse_distance_cm(" 2 ") == 2.0
    assert manual.parse_distance_cm(3) == 3.0


@pytest.mark.parametrize("text", ["abc", "", "0", "-1", "inf", "nan", "-0.0"])
def test_distance_rejects_invalid_values(text):
    with pytest.raises(ValueError):
        manual.parse_distance_cm(text)


# ── 单次移动上限 ──────────────────────────────────────────────────────


def test_max_distance_reads_environment(monkeypatch):
    assert manual.max_distance_cm({}) == 20.0
    assert manual.max_distance_cm({"MANUAL_LIFT_MAX_CM": "7.5"}) == 7.5
    for bad in ("abc", "0", "-3"):
        with pytest.raises(ValueError):
            manual.max_distance_cm({"MANUAL_LIFT_MAX_CM": bad})


def test_oversized_single_move_is_rejected_without_moving():
    adjuster, controller, lines = _make_adjuster(max_cm=1.0)
    with pytest.raises(ValueError, match="超过安全上限"):
        adjuster.execute("up", "2.0")
    assert controller.moves == []
    assert "执行完成" not in _joined(lines)


# ── 时长公式与容差 ────────────────────────────────────────────────────


def test_duration_matches_mechanism_node_formula():
    """5 mm @ 200 r/min，导程 5 mm：1 转 ×60/200×1.2 = 0.36 s。"""
    assert manual.duration_for_move(5.0, 200) == pytest.approx(0.36)
    # 与 mechanism_node._handle_move_lift 等价：(cm*10)/(lead*rpm/60)*1.2
    distance_cm = 0.5
    expected = (distance_cm * 10.0) / (5.0 * 200 / 60.0) * 1.2
    assert expected == pytest.approx(0.36)
    assert manual.duration_for_move(distance_cm * 10.0, 200) == pytest.approx(expected)


@pytest.mark.parametrize(
    "distance_mm,rpm,lead,ratio",
    [(0.0, 200, 5.0, 1.0), (5.0, 0, 5.0, 1.0), (5.0, 200, 0.0, 1.0), (5.0, 200, 5.0, 0.0)],
)
def test_duration_rejects_non_positive_inputs(distance_mm, rpm, lead, ratio):
    with pytest.raises(ValueError):
        manual.duration_for_move(
            distance_mm, rpm, screw_lead_mm=lead, motor_revs_per_screw_rev=ratio
        )


def test_tolerance_shrinks_for_short_moves():
    # 2.0 mm 容差 >= 1.5 mm 距离 → 收紧到距离的一半
    assert manual.resolve_tolerance_mm(2.0, 1.5) == pytest.approx(0.75)
    # 距离大于容差时保持原值
    assert manual.resolve_tolerance_mm(2.0, 50.0) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        manual.resolve_tolerance_mm(0.0, 50.0)


def test_execute_passes_shrunk_tolerance_to_controller():
    controller = _FakeLiftController(positions=[1000, 1000 + 3932])
    adjuster, controller, lines = _make_adjuster(controller=controller)
    adjuster.execute("up", "0.15")  # 1.5 mm < 2 mm 容差
    ((direction, distance_mm, duration_s, tolerance_mm),) = controller.moves
    assert direction == "forward"
    assert distance_mm == pytest.approx(1.5)
    assert tolerance_mm == pytest.approx(0.75)
    assert "自动收紧" in _joined(lines)


# ── 实际位移反馈 ──────────────────────────────────────────────────────


def test_execute_reports_actual_encoder_delta():
    # 26214.4 counts/mm；走 5 mm 应 +131072 counts
    controller = _FakeLiftController(positions=[0, 131_072])
    adjuster, controller, lines = _make_adjuster(controller=controller)
    result = adjuster.execute("up", "0.5")
    assert result["up_delta_mm"] == pytest.approx(5.0, abs=1e-6)
    assert "实际位移 +5.000 mm" in _joined(lines)


def test_execute_warns_on_short_move():
    controller = _FakeLiftController(positions=[0, 5243])  # 只有 0.2 mm
    adjuster, controller, lines = _make_adjuster(controller=controller)
    adjuster.execute("up", "0.5")
    assert "实际位移明显小于目标" in _joined(lines)


def test_execute_warns_on_reversed_encoder_delta():
    controller = _FakeLiftController(positions=[0, -131_072])
    adjuster, controller, lines = _make_adjuster(controller=controller)
    adjuster.execute("up", "0.5")
    assert "方向与指令相反" in _joined(lines)


def test_read_position_failure_is_reported_not_fatal():
    class _Broken(_FakeLiftController):
        def read_encoder_position(self):
            raise OSError("serial gone")

    adjuster, controller, lines = _make_adjuster(controller=_Broken())
    assert adjuster.read_position() is None
    adjuster.execute("up", "0.5")
    assert "读取编码器位置失败" in _joined(lines)
    assert "执行完成" in _joined(lines)


# ── 最高点 / 行程软限位 ───────────────────────────────────────────────


def test_mark_origin_then_block_move_past_top():
    controller = _FakeLiftController(positions=[131_072, 131_072])
    adjuster, controller, lines = _make_adjuster(controller=controller)
    adjuster.mark_origin()
    assert adjuster.origin_count == 131_072
    # 已在最高点，再上升 0.5 cm 会被拒绝
    with pytest.raises(ValueError, match="越过记录的最高点"):
        adjuster.execute("up", "0.5")
    assert controller.moves == []


def test_travel_limit_blocks_move_beyond_total_stroke():
    controller = _FakeLiftController(positions=[131_072, 131_072])
    adjuster, controller, lines = _make_adjuster(controller=controller, travel_mm=20.0)
    adjuster.mark_origin()
    with pytest.raises(ValueError, match="超过总行程"):
        adjuster.execute("down", "5.0")  # 50 mm > 20 mm
    assert controller.moves == []


def test_travel_limit_allows_move_within_stroke():
    controller = _FakeLiftController(positions=[131_072, 131_072, 65_536, 65_536])
    adjuster, controller, lines = _make_adjuster(controller=controller, travel_mm=20.0)
    adjuster.mark_origin()
    adjuster.execute("down", "0.5")  # 5 mm < 20 mm
    assert len(controller.moves) == 1
    assert controller.moves[0][0] == "reverse"


def test_origin_is_run_local_and_can_be_cleared():
    adjuster, controller, lines = _make_adjuster(positions=[1000, 1000])
    adjuster.mark_origin()
    assert adjuster.origin_count == 1000
    adjuster.clear_origin()
    assert adjuster.origin_count is None
    adjuster.clear_origin()  # 再次清除不应报错
    assert "当前没有记录最高点" in _joined(lines)


def test_mark_origin_without_travel_reports_no_soft_limit():
    adjuster, controller, lines = _make_adjuster(positions=[500, 500])
    adjuster.mark_origin()
    assert "不会做行程软限位" in _joined(lines)


def test_mark_origin_failure_keeps_origin_unset():
    class _Broken(_FakeLiftController):
        def read_encoder_position(self):
            raise OSError("serial gone")

    adjuster, controller, lines = _make_adjuster(controller=_Broken())
    adjuster.mark_origin()
    assert adjuster.origin_count is None
    assert "无法记录最高点" in _joined(lines)


# ── 交互循环 ──────────────────────────────────────────────────────────


def test_repl_moves_then_quits():
    controller = _FakeLiftController(positions=[0, 0, 131_072, 131_072])
    adjuster, controller, lines = _make_adjuster(
        controller=controller, answers=["up 0.5", "q"]
    )
    assert adjuster.repl() == 0
    assert len(controller.moves) == 1
    assert controller.moves[0][0] == "forward"
    assert "退出手动调整程序" in _joined(lines)


def test_repl_prompts_for_distance_when_omitted():
    controller = _FakeLiftController(positions=[0, 0, 131_072, 131_072])
    prompts: list[str] = []
    adjuster, controller, lines = _make_adjuster(
        controller=controller, answers=["down", "0.5", "quit"], prompts=prompts
    )
    assert adjuster.repl() == 0
    assert controller.moves[0][0] == "reverse"
    assert controller.moves[0][1] == pytest.approx(5.0)
    assert any("请输入本次移动距离" in prompt for prompt in prompts)


def test_repl_cancels_move_on_empty_distance():
    controller = _FakeLiftController(positions=[0])
    adjuster, controller, lines = _make_adjuster(
        controller=controller, answers=["up", "", "exit"]
    )
    assert adjuster.repl() == 0
    assert controller.moves == []
    assert "已取消本次移动" in _joined(lines)


@pytest.mark.parametrize("quit_word", ["q", "quit", "exit", "退出", "结束"])
def test_repl_quit_aliases(quit_word):
    adjuster, controller, lines = _make_adjuster(answers=[quit_word])
    assert adjuster.repl() == 0
    assert controller.moves == []


def test_repl_returns_zero_on_eof():
    adjuster, controller, lines = _make_adjuster(answers=[])
    assert adjuster.repl() == 0
    assert "输入结束" in _joined(lines)


def test_repl_status_and_mark_and_clear_commands():
    controller = _FakeLiftController(positions=[0, 0, 0, 0])
    adjuster, controller, lines = _make_adjuster(
        controller=controller, answers=["s", "m", "clear", "status", "q"]
    )
    assert adjuster.repl() == 0
    text = _joined(lines)
    assert "当前编码器位置" in text
    assert "记为物理最高点" in text
    assert "已清除记录的物理最高点" in text
    assert controller.moves == []


def test_repl_counts_rejected_commands_without_moving():
    adjuster, controller, lines = _make_adjuster(
        answers=["sideways 1", "up abc", "up 99", "q"]
    )
    assert adjuster.repl() == 0
    assert controller.moves == []
    assert _joined(lines).count("拒绝执行") == 3


def test_repl_stops_and_continues_after_motion_failure():
    controller = _FakeLiftController(positions=[0, 0])
    controller.move_error = RuntimeError("驱动器报警")
    adjuster, controller, lines = _make_adjuster(
        controller=controller, answers=["up 0.5", "q"]
    )
    assert adjuster.repl() == 0
    assert controller.stop_calls == [False]  # verify_off=False 的尽力停机
    text = _joined(lines)
    assert "执行失败" in text
    assert "本次未完成" in text


def test_repl_reraises_keyboard_interrupt():
    class _Interrupt(_FakeLiftController):
        def move_timed_distance(self, *args, **kwargs):
            raise KeyboardInterrupt

    adjuster, controller, lines = _make_adjuster(
        controller=_Interrupt(positions=[0]), answers=["up 0.5"]
    )
    with pytest.raises(KeyboardInterrupt):
        adjuster.repl()


def test_repl_banner_warns_about_missing_absolute_encoder():
    adjuster, controller, lines = _make_adjuster(answers=[])
    adjuster.repl()
    text = _joined(lines)
    assert "没有断电保持的多圈编码器" in text
    assert "不会自动回机械零点" in text
    assert "0.5～2 cm" in text


def test_best_effort_stop_swallows_errors():
    class _Refusing(_FakeLiftController):
        def stop(self, *, verify_off=True):
            raise RuntimeError("bus busy")

    adjuster, controller, lines = _make_adjuster(controller=_Refusing())
    adjuster.best_effort_stop()
    assert "紧急停止命令未完全确认" in _joined(lines)


# ── 命令行参数 ────────────────────────────────────────────────────────


def _defaults():
    return {
        "port": "/dev/ttyS0",
        "slave": 2,
        "rpm": 200,
        "forward_sign": 1,
        "tolerance_mm": 2.0,
        "screw_lead_mm": 5.0,
        "motor_revs_per_screw_rev": 1.0,
        "encoder_counts_per_motor_rev": 131_072,
    }


def test_parser_single_shot_positional_args():
    parser = manual.build_parser(_defaults())
    args = parser.parse_args(["up", "0.5"])
    assert args.direction == "up"
    assert args.distance_cm == "0.5"
    assert args.status is False


def test_resolve_options_uses_defaults_and_overrides(monkeypatch):
    for name in ("X2P_RPM", "X2P_FORWARD_SIGN", "MANUAL_LIFT_MAX_CM"):
        monkeypatch.delenv(name, raising=False)
    parser = manual.build_parser(_defaults())
    defaults = _defaults()

    options = manual.resolve_options(parser.parse_args([]), defaults)
    assert options.rpm == 200
    assert options.max_cm == 20.0
    assert options.tolerance_mm == 2.0
    assert options.screw_lead_mm == 5.0
    assert options.encoder_forward_sign == 1

    args = parser.parse_args(
        ["--rpm", "120", "--max-cm", "3", "--tolerance-mm", "1.5", "--travel-mm", "60"]
    )
    options = manual.resolve_options(args, defaults)
    assert options.rpm == 120
    assert options.max_cm == 3.0
    assert options.tolerance_mm == 1.5
    assert options.travel_mm == 60.0


def test_resolve_options_environment_overrides(monkeypatch):
    monkeypatch.setenv("X2P_RPM", "150")
    monkeypatch.setenv("MANUAL_LIFT_MAX_CM", "5")
    parser = manual.build_parser(_defaults())
    options = manual.resolve_options(parser.parse_args([]), _defaults())
    assert options.rpm == 150
    assert options.max_cm == 5.0
    # 命令行优先于环境变量
    options = manual.resolve_options(
        parser.parse_args(["--rpm", "90", "--max-cm", "2"]), _defaults()
    )
    assert options.rpm == 90
    assert options.max_cm == 2.0


@pytest.mark.parametrize(
    "argv",
    [["--rpm", "0"], ["--rpm", "-5"], ["--max-cm", "0"], ["--tolerance-mm", "0"],
     ["--travel-mm", "-1"], ["--forward-sign", "2"]],
)
def test_resolve_options_rejects_out_of_range_values(argv):
    parser = manual.build_parser(_defaults())
    with pytest.raises((ValueError, SystemExit)):
        manual.resolve_options(parser.parse_args(argv), _defaults())


def test_build_config_widens_limits_for_manual_rpm():
    from x2p import ControllerConfig, SafetyLimits

    options = manual.ManualAdjustOptions(
        rpm=200, max_cm=30.0, tolerance_mm=2.0, screw_lead_mm=5.0
    )
    config = manual.build_config(
        options,
        port="/dev/ttyS0",
        slave=2,
        forward_sign=1,
        config_cls=ControllerConfig,
        limits_cls=SafetyLimits,
    )
    assert config.port == "/dev/ttyS0"
    assert config.forward_sign == 1
    assert config.limits.max_rpm >= 200
    assert config.limits.max_distance_mm >= 300.0


def test_counts_per_mm_prefers_controller_config():
    config = _FakeConfig(pulses_per_mm=1000.0)
    adjuster, controller, lines = _make_adjuster(
        controller=_FakeLiftController(config=config)
    )
    assert adjuster.counts_per_mm() == pytest.approx(1000.0)

    # 没有 pulses_per_mm 时按 131072 × 1 / 5 换算
    adjuster, controller, lines = _make_adjuster()
    assert adjuster.counts_per_mm() == pytest.approx(131_072 / 5.0)


def test_encoder_sign_follows_controller_config():
    config = _FakeConfig(encoder_forward_sign=-1)
    adjuster, controller, lines = _make_adjuster(
        controller=_FakeLiftController(config=config)
    )
    assert adjuster.encoder_sign() == -1


def test_projected_offset_helper():
    # 已经低于最高点 10 mm，再下降 5 mm → -15 mm
    offset = manual.projected_up_offset_mm(
        direction="reverse",
        distance_mm=5.0,
        origin_count=0,
        current_count=-int(10 * 131_072 / 5),
        counts_per_mm=131_072 / 5,
    )
    assert offset == pytest.approx(-15.0)


# ── OFF 基线回归（status=3 固件） ─────────────────────────────────────


class _FakeDrive:
    """直连 X2PDrive 的寄存器级桩，覆盖 manual_adjust 用到的全部接口。"""

    def __init__(
        self,
        *,
        status=3,
        enable=0,
        speed=0,
        fault_code=0,
        clears_enable_on_servo_off=True,
    ):
        self.status = status
        self.enable = enable
        self.speed = speed
        self.fault_code = fault_code
        self.clears_enable_on_servo_off = clears_enable_on_servo_off
        self.servo_off_calls = 0
        self.servo_on_calls = 0
        self.speed_writes: list[int] = []

    def read_registers(self, address, count=1):
        if address == Register.STATUS:
            return [self.status]
        if address == Register.SERVO_ENABLE_STATUS:
            return [self.enable]
        if address == Register.ACTUAL_SPEED:
            return [self.speed]
        if address == Register.LAST_FAULT_CODE:
            return [self.fault_code]
        return [0] * count

    def read_signed32(self, address):
        return 0

    def write_register(self, address, value):
        if address == Register.SPEED_COMMAND:
            self.speed_writes.append(value)

    def set_speed(self, rpm):
        self.speed_writes.append(rpm)

    def servo_on(self, additional_forced_inputs=0):
        self.servo_on_calls += 1
        self.enable = 1

    def servo_off(self):
        self.servo_off_calls += 1
        if self.clears_enable_on_servo_off:
            self.enable = 0

    def diagnostic(self, value=0x51A3):
        return None

    def close(self):
        return None


def _controller(drive, **limit_overrides):
    limits = {
        "stop_timeout_s": 0.15,
        "position_timeout_s": 0.2,
    }
    limits.update(limit_overrides)
    config = manual.build_config(
        manual.ManualAdjustOptions(rpm=200, max_cm=20.0, tolerance_mm=2.0),
        port="/dev/ttyS0",
        slave=2,
        forward_sign=1,
        config_cls=__import__("x2p").ControllerConfig,
        limits_cls=__import__("x2p").SafetyLimits,
    )
    return MotionController(
        drive,
        config,
        monitor_interval_s=0.01,
        output=None,
    )


def test_prepare_off_accepts_status_3_when_un058_is_zero():
    """固件上报 status=3 而 Un058=0：旧的 status==OFF 判断会误报互锁。"""
    drive = _FakeDrive(status=3, enable=0, speed=0)
    controller = _controller(drive)
    controller.prepare_off()
    assert controller.state is MotionState.ARMED


def test_stop_verifies_off_with_un058_and_speed():
    drive = _FakeDrive(status=3, enable=1, speed=0)
    controller = _controller(drive)
    controller.stop(verify_off=True)
    assert controller.state is MotionState.OFF
    assert drive.servo_off_calls == 1
    assert 0 in drive.speed_writes


def test_status_fault_reports_e_code():
    drive = _FakeDrive(status=DriveStatus.FAULT, enable=0, speed=0, fault_code=0x0004)
    controller = _controller(drive)
    with pytest.raises(SafetyInterlockError, match="E04"):
        controller.stop(verify_off=True)


def test_wait_disabled_times_out_when_un058_stays_set():
    from x2p.errors import MotionTimeoutError

    drive = _FakeDrive(status=3, enable=1, clears_enable_on_servo_off=False)
    controller = _controller(drive)
    with pytest.raises((MotionTimeoutError, SafetyInterlockError)) as excinfo:
        controller.stop(verify_off=True)
    assert "仍未停机" in str(excinfo.value)


def test_wait_enabled_requires_un058_to_change():
    from x2p.errors import MotionTimeoutError

    class _StubbornDrive(_FakeDrive):
        def servo_on(self, additional_forced_inputs=0):
            self.servo_on_calls += 1  # 不改变 Un058

    controller = _controller(_StubbornDrive(status=3, enable=0))
    with pytest.raises(MotionTimeoutError, match="伺服使能超时"):
        controller._enable_and_verify()


def test_controller_no_longer_waits_on_undocumented_status_register():
    controller = _controller(_FakeDrive())
    assert not hasattr(controller, "_wait_status")


def test_enable_timeout_includes_chain_snapshot_and_verdict():
    """使能超时的报错要带寄存器快照和判读，不能只给一句“检查接线”。"""
    from x2p.errors import MotionTimeoutError

    class _ChainDrive(_FakeDrive):
        def servo_on(self, additional_forced_inputs=0):
            self.servo_on_calls += 1  # 不改变 Un058

        def read_enable_chain(self):
            return {
                "P400_DI1功能": 1,
                "P415_强制输入": 0x01,
                "Un032_DI状态": 0x00,
                "Un058_伺服使能": 0,
                "STATUS_0x3E00": 3,
                "Un100_故障码": 0,
            }

    controller = _controller(_ChainDrive(status=3, enable=0))
    with pytest.raises(MotionTimeoutError) as excinfo:
        controller._enable_and_verify()
    message = str(excinfo.value)
    assert "使能链路快照" in message
    assert "Un032_DI状态=0" in message
    assert "24V/DI 公共端" in message


def test_enable_chain_hints_flag_di_active_but_refused():
    hints = MotionController._enable_chain_hints(
        {
            "P400_DI1功能": 1,
            "P415_强制输入": 0x01,
            "Un032_DI状态": 0x01,
            "Un058_伺服使能": 0,
        }
    )
    assert any("主动拒绝使能" in hint for hint in hints)


def test_enable_chain_hint_is_optional_for_stub_drives():
    """桩驱动没有 read_enable_chain 时，原报错必须保持原样。"""
    controller = _controller(_FakeDrive(status=3, enable=0))
    assert controller._enable_chain_hint() == ""


def test_read_enable_chain_reports_per_register_failures():
    """快照里单个寄存器读失败只记文字，不能让整份诊断中断。"""
    from x2p.drive import X2PDrive
    from x2p.registers import Register

    class _PartialClient:
        def read_registers(self, address, count=1):
            if address == Register.DIGITAL_INPUT_STATUS:
                raise RuntimeError("timeout")
            return [1]

        def close(self):
            return None

    drive = object.__new__(X2PDrive)  # 仅注入桩 client，绕过串口构造
    drive.client = _PartialClient()
    snapshot = drive.read_enable_chain()
    assert snapshot["Un032_DI状态"] == "读取失败(timeout)"
    assert snapshot["Un058_伺服使能"] == 1


def test_is_disabled_and_stopped_requires_zero_speed():
    controller = _controller(_FakeDrive(status=3, enable=0, speed=0))
    assert controller._is_disabled_and_stopped() is True
    controller = _controller(_FakeDrive(status=3, enable=0, speed=12))
    assert controller._is_disabled_and_stopped() is False


def test_manual_adjuster_drives_real_controller_move_path(monkeypatch):
    """手动程序可以直接驱动 MotionController（encoder 闭环路径）。"""
    drive = _FakeDrive(status=3, enable=0, speed=0)
    controller = _controller(drive)
    monkeypatch.setattr(controller, "read_encoder_position", lambda: 0)
    moves: list[tuple] = []

    def _fake_move(direction, distance_mm, duration_s, *, tolerance_mm=None):
        moves.append((direction, distance_mm, duration_s, tolerance_mm))
        return object()

    monkeypatch.setattr(controller, "move_timed_distance", _fake_move)
    adjuster = manual.ManualLiftAdjuster(
        controller,
        manual.ManualAdjustOptions(rpm=200, max_cm=20.0, tolerance_mm=2.0),
        output=list().append,
        input_fn=_scripted_input([]),
    )
    adjuster.execute("down", "0.5")
    assert moves == [("reverse", 5.0, pytest.approx(0.36), pytest.approx(2.0))]
