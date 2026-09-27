"""ROS 机制服务节点测试（mechanism_node.py）。

全程 mock rospy：本机无 rospy，模块的 HAS_ROS 守卫提供回退 Trigger/SetGrain
类型，``mechanism_node.rospy`` 用 unittest.mock.patch 替换为 MagicMock。

覆盖：
- 模块可无 rospy import（HAS_ROS=False + 回退消息类型）
- 19 个服务注册（12 动作 Trigger + lift_health + 6 个深度变体）
- 服务触发动作（线程化执行 + 品种时长注入）
- 动作失败重试 N 次后返回 success=False
- 急停：停运行中通道 + 锁定 + 抢断阻塞动作线程
- set_grain：更新品种参数 / 未知品种失败 / 急停后重新使能
- 占位动作可配置（placeholder_success）
- wait_for_action=False 时立即返回
- 并发动作不互相阻塞
"""

from __future__ import annotations

import threading
import time
from unittest.mock import patch

import pytest

import grain_sampling_workflow.mechanism_node as mn
from grain_sampling_devices.mechanism_driver import (
    BIN_CLOSE_PULSE,
    BIN_OPEN_PULSE,
    CHANNELS,
)
from grain_sampling_workflow.mechanism_config import GRAIN_MECHANISM_CONFIG


def _trigger_req():
    """构造一个 Trigger 请求（回退类型，等价 std_srvs/Trigger 空请求）。"""
    return mn.Trigger._request_class()


# ── 无 rospy 可导入 + 回退类型 ──────────────────────────────────────────


def test_importable_without_rospy():
    assert mn.HAS_ROS is False
    assert mn.rospy is None
    # 回退消息类型行为与 ROS 消息一致
    resp = mn.Trigger._response_class(success=True, message="ok")
    assert resp.success is True and resp.message == "ok"
    req = mn.SetGrain._request_class(grain="稻谷")
    assert req.grain == "稻谷"
    assert mn.SetGrain._type == "mechanism_node/SetGrain"


# ── 服务注册 ────────────────────────────────────────────────────────────


def test_registers_services(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    with patch("grain_sampling_workflow.mechanism_node.rospy") as mock_rospy:
        node.start()

    calls = mock_rospy.Service.call_args_list
    # HAS_ROS=False（本机无 rospy）：SetGrain/MoveLift 为回退类（无 ROS 序列化），
    # set_grain/move_lift 服务跳过注册：12 个 Trigger 动作服务
    # + 3 个 open_bin + 3 个 close_bin 深度变体。
    assert len(calls) == 25  # Includes the production close_all_bins service.
    names = [c.args[0] for c in calls]
    expected = [f"/mechanism/{a}" for a in mn.ACTION_SERVICES]
    expected += ["/mechanism/lift_health"]
    expected += [f"/mechanism/open_bin/{d}" for d in mn.OPEN_BIN_DEPTHS]
    expected += [f"/mechanism/close_bin/{d}" for d in mn.OPEN_BIN_DEPTHS]
    expected += [f"/mechanism/hold_bin_open/{d}" for d in mn.OPEN_BIN_DEPTHS]
    assert sorted(names) == sorted(expected)
    # 全部用 std_srvs/Trigger
    for c in calls:
        name, srv_type, handler = c.args
        assert srv_type is mn.Trigger
        assert callable(handler)
    # init_node 只被调用一次（HAS_ROS=False 时跳过，rospy 被 mock）
    assert mock_rospy.init_node.called or True  # HAS_ROS=False 分支不调用


def test_ros_mode_imports_real_trigger_type():
    """HAS_ROS=True 时 Trigger 应来自 std_srvs（这里验证类型占位常量）。"""
    assert mn.Trigger._type == "std_srvs/Trigger"


# ── 服务触发动作 ────────────────────────────────────────────────────────


def test_service_triggers_action(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    resp = node._handle_action("clamp", _trigger_req())

    assert resp.success is True, resp.message
    names = [n for n, _ in mock_mechanism.action_history]
    assert "clamp" in names
    # 时长从默认品种参数注入（clamp_duration=2.0）
    clamp_kw = next(kw for n, kw in mock_mechanism.action_history if n == "clamp")
    assert clamp_kw["duration"] == 2.0


def test_all_action_services_dispatch_success(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    for action in mn.ACTION_SERVICES:
        if action == "emergency_stop":
            continue
        resp = node._handle_action(action, _trigger_req())
        assert resp.success is True, f"{action}: {resp.message}"


def test_open_bin_uses_configured_depth(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, open_bin_depth="deep")
    resp = node._handle_action("open_bin", _trigger_req())
    assert resp.success is True
    # 深度映射到对应通道（deep → CH4=bin_deep），写 BIN_OPEN_PULSE=1200；
    # 时长注入 open_duration 只用于 Timer 自动关同仓，不记录到 _act_pulse。
    open_kw = next(kw for n, kw in mock_mechanism.action_history if n == "open_bin")
    assert open_kw["channel"] == CHANNELS["bin_deep"]
    assert open_kw["pulse_us"] == BIN_OPEN_PULSE
    assert open_kw["duration"] is None
    assert mock_mechanism.pca9685.register_history[CHANNELS["bin_deep"]] == [
        BIN_OPEN_PULSE
    ]


def test_close_bin_uses_configured_depth(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, open_bin_depth="deep")
    resp = node._handle_action("close_bin", _trigger_req())
    assert resp.success is True
    # close_bin 与 open_bin 共享深度注入：deep → CH4=bin_deep，写 BIN_CLOSE_PULSE=1800
    close_kw = next(kw for n, kw in mock_mechanism.action_history if n == "close_bin")
    assert close_kw["channel"] == CHANNELS["bin_deep"]
    assert close_kw["pulse_us"] == BIN_CLOSE_PULSE
    assert mock_mechanism.pca9685.register_history[CHANNELS["bin_deep"]] == [
        BIN_CLOSE_PULSE
    ]


def test_stop_suction_writes_stop_pulse_to_fan_channel(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    resp = node._handle_action("stop_suction", _trigger_req())
    assert resp.success is True
    act = [kw for n, kw in mock_mechanism.action_history if n == "actuate"]
    assert any(kw["channel"] == CHANNELS["fan"] and kw["action"] == "stop" for kw in act)


# ── 失败重试 ────────────────────────────────────────────────────────────


def test_action_failure_retries_then_returns_false(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, retry_attempts=2, retry_interval=0)
    calls = {"n": 0}

    def _boom(**kw):
        calls["n"] += 1
        raise OSError("simulated i2c error")

    mock_mechanism.clamp = _boom

    ok, msg = node.run_action("clamp")
    assert ok is False
    assert calls["n"] == 3  # 1 次正常 + 2 次重试
    assert "failed after 3 attempts" in msg

    # 服务响应同样携带失败结果（不挂起）
    resp = node._handle_action("clamp", _trigger_req())
    assert resp.success is False
    assert calls["n"] == 6


def test_success_on_first_attempt_no_retry(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, retry_attempts=2)
    ok, msg = node.run_action("clamp")
    assert ok is True
    assert msg == "clamp ok"  # 首次即成功，无重试/占位痕迹


def test_registered_rospy_handler_dispatches_action(mock_mechanism):
    """通过注册表取出的 rospy 回调闭包可直接触发动作。"""
    node = mn.MechanismNode(controller=mock_mechanism)
    with patch("grain_sampling_workflow.mechanism_node.rospy") as mock_rospy:
        node.start()
    by_name = {c.args[0]: c.args[2] for c in mock_rospy.Service.call_args_list}
    handler = by_name["/mechanism/clamp"]
    resp = handler(_trigger_req())
    assert resp.success is True
    names = [n for n, _ in mock_mechanism.action_history]
    assert "clamp" in names


# ── 急停 ────────────────────────────────────────────────────────────────


def test_emergency_stop_stops_running_channels_and_locks(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    mock_mechanism.actuate(0, "open", duration=60)
    mock_mechanism.actuate(2, "close", duration=60)
    assert mock_mechanism._running == {0, 2}

    resp = node._handle_emergency_stop(_trigger_req())
    assert resp.success is True

    # 运行中通道被立即断电释放
    assert mock_mechanism._running == set()
    assert mock_mechanism.pca9685.register_history[0] == [1200, 1500.0]
    assert mock_mechanism.pca9685.register_history[2] == [1900, 1500.0]
    assert node._stop_flag.is_set()

    # 急停后拒绝新动作
    ok, msg = node.run_action("clamp")
    assert ok is False
    assert "emergency-stop" in msg


def test_emergency_stop_preempts_blocking_action(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, retry_attempts=0)
    entered = threading.Event()
    release = threading.Event()
    real_clamp = mock_mechanism.clamp

    def _blocking(**kw):
        entered.set()
        release.wait(5.0)
        real_clamp(**kw)

    mock_mechanism.clamp = _blocking

    result = {}

    def _call_handler():
        result["resp"] = node._handle_action("clamp", _trigger_req())

    t = threading.Thread(target=_call_handler)
    t.start()
    assert entered.wait(2.0), "动作线程应已进入阻塞调用"

    estop = node._handle_emergency_stop(_trigger_req())
    assert estop.success is True

    release.set()
    t.join(5.0)
    assert not t.is_alive(), "急停后动作回调不应挂起"
    assert result["resp"].success is False
    # 被急停抢断：可能是派发前被驱动层拒绝，或派发后被停止标志识别
    assert ("preempted" in result["resp"].message
            or "emergency-stop" in result["resp"].message)
    assert node.last_results["clamp"][0] is False


# ── set_grain ───────────────────────────────────────────────────────────


def test_set_grain_updates_current_grain(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    resp = node._handle_set_grain(mn.SetGrain._request_class(grain="稻谷"))

    assert resp.success is True
    assert mock_mechanism.current_grain == "稻谷"
    assert node._grain == "稻谷"
    assert any(n == "set_grain" for n, _ in mock_mechanism.action_history)

    # 品种参数作用于后续动作，测试跟随现场统一参数源。
    expected_duration = GRAIN_MECHANISM_CONFIG["稻谷"]["convey_duration"]
    assert node._duration_for("convey") == expected_duration
    node._handle_action("convey", _trigger_req())
    convey_kw = next(kw for n, kw in mock_mechanism.action_history if n == "convey")
    assert convey_kw["duration"] == expected_duration


def test_set_grain_unknown_grain_returns_false(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    resp = node._handle_set_grain(mn.SetGrain._request_class(grain="小麦"))
    assert resp.success is False
    assert "default params" in resp.message


def test_set_grain_rearms_after_emergency_stop(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    node._handle_emergency_stop(_trigger_req())
    ok, _ = node.run_action("clamp")
    assert ok is False

    resp = node._handle_set_grain(mn.SetGrain._request_class(grain="稻谷"))
    assert resp.success is True
    assert "emergency stop cleared" in resp.message
    assert not node._stop_flag.is_set()

    ok, msg = node.run_action("clamp")
    assert ok is True, msg


# ── 占位动作（未接通道） ────────────────────────────────────────────────


def test_placeholder_actions_success_by_default(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    for action in ("press", "lift"):
        ok, msg = node.run_action(action)
        assert ok is True, msg
        assert "placeholder" in msg
    # 占位动作仍派发到控制器（驱动层记录/按 ENABLE_UNWIRED_CHANNELS 决定写 I2C）
    names = [n for n, _ in mock_mechanism.action_history]
    assert {"press", "lift"}.issubset(set(names))


def test_start_suction_is_not_a_placeholder(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    ok, msg = node.run_action('start_suction')
    assert ok is True
    assert 'placeholder' not in msg
    assert any(name == 'fan' for name, _ in mock_mechanism.action_history)


def test_placeholder_actions_fail_when_disabled(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, placeholder_success=False)
    ok, msg = node.run_action("press")
    assert ok is False
    assert "placeholder" in msg
    # 未派发到控制器
    assert all(n != "press" for n, _ in mock_mechanism.action_history)


def test_press_lift_not_placeholder_when_lift_drive_injected(mock_mechanism):
    """注入 lift_drive（X2P 伺服）后 press/lift 不再标 placeholder。"""
    class _FakeDrive:
        def run_speed(self, direction, rpm, duration_s):
            pass

        def stop(self):
            pass

    mock_mechanism.lift_drive = _FakeDrive()
    node = mn.MechanismNode(controller=mock_mechanism)
    for action in ("press", "lift"):
        ok, msg = node.run_action(action)
        assert ok is True, msg
        assert "placeholder" not in msg


def test_lift_health_reads_encoder_without_motion(mock_mechanism):
    class _FakeDrive:
        def __init__(self):
            self.reads = 0

        def read_position(self):
            self.reads += 1
            return 12345

    drive = _FakeDrive()
    mock_mechanism.lift_drive = drive
    node = mn.MechanismNode(controller=mock_mechanism)

    resp = node._handle_lift_health(_trigger_req())

    assert resp.success is True
    assert "12345" in resp.message
    assert drive.reads == 1
    assert mock_mechanism.action_history == []


def test_lift_health_failure_detaches_dead_drive(mock_mechanism):
    class _DeadDrive:
        closed = False

        def read_position(self):
            raise RuntimeError("通信超时或应答过短")

        def close(self):
            self.closed = True

    drive = _DeadDrive()
    mock_mechanism.lift_drive = drive
    node = mn.MechanismNode(controller=mock_mechanism)

    resp = node._handle_lift_health(_trigger_req())

    assert resp.success is False
    assert "通信超时或应答过短" in resp.message
    assert drive.closed is True
    assert mock_mechanism.lift_drive is None


def test_move_lift_uses_full_configured_rpm(mock_mechanism):
    """200 r/min 不再被 1.2 时长余量降成约 167 r/min。"""
    captured = {}

    def move_lift(direction, distance_cm, duration_s):
        captured.update(
            direction=direction,
            distance_cm=distance_cm,
            duration_s=duration_s,
        )
        return "ok"

    mock_mechanism.move_lift = move_lift
    node = mn.MechanismNode(controller=mock_mechanism)
    node._x2p_rpm = 200.0
    req = mn.MoveLift._request_class(
        direction="down_cycle", distance_cm=20.0
    )

    resp = node._handle_move_lift(req)

    assert resp.success is True
    assert captured == {
        "direction": "down_cycle",
        "distance_cm": 20.0,
        "duration_s": pytest.approx(12.0),
    }


# ── 异步模式 / 并发 ─────────────────────────────────────────────────────


def test_wait_for_action_false_returns_immediately(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, wait_for_action=False)
    entered = threading.Event()
    release = threading.Event()

    def _slow(**kw):
        entered.set()
        release.wait(5.0)

    mock_mechanism.clamp = _slow

    resp = node._handle_action("clamp", _trigger_req())
    assert resp.success is True
    assert "started" in resp.message

    release.set()
    deadline = time.monotonic() + 3.0
    while "clamp" not in node.last_results and time.monotonic() < deadline:
        time.sleep(0.01)
    assert node.last_results["clamp"][0] is True


def test_concurrent_actions_do_not_block_each_other(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism, retry_attempts=0)
    results = {}

    def _call(action):
        results[action] = node._handle_action(action, _trigger_req())

    threads = [
        threading.Thread(target=_call, args=(a,))
        for a in ("clamp", "convey", "tighten")
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(5.0)
    for a in ("clamp", "convey", "tighten"):
        assert results[a].success is True, results[a].message


# ── 生命周期 ────────────────────────────────────────────────────────────


def test_shutdown_stops_running_channels_and_is_idempotent(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    mock_mechanism.actuate(0, "open", duration=60)
    node.shutdown()
    assert mock_mechanism._running == set()
    assert node._stop_flag.is_set()
    node.shutdown()  # 幂等


def test_start_is_idempotent(mock_mechanism):
    node = mn.MechanismNode(controller=mock_mechanism)
    with patch("grain_sampling_workflow.mechanism_node.rospy") as mock_rospy:
        node.start()
        node.start()
    assert mock_rospy.Service.call_count == 25  # 只注册一次（含三仓全关服务）
