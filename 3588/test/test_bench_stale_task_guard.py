"""Safety checks for one-key bench-session replacement (no hardware)."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "bench_stale_task_guard.py"
SPEC = importlib.util.spec_from_file_location("bench_stale_task_guard", SCRIPT)
assert SPEC and SPEC.loader
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def make_args(tmp_path: Path, *, software_stop: bool) -> argparse.Namespace:
    marker = tmp_path / "current_task.json"
    marker.write_text(json.dumps({"task_id": "T-local", "source": "local"}), encoding="utf-8")
    log_root = tmp_path / "bench"
    log = log_root / "session" / "ui-stdout.log"
    log.parent.mkdir(parents=True)
    lines = ["UI session=/tmp/ui-123.log"]
    if software_stop:
        lines.append(
            "[ERROR] Stopping FSM: mechanism set_grain failed: "
            "chassis must be stopped, online and in auto mode"
        )
    lines.append("FSM: SamplingState.NAVIGATE_TO_POINT -> SamplingState.STOPPED")
    log.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return argparse.Namespace(ui_pid=123, daemon_pid=456, marker=marker, log_root=log_root)


def test_known_premotion_software_estop_is_cleared(monkeypatch, tmp_path):
    args = make_args(tmp_path, software_stop=True)
    monkeypatch.setattr(guard, "ui_window_exists", lambda pid: True)
    monkeypatch.setattr(guard, "daemon_socket", lambda pid: "/tmp/grain-bench.test/control.sock")
    monkeypatch.setattr(guard.time, "sleep", lambda delay: None)
    calls = []
    locked = True

    def request(path, action, **values):
        nonlocal locked
        calls.append((action, values))
        if action == "clear_estop":
            assert values == {"mechanical_reset_confirmed": False}
            locked = False
            return {"ok": True}
        return {"ok": True, "chassis": {
            "motion_armed": False, "chassis_link": "online",
            "estop_latched": locked, "rc_mode": "auto", "rc_valid": True,
            "faults": 0,
        }, "navigation": None}

    monkeypatch.setattr(guard, "ipc_request", request)
    guard.check(args)
    assert [action for action, _ in calls].count("clear_estop") == 1
    assert args.marker.exists()  # Guard never deletes the task itself.


def test_unknown_estop_origin_is_not_cleared(monkeypatch, tmp_path):
    args = make_args(tmp_path, software_stop=False)
    monkeypatch.setattr(guard, "ui_window_exists", lambda pid: True)
    monkeypatch.setattr(guard, "daemon_socket", lambda pid: "/tmp/grain-bench.test/control.sock")
    calls = []

    def request(path, action, **values):
        calls.append(action)
        return {"ok": True, "chassis": {
            "motion_armed": False, "chassis_link": "online",
            "estop_latched": True, "rc_mode": "auto", "rc_valid": True,
            "faults": 0,
        }}

    monkeypatch.setattr(guard, "ipc_request", request)
    with pytest.raises(RuntimeError, match="急停来源不是已识别"):
        guard.check(args)
    assert "clear_estop" not in calls


def test_offline_premotion_stopped_task_can_be_archived(tmp_path):
    args = make_args(tmp_path, software_stop=False)
    log = args.log_root / "session" / "ui-stdout.log"
    log.write_text(
        "UI session=/tmp/ui-123.log\n"
        "Current task persisted: T-local (local)\n"
        "FSM: SamplingState.NAVIGATE_TO_POINT -> SamplingState.STOPPED\n",
        encoding="utf-8",
    )
    newer = args.log_root / "newer-session" / "ui-stdout.log"
    newer.parent.mkdir(parents=True)
    newer.write_text("UI opened without a task\n", encoding="utf-8")
    guard.check_offline(args)
    assert args.marker.exists()


def test_offline_task_with_mechanism_action_is_not_discarded(tmp_path):
    args = make_args(tmp_path, software_stop=False)
    log = args.log_root / "session" / "ui-stdout.log"
    log.write_text(
        "UI session=/tmp/ui-123.log\n"
        "Current task persisted: T-local (local)\n"
        "Mechanism clamp begin attempt=1/3\n"
        "FSM: SamplingState.PRESS_AND_SUCTION -> SamplingState.STOPPED\n",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="已开始机构动作"):
        guard.check_offline(args)
