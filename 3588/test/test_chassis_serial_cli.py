import importlib.util
from pathlib import Path
import sys
import subprocess
from types import SimpleNamespace

import pytest

from grain_sampling_devices import chassis_protocol as p
from grain_sampling_devices.chassis_serial import ChassisError

spec = importlib.util.spec_from_file_location(
    "chassis_test_cli", Path(__file__).resolve().parents[1] / "scripts" / "test_chassis_serial.py")
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class FakeLink:
    def __init__(self):
        self.now = 0.0
        self.status_time = -1
        self.sequence = 0
        self.commands = []
        self.status = dict(state=2, mode=2, flags=4, faults=0, rc_age_ms=0,
                           ch1=1500, ch3=1500, ch8=2000, left=0, right=0,
                           pwm_left=1500, pwm_right=1500, pwm_enabled=1,
                           motion_sequence=0)

    def clock(self):
        return self.now

    def sleep(self, delay):
        self.now += delay

    def receive(self):
        self.now += 0.01
        self.status_time = self.now

    def request(self, kind):
        self.commands.append(kind)
        if kind == p.AUTO_STOP:
            self.status.update(state=2, flags=4, left=0, right=0, pwm_left=1500, pwm_right=1500)

    stream_control = request

    def token_request(self, kind):
        self.commands.append(kind)
        self.status.update(state=3, flags=5)

    def stream_effort(self, forward, turn):
        self.sequence += 1
        left, right = forward - turn, forward + turn
        self.status.update(left=left, right=right, pwm_left=1500 + int(left / 4),
                           pwm_right=1500 + int(right / 4), motion_sequence=self.sequence)


@pytest.mark.parametrize("direction,expected", [
    ("forward", (200, 200)), ("backward", (-200, -200)),
    ("left", (-200, 200)), ("right", (200, -200))])
def test_motion_directions_and_stop_sent(direction, expected, capsys):
    link = FakeLink()
    outputs = []
    original = link.stream_effort

    def effort(forward, turn):
        original(forward, turn)
        outputs.append((link.status["left"], link.status["right"]))

    link.stream_effort = effort
    cli.move(link, direction, 0.2, 0.3, link.clock, link.sleep)
    assert outputs and all(output == expected for output in outputs)
    assert link.commands == [p.AUTO_STOP, p.AUTO_STOP]
    assert link.status["left"] == link.status["right"] == 0
    assert "发送结束" in capsys.readouterr().out


def test_missing_motion_feedback_does_not_block_stream(capsys):
    link = FakeLink()
    link.stream_effort = lambda *args: None
    cli.move(link, "forward", 0.2, 0.3, link.clock, link.sleep)
    assert "不代表单片机执行或实际运动已验证" in capsys.readouterr().out
    assert link.commands[-1] == p.AUTO_STOP


@pytest.mark.parametrize("arguments", [
    ["--effort", "nan"], ["--effort", "inf"], ["--effort", "1.1"],
    ["--seconds", "0"], ["--seconds", "11"]])
def test_invalid_motion_parameters_rejected_before_open(arguments):
    with pytest.raises(SystemExit):
        cli.parser().parse_args(["forward"] + arguments)


@pytest.mark.parametrize("failure,code", [(KeyboardInterrupt, 130), (ChassisError, 1)])
def test_main_stops_and_closes_after_motion_failure(monkeypatch, failure, code):
    link = FakeLink()
    link.boot, link.session, link.epoch = 1, 2, 3
    link.close = lambda: link.commands.append("close")
    port = SimpleNamespace(reset_input_buffer=lambda: None)
    monkeypatch.setitem(sys.modules, "serial", SimpleNamespace(Serial=lambda *a, **kw: port))
    monkeypatch.setattr(cli, "ChassisSerial", lambda port: link)

    def failed_move(*args):
        raise failure()

    monkeypatch.setattr(cli, "move", failed_move)
    assert cli.main(["forward", "--port", "fake"]) == code
    assert link.commands[-2:] == [p.AUTO_STOP, "close"]


def test_help_runs_outside_project_without_pythonpath(tmp_path):
    result = subprocess.run([sys.executable, str(Path(cli.__file__)), "--help"],
                            cwd=tmp_path, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert b"--port" in result.stdout
