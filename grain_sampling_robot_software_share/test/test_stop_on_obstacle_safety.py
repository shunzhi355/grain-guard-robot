"""Safety regressions for obstacle handling."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_obstacle_node_never_sends_motor_udp_commands():
    source = (
        ROOT / "src/grain_sampling_workflow/stop_on_obstacle.py"
    ).read_text(encoding="utf-8")

    assert "sendto(" not in source
    assert "lr 0.1 0.1" not in source
    assert "Publisher('/obstacle_detected'" in source
