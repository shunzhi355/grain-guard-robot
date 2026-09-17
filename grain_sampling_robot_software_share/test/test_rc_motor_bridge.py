from __future__ import annotations

import pytest

from grain_sampling_workflow.rc_motor_bridge import (
    RCTrackMotorBridge,
    rc_cmd_vel_to_tracks,
)


class FakeSocket:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, tuple[str, int]]] = []
        self.closed = False

    def sendto(self, data: bytes, address: tuple[str, int]) -> None:
        self.sent.append((data, address))

    def close(self) -> None:
        self.closed = True


class FakeNavigationBridge:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def cancel_goal(self) -> bool:
        self.cancel_calls += 1
        return True


def test_channel_mapping_is_independent() -> None:
    assert rc_cmd_vel_to_tracks(0.3, 0.0) == pytest.approx((1.0, 0.0))
    assert rc_cmd_vel_to_tracks(0.0, 0.8) == pytest.approx((0.0, -1.0))
    assert rc_cmd_vel_to_tracks(-0.3, -0.8) == pytest.approx((-1.0, 1.0))


def test_bridge_sends_lr_and_stops_on_takeover_and_close() -> None:
    nav = FakeNavigationBridge()
    sock = FakeSocket()
    bridge = RCTrackMotorBridge(nav, sock=sock)

    bridge.cancel_goal()
    bridge.publish_cmd_vel(0.15, -0.4)
    bridge.close()

    assert nav.cancel_calls == 1
    assert [item[0] for item in sock.sent] == [
        b"stop",
        b"lr 0.5000 0.5000",
        b"stop",
    ]
    assert sock.closed
