from grain_sampling_local.localization_health import LocalizationHealthGate


def sample(gate, stamp, x=0.0, y=0.0, yaw=0.0, now=None, frame="camera_init"):
    return gate.accept(now=stamp if now is None else now, stamp=stamp,
                       x=x, y=y, yaw=yaw, frame_id=frame)


def test_requires_consecutive_fresh_samples():
    gate = LocalizationHealthGate(required_samples=3)
    assert sample(gate, 1.0) is False
    assert sample(gate, 1.1) is False
    assert sample(gate, 1.2) is True
    assert gate.reason == "healthy"


def test_rejects_frame_timestamp_and_jump_faults():
    gate = LocalizationHealthGate(required_samples=1)
    assert sample(gate, 1.0) is True
    assert sample(gate, 1.0) is False
    assert gate.reason == "timestamp_not_increasing"
    assert sample(gate, 1.1, frame="body") is False
    assert gate.reason == "frame_mismatch"
    assert sample(gate, 1.2, x=2.0) is False
    assert gate.reason == "pose_jump"


def test_timeout_invalidates_after_recovery():
    gate = LocalizationHealthGate(required_samples=1, max_stream_gap=0.5)
    assert sample(gate, 1.0) is True
    assert gate.tick(now=1.6) is False
    assert gate.reason == "odometry_timeout"
    assert sample(gate, 1.7) is True


def test_stale_sample_never_becomes_valid():
    gate = LocalizationHealthGate(required_samples=1, max_sample_age=0.2)
    assert sample(gate, 1.0, now=1.3) is False
    assert gate.reason == "stale_or_future"
