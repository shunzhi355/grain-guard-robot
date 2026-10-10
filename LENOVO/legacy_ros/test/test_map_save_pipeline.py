from __future__ import annotations

import os
from pathlib import Path
import signal
import struct
from unittest.mock import mock_open

import pytest

from grain_sampling_workflow import map_save, slam_bridge
from grain_sampling_workflow.slam_bridge import SlamBridge
from scripts.recover_sfast_scans import merge_scans


class FakeProcess:
    def __init__(self, pid: int, returncode=None):
        self.pid = pid
        self.returncode = returncode
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        return self.returncode

    def terminate(self):
        self.returncode = -signal.SIGTERM


def _attach_owned_process(monkeypatch, bridge, process, pgid=None):
    actual_pgid = process.pid if pgid is None else pgid
    bridge._mapping_process = process
    bridge._mapping_pid = process.pid
    bridge._mapping_pgid = actual_pgid
    monkeypatch.setattr(slam_bridge.os, "getpgid", lambda _pid: actual_pgid, raising=False)


def test_content_identity_ignores_timestamp_only_change(tmp_path):
    path = tmp_path / "GlobalMap.pcd"
    path.write_bytes(b"old-map")
    before = map_save.snapshot_map(str(path))
    os.utime(path, ns=(before.mtime_ns + 1_000_000, before.mtime_ns + 1_000_000))
    after = map_save.snapshot_map(str(path))
    assert not map_save.map_content_changed(before, after)


def test_copy_fresh_map_rejects_stale_and_creates_no_copy(tmp_path):
    source = tmp_path / "GlobalMap.pcd"
    destination = tmp_path / "named.pcd"
    source.write_bytes(b"old-map")
    before = map_save.snapshot_map(str(source))
    with pytest.raises(map_save.StaleMapError):
        map_save.copy_fresh_map(str(source), str(destination), before)
    assert not destination.exists()


def test_copy_fresh_map_accepts_changed_content(tmp_path):
    source = tmp_path / "GlobalMap.pcd"
    destination = tmp_path / "named.pcd"
    source.write_bytes(b"old-map")
    before = map_save.snapshot_map(str(source))
    source.write_bytes(b"new-map")
    copied = map_save.copy_fresh_map(str(source), str(destination), before)
    assert copied.sha256 == map_save.snapshot_map(str(source)).sha256


def test_bridge_rejects_process_exit_without_update(tmp_path, monkeypatch):
    global_map = tmp_path / "GlobalMap.pcd"
    global_map.write_bytes(b"old-map")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    process = FakeProcess(101)
    _attach_owned_process(monkeypatch, bridge, process)

    def _killpg(_pgid, _signal):
        process.returncode = 0

    monkeypatch.setattr(slam_bridge.os, "killpg", _killpg, raising=False)
    assert not bridge.save_current_map(timeout=1)


def test_bridge_accepts_new_map_after_clean_exit(tmp_path, monkeypatch):
    global_map = tmp_path / "GlobalMap.pcd"
    global_map.write_bytes(b"old-map")
    before = map_save.snapshot_map(str(global_map))
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    process = FakeProcess(202)
    _attach_owned_process(monkeypatch, bridge, process)

    def _signal(_pgid, sig):
        assert sig == signal.SIGINT
        global_map.write_bytes(b"new-map")
        process.returncode = 0

    monkeypatch.setattr(slam_bridge.os, "killpg", _signal, raising=False)
    assert bridge.save_current_map(before_identity=before, timeout=1)
    assert bridge.mapping_pid is None


def test_bridge_timeout_never_reports_existing_old_map(tmp_path, monkeypatch):
    (tmp_path / "GlobalMap.pcd").write_bytes(b"old-map")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    process = FakeProcess(303)
    _attach_owned_process(monkeypatch, bridge, process)
    monkeypatch.setattr(slam_bridge.os, "killpg", lambda *_: None, raising=False)
    assert not bridge.save_current_map(timeout=0)


def test_mapping_launch_execs_real_process_in_owned_session(monkeypatch):
    bridge = SlamBridge()
    process = FakeProcess(404)
    calls = []

    def _popen(args, **kwargs):
        calls.append((args, kwargs))
        return process

    monkeypatch.setattr("builtins.open", mock_open())
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.subprocess.Popen", _popen)
    monkeypatch.setattr(slam_bridge.os, "getpgid", lambda pid: pid, raising=False)

    assert bridge._launch_mapping_process()
    args, kwargs = calls[0]
    command = args[2]
    assert kwargs["start_new_session"] is True
    assert "exec " in command
    assert slam_bridge.SFAST_EXECUTABLE in command
    assert "setsid" not in command
    assert "nohup" not in command
    assert not command.rstrip().endswith("&")
    assert bridge.mapping_pid == 404
    assert bridge.mapping_pgid == 404


def test_wrapper_exit_with_surviving_child_is_not_owned(monkeypatch, tmp_path):
    """A dead wrapper must never authorize signalling a detached child."""
    (tmp_path / "GlobalMap.pcd").write_bytes(b"old-map")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    wrapper = FakeProcess(505, returncode=0)
    _attach_owned_process(monkeypatch, bridge, wrapper)
    signals = []
    monkeypatch.setattr(
        slam_bridge.os, "killpg", lambda *args: signals.append(args), raising=False
    )
    assert not bridge.save_current_map(timeout=1)
    assert signals == []


def test_stale_pid_or_pgid_is_rejected(monkeypatch):
    bridge = SlamBridge()
    process = FakeProcess(606)
    bridge._mapping_process = process
    bridge._mapping_pid = 606
    bridge._mapping_pgid = 606
    monkeypatch.setattr(slam_bridge.os, "getpgid", lambda _pid: 999, raising=False)
    signals = []
    monkeypatch.setattr(
        slam_bridge.os, "killpg", lambda *args: signals.append(args), raising=False
    )
    assert not bridge._signal_owned_mapping(signal.SIGINT)
    assert signals == []


def test_repeated_sessions_signal_only_current_pgid(monkeypatch, tmp_path):
    global_map = tmp_path / "GlobalMap.pcd"
    global_map.write_bytes(b"map-0")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    groups = []

    for index, pid in enumerate((701, 702), start=1):
        process = FakeProcess(pid)
        _attach_owned_process(monkeypatch, bridge, process)

        def _killpg(pgid, sig, *, current=process, content=f"map-{index}".encode()):
            groups.append((pgid, sig))
            global_map.write_bytes(content)
            current.returncode = 0

        monkeypatch.setattr(slam_bridge.os, "killpg", _killpg, raising=False)
        assert bridge.save_current_map(timeout=1)

    assert groups == [(701, signal.SIGINT), (702, signal.SIGINT)]


def test_mapping_save_does_not_use_global_pkill(monkeypatch, tmp_path):
    global_map = tmp_path / "GlobalMap.pcd"
    global_map.write_bytes(b"old")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    process = FakeProcess(808)
    _attach_owned_process(monkeypatch, bridge, process)
    monkeypatch.setattr(
        "grain_sampling_workflow.slam_bridge._pkill",
        lambda *_: pytest.fail("mapping save must not use global pkill"),
    )

    def _killpg(_pgid, _sig):
        global_map.write_bytes(b"new")
        process.returncode = 0

    monkeypatch.setattr(slam_bridge.os, "killpg", _killpg, raising=False)
    assert bridge.save_current_map(timeout=1)


def _write_binary_pcd(path: Path, points: list[tuple[float, float, float, float]]) -> None:
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        for point in points:
            stream.write(struct.pack("<ffff", *point))


def test_recovery_merges_compatible_world_frame_chunks(tmp_path):
    _write_binary_pcd(tmp_path / "scans_1.pcd", [(1, 2, 3, 4), (-1, 0, 2, 5)])
    _write_binary_pcd(tmp_path / "scans_2.pcd", [(4, 5, 6, 7)])
    destination = tmp_path / "recovered.pcd"
    points, bounds, digest = merge_scans(tmp_path, destination)
    assert points == 3
    assert bounds == (-1.0, 0.0, 2.0, 4.0, 5.0, 6.0)
    assert digest == map_save.snapshot_map(str(destination)).sha256


def test_ui_save_is_dispatched_to_background_worker():
    source = (
        Path(__file__).resolve().parents[1]
        / "src/grain_sampling_ui/pages/mapping_page.py"
    ).read_text(encoding="utf-8")
    handler = source[source.index("    def _on_save_clicked"):source.index("    def _save_map_worker")]
    assert "threading.Thread(" in handler
    assert ".save_current_map(" not in handler
    assert "if self._save_in_progress:" in handler


def test_reproducible_patch_fixes_empty_remainder_gate():
    root = Path(__file__).resolve().parents[1]
    patch = (root / "deploy/mapping/patches/sfast-lio-save-finalize.patch").read_text()
    prepare = (root / "deploy/mapping/prepare_sfast.sh").read_text()
    assert "-    if (pcl_wait_save->size() > 0 && pcd_save_en)" in patch
    assert "+    if (pcd_save_en)" in patch
    assert "+        *cloud += *pcl_wait_save;" in patch
    assert "GlobalMap.pcd.tmp" in patch
    assert "sfast-lio-save-finalize.patch" in prepare
