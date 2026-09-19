from __future__ import annotations

import os
from pathlib import Path
import struct

import pytest

from grain_sampling_workflow import map_save
from grain_sampling_workflow.slam_bridge import SlamBridge
from scripts.recover_sfast_scans import merge_scans


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
    states = iter([True, False])
    bridge = SlamBridge()
    monkeypatch.setattr(bridge, "_mapping_running", lambda: next(states))
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge._pkill", lambda *_: True)
    assert not bridge.save_current_map(timeout=1)


def test_bridge_accepts_new_map_after_clean_exit(tmp_path, monkeypatch):
    global_map = tmp_path / "GlobalMap.pcd"
    global_map.write_bytes(b"old-map")
    before = map_save.snapshot_map(str(global_map))
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    states = iter([True, False])
    monkeypatch.setattr(bridge, "_mapping_running", lambda: next(states))

    def _signal(*_args):
        global_map.write_bytes(b"new-map")
        return True

    monkeypatch.setattr("grain_sampling_workflow.slam_bridge._pkill", _signal)
    assert bridge.save_current_map(before_identity=before, timeout=1)


def test_bridge_timeout_never_reports_existing_old_map(tmp_path, monkeypatch):
    (tmp_path / "GlobalMap.pcd").write_bytes(b"old-map")
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge.PCD_DIR", str(tmp_path))
    bridge = SlamBridge()
    monkeypatch.setattr(bridge, "_mapping_running", lambda: True)
    monkeypatch.setattr("grain_sampling_workflow.slam_bridge._pkill", lambda *_: True)
    assert not bridge.save_current_map(timeout=0)


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
