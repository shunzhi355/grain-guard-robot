from __future__ import annotations

import json

from grain_sampling_workflow.map_manifest import (
    build_manifest,
    load_active_manifest,
    load_manifest,
    write_manifest,
)


def test_manifest_is_versioned_and_active_pointer_is_atomic(tmp_path):
    pcd = tmp_path / "warehouse_20261013.pcd"
    pcd.write_bytes(b"pcd-data-v1\n")

    manifest = build_manifest(pcd, "warehouse")
    manifest_path = write_manifest(manifest)

    assert manifest_path.exists()
    assert load_manifest(manifest_path) == manifest
    assert load_active_manifest(tmp_path) == manifest
    active = json.loads((tmp_path / "active_map.json").read_text(encoding="utf-8"))
    assert active["map_id"] == manifest.map_id
    assert active["frame"] == "camera_init"


def test_manifest_rejects_changed_pcd(tmp_path):
    pcd = tmp_path / "warehouse.pcd"
    pcd.write_bytes(b"before")
    manifest = build_manifest(pcd, "warehouse")
    write_manifest(manifest)
    pcd.write_bytes(b"after")
    assert load_active_manifest(tmp_path) is None
