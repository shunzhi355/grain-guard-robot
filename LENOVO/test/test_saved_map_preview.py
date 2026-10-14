import hashlib
import json
from pathlib import Path
import pytest
from grain_sampling_workflow.saved_map_preview import load_saved_preview


def _hash(data):
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def artifacts(tmp_path):
    preview = tmp_path / "preview"
    preview.mkdir()
    (tmp_path / "PCD").mkdir()
    (tmp_path / "PCD/GlobalMap.pcd").write_bytes(b"test point cloud")
    (preview / "map_preview.png").write_bytes(b"raster")
    manifest = {"map_id": "map-a", "map_frame": "camera_init", "map_version": "r1",
                "pcd_sha256": _hash(b"test point cloud")}
    raw = json.dumps(manifest).encode()
    (preview / "map_manifest.json").write_bytes(raw)
    metadata = {"manifest_sha256": _hash(raw), "pcd_sha256": _hash(b"test point cloud"),
                "preview_sha256": _hash(b"raster"), "world_bounds": [-2, 3, -1, 5],
                "z_slice": [0.4, 2.0], "width_px": 100, "height_px": 80}
    (preview / "preview_metadata.json").write_text(json.dumps(metadata))
    pointer = tmp_path / "saved_map_preview.json"
    pointer.write_text(json.dumps({"metadata_path": "preview/preview_metadata.json"}))
    return pointer


def test_current_map_preview_returns_original_world_coordinates(artifacts):
    data = load_saved_preview(artifacts, map_id="map-a", frame="camera_init")
    assert data["world_bounds"] == [-2, 3, -1, 5]
    assert data["map_version"] == "r1"
    assert data["z_slice"] == [0.4, 2.0]


@pytest.mark.parametrize("map_id,frame", [("old-map", "camera_init"), ("map-a", "body")])
def test_mismatched_live_map_is_never_displayed(artifacts, map_id, frame):
    with pytest.raises(ValueError, match="identity/frame"):
        load_saved_preview(artifacts, map_id=map_id, frame=frame)


@pytest.mark.parametrize("target", ["preview/map_preview.png", "preview/map_manifest.json", "PCD/GlobalMap.pcd"])
def test_changed_map_or_preview_fails_hash_check(artifacts, target):
    path = artifacts.parent / target
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="SHA256"):
        load_saved_preview(artifacts, map_id="map-a", frame="camera_init")


def test_navigation_default_uses_active_pointer_without_overwriting_candidate(tmp_path):
    from grain_sampling_workflow.saved_map_preview import default_preview_pointer
    candidate=tmp_path/"saved_map_preview.json";candidate.write_text('{"candidate":true}')
    assert default_preview_pointer(tmp_path)==candidate
    active=tmp_path/"active_map_preview.json";active.write_text('{"candidate":false}')
    assert default_preview_pointer(tmp_path)==active
    assert candidate.read_text()=='{"candidate":true}'


def test_select_saved_map_updates_ui_watched_pointer(tmp_path, monkeypatch):
    from grain_sampling_local.slam import LocalSlamBridge
    package=tmp_path/'map_package'; (package/'preview').mkdir(parents=True)
    source=package/'saved_map_preview.json'
    metadata=package/'preview'/'preview_metadata.json'
    metadata.write_text('{}')
    (package/'preview'/'map_manifest.json').write_text(json.dumps({
        'map_id': 'map-a', 'map_version': 1, 'map_frame': 'camera_init',
        'pcd_sha256': 'a' * 64
    }))
    source.write_text(json.dumps({'metadata_path':str(metadata),'candidate':True}))
    bridge=LocalSlamBridge.__new__(LocalSlamBridge)
    bridge.base=tmp_path/'maps'
    watched=tmp_path/'state'/'active_map_preview.json'
    monkeypatch.setenv('GRAIN_SAVED_MAP_PREVIEW',str(watched))
    # Preview-only selection: a non-local runtime must not touch Golden.
    monkeypatch.delenv('GRAIN_LOCAL_RUNTIME',raising=False)
    bridge.last_error='old'
    bridge.select_saved_map({'preview_pointer':str(source)})
    selected=json.loads(watched.read_text())
    assert selected['candidate'] is True
    assert selected['source_pointer']==str(source.resolve())
    assert selected['metadata_path']==str(metadata.resolve())
    assert bridge.last_error==''


def test_local_selection_marks_preview_active_after_golden_switch(tmp_path, monkeypatch):
    from grain_sampling_local.slam import LocalSlamBridge
    package=tmp_path/'map_package'; (package/'preview').mkdir(parents=True)
    source=package/'saved_map_preview.json'
    metadata=package/'preview'/'preview_metadata.json'
    metadata.write_text('{}')
    (package/'preview'/'map_manifest.json').write_text(json.dumps({
        'map_id': 'map-a', 'map_version': 1, 'map_frame': 'camera_init',
        'pcd_sha256': 'a' * 64
    }))
    source.write_text(json.dumps({'metadata_path':str(metadata),'candidate':True}))
    bridge=LocalSlamBridge.__new__(LocalSlamBridge)
    bridge.base=tmp_path/'maps'
    watched=tmp_path/'state'/'active_map_preview.json'
    monkeypatch.setenv('GRAIN_SAVED_MAP_PREVIEW',str(watched))
    monkeypatch.setenv('GRAIN_LOCAL_RUNTIME','1')
    monkeypatch.setattr('grain_sampling_local.slam.subprocess.run', lambda *args, **kwargs: type('R', (), {'returncode':0, 'stderr':'', 'stdout':''})())
    monkeypatch.setattr('grain_sampling_local.slam.Path.is_file', lambda self: True)
    bridge.select_saved_map({'preview_pointer':str(source)})
    selected=json.loads(watched.read_text())
    assert selected['candidate'] is False



def test_local_selection_skips_recreating_already_loaded_golden_map(tmp_path, monkeypatch):
    from grain_sampling_local.slam import LocalSlamBridge
    package=tmp_path/'map_package'; (package/'preview').mkdir(parents=True)
    source=package/'saved_map_preview.json'
    metadata=package/'preview'/'preview_metadata.json'
    metadata.write_text('{}')
    manifest={"map_id":"map-a","map_version":1,"map_frame":"camera_init",
              "pcd_sha256":"a"*64}
    (package/'preview'/'map_manifest.json').write_text(json.dumps(manifest))
    source.write_text(json.dumps({'metadata_path':str(metadata),'candidate':True}))
    bridge=LocalSlamBridge.__new__(LocalSlamBridge)
    bridge.base=tmp_path/'maps'
    watched=tmp_path/'state'/'active_map_preview.json'
    monkeypatch.setenv('GRAIN_SAVED_MAP_PREVIEW',str(watched))
    monkeypatch.setenv('GRAIN_LOCAL_RUNTIME','1')
    calls=[]
    def run(args, **kwargs):
        calls.append(args)
        if args[1] == 'inspect':
            return type('R', (), {'returncode':0,'stdout':'running','stderr':''})()
        if args[1] == 'exec':
            return type('R', (), {'returncode':0,'stdout':json.dumps(manifest),'stderr':''})()
        raise AssertionError(f"unexpected subprocess: {args}")
    monkeypatch.setattr('grain_sampling_local.slam.subprocess.run',run)
    bridge.select_saved_map({'preview_pointer':str(source)})
    assert len(calls)==2
    assert json.loads(watched.read_text())['candidate'] is False

