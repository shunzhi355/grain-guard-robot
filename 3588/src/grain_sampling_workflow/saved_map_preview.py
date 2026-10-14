"""Read and verify an existing saved-map raster; never mutate map artifacts."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def default_preview_pointer(directory):
    directory = Path(directory)
    active = directory / "active_map_preview.json"
    return active if active.is_file() else directory / "saved_map_preview.json"


def load_runtime_map_binding(pointer):
    """Return the runtime map binding recorded beside the active preview.

    The binding is written when a saved map is actually loaded into the Golden
    runtime.  It pins the package hashes so the UI never compares two unrelated
    manifests (the saved-map package manifest and the Golden startup manifest).
    """
    pointer = Path(pointer)
    candidate = pointer.parent / "active_runtime_map.json"
    try:
        value = json.loads(candidate.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def preview_paths(pointer):
    pointer = Path(pointer)
    value = json.loads(pointer.read_text(encoding="utf-8"))
    metadata = Path(value["metadata_path"]).expanduser()
    if not metadata.is_absolute():
        metadata = pointer.parent / metadata
    return (pointer, metadata, metadata.parent / "map_manifest.json",
            metadata.parent / "map_preview.png", metadata.parent.parent / "PCD/GlobalMap.pcd")


def load_saved_preview(pointer, *, map_id=None, frame=None, allow_candidate=False):
    _, metadata_path, manifest_path, image_path, pcd_path = preview_paths(pointer)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not allow_candidate and (not map_id or manifest["map_id"] != map_id or manifest["map_frame"] != frame):
        raise ValueError("saved preview map identity/frame differs from LIVE")
    for path, key in ((manifest_path, "manifest_sha256"), (image_path, "preview_sha256"),
                      (pcd_path, "pcd_sha256")):
        if _sha256(path) != metadata[key]:
            raise ValueError(f"saved preview SHA256 mismatch: {path.name}")
    if manifest["pcd_sha256"] != metadata["pcd_sha256"]:
        raise ValueError("preview and manifest refer to different PCD maps")
    bounds = metadata["world_bounds"]
    if (len(bounds) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in bounds)
            or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]):
        raise ValueError("invalid preview world bounds")
    return {"image_path": str(image_path), "world_bounds": list(bounds),
            "map_id": manifest["map_id"], "map_version": manifest["map_version"],
            "frame": manifest["map_frame"], "manifest_sha256": metadata["manifest_sha256"], "z_slice": metadata["z_slice"],
            "pcd_sha256": manifest["pcd_sha256"],
            "width_px": metadata["width_px"], "height_px": metadata["height_px"]}
