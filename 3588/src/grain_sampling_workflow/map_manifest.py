"""Versioned map identity and handoff helpers.

The mapping UI writes an immutable manifest beside each PCD and atomically
updates one active pointer.  Runtime consumers can therefore agree on the
same map without copying a stale ``map_id`` through several config files.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile

from grain_sampling_workflow.map_save import snapshot_map


DEFAULT_FRAME = "camera_init"


@dataclass(frozen=True)
class MapManifest:
    map_id: str
    map_version: int
    frame: str
    pcd_path: str
    sha256: str
    created_at: str
    source: str = "ui_mapping"

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def active_manifest_path(pcd_dir: str | os.PathLike[str] | None = None) -> Path:
    if pcd_dir is None:
        explicit = os.environ.get("GRAIN_ACTIVE_MAP_MANIFEST")
        if explicit:
            return Path(explicit)
    root = Path(pcd_dir or os.environ.get("SFAST_PCD_DIR", "~/fastlio2_ws/src/S-FAST_LIO/PCD")).expanduser()
    return root / "active_map.json"


def _next_version(directory: Path) -> int:
    versions = []
    for path in directory.glob("*.manifest.json"):
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            versions.append(int(value.get("map_version", 0)))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            continue
    return max(versions, default=0) + 1


def build_manifest(
    pcd_path: str | os.PathLike[str],
    map_name: str,
    *,
    frame: str = DEFAULT_FRAME,
    version: int | None = None,
    source: str = "ui_mapping",
) -> MapManifest:
    path = Path(pcd_path).expanduser().resolve()
    identity = snapshot_map(str(path))
    if not identity.exists or not identity.sha256:
        raise FileNotFoundError(f"map PCD is missing or unreadable: {path}")
    clean_name = "_".join(part for part in map_name.strip().split() if part)
    if not clean_name:
        raise ValueError("map name must not be empty")
    safe_name = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in clean_name)
    return MapManifest(
        map_id=f"{safe_name}_{identity.sha256[:12]}",
        map_version=int(version if version is not None else _next_version(path.parent)),
        frame=frame,
        pcd_path=str(path),
        sha256=identity.sha256,
        created_at=datetime.now(timezone.utc).isoformat(),
        source=source,
    )


def validate_manifest(manifest: MapManifest) -> MapManifest:
    if not manifest.map_id or manifest.map_version < 1 or not manifest.frame:
        raise ValueError("invalid map manifest identity")
    identity = snapshot_map(manifest.pcd_path)
    if not identity.exists or identity.sha256 != manifest.sha256:
        raise ValueError(f"map manifest SHA256 mismatch: {manifest.pcd_path}")
    return manifest


def write_manifest(manifest: MapManifest, *, active: bool = True) -> Path:
    path = Path(manifest.pcd_path).with_suffix(Path(manifest.pcd_path).suffix + ".manifest.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(manifest.to_dict(), ensure_ascii=False, indent=2) + "\n"
    _atomic_write(path, payload)
    if active:
        _atomic_write(active_manifest_path(path.parent), payload)
    return path


def load_manifest(path: str | os.PathLike[str]) -> MapManifest:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("map manifest must be a JSON object")
    manifest = MapManifest(
        map_id=str(value["map_id"]),
        map_version=int(value["map_version"]),
        frame=str(value["frame"]),
        pcd_path=str(value["pcd_path"]),
        sha256=str(value["sha256"]),
        created_at=str(value["created_at"]),
        source=str(value.get("source", "unknown")),
    )
    return validate_manifest(manifest)


def load_active_manifest(path: str | os.PathLike[str] | None = None) -> MapManifest | None:
    candidate = Path(path).expanduser() if path else active_manifest_path()
    if candidate.is_dir():
        candidate = candidate / "active_map.json"
    try:
        return load_manifest(candidate)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return None


def _atomic_write(path: Path, payload: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
