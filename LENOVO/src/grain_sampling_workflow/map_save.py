"""File-identity helpers for reliable S-FAST_LIO map saves."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os
import shutil
import tempfile


@dataclass(frozen=True)
class MapFileIdentity:
    exists: bool
    mtime_ns: int | None = None
    size: int | None = None
    sha256: str | None = None


class StaleMapError(RuntimeError):
    """Raised when a save did not produce new GlobalMap content."""


def snapshot_map(path: str) -> MapFileIdentity:
    """Return a content identity for ``path`` without trusting timestamps alone."""
    try:
        stat = os.stat(path)
    except FileNotFoundError:
        return MapFileIdentity(False)

    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return MapFileIdentity(True, stat.st_mtime_ns, stat.st_size, digest.hexdigest())


def map_content_changed(before: MapFileIdentity, after: MapFileIdentity) -> bool:
    """Require a present map whose content differs from the pre-save snapshot."""
    if not after.exists or not after.sha256:
        return False
    return not before.exists or before.sha256 != after.sha256


def copy_fresh_map(
    source: str,
    destination: str,
    before: MapFileIdentity,
) -> MapFileIdentity:
    """Atomically copy a newly generated map, rejecting stale source content."""
    after = snapshot_map(source)
    if not map_content_changed(before, after):
        raise StaleMapError("GlobalMap.pcd content did not change")

    destination_dir = os.path.dirname(destination) or "."
    os.makedirs(destination_dir, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".map-save-", suffix=".pcd", dir=destination_dir)
    os.close(fd)
    try:
        shutil.copy2(source, temporary)
        copied = snapshot_map(temporary)
        if copied.sha256 != after.sha256 or copied.size != after.size:
            raise OSError("copied map identity does not match GlobalMap.pcd")
        os.replace(temporary, destination)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return snapshot_map(destination)
