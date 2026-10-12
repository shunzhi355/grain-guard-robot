"""Local SLAM selection with one host/container runtime entry point."""
from __future__ import annotations

import os
from pathlib import Path
import subprocess


class LocalSlamBridge:
    is_remote = False

    def __init__(self):
        self.last_error = ""
        self._bridge = None

    def _get(self):
        if not Path("/opt/ros/noetic/setup.bash").exists():
            checker = Path(__file__).resolve().parents[2] / "scripts" / "start_noetic_slam.sh"
            container = os.environ.get("GRAIN_NOETIC_CONTAINER", "")
            if not container or not checker.exists():
                raise RuntimeError("Noetic runtime unavailable; set GRAIN_NOETIC_CONTAINER to the task-owned container")
            result = subprocess.run(["bash", str(checker), "--check", "--container", container],
                                    capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "Noetic container is not ready")
            raise RuntimeError("UI must run inside the verified Noetic container runtime; use start_noetic_slam.sh")
        if self._bridge is None:
            from grain_sampling_workflow.slam_bridge import SlamBridge
            self._bridge = SlamBridge()
        return self._bridge

    def _call(self, name, *args, **kwargs):
        try:
            result = getattr(self._get(), name)(*args, **kwargs)
            self.last_error = "" if result else f"local SLAM {name} failed; check mapping environment"
            return result
        except (RuntimeError, OSError) as exc:
            self.last_error = str(exc)
            return False

    def start_mapping(self):
        return self._call("start_mapping")

    def stop_mapping(self):
        return self._call("stop_mapping")

    def save_current_map(self, *args, **kwargs):
        return self._call("save_current_map", *args, **kwargs)

    def start_relocalization(self, path):
        return self._call("start_relocalization", path)

    def stop_relocalization(self):
        return self._call("stop_relocalization")

    def find_map_by_warehouse(self, name):
        return self._call("find_map_by_warehouse", name) or None

    def find_latest_saved_map(self):
        return self._call("find_latest_saved_map") or None

    def connected(self):
        return bool(self._call("connected"))
