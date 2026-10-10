"""In-process local SLAM selection; never contacts the RK3588 map proxy."""
from __future__ import annotations

import os
from pathlib import Path


class LocalSlamBridge:
    is_remote = False

    def __init__(self):
        self.last_error = ""
        self._bridge = None

    def _get(self):
        if not Path("/opt/ros/noetic/setup.bash").exists():
            raise RuntimeError("本机 ROS1 在容器内：建图 UI 须在配置好的 Noetic 容器环境启动；不会接管当前雷达任务")
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
