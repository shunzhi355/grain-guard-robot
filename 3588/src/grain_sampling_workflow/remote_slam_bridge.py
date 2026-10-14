"""3588 UI proxy for SLAM and maps owned by the Lenovo host."""
from __future__ import annotations

import time

from grain_sampling_workflow.robot_bridge import RobotClient


class RemoteSlamBridge:
    is_remote = True

    def __init__(self, client=None):
        self.client = client or RobotClient()
        self.last_error = ""

    def _send(self, operation: str, **fields) -> bool:
        try:
            self.client.request("slam", command={"operation": operation, **fields})
            self.last_error = ""
            return True
        except (OSError, RuntimeError, ValueError) as exc:
            self.last_error = str(exc)
            return False

    def start_mapping(self) -> bool:
        return self._send("START_MAPPING")

    def stop_mapping(self) -> bool:
        return self._send("STOP_MAPPING")

    def save_current_map(self, before_identity=None, name: str = "") -> bool:
        if not name:
            self.last_error = "map name required"
            return False
        try:
            response = self.client.request("slam", command={"operation": "SAVE_MAP", "map_name": name})
            request_id = response["request_id"]
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                status = self.client.request("status")
                result = status.get("slam_response") or {}
                if result.get("request_id") == request_id:
                    if result.get("accepted") is False or result.get("success") is False:
                        self.last_error = result.get("message", "map save failed")
                        return False
                    if result.get("success") is True and result.get("map_id"):
                        return True
                time.sleep(0.2)
            self.last_error = "map save confirmation timed out"
            return False
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
            return False

    def start_relocalization(self, map_id: str) -> bool:
        return self._send("START_LOCALIZATION", map_id=map_id)

    def stop_relocalization(self) -> bool:
        return self._send("STOP_LOCALIZATION")

    def list_maps(self) -> list[dict]:
        try:
            request_id = self.client.request("map_list")["request_id"]
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                response = self.client.request("status").get("map_list_response") or {}
                if response.get("request_id") == request_id:
                    return response.get("maps") or []
                time.sleep(0.1)
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            self.last_error = str(exc)
        return []

    def find_map_by_warehouse(self, warehouse: str) -> str | None:
        for item in self.list_maps():
            if item.get("name") == warehouse or item.get("map_id") == warehouse:
                return item.get("map_id")
        return None

    def find_latest_saved_map(self) -> str | None:
        maps = self.list_maps()
        return max(maps, key=lambda item: item.get("created_at_ms", 0)).get("map_id") if maps else None

    def connected(self) -> bool:
        try:
            return bool(self.client.request("status").get("lenovo_online"))
        except (OSError, RuntimeError, ValueError):
            return False
