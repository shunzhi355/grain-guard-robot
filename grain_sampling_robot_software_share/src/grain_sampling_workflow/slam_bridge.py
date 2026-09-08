"""SLAM bridge — manages S-FAST_LIO mapping/relocalization processes.

Process management
------------------
S-FAST_LIO runs two mutually-exclusive ROS nodes that both publish /Odometry:

  - ``sfastlio_mapping``   (纯建图/里程计): accumulates a map and, on SIGINT,
    merges all scan chunks and writes ``GlobalMap.pcd`` /
    ``GlobalMap_ikdtree.pcd`` — but only when ``pcd_save_en: true``.
  - ``fastlio_mapping_re`` (重定位): loads a saved map and localizes against it.

They must never run at the same time.  Because ``sfastlio_mapping`` may be
launched either by ``start_all_full.sh`` (nohup, detached) or by this bridge,
liveness is detected via ``pgrep`` and shutdown via ``pkill`` — never via a
subprocess handle, which cannot see externally-launched processes.

Signals (S-FAST_LIO's ``SigHandle`` only handles SIGINT):
  - ``SIGINT``  -> sets flg_exit, main() writes GlobalMap.pcd, then exits
                   (save + stop).
  - ``SIGTERM`` -> immediate stop, no save.

UI state machine:
  idle -> (select task) -> relocalizing -> (start mapping) -> mapping
       -> (save) -> GlobalMap.pcd written -> saved/idle
"""

from __future__ import annotations

import logging
import os
import subprocess
import time

logger = logging.getLogger(__name__)

PCD_DIR = os.path.expanduser("~/fastlio2_ws/src/S-FAST_LIO/PCD")
MID360_YAML = os.path.expanduser("~/fastlio2_ws/src/S-FAST_LIO/config/mid360.yaml")

#: Process-name patterns matched by ``pgrep``/``pkill -f``.
MAPPING_PATTERN = "sfastlio_mapping"
RELOC_PATTERN = "fastlio_mapping_re"


def _pgrep(pattern: str) -> bool:
    """Return True if a process matching ``pattern`` is currently running."""
    try:
        r = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=5,
        )
        return r.returncode == 0
    except Exception:
        return False


def _pkill(pattern: str, sig: str) -> bool:
    """Send signal ``sig`` (e.g. 'INT', 'TERM') to processes matching pattern."""
    try:
        subprocess.run(
            ["pkill", f"-{sig}", "-f", pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=15,
        )
        return True
    except Exception:
        return False


class SlamBridge:
    """Wraps S-FAST_LIO mapping/relocalization process control."""

    def __init__(self) -> None:
        self._connected = False

    # ── Mapping (pure mapping / odometry) ───────────────────

    def _mapping_running(self) -> bool:
        return _pgrep(MAPPING_PATTERN)

    def _reloc_running(self) -> bool:
        return _pgrep(RELOC_PATTERN)

    def _ensure_pcd_save_enabled(self) -> None:
        """Force ``pcd_save_en: true`` so SIGINT actually writes GlobalMap.pcd.

        S-FAST_LIO's save path is gated by ``pcd_save_en``; when false the map
        is silently discarded on exit.  Idempotent (sed no-ops if already true).
        """
        try:
            subprocess.run(
                ["sed", "-i", "s/pcd_save_en: false/pcd_save_en: true/", MID360_YAML],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except Exception:
            logger.exception("Failed to ensure pcd_save_en=true")

    def start_mapping(self) -> bool:
        """Ensure sfastlio_mapping is running (idempotent across processes)."""
        if self._mapping_running():
            logger.info("sfastlio_mapping already running")
            return True
        # Mutually exclusive: never run mapping + relocalization together.
        self.stop_relocalization()
        self._ensure_pcd_save_enabled()
        cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source ~/fastlio_ws/devel/setup.bash && "
            "source ~/fastlio2_ws/devel/setup.bash && "
            "rosparam load ~/fastlio2_ws/src/S-FAST_LIO/config/mid360.yaml && "
            "cd ~/fastlio2_ws && "
            "setsid nohup ~/fastlio2_ws/devel/lib/sfast_lio/sfastlio_mapping "
            "> /tmp/sfast.log 2>&1 < /dev/null &"
        )
        try:
            subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return True
        except Exception:
            logger.exception("Failed to start sfastlio_mapping")
            return False

    def save_current_map(self) -> bool:
        """Save the map by SIGINT-ing sfastlio_mapping and wait for GlobalMap.pcd.

        S-FAST_LIO has no ``save_map`` service; its only save path is the SIGINT
        handler.  Blocks until ``GlobalMap.pcd`` is (re)written or the process
        exits, whichever comes first.
        """
        if not self._mapping_running():
            logger.warning("save_current_map: sfastlio_mapping not running")
            return False
        global_map = os.path.join(PCD_DIR, "GlobalMap.pcd")
        before = os.path.getmtime(global_map) if os.path.exists(global_map) else None
        _pkill(MAPPING_PATTERN, "INT")
        deadline = time.time() + 60
        while time.time() < deadline:
            if os.path.exists(global_map):
                mtime = os.path.getmtime(global_map)
                if before is None or mtime > before:
                    logger.info("GlobalMap.pcd saved")
                    return True
            if not self._mapping_running():
                break
            time.sleep(0.5)
        logger.warning("save_current_map: GlobalMap.pcd not updated")
        return os.path.exists(global_map)

    def stop_mapping(self) -> bool:
        """Stop mapping immediately (SIGTERM, no save)."""
        return _pkill(MAPPING_PATTERN, "TERM")

    # ── Relocalization ─────────────────────────────────────

    def start_relocalization(self, pcd_path: str) -> bool:
        """Stop mapping, point ``map_file_path`` at ``pcd_path``, launch reloc."""
        if self._reloc_running():
            logger.info("fastlio_mapping_re already running")
            return True
        # Mutually exclusive with mapping.
        self.stop_mapping()
        try:
            subprocess.run(
                ["sed", "-i", f"s|map_file_path:.*|map_file_path: {pcd_path}|", MID360_YAML],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except Exception:
            logger.exception("Failed to update map_file_path")
            return False
        cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source ~/fastlio_ws/devel/setup.bash && "
            "source ~/fastlio2_ws/devel/setup.bash && "
            "export QT_QPA_PLATFORM=offscreen && "
            "roslaunch sfast_lio mapping_mid360_relocalization.launch rviz:=false"
        )
        try:
            subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            return True
        except Exception:
            logger.exception("Failed to start relocalization")
            return False

    def stop_relocalization(self) -> bool:
        """Stop the relocalization node (SIGTERM)."""
        return _pkill(RELOC_PATTERN, "TERM")

    # ── Map file discovery ──────────────────────────────────

    def find_map_by_warehouse(self, warehouse: str) -> str | None:
        """Scan PCD_DIR for the latest map matching ``{warehouse}_*.pcd``."""
        if not os.path.isdir(PCD_DIR):
            logger.warning("PCD_DIR %s does not exist", PCD_DIR)
            return None
        try:
            matches = [
                f for f in os.listdir(PCD_DIR)
                if f.startswith(f"{warehouse}_") and f.endswith(".pcd")
            ]
        except OSError:
            return None
        if not matches:
            logger.info("No map found for warehouse: %s", warehouse)
            return None
        matches.sort(key=lambda f: os.path.getmtime(os.path.join(PCD_DIR, f)), reverse=True)
        return os.path.join(PCD_DIR, matches[0])

    def find_latest_saved_map(self) -> str | None:
        """Scan PCD_DIR for the most recent saved map (exclude scans_*.pcd)."""
        if not os.path.isdir(PCD_DIR):
            logger.warning("PCD_DIR %s does not exist", PCD_DIR)
            return None
        try:
            files = [
                f for f in os.listdir(PCD_DIR)
                if f.endswith(".pcd") and not f.startswith("scans_")
            ]
        except OSError:
            return None
        if not files:
            logger.info("No saved maps found in %s", PCD_DIR)
            return None
        files.sort(key=lambda f: os.path.getmtime(os.path.join(PCD_DIR, f)), reverse=True)
        return os.path.join(PCD_DIR, files[0])

    @property
    def connected(self) -> bool:
        return self._connected
