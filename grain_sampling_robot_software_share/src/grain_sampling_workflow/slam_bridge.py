"""SLAM bridge — manages S-FAST_LIO mapping/relocalization processes.

Process management
------------------
S-FAST_LIO runs two mutually-exclusive ROS nodes that both publish /Odometry:

  - ``sfastlio_mapping``   (纯建图/里程计): accumulates a map and, on SIGINT,
    merges all scan chunks and writes ``GlobalMap.pcd`` /
    ``GlobalMap_ikdtree.pcd`` — but only when ``pcd_save_en: true``.
  - ``fastlio_mapping_re`` (重定位): loads a saved map and localizes against it.

They must never run at the same time.  A mapping session started by this bridge
is kept in the foreground of a dedicated process group.  The shell sources the
ROS workspaces and then ``exec`` replaces itself with ``sfastlio_mapping``, so
the stored Popen PID is the real mapping PID and process-group leader.  External
mapping processes are detected only as conflicts; they are never adopted or
signalled by this bridge.

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
import signal
import shlex
import subprocess
import time

from grain_sampling_workflow.map_save import (
    MapFileIdentity,
    map_content_changed,
    snapshot_map,
)

logger = logging.getLogger(__name__)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MAPPING_RUNTIME = os.path.join(PROJECT_ROOT, "deploy", "mapping", "runtime.sh")
MAPPING_ENV = os.path.join(PROJECT_ROOT, "deploy", "mapping", "mapping.env")
SFAST_WS = os.environ.get("SFAST_WS", os.path.expanduser("~/fastlio2_ws"))
LIVOX_WS = os.environ.get("LIVOX_WS", os.path.expanduser("~/fastlio_ws"))
PCD_DIR = os.environ.get("SFAST_PCD_DIR", os.path.join(SFAST_WS, "src", "S-FAST_LIO", "PCD"))
MID360_YAML = os.environ.get("SFAST_CONFIG", os.path.join(SFAST_WS, "src", "S-FAST_LIO", "config", "mid360.yaml"))
LIVOX_PACKAGE = os.environ.get("LIVOX_PACKAGE", "livox_ros_driver2")
LIVOX_LAUNCH = os.environ.get("LIVOX_LAUNCH", "msg_MID360.launch")
SFAST_PACKAGE = os.environ.get("SFAST_PACKAGE", "sfast_lio")
SFAST_EXECUTABLE = os.environ.get("SFAST_MAPPING_EXECUTABLE", os.path.join(SFAST_WS, "devel", "lib", SFAST_PACKAGE, "sfastlio_mapping"))
SFAST_RELOCALIZATION_LAUNCH = os.environ.get("SFAST_RELOCALIZATION_LAUNCH", "mapping_mid360_relocalization.launch")
LIVOX_LIDAR_TOPIC = os.environ.get("LIVOX_LIDAR_TOPIC", "/livox/lidar")
LIVOX_IMU_TOPIC = os.environ.get("LIVOX_IMU_TOPIC", "/livox/imu")

#: Process-name patterns used for conflict/liveness discovery.
MAPPING_PATTERN = "sfastlio_mapping"
RELOC_PATTERN = "fastlio_mapping_re"
LIVOX_PATTERN = "livox_ros_driver2_node"


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
        result = subprocess.run(
            ["pkill", f"-{sig}", "-f", pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=15,
        )
        return result.returncode == 0
    except Exception:
        return False


class SlamBridge:
    """Wraps S-FAST_LIO mapping/relocalization process control."""

    def __init__(self) -> None:
        self._connected = False
        self._mapping_process: subprocess.Popen | None = None
        self._mapping_pid: int | None = None
        self._mapping_pgid: int | None = None

    # ── Mapping (pure mapping / odometry) ───────────────────

    def _clear_mapping_session(self) -> None:
        self._mapping_process = None
        self._mapping_pid = None
        self._mapping_pgid = None

    def _owned_mapping_running(self) -> bool:
        """Return true only for the live process group created by this bridge."""
        process = self._mapping_process
        if process is None or self._mapping_pid is None or self._mapping_pgid is None:
            return False
        if process.pid != self._mapping_pid or process.poll() is not None:
            return False
        try:
            return os.getpgid(self._mapping_pid) == self._mapping_pgid
        except ProcessLookupError:
            return False

    def _signal_owned_mapping(self, sig: signal.Signals) -> bool:
        """Signal the exact mapping group, rejecting stale or reused PIDs."""
        if not self._owned_mapping_running():
            logger.warning("Mapping session ownership is missing or stale")
            return False
        assert self._mapping_pgid is not None
        try:
            os.killpg(self._mapping_pgid, sig)
            return True
        except (ProcessLookupError, PermissionError, OSError):
            logger.exception("Failed to signal mapping process group %s", self._mapping_pgid)
            return False

    def _reloc_running(self) -> bool:
        return _pgrep(RELOC_PATTERN)

    def _mapping_shell(self) -> str:
        """Return a shell prelude using the parameterized mapping runtime."""
        return (
            f"source {shlex.quote(MAPPING_RUNTIME)} && "
            f"source {shlex.quote(MAPPING_ENV)} && "
            "mapping_load_config && mapping_source_ros1 && "
            f"mapping_source_workspace {shlex.quote(LIVOX_WS)} && "
            f"mapping_source_workspace {shlex.quote(SFAST_WS)}"
        )

    def _mapping_environment_ready(self) -> bool:
        """Check runtime prerequisites without changing board configuration."""
        checker = os.path.join(PROJECT_ROOT, "scripts", "check_mapping_env.sh")
        try:
            return subprocess.run(["bash", checker, "--runtime"], timeout=45).returncode == 0
        except Exception:
            logger.exception("Mapping environment check failed")
            return False

    def _run_mapping_command(self, command: str, timeout: int = 60) -> bool:
        """Run a command inside the configured ROS1 mapping environment."""
        try:
            result = subprocess.run(
                ["bash", "-c", f"{self._mapping_shell()} && {command}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=timeout,
            )
            return result.returncode == 0
        except Exception:
            logger.exception("Mapping command failed: %s", command)
            return False

    def _ensure_mapping_network(self) -> bool:
        helper = os.path.join(PROJECT_ROOT, "scripts", "ensure_mapping_network.sh")
        try:
            return subprocess.run(["bash", helper], timeout=30).returncode == 0
        except Exception:
            logger.exception("Failed to prepare the MID360 network route")
            return False

    def _ensure_ros_master(self) -> bool:
        if self._run_mapping_command("timeout 3 rosnode list", timeout=8):
            return True
        try:
            subprocess.Popen(
                ["bash", "-c", f"{self._mapping_shell()} && setsid nohup roscore > /tmp/roscore.log 2>&1 < /dev/null &"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
        except Exception:
            logger.exception("Failed to start roscore")
            return False
        for _ in range(20):
            if self._run_mapping_command("timeout 3 rosnode list", timeout=8):
                return True
            time.sleep(0.5)
        logger.error("ROS master did not become ready")
        return False

    def _ensure_livox_driver(self) -> bool:
        if not _pgrep(LIVOX_PATTERN):
            cmd = (
                f"{self._mapping_shell()} && "
                f"setsid nohup roslaunch {shlex.quote(LIVOX_PACKAGE)} "
                f"{shlex.quote(LIVOX_LAUNCH)} > /tmp/livox.log 2>&1 < /dev/null &"
            )
            try:
                subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except Exception:
                logger.exception("Failed to start livox_ros_driver2")
                return False
        return (
            self._run_mapping_command(
                f"mapping_wait_for_topic {shlex.quote(LIVOX_LIDAR_TOPIC)}", timeout=45
            )
            and self._run_mapping_command(
                f"mapping_wait_for_topic {shlex.quote(LIVOX_IMU_TOPIC)}", timeout=45
            )
        )

    def _ensure_pcd_directory(self) -> bool:
        """Create the S-FAST_LIO output directory before periodic PCD writes."""
        try:
            os.makedirs(PCD_DIR, exist_ok=True)
            return os.path.isdir(PCD_DIR) and os.access(PCD_DIR, os.W_OK)
        except OSError:
            logger.exception("Failed to prepare the S-FAST_LIO PCD directory: %s", PCD_DIR)
            return False

    def _launch_mapping_process(self) -> bool:
        """Launch and own the real mapping process in a new session."""
        command = (
            f"{self._mapping_shell()} && "
            f"mapping_wait_for_topic {shlex.quote(LIVOX_LIDAR_TOPIC)} && "
            f"mapping_wait_for_topic {shlex.quote(LIVOX_IMU_TOPIC)} && "
            f"rosparam load {shlex.quote(MID360_YAML)} && "
            f"cd {shlex.quote(SFAST_WS)} && "
            f"exec {shlex.quote(SFAST_EXECUTABLE)}"
        )
        try:
            with open("/tmp/sfast.log", "ab", buffering=0) as log_stream:
                process = subprocess.Popen(
                    ["bash", "-c", command],
                    stdin=subprocess.DEVNULL,
                    stdout=log_stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            pgid = os.getpgid(process.pid)
        except Exception:
            logger.exception("Failed to start owned sfastlio_mapping session")
            return False

        if pgid != process.pid:
            logger.error(
                "Mapping process is not its process-group leader: pid=%s pgid=%s",
                process.pid,
                pgid,
            )
            try:
                process.terminate()
            except OSError:
                pass
            return False

        self._mapping_process = process
        self._mapping_pid = process.pid
        self._mapping_pgid = pgid
        logger.info("Owned mapping session started: pid=%s pgid=%s", process.pid, pgid)
        return True

    def start_mapping(self) -> bool:
        """Start and retain ownership of this bridge's mapping process group."""
        if self._owned_mapping_running():
            logger.info("Owned sfastlio_mapping session already running")
            return True
        if self._mapping_process is not None:
            self._clear_mapping_session()
        if _pgrep(MAPPING_PATTERN):
            logger.error("Unowned sfastlio_mapping process exists; refusing to adopt it")
            return False
        # Mutually exclusive: never run mapping + relocalization together.
        self.stop_relocalization()
        if not self._ensure_mapping_network():
            logger.error("MID360 network route is not ready")
            return False
        if not self._mapping_environment_ready():
            logger.error("Mapping environment is not ready")
            return False
        if not self._ensure_ros_master() or not self._ensure_livox_driver():
            logger.error("ROS master or Livox Driver2 is not ready")
            return False
        if not os.path.isfile(MID360_YAML) or not os.access(MID360_YAML, os.R_OK):
            logger.error("S-FAST_LIO config is missing: %s", MID360_YAML)
            return False
        if not self._ensure_pcd_directory():
            logger.error("S-FAST_LIO PCD directory is not writable: %s", PCD_DIR)
            return False
        if not self._launch_mapping_process():
            return False
        ready = (
            self._run_mapping_command("mapping_wait_for_topic /Odometry", timeout=45)
            and self._run_mapping_command("mapping_wait_for_topic /cloud_registered", timeout=45)
            and self._run_mapping_command("mapping_wait_for_topic /Laser_map", timeout=45)
        )
        if not ready:
            self.stop_mapping()
        return ready

    def save_current_map(
        self,
        before_identity: MapFileIdentity | None = None,
        timeout: float = 60.0,
    ) -> bool:
        """Save the map by SIGINT-ing sfastlio_mapping and wait for GlobalMap.pcd.

        S-FAST_LIO has no ``save_map`` service; its only save path is the SIGINT
        handler.  The save succeeds only after the map content changes and the
        mapping process exits normally.  Callers should invoke this method off
        the GUI thread.
        """
        if not self._owned_mapping_running():
            logger.warning("save_current_map: no live owned mapping session")
            return False
        process = self._mapping_process
        assert process is not None
        global_map = os.path.join(PCD_DIR, "GlobalMap.pcd")
        before = before_identity or snapshot_map(global_map)
        if not self._signal_owned_mapping(signal.SIGINT):
            logger.warning("save_current_map: SIGINT delivery failed")
            return False
        deadline = time.monotonic() + timeout
        changed = False
        while time.monotonic() < deadline:
            changed = map_content_changed(before, snapshot_map(global_map))
            running = process.poll() is None
            if changed and not running:
                logger.info("GlobalMap.pcd saved and mapping process exited")
                self._clear_mapping_session()
                return True
            if not running and not changed:
                logger.warning("save_current_map: process exited without a new GlobalMap.pcd")
                self._clear_mapping_session()
                return False
            time.sleep(0.5)
        logger.warning(
            "save_current_map: timed out (updated=%s, running=%s)",
            changed,
            process.poll() is None,
        )
        return False

    def stop_mapping(self) -> bool:
        """Stop mapping immediately (SIGTERM, no save)."""
        process = self._mapping_process
        if process is None or not self._signal_owned_mapping(signal.SIGTERM):
            return False
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            logger.error("Owned mapping process did not exit after SIGTERM")
            return False
        self._clear_mapping_session()
        return True

    @property
    def mapping_pid(self) -> int | None:
        return self._mapping_pid if self._owned_mapping_running() else None

    @property
    def mapping_pgid(self) -> int | None:
        return self._mapping_pgid if self._owned_mapping_running() else None

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
        if not self._mapping_environment_ready():
            logger.error("Mapping environment is not ready")
            return False
        cmd = (
            f"{self._mapping_shell()} && "
            f"mapping_wait_for_topic {shlex.quote(LIVOX_LIDAR_TOPIC)} && "
            f"mapping_wait_for_topic {shlex.quote(LIVOX_IMU_TOPIC)} && "
            "export QT_QPA_PLATFORM=offscreen && "
            f"roslaunch {shlex.quote(SFAST_PACKAGE)} {shlex.quote(SFAST_RELOCALIZATION_LAUNCH)} rviz:=false"
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
