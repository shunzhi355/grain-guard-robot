"""Task043 local UI bridge for isolated Noetic mapping sessions."""
from __future__ import annotations

import json
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import time


class LocalSlamBridge:
    is_remote = False
    isolated_map_session = True

    def __init__(self):
        self.last_error = ""
        self._script = Path(__file__).resolve().parents[2] / "scripts" / "task043_mapping_session.sh"
        self.base = Path(os.environ.get("GRAIN_MAPPING_BASE", Path.home() / ".cache/grain-diagnostics/GSR-NAV-UI-SOFTWARE-INTEGRATION-043/maps"))
        self.pointer = self.base / "active_mapping.json"
        self.map_root = self.base / "current"
        self.pcd_dir = self.map_root / "PCD"
        self._sync_root()

    def _sync_root(self):
        try:
            value = json.loads(self.pointer.read_text(encoding="utf-8"))
            self.map_root = Path(value["map_root"]).expanduser()
            self.pcd_dir = self.map_root / "PCD"
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def _run(self, command, *args, timeout=30):
        if not self._script.exists():
            raise RuntimeError("task043 mapping session script is missing")
        result = subprocess.run(["bash", str(self._script), command, *map(str, args)],
                                capture_output=True, text=True, timeout=timeout)
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or f"mapping {command} failed")
        self._sync_root()
        return result.stdout.strip()

    def _call(self, command, *args, timeout=30):
        try:
            self._run(command, *args, timeout=timeout)
            self.last_error = ""
            return True
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.last_error = str(exc)
            return False

    def start_mapping(self, name="new_map"):
        return self._call("start", name, timeout=60)

    def stop_mapping(self):
        return self._call("stop", timeout=25)

    def save_current_map(self, *args, **kwargs):
        return self._call("save", timeout=75)

    def save_map_package(self, name):
        if not self._call("save", timeout=75): return None
        self._sync_root()
        source=self.pcd_dir/"GlobalMap.pcd"
        ikdtree=self.pcd_dir/"GlobalMap_ikdtree.pcd"
        if not source.is_file() or not ikdtree.is_file():
            self.last_error="本轮地图或ikdtree缺失"; return None
        package=self.map_root/"map_package"
        helper=Path(__file__).resolve().parents[2]/"scripts"/"task043_build_preview.py"
        python=os.environ.get("GRAIN_MAP_PREVIEW_PYTHON",os.sys.executable)
        try:
            args=[python,str(helper),str(source),str(ikdtree),str(package),name]
            # Qt text can contain an invisible NUL after clipboard/paste events;
            # subprocess rejects it before the preview helper can report an error.
            args=[str(value).replace("\x00", "") for value in args]
            if any(not value for value in args):
                raise RuntimeError("预览参数为空")
            result=subprocess.run(args, capture_output=True,text=True,timeout=240)
            if result.returncode: raise RuntimeError(result.stderr.strip() or "地图预览生成失败")
            # Show the freshly saved map immediately as a candidate.  It is
            # intentionally not admitted for navigation until its own
            # localization startup reaches READY.
            self.select_saved_map({"preview_pointer": str(package / "saved_map_preview.json")})
            self.last_candidate_preview=package/"saved_map_preview.json"
            self.last_error=""
            return str(source)
        except (OSError,RuntimeError,subprocess.TimeoutExpired) as exc:
            self.last_error=str(exc); return None

    def mapping_status(self):
        try:
            return self._run("status", timeout=8)
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.last_error=str(exc); return "MAPPING=UNAVAILABLE"

    def list_saved_maps(self):
        results=[]
        if not self.base.exists(): return results
        for root in sorted(self.base.iterdir(), key=lambda p:p.stat().st_mtime, reverse=True):
            pcd=root/"PCD"/"GlobalMap.pcd"
            if not pcd.is_file(): continue
            packages=sorted((root/"PCD").glob("*.map"),key=lambda p:p.stat().st_mtime,reverse=True)
            package=packages[0] if packages else None
            manifest=root/"map_package"/"preview"/"map_manifest.json"
            if not manifest.is_file(): continue
            try: identity=json.loads(manifest.read_text(encoding="utf-8"))
            except (OSError,ValueError): identity={}
            preview=root/"map_package"/"preview"/"map_preview.png"
            pointer = root / "map_package" / "saved_map_preview.json"
            results.append({"name":root.name,"map_id":identity.get("map_id",root.name),
                            "map_version":identity.get("map_version"),
                            "frame":identity.get("map_frame", "camera_init"),
                            "path":str(pcd), "preview":str(preview), "map_root":str(root),
                            "preview_pointer":str(pointer), "candidate_preview":str(pointer),
                            "selected": self._selected_preview_pointer() == str(pointer.resolve())})
        return results

    def _active_preview_pointer(self):
        configured = os.environ.get("GRAIN_SAVED_MAP_PREVIEW")
        return Path(configured).expanduser() if configured else self.base / "active_map_preview.json"

    def _selected_preview_pointer(self):
        pointer = self._active_preview_pointer()
        try:
            value = json.loads(pointer.read_text(encoding="utf-8"))
            source = value.get("source_pointer")
            if source:
                return str(Path(source).expanduser().resolve())
            metadata = Path(value["metadata_path"]).expanduser()
            return str(metadata.resolve())
        except (OSError, ValueError, TypeError, KeyError):
            return ""

    def select_saved_map(self, entry):
        """Select a map and, in local mode, bind it to the Golden runtime."""
        source = Path(entry["preview_pointer"]).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"地图预览指针不存在: {source}")
        value = json.loads(source.read_text(encoding="utf-8"))
        metadata = Path(value["metadata_path"]).expanduser()
        if not metadata.is_absolute():
            metadata = (source.parent / metadata).resolve()
        value["metadata_path"] = str(metadata)
        value["candidate"] = True
        value["source_pointer"] = str(source)
        runtime_bound = False
        if os.environ.get("GRAIN_LOCAL_RUNTIME") == "1":
            manifest = json.loads((metadata.parent / "map_manifest.json").read_text(encoding="utf-8"))
            runtime = os.environ.get("TASK043_RUNTIME_CONTAINER", "task043-new-map-relocalization-20261012")
            try:
                state = subprocess.run(["docker", "inspect", "-f", "{{.State.Status}}", runtime],
                                       capture_output=True, text=True, timeout=5)
                if state.returncode == 0 and state.stdout.strip() == "running":
                    current = subprocess.run(["docker", "exec", runtime, "cat",
                                              "/task043/out/startup_map_manifest.json"],
                                             capture_output=True, text=True, timeout=5)
                    if current.returncode == 0:
                        active = json.loads(current.stdout)
                        runtime_bound = all((
                            active.get("map_id") == manifest.get("map_id"),
                            str(active.get("map_version")) == str(manifest.get("map_version")),
                            active.get("map_frame") == manifest.get("map_frame"),
                            active.get("pcd_sha256") == manifest.get("pcd_sha256"),
                        ))
            except (OSError, subprocess.TimeoutExpired, ValueError):
                runtime_bound = False
            if not runtime_bound:
                script = Path(__file__).resolve().parents[2] / "scripts" / "task043_switch_map.sh"
                if not script.is_file():
                    raise RuntimeError("地图运行时切换脚本缺失")
                result = subprocess.run(["bash", str(script), str(source)],
                                        capture_output=True, text=True, timeout=180)
                if result.returncode:
                    raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "Golden 地图切换失败")
                runtime_bound = True
        # A successful local switch means this preview is now the active map;
        # keep candidate=true only for preview-only/non-local selection.
        value["candidate"] = not runtime_bound
        if runtime_bound:
            self._write_runtime_binding(manifest, metadata.parent / "map_manifest.json", runtime)
        target = self._active_preview_pointer()
        target.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=".active_map_preview.", dir=target.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self.last_error = ""
        return str(target)

    def _write_runtime_binding(self, manifest, package_manifest, runtime):
        """Record which saved map the Golden runtime is actually running.

        The UI compares the preview against this record.  Comparing the preview
        manifest hash with the Golden startup manifest hash would never match
        because they are different artifacts.
        """
        digest = hashlib.sha256()
        with Path(package_manifest).open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        binding = {
            "map_id": manifest["map_id"],
            "map_version": manifest["map_version"],
            "frame": manifest["map_frame"],
            "pcd_sha256": manifest.get("pcd_sha256", ""),
            "package_manifest_sha256": digest.hexdigest(),
            "runtime_container": runtime,
            "updated_at": time.time(),
        }
        target = self._active_preview_pointer().parent / "active_runtime_map.json"
        target.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(prefix=".active_runtime_map.", dir=target.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(binding, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def start_relocalization(self, path):
        self.last_error=("新地图已独立保存。请先确认地图启动位置与航向，再通过定位启动门控加载；旧地图未被替换。")
        return False

    def confirm_map_origin(self):
        """Write an explicit map-origin confirmation for the task043 guard."""
        status=self.read_localization_startup_status()
        if status.get("state") in ("VALIDATING_STATIC", "READY"):
            self.last_error=""
            return True  # idempotent: never interrupt an ongoing validation
        if status.get("state") == "UNAVAILABLE":
            self.last_error=status.get("reason", "定位启动状态不可用")
            return False
        script=Path(__file__).resolve().parents[2]/"scripts"/"task043_initialize_localization.sh"
        if not script.is_file():
            self.last_error="定位初始化脚本缺失"
            return False
        try:
            result=subprocess.run(["bash",str(script),"--map-origin-confirmed"],
                capture_output=True,text=True,timeout=20)
            if result.returncode:
                self.last_error=result.stderr.strip() or result.stdout.strip() or "定位启动确认失败"
                return False
            self.last_error=""
            return True
        except (OSError,subprocess.TimeoutExpired) as exc:
            self.last_error=str(exc)
            return False

    def read_localization_startup_status(self):
        """Read the task-owned startup guard state without changing runtime state."""
        runtime=os.environ.get("TASK043_RUNTIME_CONTAINER","task043-new-map-relocalization-20261012")
        try:
            result=subprocess.run(["docker","exec",runtime,"cat","/task043/out/localization_startup_status.json"],
                capture_output=True,text=True,timeout=5)
            if result.returncode:
                return {"state":"UNAVAILABLE","reason":result.stderr.strip() or "status unavailable"}
            return json.loads(result.stdout)
        except (OSError,subprocess.TimeoutExpired,ValueError) as exc:
            return {"state":"UNAVAILABLE","reason":str(exc)}

    def stop_relocalization(self):
        return True

    def find_map_by_warehouse(self,name):
        for item in self.list_saved_maps():
            if item["name"]==name or item["map_id"]==name: return item["path"]
        return None

    def find_latest_saved_map(self):
        maps=self.list_saved_maps()
        return maps[0]["path"] if maps else None

    def connected(self):
        try:
            result=subprocess.run(["docker","inspect","-f","{{.State.Status}}",
                os.environ.get("TASK043_RUNTIME_CONTAINER","task043-new-map-relocalization-20261012")],
                capture_output=True,text=True,timeout=5)
            return result.returncode==0 and result.stdout.strip()=="running"
        except (OSError,subprocess.TimeoutExpired): return False
