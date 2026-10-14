#!/usr/bin/env bash
set -Eeuo pipefail
POINTER="${1:?usage: task043_switch_map.sh SAVED_MAP_PREVIEW_POINTER}"
RUNTIME="${TASK043_RUNTIME_CONTAINER:-task043-new-map-relocalization-20261012}"
ADAPTER="${TASK043_ADAPTER_CONTAINER:-task043-live-nav-adapter-current}"
STATE="${TASK043_SOCKET_DIR:-$HOME/.local/state/grain-lenovo}"
BASE="${GRAIN_MAPPING_BASE:-$HOME/.cache/grain-diagnostics/GSR-NAV-UI-SOFTWARE-INTEGRATION-043/maps}"
[[ "$RUNTIME" =~ ^task043-new-map-relocalization-[A-Za-z0-9._-]+$ ]] || { echo 'unknown runtime owner' >&2; exit 1; }
[[ -f "$POINTER" ]] || { echo "preview pointer missing: $POINTER" >&2; exit 1; }
python3 - "$POINTER" "$RUNTIME" "$ADAPTER" "$STATE" "$BASE" <<'PY'
import hashlib, json, os, pathlib, shutil, subprocess, sys, tempfile, time, signal
pointer = pathlib.Path(sys.argv[1])
runtime = sys.argv[2]
adapter = sys.argv[3]
state_dir = pathlib.Path(sys.argv[4])
base = pathlib.Path(sys.argv[5])

def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()

def die(msg): raise SystemExit(msg)
value=json.loads(pointer.read_text(encoding='utf-8'))
metadata=pathlib.Path(value['metadata_path']).expanduser().resolve()
if not metadata.is_file(): die('preview metadata missing')
try: metadata.relative_to(base.resolve())
except ValueError: die('selected map is outside the task043 map store')
preview=metadata.parent
manifest=json.loads((preview/'map_manifest.json').read_text(encoding='utf-8'))
meta=json.loads(metadata.read_text(encoding='utf-8'))
pcd=(preview.parent/'PCD'/'GlobalMap.pcd').resolve()
ikd=(preview.parent/'PCD'/'GlobalMap_ikdtree.pcd').resolve()
if sha(preview/'map_manifest.json') != meta['manifest_sha256']: die('map manifest SHA256 mismatch')
if sha(preview/'map_preview.png') != meta['preview_sha256']: die('map preview SHA256 mismatch')
if sha(pcd) != meta['pcd_sha256'] or manifest.get('pcd_sha256') != meta['pcd_sha256']: die('map PCD SHA256 mismatch')
if sha(ikd) != manifest.get('ikdtree_sha256'): die('map ikdtree SHA256 mismatch')
if manifest.get('map_frame') != 'camera_init': die('map frame must be camera_init')
# Never interrupt an active goal or physical mechanism task.
status_path=state_dir/'control.sock'
if status_path.exists():
 import socket
 with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
  s.settimeout(3); s.connect(str(status_path)); s.sendall(b'{"action":"status"}\n')
  status=json.loads(s.makefile('rb').readline())
 if (status.get('chassis') or {}).get('motion_armed') or (status.get('navigation') or {}).get('result') in ('ACTIVE','RUNNING','PENDING'):
  die('cannot switch map while motion or navigation goal is active')
info=json.loads(subprocess.check_output(['docker','inspect',runtime],text=True))[0]
if info['State']['Status'] != 'running': die('Golden runtime must be running')
if info['Config']['Image'] != 'grain-nav:noetic-x86': die('Golden runtime image mismatch')
# Stop only task-owned adapter/runtime. The supervisor will not restore an adapter
# until the new startup guard reaches READY.
if subprocess.run(['docker','inspect',adapter],capture_output=True).returncode==0:
 labels=json.loads(subprocess.check_output(['docker','inspect','-f','{{json .Config.Labels}}',adapter],text=True))
 if labels.get('io.grain.task') != 'task043-live-navigation': die('adapter ownership mismatch')
 subprocess.run(['docker','stop','-t','5',adapter],check=False,stdout=subprocess.DEVNULL)
# Locate the current bind sources.
mounts={m['Destination']:m for m in info['Mounts']}
out_mount=mounts.get('/task043/out'); pcd_mount=mounts.get('/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD')
if not out_mount or not pcd_mount: die('Golden runtime map/output mounts missing')
old_out=pathlib.Path(out_mount['Source']).resolve()
run_root=base/'runtime-maps'/manifest['map_id']/time.strftime('%Y%m%d-%H%M%S')
new_out=run_root/'out'; backup=state_dir/('map-switch-'+time.strftime('%Y%m%d-%H%M%S'))
backup.mkdir(parents=True,exist_ok=True); new_out.mkdir(parents=True,exist_ok=False)
# Snapshot the task-owned runtime output and construct a new identity-bound config.
shutil.copytree(old_out,new_out,dirs_exist_ok=True,symlinks=True)
# A pose or READY state is bound to the previous map and must never carry over.
for stale in ('initial_pose_confirmation.json','localization_startup_status.json','fault_diagnostics.json'):
 (new_out/stale).unlink(missing_ok=True)
shutil.copy2(old_out/'frozen_config.yaml',backup/'frozen_config.yaml.before')
shutil.copy2(old_out/'localization_startup_config.json',backup/'localization_startup_config.json.before')
shutil.copy2(old_out/'startup_map_manifest.json',backup/'startup_map_manifest.json.before')
frozen=(new_out/'frozen_config.yaml').read_text(encoding='utf-8')
frozen_lines=[]
for line in frozen.splitlines():
 frozen_lines.append('map_file_path: "/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD/GlobalMap.pcd"' if line.lstrip().startswith('map_file_path:') else line)
(new_out/'frozen_config.yaml').write_text('\n'.join(frozen_lines)+'\n',encoding='utf-8')
runtime_manifest={
 'schema_version':2,'task_id':'GSR-NAV-UI-SOFTWARE-INTEGRATION-043',
 'created_utc':manifest.get('created_at',time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())),
 'generation':'task043 UI saved map candidate promoted to task-owned relocalization runtime',
 'map_id':manifest['map_id'],'map_version':str(manifest['map_version']),
 'map_frame':manifest['map_frame'],'map_frame_semantics':'Native Golden camera_init world frame; no added transform',
 'pcd_sha256':manifest['pcd_sha256'],'ikdtree_sha256':manifest['ikdtree_sha256'],
 'point_count':manifest.get('point_count'), 'map_bbox':manifest.get('map_bbox'),
 'origin':'mapping_start','origin_pose':manifest.get('origin_pose',[0,0,0,0,0,0,1]),
 'source_topics':{'imu':'/livox/imu','lidar':'/livox/lidar'},
 'runtime_algorithm_sha256':json.loads((old_out/'startup_map_manifest.json').read_text()).get('runtime_algorithm_sha256','')}
(new_out/'startup_map_manifest.json').write_text(json.dumps(runtime_manifest,indent=2)+'\n',encoding='utf-8')
config=json.loads((new_out/'localization_startup_config.json').read_text())
config.update(map_id=runtime_manifest['map_id'],map_version=runtime_manifest['map_version'],manifest='/task043/out/startup_map_manifest.json',manifest_sha256=sha(new_out/'startup_map_manifest.json'),frozen_config='/task043/out/frozen_config.yaml',frozen_config_sha256=sha(new_out/'frozen_config.yaml'),pcd_directory='/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD')
(new_out/'localization_startup_config.json').write_text(json.dumps(config,indent=2)+'\n',encoding='utf-8')
# Recreate the same task-owned container, changing only the PCD and /task043/out binds.
# Pause only the verified task043 supervisor while replacing its owned runtime.
supervisor=subprocess.check_output(['pgrep','-f','^bash /home/ps02/grain_robot/staging/lenovo-local-20261010/LENOVO/scripts/task043_stack_supervisor.sh$'],text=True).split()
if len(supervisor)!=1: die('expected exactly one task043 stack supervisor')
supervisor_pid=int(supervisor[0])
cmdline=pathlib.Path(f'/proc/{supervisor_pid}/cmdline').read_bytes().replace(b'\0',b' ').decode()
if 'task043_stack_supervisor.sh' not in cmdline: die('supervisor identity changed')
os.kill(supervisor_pid, signal.SIGSTOP)
replacement=runtime+'-switching'
try:
 # Build a replacement container first. Keep the old runtime until checks pass.
 subprocess.run(['docker','rm','-f',replacement],check=False,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 host_config=info.get('HostConfig') or {}
 cmd=['docker','run','-d','--name',replacement]
 network=host_config.get('NetworkMode') or 'host'
 cmd += ['--network',network]
 labels=dict(info['Config'].get('Labels') or {})
 labels['io.grain.task']='task043-localization'
 for key,value in labels.items():
  if value is not None: cmd += ['--label',f'{key}={value}']
 for env in info['Config'].get('Env') or []: cmd += ['-e',env]
 if info['Config'].get('WorkingDir'): cmd += ['--workdir',info['Config']['WorkingDir']]
 if info['Config'].get('User'): cmd += ['--user',info['Config']['User']]
 entrypoint=info['Config'].get('Entrypoint')
 if entrypoint: cmd += ['--entrypoint',entrypoint[0] if isinstance(entrypoint,list) else entrypoint]
 for dest,m in mounts.items():
  src=str(pcd.parent.resolve()) if dest=='/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD' else str(new_out) if dest=='/task043/out' else m['Source']
  mode='ro' if not m.get('RW') else 'rw'
  cmd += ['--mount',f'type=bind,source={src},target={dest},readonly' if mode=='ro' else f'type=bind,source={src},target={dest}']
 cmd += [info['Config']['Image']] + (info['Config'].get('Cmd') or ['/task043/out/launch_runtime.sh'])
 subprocess.run(cmd,check=True)
 # Verify the replacement uses the selected PCD and its matching startup identity.
 current=json.loads(subprocess.check_output(['docker','inspect',replacement],text=True))[0]
 new_mounts={m['Destination']:pathlib.Path(m['Source']).resolve() for m in current['Mounts']}
 if new_mounts.get('/task032/runtime/repaired/ws/src/S-FAST_LIO/PCD') != pcd.parent.resolve(): die('new PCD bind mismatch')
 if new_mounts.get('/task043/out') != new_out.resolve(): die('new startup output bind mismatch')
 subprocess.run(['docker','stop','-t','10',runtime],check=True,stdout=subprocess.DEVNULL)
 subprocess.run(['docker','rm',runtime],check=True,stdout=subprocess.DEVNULL)
 subprocess.run(['docker','rename',replacement,runtime],check=True)
 # Keep the host daemon and the next adapter launch on the same selected map.
 selected_state=state_dir/'active_runtime_map.json'
 if selected_state.exists(): shutil.copy2(selected_state, backup/'active_runtime_map.json.before')
 selected_state.write_text(json.dumps({'map_id':runtime_manifest['map_id'],
                                       'map_version':int(runtime_manifest['map_version']),
                                       'frame':runtime_manifest['map_frame'],
                                       'pcd_sha256':manifest.get('pcd_sha256',''),
                                       'package_manifest_sha256':meta.get('manifest_sha256',''),
                                       'runtime_container':runtime,
                                       'manifest_sha256':config['manifest_sha256']}, indent=2)+'\n', encoding='utf-8')
 host_config=pathlib.Path('/home/ps02/grain_robot/staging/lenovo-local-20261010/LENOVO/config/local_robot.local-live.json')
 if host_config.exists():
  shutil.copy2(host_config, backup/'local_robot.local-live.json.before')
  host_value=json.loads(host_config.read_text(encoding='utf-8'))
  host_value.update({'map_id':runtime_manifest['map_id'], 'map_version':int(runtime_manifest['map_version']),
                     'localization_startup_map_version':runtime_manifest['map_version'],
                     'localization_startup_manifest_sha256':config['manifest_sha256']})
  fd,tmp=tempfile.mkstemp(prefix='.local_robot.local-live.',dir=host_config.parent)
  with os.fdopen(fd,'w',encoding='utf-8') as stream:
   json.dump(host_value,stream,ensure_ascii=False,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
  os.replace(tmp,host_config)

 print(json.dumps({'map_id':runtime_manifest['map_id'],'map_version':runtime_manifest['map_version'],'runtime_root':str(run_root),'backup':str(backup),'container_recreated':True},indent=2))
finally:
 try: subprocess.run(['docker','rm','-f',replacement],check=False,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 except Exception: pass
 try: os.kill(supervisor_pid, signal.SIGCONT)
 except ProcessLookupError: pass
PY
