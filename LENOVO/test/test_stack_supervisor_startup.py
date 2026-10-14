"""Cold-start dependency checks execute production shell functions with fake Docker."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import pytest

ROOT=Path(__file__).resolve().parents[2]
BASH=shutil.which("bash")

def shell_path(path):
    value=str(path).replace("\\","/")
    if os.name=="nt":return "/"+value[0].lower()+value[2:]
    return value

@pytest.mark.parametrize("ready,image,state,owner,action,success", [
    ("no","grain-nav:noetic-x86","exited","task043-live-navigation","ensure_runtime",False),
    ("yes","different-image","exited","task043-live-navigation","ensure_runtime",False),
    ("yes","grain-nav:noetic-x86","exited","task043-live-navigation","ensure_runtime",True),
    ("yes","grain-nav:noetic-x86","exited","someone-else","ensure_adapter",False),
])
def test_boot_dependency_and_owner_gates(tmp_path,ready,image,state,owner,action,success):
    if not BASH:pytest.skip("Bash unavailable")
    fake=tmp_path/"bin";fake.mkdir()
    docker=fake/"docker"
    docker.write_text('''#!/usr/bin/env bash
printf '%s\n' "$*" >>"$CALL_LOG"
case "$1" in
 info) [[ "$FAKE_READY" == yes ]] ;;
 inspect)
   case "$*" in
     *State.Status*) printf '%s\n' "$FAKE_STATE" ;;
     *Config.Image*) printf '%s\n' "$FAKE_IMAGE" ;;
     *Config.Labels*) printf '%s\n' "$FAKE_OWNER" ;;
   esac ;;
 start) exit 0 ;;
 *) exit 97 ;;
esac
''')
    docker.chmod(0o755)
    python=fake/"python3"
    python.write_text(f'#!/usr/bin/env bash\nexec "{shell_path(sys.executable)}" "$@"\n')
    python.chmod(0o755)
    timeout=fake/"timeout"
    timeout.write_text('#!/usr/bin/env bash\nshift; exec "$@"\n')
    timeout.chmod(0o755)
    source=(ROOT/"LENOVO/scripts/task043_stack_supervisor.sh").read_text()
    functions=tmp_path/"functions.sh"
    functions.write_text(source[:source.index("while :; do")])
    env=dict(os.environ, FAKE_READY=ready,FAKE_IMAGE=image,FAKE_STATE=state,FAKE_OWNER=owner,
             CALL_LOG=shell_path(tmp_path/"calls.log"),TASK043_SOCKET_DIR=shell_path(tmp_path/"state"),
             TASK043_LOG_DIR=shell_path(tmp_path/"logs"),
             TASK043_RUNTIME_CONTAINER="task043-new-map-relocalization-test",
             TASK043_ADAPTER_CONTAINER="task043-live-nav-adapter-test")
    result=subprocess.run([BASH,"-c",'export PATH="$1:$PATH"; source "$2"; "$3"',"_",
                           shell_path(fake),shell_path(functions),action],env=env,capture_output=True,text=True)
    assert (result.returncode==0)==success,result.stdout+result.stderr
    calls=(tmp_path/"calls.log").read_text()
    if success:assert "start task043-new-map-relocalization-test" in calls
    else:assert "start " not in calls and "rm " not in calls


@pytest.mark.parametrize("adapter_start,runtime_start,registered,owner,reconnect", [
    ("2026-10-13T17:42:40Z", "2026-10-13T17:55:00Z", "yes", "task043-live-navigation", True),
    ("2026-10-13T17:56:00Z", "2026-10-13T17:55:00Z", "no", "task043-live-navigation", True),
    ("2026-10-13T17:56:00Z", "2026-10-13T17:55:00Z", "yes", "task043-live-navigation", False),
    ("2026-10-13T17:42:40Z", "2026-10-13T17:55:00Z", "no", "other-owner", False),
])
def test_production_supervisor_recovers_only_its_disconnected_ros_adapter(
        tmp_path, adapter_start, runtime_start, registered, owner, reconnect):
    if not BASH: pytest.skip("Bash unavailable")
    fake=tmp_path/"bin"; fake.mkdir()
    docker=fake/"docker"
    docker.write_text('''#!/usr/bin/env bash
printf '%s\n' "$*" >>"$CALL_LOG"
case "$1" in
 inspect)
   case "$*" in
    *State.Status*) echo running ;;
    *Config.Image*) echo grain-nav:noetic-x86 ;;
    *Config.Labels*) echo "$FAKE_OWNER" ;;
    *State.StartedAt*)
      if [[ "$*" == *relocalization-test* ]]; then echo "$RUNTIME_START";
      else echo "$ADAPTER_START"; fi ;;
   esac ;;
 exec)
   if [[ "$*" == *localization_startup_status.json* ]]; then
     echo '{"state":"READY"}';
   else cat >/dev/null; [[ "$FAKE_REGISTERED" == yes ]]; fi ;;
 stop) exit 0 ;;
 *) exit 97 ;;
esac
''')
    docker.chmod(0o755)
    python=fake/"python3"
    python.write_text(f'#!/usr/bin/env bash\nexec "{shell_path(sys.executable)}" "$@"\n')
    python.chmod(0o755)
    timeout=fake/"timeout"
    timeout.write_text('#!/usr/bin/env bash\nshift; exec "$@"\n')
    timeout.chmod(0o755)
    source=(ROOT/"LENOVO/scripts/task043_stack_supervisor.sh").read_text()
    functions=tmp_path/"functions.sh"
    functions.write_text(source[:source.index("while :; do")])
    env=dict(os.environ, FAKE_OWNER=owner, ADAPTER_START=adapter_start,
             RUNTIME_START=runtime_start, FAKE_REGISTERED=registered,
             CALL_LOG=shell_path(tmp_path/"calls.log"), TASK043_SOCKET_DIR=shell_path(tmp_path/"state"),
             TASK043_LOG_DIR=shell_path(tmp_path/"logs"),
             TASK043_RUNTIME_CONTAINER="task043-new-map-relocalization-test",
             TASK043_ADAPTER_CONTAINER="task043-live-nav-adapter-test")
    result=subprocess.run([BASH,"-c",'export PATH="$1:$PATH"; source "$2"; ensure_adapter',"_",
                           shell_path(fake),shell_path(functions)],env=env,capture_output=True,text=True)
    calls=(tmp_path/"calls.log").read_text()
    assert ("stop -t 5 task043-live-nav-adapter-test" in calls)==reconnect
    assert "stop -t 5 task043-new-map" not in calls
    assert "clear_estop" not in calls and "nav_velocity" not in calls
    assert (result.returncode==0)==(registered=="yes" and not reconnect and owner=="task043-live-navigation")


@pytest.mark.parametrize("fault", [None,"missing_health","other_owner","missing_odom","missing_startup","node_unreachable"])
def test_production_ros_probe_requires_real_registered_adapter(monkeypatch, fault):
    import builtins, io, json, socket, xmlrpc.client
    source=(ROOT/"LENOVO/scripts/task043_stack_supervisor.sh").read_text()
    block=source.split("adapter_ros_registered() {",1)[1].split("\nPY\n}",1)[0]
    code=block.split("\n",2)[2]
    node="/grain_lenovo_local_navigation_test"
    config={"odom_topic":"/test/odom","localization_valid_topic":"/test/valid",
            "localization_startup_status_topic":"/test/startup"}
    pubs={"/test/valid":[node]}; subs={"/test/odom":[node],"/test/startup":[node]}
    if fault=="missing_health":pubs={}
    if fault=="other_owner":pubs["/test/valid"]=["/other_task"]
    if fault=="missing_odom":subs.pop("/test/odom")
    if fault=="missing_startup":subs.pop("/test/startup")
    class Proxy:
        def getSystemState(self,caller):return [1,"ok",[list(pubs.items()),list(subs.items()),[]]]
        def lookupNode(self,caller,name):return [1,"ok","http://test-adapter:1234"]
        def getPid(self,caller):
            if fault=="node_unreachable":raise OSError("node offline")
            return [1,"ok",99]
    real_open=builtins.open
    monkeypatch.setattr(builtins,"open",lambda path,*a,**kw:
        io.StringIO(json.dumps(config)) if path=="/run/task043-config/local_robot.json" else real_open(path,*a,**kw))
    monkeypatch.setattr(xmlrpc.client,"ServerProxy",lambda *a,**kw:Proxy())
    monkeypatch.setattr(socket,"setdefaulttimeout",lambda value:None)
    monkeypatch.setenv("ROS_MASTER_URI","http://test-master:11311")
    try:exec(compile(code,"production_adapter_probe","exec"),{})
    except SystemExit as exc:assert (exc.code==0)==(fault is None)
    except OSError:assert fault=="node_unreachable"
