"""Exercise production admission reporting with a fake read-only status client."""
import re
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]

@pytest.mark.parametrize("fault",[None,"invalid_pose","stale","manual","locked","hardware","offline"])
def test_startup_is_not_motion_admission(monkeypatch,capsys,fault):
    from grain_sampling_workflow import robot_bridge
    status={"pose":{"localization_valid":True,"odom_at":time.monotonic()},
            "chassis":{"rc_mode":"auto","rc_valid":True,"faults":0,"chassis_link":"online",
                       "estop_latched":False},"safety_latched":False}
    if fault=="invalid_pose":status["pose"]["localization_valid"]=False
    if fault=="stale":status["pose"]["odom_at"]-=2
    if fault=="manual":status["chassis"]["rc_mode"]="manual"
    if fault=="locked":status["safety_latched"]=True
    if fault=="hardware":status["chassis"]["faults"]=1
    calls=[]
    def request(action):
        calls.append(action)
        if fault=="offline":raise OSError("offline test")
        return status
    monkeypatch.setattr(robot_bridge,"RobotClient",lambda path:SimpleNamespace(request=request))
    monkeypatch.setattr(sys,"argv",["test","/mock/control.sock"])
    source=(ROOT/"LENOVO/scripts/task043_navigation_gate.sh").read_text()
    block=source.split("report_navigation_admission() {",1)[1]
    lines=block.splitlines()
    begin=next(i for i,line in enumerate(lines) if "<<'PY'" in line)
    end=lines.index("PY",begin)
    script=chr(10).join(lines[begin+1:end])
    if fault:
        with pytest.raises(SystemExit) as exc:exec(compile(script,"production-admission","exec"),{})
        assert exc.value.code==1
        assert "NAVIGATION_ADMISSION=BLOCKED" in capsys.readouterr().out
    else:
        exec(compile(script,"production-admission","exec"),{})
        assert "NAVIGATION_ADMISSION=READY" in capsys.readouterr().out
    assert calls==["status"]
