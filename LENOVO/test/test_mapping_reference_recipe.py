"""Generate the real mapping configuration and compare the verified 042 recipe."""
import os,shutil,subprocess
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[2]
def shell_path(path):
    s=str(path).replace("\\","/")
    return "/"+s[0].lower()+s[2:] if os.name=="nt" else s

def test_mapping_generated_config_preserves_complete_reference_recipe(tmp_path):
    bash=shutil.which("bash")
    if not bash:pytest.skip("Bash unavailable")
    import yaml
    source=(ROOT/"LENOVO/scripts/task043_mapping_session.sh").read_text(encoding="utf-8")
    funcs=tmp_path/"mapping_functions.sh"
    funcs.write_text(source[:source.index('case "${1:-}" in')],encoding="utf-8")
    env={**os.environ,"GRAIN_MAPPING_BASE":shell_path(tmp_path/"maps")}
    result=subprocess.run([bash,"-c",'source "$1"; write_params "$2"',"_",shell_path(funcs),shell_path(tmp_path)],env=env,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    p=yaml.safe_load((tmp_path/"mapping.yaml").read_text())
    assert p["cube_side_length"]==1000.0
    assert p["common"]=={"lid_topic":"/livox/lidar","imu_topic":"/livox/imu","time_sync_en":False,"time_offset_lidar_to_imu":0.0}
    assert p["mapping"]["det_range"]==100.0
    assert p["mapping"]["extrinsic_est_en"] is False
    assert p["mapping"]["extrinsic_T"]==[0.,0.,0.]
    assert p["mapping"]["extrinsic_R"]==[1,0,0,0,1,0,0,0,1]
    assert p["publish"]["scan_bodyframe_pub_en"] is False
    assert p["preprocess"]["timestamp_unit"]==3
