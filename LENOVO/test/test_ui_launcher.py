"""Exercise the actual launcher with no Qt, Docker, socket or serial access."""
import os
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[2]
BASH = shutil.which("bash")

@pytest.mark.parametrize("gate_exit,mode,expect_ui", [
    (0, "start", True),
    (1, "start", True),    # blocked gate still opens the console for fault display
    (1, "check", False),   # an explicit check must fail loudly
])
def test_host_ui_uses_shared_config_and_reports_gate_state(tmp_path, gate_exit, mode, expect_ui):
    if not BASH or not Path(BASH).exists():
        pytest.skip("Bash unavailable")
    scripts = tmp_path / "LENOVO/scripts"
    scripts.mkdir(parents=True)
    config = tmp_path / "LENOVO/config/local_robot.local-live.json"
    config.parent.mkdir()
    config.write_text("{}")
    (scripts / "local_robot.py").write_text("")
    launcher = scripts / "start_mapping_ui.sh"
    shutil.copyfile(ROOT / "LENOVO/scripts/start_mapping_ui.sh", launcher)
    (scripts / "task043_navigation_gate.sh").write_text(f"exit {gate_exit}\n")
    python = tmp_path / ".venv-ui/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text('#!/usr/bin/env bash\nif [[ "$1" == -c ]]; then exit 0; fi\nprintf "HOST_UI %s\\n" "$*"\n')
    python.chmod(0o755)
    env = dict(os.environ)
    for name in ("PYTHON", "GRAIN_UI_PYTHON", "GRAIN_LOCAL_CONFIG", "TASK043_SOCKET_DIR"):
        env.pop(name, None)
    result = subprocess.run([BASH, str(launcher), "--task043-gate", mode],
                            env=env, capture_output=True, text=True)
    if not expect_ui:
        assert result.returncode == 1, result.stderr
        assert "HOST_UI" not in result.stdout
        return
    assert result.returncode == 0, result.stderr
    assert "HOST_UI" in result.stdout
    assert "local_robot.py ui --config" in result.stdout
    assert "--runtime-dir" in result.stdout
    assert "start_noetic_slam" not in result.stdout
    if gate_exit:
        assert "fault-display mode" in result.stderr
