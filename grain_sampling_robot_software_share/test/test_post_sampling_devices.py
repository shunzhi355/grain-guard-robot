from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "test_post_sampling_devices.py"
spec = importlib.util.spec_from_file_location("post_sampling_device_test", SCRIPT)
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def _options(**overrides):
    values = dict(
        dry_run=True,
        assume_yes=False,
        grain="稻谷",
        waste_seconds=0.0,
        convey_seconds=0.0,
        bin_hold_seconds=0.0,
        service_timeout=1.0,
        skip_waste=False,
    )
    values.update(overrides)
    return module.Options(**values)


def test_dry_run_uses_production_services_and_all_three_bins(capsys):
    flow = module.PostSamplingDeviceTest(_options())
    flow.run()
    output = capsys.readouterr().out

    assert "/mechanism/start_suction" in output
    assert "/mechanism/stop_suction" in output
    assert "/mechanism/convey" in output
    for name in ("shallow", "mid", "deep"):
        assert f"/mechanism/open_bin/{name}" in output
        assert f"/mechanism/close_bin/{name}" in output
    assert "导航、插管、夹紧、拧管或升降" in output


def test_skip_waste_omits_suction_calls(capsys):
    flow = module.PostSamplingDeviceTest(_options(skip_waste=True))
    flow.run()
    output = capsys.readouterr().out

    assert "[\u8df3\u8fc7] --skip-waste" in output
    assert "[预演] 启动排废粮" not in output
    assert "/mechanism/convey" in output
