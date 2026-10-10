import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "sampling_press", Path(__file__).resolve().parents[1] / "scripts" / "sampling_press.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_script_uses_production_cycle_and_return():
    calls = []
    module.run_segments(25, 20, lambda name: calls.append(name),
                        lambda name, cm: calls.append((name, cm)))
    assert calls == ["clamp", ("down_cycle", 20), "unclamp", ("return", 20), "clamp",
                     "clamp", ("down_cycle", 5), "unclamp", ("return", 5), "clamp"]


def test_script_failure_does_not_unclamp_or_return():
    calls = []

    def fail(*args):
        raise RuntimeError("fault")

    with pytest.raises(RuntimeError):
        module.run_segments(20, 20, calls.append, fail)
    assert calls == ["clamp"]
