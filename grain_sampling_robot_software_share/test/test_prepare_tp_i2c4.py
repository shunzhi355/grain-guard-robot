"""Synthetic FIT regressions: no target boot partition is accessed."""
import hashlib
import importlib.util
from pathlib import Path
import struct
import subprocess
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/prepare_tp_i2c4.py"
spec = importlib.util.spec_from_file_location("tp_patch", SCRIPT)
patcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)


def cells(*values):
    return struct.pack(">" + "I" * len(values), *values)


def fdt(tree):
    strings = bytearray()
    body = bytearray()

    def aligned(data):
        return data + b"\0" * (-len(data) % 4)

    def visit(name, props, children):
        body.extend(cells(1) + aligned(name.encode() + b"\0"))
        for key, value in props.items():
            nameoff = len(strings)
            strings.extend(key.encode() + b"\0")
            body.extend(cells(3, len(value), nameoff) + aligned(value))
        for child, content in children.items():
            visit(child, *content)
        body.extend(cells(2))

    visit("", *tree)
    body.extend(cells(9))
    return (cells(0xD00DFEED, 56 + len(body) + len(strings), 56,
                  56 + len(body), 40, 17, 16, 0, len(strings), len(body))
            + bytes(16) + body + strings)


def fixture_image(signed=False, wrong_pin=False):
    blob = fdt(({"compatible": b"neardi,lpb3588-linux-f0,\0rockchip,rk3588\0"}, {
        "aliases": ({"i2c4": b"/i2c@feac0000\0"}, {}),
        "i2c@feac0000": ({"status": b"disabled\0", "pinctrl-0": cells(409)}, {}),
        "pinctrl": ({}, {"i2c4": ({}, {"i2c4m0-xfer": ({
            "phandle": cells(409),
            "rockchip,pins": cells(3, 7 if wrong_pin else 6, 9, 500, 3, 5, 9, 500)
        }, {})})})
    }))
    images = {}
    payloads = (("fdt", 2048, blob), ("kernel", 4096, b"kernel bytes"),
                ("resource", 8192, b"resource bytes"))
    for name, offset, data in payloads:
        images[name] = ({"data-position": cells(offset), "data-size": cells(len(data)),
                         "compression": b"none\0"}, {"hash": ({
            "algo": b"sha256\0", "value": hashlib.sha256(data).digest()}, {})})
    signature = {"algo": b"sha256,rsa2048\0"}
    if signed:
        signature["value"] = bytes(256)
    header = fdt(({}, {"images": ({}, images), "configurations": (
        {"default": b"conf\0"}, {"conf": ({"fdt": b"fdt\0", "kernel": b"kernel\0",
        "multi": b"resource\0"}, {"signature": (signature, {})})})}))
    assert len(header) <= 2048
    result = bytearray(16384)
    result[:len(header)] = header
    for _, offset, data in payloads:
        result[offset:offset + len(data)] = data
    return result


def test_enable_and_hash_preserve_offsets_and_other_payloads():
    original = fixture_image()
    result = patcher.prepare(original)
    before, after = patcher.FDT(original), patcher.FDT(result)
    assert len(result) == len(original)
    start = after.number("/images/fdt", "data-position")
    size = after.number("/images/fdt", "data-size")
    assert patcher.FDT(result[start:start + size]).get("/i2c@feac0000", "status") == b"okay\0"
    assert hashlib.sha256(result[start:start + size]).digest() == after.get("/images/fdt/hash", "value")
    for key in before.props:
        if key != ("/images/fdt/hash", "value"):
            assert before.props[key] == after.props[key]
    assert result[4096:] == original[4096:]


@pytest.mark.parametrize("offset", [2048 + 60, 4096, 8192])
def test_refuse_corrupt_original(offset):
    original = fixture_image()
    original[offset] ^= 1
    with pytest.raises(ValueError, match="checksum"):
        patcher.prepare(original)


def test_refuse_real_signature():
    with pytest.raises(ValueError, match="signing key"):
        patcher.prepare(fixture_image(signed=True))


def test_refuse_wrong_pin_mux():
    with pytest.raises(ValueError, match="pin mux"):
        patcher.prepare(fixture_image(wrong_pin=True))


def test_refuse_already_enabled():
    with pytest.raises(ValueError, match="disabled"):
        patcher.prepare(patcher.prepare(fixture_image()))


def test_cli_creates_copy_and_refuses_overwrite(tmp_path):
    original = fixture_image()
    backup, output = tmp_path / "original.img", tmp_path / "tp.img"
    backup.write_bytes(original)
    command = [sys.executable, str(SCRIPT), str(backup), str(output)]
    first = subprocess.run(command, capture_output=True, text=True)
    assert first.returncode == 0, first.stderr
    expected = output.read_bytes()
    second = subprocess.run(command, capture_output=True, text=True)
    assert second.returncode != 0
    assert output.read_bytes() == expected
    assert backup.read_bytes() == original
