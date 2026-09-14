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


def resource_image(blob, hash_size=32):
    result = bytearray(4096)
    struct.pack_into("<4sHHBBBxI", result, 0, b"RSCE", 0, 0, 1, 1, 1, 2)
    for i, (name, block, data) in enumerate(((b"rk-kernel.dtb", 3, blob),
                                           (b"logo.bmp", 7, b"logo contents"))):
        entry = 512 * (i + 1)
        result[entry:entry + 4] = b"ENTR"
        result[entry + 4:entry + 4 + len(name)] = name
        if hash_size:
            digest = (hashlib.sha1 if hash_size == 20 else hashlib.sha256)(data).digest()
            result[entry + 224:entry + 224 + hash_size] = digest
        struct.pack_into("<III", result, entry + 256, hash_size, block, len(data))
        result[block * 512:block * 512 + len(data)] = data
    return result


def fixture_image(signed=False, wrong_pin=False, resource_hash_size=32):
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
                ("resource", 8192, resource_image(blob, resource_hash_size)))
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


@pytest.mark.parametrize("hash_size", [0, 20, 32])
def test_enable_and_hash_preserve_offsets_and_other_payloads(hash_size):
    original = fixture_image(resource_hash_size=hash_size)
    result = patcher.prepare(original)
    before, after = patcher.FDT(original), patcher.FDT(result)
    assert len(result) == len(original)
    start = after.number("/images/fdt", "data-position")
    size = after.number("/images/fdt", "data-size")
    assert patcher.FDT(result[start:start + size]).get("/i2c@feac0000", "status") == b"okay\0"
    assert hashlib.sha256(result[start:start + size]).digest() == after.get("/images/fdt/hash", "value")
    for key in before.props:
        if key not in (("/images/fdt/hash", "value"), ("/images/resource/hash", "value")):
            assert before.props[key] == after.props[key]
    assert result[4096:8192] == original[4096:8192]
    resource = result[8192:12288]
    entries = patcher.resource_entries(resource)
    name, start, size, _, _ = entries[0]
    assert name == "rk-kernel.dtb"
    assert patcher.FDT(resource[start:start + size]).get("/i2c@feac0000", "status") == b"okay\0"
    assert result[8192 + 7 * 512:] == original[8192 + 7 * 512:]
    assert hashlib.sha256(resource).digest() == after.get("/images/resource/hash", "value")


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


def test_refuse_resource_entry_bad_hash_even_with_valid_fit_hash():
    original = fixture_image()
    original[8192 + 512 + 224] ^= 1
    fit = patcher.FDT(original)
    _, pos, _ = fit.props["/images/resource/hash", "value"]
    original[pos:pos + 32] = hashlib.sha256(original[8192:12288]).digest()
    with pytest.raises(ValueError, match="Resource entry checksum"):
        patcher.prepare(original)


def test_refuse_resource_overlap():
    original = fixture_image()
    resource = original[8192:12288]
    struct.pack_into("<I", resource, 1024 + 260, 3)
    with pytest.raises(ValueError, match="Overlapping"):
        patcher.resource_entries(resource)


def test_refuse_multiple_resource_dtbs():
    original = fixture_image()
    resource = original[8192:12288]
    resource[1028:1248] = b"other.dtb\0".ljust(220, b"\0")
    with pytest.raises(ValueError, match="single rk-kernel"):
        patcher.enable_resource(resource)


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
