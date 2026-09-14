#!/usr/bin/env python3
"""Prepare an LPB3588 FIT copy with I2C4 enabled; never write a block device.

Uses only Python's standard library. Preserves image size, external data offsets,
kernel and resource bytes. Refuses populated signature nodes; does not remove or
bypass signatures. The original backup must be retained for recovery.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct


def u32(data, offset):
    return struct.unpack_from(">I", data, offset)[0]


class FDT:
    """Read property locations without reserializing an FDT or its string table."""

    def __init__(self, data):
        if len(data) < 40 or u32(data, 0) != 0xD00DFEED:
            raise ValueError("Not an FDT/FIT")
        total, pos, strings = (u32(data, n) for n in (4, 8, 12))
        strings_end = strings + u32(data, 32)
        end = pos + u32(data, 36)
        if not (40 <= pos <= end <= total <= len(data)
                and 40 <= strings <= strings_end <= total):
            raise ValueError("Invalid FDT bounds")
        self.props = {}
        self.nodes = set()
        stack = []
        while pos + 4 <= end:
            token = u32(data, pos)
            pos += 4
            if token == 1:
                stop = data.index(b"\0", pos, end)
                name = data[pos:stop].decode("ascii")
                stack.append(name)
                self.nodes.add("/" + "/".join(stack[1:]))
                pos = (stop + 4) & ~3
            elif token == 2:
                if not stack:
                    raise ValueError("Unbalanced FDT")
                stack.pop()
            elif token == 3:
                if not stack or pos + 8 > end:
                    raise ValueError("Invalid FDT property")
                size, nameoff = u32(data, pos), u32(data, pos + 4)
                value = pos + 8
                if value + ((size + 3) & ~3) > end or strings + nameoff >= strings_end:
                    raise ValueError("Invalid FDT property bounds")
                stop = data.index(b"\0", strings + nameoff, strings_end)
                name = data[strings + nameoff:stop].decode("ascii")
                key = ("/" + "/".join(stack[1:]), name)
                if key in self.props:
                    raise ValueError("Duplicate FDT property")
                self.props[key] = (pos, value, bytes(data[value:value + size]))
                pos = value + ((size + 3) & ~3)
            elif token == 4:
                pass  # FDT_NOP
            elif token == 9 and not stack:
                return
            else:
                raise ValueError("Invalid FDT token or nesting")
        raise ValueError("Missing FDT_END")

    def get(self, path, name):
        return self.props[path, name][2]

    def number(self, path, name):
        value = self.get(path, name)
        if len(value) != 4:
            raise ValueError("Expected one 32-bit cell")
        return u32(value, 0)

    def string(self, path, name):
        return self.get(path, name).rstrip(b"\0").decode("ascii")

    def children(self, path):
        prefix = path.rstrip("/") + "/"
        return sorted(n for n in self.nodes if n.startswith(prefix)
                      and "/" not in n[len(prefix):])


def prepare(original):
    fit = FDT(original)
    for (path, name), (_, _, value) in fit.props.items():
        if name == "value" and value and any(
                part.startswith("signature") for part in path.split("/")):
            raise ValueError("Actual FIT signature found; matching signing key required")
    if fit.children("/images") != ["/images/fdt", "/images/kernel", "/images/resource"]:
        raise ValueError("Unexpected FIT images; inspect manually")
    if fit.children("/configurations") != ["/configurations/conf"]:
        raise ValueError("Unexpected FIT configurations")
    if fit.string("/configurations", "default") != "conf":
        raise ValueError("Unexpected default configuration")
    for prop, expected in (("fdt", "fdt"), ("kernel", "kernel"), ("multi", "resource")):
        if fit.string("/configurations/conf", prop) != expected:
            raise ValueError("Unexpected configuration references")

    ranges = []
    for path in fit.children("/images"):
        start = fit.number(path, "data-position")
        size = fit.number(path, "data-size")
        if start < u32(original, 4) or size <= 0 or start + size > len(original):
            raise ValueError("Invalid external image bounds")
        if any(start < b and start + size > a for a, b in ranges):
            raise ValueError("Overlapping external images")
        ranges.append((start, start + size))
        if fit.children(path) != [path + "/hash"]:
            raise ValueError("Unexpected hash layout")
        if fit.string(path + "/hash", "algo") != "sha256":
            raise ValueError("Unsupported hash algorithm")
        if hashlib.sha256(original[start:start + size]).digest() != fit.get(path + "/hash", "value"):
            raise ValueError("Original image checksum failed: " + path)

    if fit.string("/images/fdt", "compression") != "none":
        raise ValueError("Compressed device tree is unsupported")
    start = fit.number("/images/fdt", "data-position")
    size = fit.number("/images/fdt", "data-size")
    blob = bytearray(original[start:start + size])
    dt = FDT(blob)
    if b"neardi,lpb3588-linux-f0," not in dt.get("/", "compatible").split(b"\0"):
        raise ValueError("Not the expected LPB3588 board configuration")
    node = "/i2c@feac0000"
    if dt.string("/aliases", "i2c4") != node:
        raise ValueError("Unexpected I2C4 alias")
    pin = "/pinctrl/i2c4/i2c4m0-xfer"
    if dt.get(node, "pinctrl-0") != dt.get(pin, "phandle"):
        raise ValueError("I2C4 does not select the expected m0 pin group")
    pins = struct.unpack(">8I", dt.get(pin, "rockchip,pins"))
    if pins[:3] != (3, 6, 9) or pins[4:7] != (3, 5, 9):
        raise ValueError("Unexpected I2C4 pin mux")
    length_pos, value_pos, old = dt.props[node, "status"]
    if old != b"disabled\0":
        raise ValueError("Expected I2C4 status disabled")

    # 9-byte value occupies 12 aligned bytes. A 5-byte value occupies 8;
    # replace the remaining word by a legal FDT_NOP, keeping ALL offsets fixed.
    struct.pack_into(">I", blob, length_pos, 5)
    blob[value_pos:value_pos + 12] = b"okay\0\0\0\0" + struct.pack(">I", 4)
    changed_dt = FDT(blob)
    if changed_dt.get(node, "status") != b"okay\0":
        raise ValueError("Patched status validation failed")
    if dt.nodes != changed_dt.nodes or dt.props.keys() != changed_dt.props.keys():
        raise ValueError("Device tree structure changed unexpectedly")
    for key in dt.props:
        if key != (node, "status") and dt.props[key][2] != changed_dt.props[key][2]:
            raise ValueError("Unrelated device tree property changed")

    result = bytearray(original)
    result[start:start + size] = blob
    _, hash_pos, _ = fit.props["/images/fdt/hash", "value"]
    digest = hashlib.sha256(blob).digest()
    result[hash_pos:hash_pos + 32] = digest
    updated = FDT(result)
    if updated.get("/images/fdt/hash", "value") != digest or len(result) != len(original):
        raise ValueError("Patched FIT validation failed")
    for path in ("/images/kernel", "/images/resource"):
        pos = fit.number(path, "data-position")
        count = fit.number(path, "data-size")
        if result[pos:pos + count] != original[pos:pos + count]:
            raise ValueError("Kernel/resource changed")
    allowed = set(range(start + length_pos, start + length_pos + 4))
    allowed.update(range(start + value_pos, start + value_pos + 12))
    allowed.update(range(hash_pos, hash_pos + 32))
    if any(a != b and i not in allowed for i, (a, b) in enumerate(zip(original, result))):
        raise ValueError("Unexpected changed bytes")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("backup", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if not args.backup.is_file():
        parser.error("Input must be a regular backup file")
    original = args.backup.read_bytes()
    try:
        result = prepare(original)
        # Exclusive create: refuses existing files, symlinks and block devices.
        with args.output.open("xb") as handle:
            handle.write(result)
            handle.flush()
            os.fsync(handle.fileno())
        if args.output.read_bytes() != result:
            raise ValueError("Output readback failed; do not use output")
    except (ValueError, KeyError, OSError, struct.error) as exc:
        parser.exit(1, "STOP: " + str(exc) + "\n")
    print(json.dumps({
        "output": str(args.output.resolve()),
        "original_sha256": hashlib.sha256(original).hexdigest(),
        "output_sha256": hashlib.sha256(result).hexdigest(),
        "i2c4_status": "okay", "pin_group": "i2c4m0-xfer",
        "kernel_resource_unchanged": True,
        "notice": "Prepared copy only. Not flashed or hardware tested."
    }, indent=2))


if __name__ == "__main__":
    main()
