#!/usr/bin/env python3
"""Prepare an LPB3588 FIT copy with I2C4 enabled; never write a block device.

Uses only Python's standard library. Preserves image size, external data offsets,
kernel and non-DTB resources. Updates both FIT fdt and resource DTB copies. Refuses populated signature nodes; does not remove or
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


def reject_pin_conflicts(dt, target, group):
    def pins(path):
        raw = dt.props.get((path, 'rockchip,pins'), (0, 0, b''))[2]
        if len(raw) % 16:
            raise ValueError('Malformed pin group: ' + path)
        return {(u32(raw, n), u32(raw, n + 4)) for n in range(0, len(raw), 16)}

    handles = {dt.number(path, prop): path for path, prop in dt.props
               if prop in ('phandle', 'linux,phandle')}
    wanted = pins(group)
    for (node, prop), (_, _, raw) in dt.props.items():
        if node == target or not prop.startswith('pinctrl-') or not prop[8:].isdigit():
            continue
        parent = node
        enabled = True
        while parent:
            value = dt.props.get((parent, 'status'), (0, 0, b'okay\0'))[2]
            if value.rstrip(b'\0') not in (b'okay', b'ok'):
                enabled = False
            parent = parent.rsplit('/', 1)[0]
        if not enabled:
            continue
        if len(raw) % 4:
            raise ValueError('Malformed pinctrl reference: ' + node)
        for offset in range(0, len(raw), 4):
            other = handles.get(u32(raw, offset))
            if other and wanted & pins(other):
                raise ValueError('I2C4 pin conflict with enabled device ' + node + ' (' + other + ')')


def enable_dtb(original, disable_gmac1=False):
    blob = bytearray(original)
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

    expected = {(node, 'status'): b'okay\0'}
    if disable_gmac1:
        ethernet = '/ethernet@fe1c0000'
        ep, ev, old_eth = dt.props[ethernet, 'status']
        if old_eth != b'okay\0':
            raise ValueError('Expected GMAC1 status okay before disabling it')
        expected[ethernet, 'status'] = b'disabled\0'
        # Exchange four bytes of aligned property space. Total structure size,
        # string table and all external FIT payload offsets remain unchanged.
        edits = [(length_pos, value_pos + 12,
                  struct.pack('>I', 5) + blob[length_pos + 4:value_pos] + b'okay\0\0\0\0'),
                 (ep, ev + 8,
                  struct.pack('>I', 9) + blob[ep + 4:ev] + b'disabled\0\0\0\0')]
        for start, end, data in sorted(edits, reverse=True):
            blob[start:end] = data
        allowed = set(range(min(length_pos, ep), max(value_pos + 12, ev + 8)))
    else:
        # Shorter value plus FDT_NOP keeps every offset fixed.
        struct.pack_into('>I', blob, length_pos, 5)
        blob[value_pos:value_pos + 12] = b'okay\0\0\0\0' + struct.pack('>I', 4)
        allowed = set(range(length_pos, length_pos + 4))
        allowed.update(range(value_pos, value_pos + 12))
    changed_dt = FDT(blob)
    if len(blob) != len(original):
        raise ValueError('DTB size changed')
    for key, value in expected.items():
        if changed_dt.get(*key) != value:
            raise ValueError('Patched status validation failed')
    if dt.nodes != changed_dt.nodes or dt.props.keys() != changed_dt.props.keys():
        raise ValueError("Device tree structure changed unexpectedly")
    for key in dt.props:
        if key not in expected and dt.props[key][2] != changed_dt.props[key][2]:
            raise ValueError("Unrelated device tree property changed")
    reject_pin_conflicts(changed_dt, node, pin)
    return blob, allowed


def resource_entries(data):
    """Parse Rockchip RSCE v0 / ENTR (512-byte blocks, little-endian cells).

    Layout: rockchip-linux/u-boot arch/arm/mach-rockchip/resource_img.c.
    Reject variants and multiple DTBs rather than guessing hardware selection.
    """
    if len(data) < 512 or data[:4] != b"RSCE":
        raise ValueError("Resource is not RSCE; inspect actual image")
    version, content_version = struct.unpack_from("<HH", data, 4)
    header_blocks, table_block, entry_blocks = data[8:11]
    count = struct.unpack_from("<I", data, 12)[0]
    if (version != 0 or content_version != 0 or header_blocks != 1
            or table_block != 1 or entry_blocks != 1 or count < 1):
        raise ValueError("Unsupported RSCE header")
    table_end = (table_block + count * entry_blocks) * 512
    if table_end > len(data):
        raise ValueError("Resource entry table out of bounds")
    entries, ranges, names = [], [], set()
    for i in range(count):
        entry = (table_block + i * entry_blocks) * 512
        if data[entry:entry + 4] != b"ENTR":
            raise ValueError("Invalid resource entry")
        raw_name = data[entry + 4:entry + 224]
        if b"\0" not in raw_name:
            raise ValueError("Unterminated resource name")
        name = raw_name.split(b"\0", 1)[0].decode("ascii")
        hash_size, block, size = struct.unpack_from("<III", data, entry + 256)
        start = block * 512
        if not name or name in names or hash_size not in (0, 20, 32):
            raise ValueError("Invalid resource name/hash type")
        if size <= 0 or start < table_end or start + size > len(data):
            raise ValueError("Invalid resource data bounds")
        if any(start < b and start + size > a for a, b in ranges):
            raise ValueError("Overlapping resource entries")
        names.add(name)
        ranges.append((start, start + size))
        if hash_size:
            digest = (hashlib.sha1 if hash_size == 20 else hashlib.sha256)(
                data[start:start + size]).digest()
            if digest != data[entry + 224:entry + 224 + hash_size]:
                raise ValueError("Resource entry checksum failed: " + name)
        entries.append((name, start, size, entry + 224, hash_size))
    return entries


def enable_resource(original, disable_gmac1=False):
    entries = resource_entries(original)
    dtbs = [e for e in entries if e[0].endswith(".dtb")
            or original[e[1]:e[1] + 4] == b"\xd0\x0d\xfe\xed"]
    if len(dtbs) != 1 or dtbs[0][0] != "rk-kernel.dtb":
        raise ValueError("Expected single rk-kernel.dtb resource; found "
                         + repr([e[0] for e in dtbs]))
    name, start, size, hash_pos, hash_size = dtbs[0]
    blob = original[start:start + size]
    if u32(blob, 4) != size:
        raise ValueError("Resource DTB size differs from FDT totalsize")
    blob, changes = enable_dtb(blob, disable_gmac1)
    result = bytearray(original)
    result[start:start + size] = blob
    allowed = {start + i for i in changes}
    if hash_size:
        digest = (hashlib.sha1 if hash_size == 20 else hashlib.sha256)(blob).digest()
        result[hash_pos:hash_pos + hash_size] = digest
        allowed.update(range(hash_pos, hash_pos + hash_size))
    if resource_entries(result) != entries:
        raise ValueError("Resource entry layout changed")
    if any(a != b and i not in allowed for i, (a, b) in enumerate(zip(original, result))):
        raise ValueError("Non-DTB resource bytes changed")
    return result, allowed


def prepare(original, disable_gmac1=False):
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
    blob, dt_changes = enable_dtb(blob, disable_gmac1)

    result = bytearray(original)
    result[start:start + size] = blob
    _, hash_pos, _ = fit.props["/images/fdt/hash", "value"]
    digest = hashlib.sha256(blob).digest()
    result[hash_pos:hash_pos + 32] = digest
    updated = FDT(result)
    if updated.get("/images/fdt/hash", "value") != digest or len(result) != len(original):
        raise ValueError("Patched FIT validation failed")
    resource_start = fit.number("/images/resource", "data-position")
    resource_size = fit.number("/images/resource", "data-size")
    if fit.string("/images/resource", "compression") != "none":
        raise ValueError("Compressed resource is unsupported")
    resource, resource_changes = enable_resource(
        original[resource_start:resource_start + resource_size], disable_gmac1)
    result[resource_start:resource_start + resource_size] = resource
    _, resource_hash_pos, _ = fit.props["/images/resource/hash", "value"]
    result[resource_hash_pos:resource_hash_pos + 32] = hashlib.sha256(resource).digest()
    updated = FDT(result)
    for path in fit.children("/images"):
        pos = fit.number(path, "data-position")
        count = fit.number(path, "data-size")
        if hashlib.sha256(result[pos:pos + count]).digest() != updated.get(path + "/hash", "value"):
            raise ValueError("Output hash verification failed: " + path)
    allowed = {start + i for i in dt_changes}
    allowed.update(range(hash_pos, hash_pos + 32))
    allowed.update(resource_start + i for i in resource_changes)
    allowed.update(range(resource_hash_pos, resource_hash_pos + 32))
    if any(a != b and i not in allowed for i, (a, b) in enumerate(zip(original, result))):
        raise ValueError("Unexpected changed bytes")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("backup", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument('--disable-gmac1', action='store_true',
                        help='Explicitly disable /ethernet@fe1c0000 to release I2C4 m0 pins')
    args = parser.parse_args()
    if not args.backup.is_file():
        parser.error("Input must be a regular backup file")
    original = args.backup.read_bytes()
    try:
        result = prepare(original, args.disable_gmac1)
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
        "script_version": 3,
        "gmac1_disabled": args.disable_gmac1,
        "resource_dtb_status": "okay",
        "kernel_and_non_dtb_resources_unchanged": True,
        "notice": "Prepared copy only. Not flashed or hardware tested."
    }, indent=2))


if __name__ == "__main__":
    main()
