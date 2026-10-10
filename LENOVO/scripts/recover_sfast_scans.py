#!/usr/bin/env python3
"""Recover one S-FAST_LIO map from compatible world-frame ``scans_*.pcd`` files."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import struct
import tempfile


class PcdFormatError(RuntimeError):
    pass


def _read_header(path: Path) -> tuple[list[str], dict[str, list[str]], int]:
    lines: list[str] = []
    values: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise PcdFormatError(f"missing DATA header: {path}")
            try:
                line = raw.decode("ascii").rstrip("\r\n")
            except UnicodeDecodeError as exc:
                raise PcdFormatError(f"non-ASCII PCD header: {path}") from exc
            lines.append(line)
            if line and not line.startswith("#"):
                parts = line.split()
                values[parts[0].upper()] = parts[1:]
            if line.upper().startswith("DATA "):
                return lines, values, stream.tell()


def _required(values: dict[str, list[str]], key: str, path: Path) -> list[str]:
    if key not in values or not values[key]:
        raise PcdFormatError(f"missing {key}: {path}")
    return values[key]


def _schema(values: dict[str, list[str]], path: Path) -> tuple[tuple[str, ...], ...]:
    fields = tuple(_required(values, "FIELDS", path))
    sizes = tuple(_required(values, "SIZE", path))
    types = tuple(_required(values, "TYPE", path))
    counts = tuple(values.get("COUNT", ["1"] * len(fields)))
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise PcdFormatError(f"inconsistent field schema: {path}")
    return fields, sizes, types, counts


def _point_step(schema: tuple[tuple[str, ...], ...]) -> int:
    _, sizes, _, counts = schema
    return sum(int(size) * int(count) for size, count in zip(sizes, counts))


def _xyz_layout(schema: tuple[tuple[str, ...], ...]) -> list[tuple[int, str]]:
    fields, sizes, types, counts = schema
    offsets: dict[str, tuple[int, str]] = {}
    offset = 0
    for name, size_text, type_name, count_text in zip(fields, sizes, types, counts):
        size = int(size_text)
        count = int(count_text)
        if name in {"x", "y", "z"} and count == 1 and type_name == "F" and size in {4, 8}:
            offsets[name] = (offset, "<f" if size == 4 else "<d")
        offset += size * count
    if set(offsets) != {"x", "y", "z"}:
        raise PcdFormatError("x/y/z must be scalar float fields")
    return [offsets[axis] for axis in ("x", "y", "z")]


def _number(values: dict[str, list[str]], key: str, path: Path) -> int:
    try:
        return int(_required(values, key, path)[0])
    except ValueError as exc:
        raise PcdFormatError(f"invalid {key}: {path}") from exc


def _natural_scan_key(path: Path) -> int:
    match = re.fullmatch(r"scans_(\d+)\.pcd", path.name)
    if not match:
        raise PcdFormatError(f"unexpected scan filename: {path.name}")
    return int(match.group(1))


def merge_scans(source_dir: Path, destination: Path) -> tuple[int, tuple[float, ...], str]:
    scans = sorted(source_dir.glob("scans_*.pcd"), key=_natural_scan_key)
    if not scans:
        raise PcdFormatError(f"no scans_*.pcd files in {source_dir}")

    metadata = []
    first_schema = None
    total_points = 0
    for scan in scans:
        lines, values, data_offset = _read_header(scan)
        if _required(values, "DATA", scan)[0].lower() != "binary":
            raise PcdFormatError(f"only DATA binary is supported: {scan}")
        if _number(values, "HEIGHT", scan) != 1:
            raise PcdFormatError(f"organized clouds are not supported: {scan}")
        schema = _schema(values, scan)
        if first_schema is None:
            first_schema = schema
            first_lines = lines
        elif schema != first_schema:
            raise PcdFormatError(f"PCD schema differs: {scan}")
        points = _number(values, "POINTS", scan)
        step = _point_step(schema)
        payload_size = scan.stat().st_size - data_offset
        if payload_size != points * step:
            raise PcdFormatError(
                f"payload size mismatch for {scan}: {payload_size} != {points * step}"
            )
        metadata.append((scan, data_offset, points))
        total_points += points

    assert first_schema is not None
    xyz = _xyz_layout(first_schema)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    os.close(fd)
    temporary = Path(temporary_name)
    bounds = [float("inf"), float("inf"), float("inf"),
              float("-inf"), float("-inf"), float("-inf")]
    step = _point_step(first_schema)

    try:
        with temporary.open("wb") as output:
            for line in first_lines:
                key = line.split(maxsplit=1)[0].upper() if line else ""
                if key == "WIDTH":
                    line = f"WIDTH {total_points}"
                elif key == "HEIGHT":
                    line = "HEIGHT 1"
                elif key == "POINTS":
                    line = f"POINTS {total_points}"
                output.write((line + "\n").encode("ascii"))

            for scan, data_offset, points in metadata:
                with scan.open("rb") as source:
                    source.seek(data_offset)
                    for _ in range(points):
                        record = source.read(step)
                        if len(record) != step:
                            raise PcdFormatError(f"truncated point record: {scan}")
                        coords = [struct.unpack_from(fmt, record, offset)[0] for offset, fmt in xyz]
                        bounds[0] = min(bounds[0], coords[0])
                        bounds[1] = min(bounds[1], coords[1])
                        bounds[2] = min(bounds[2], coords[2])
                        bounds[3] = max(bounds[3], coords[0])
                        bounds[4] = max(bounds[4], coords[1])
                        bounds[5] = max(bounds[5], coords[2])
                        output.write(record)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()

    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    return total_points, tuple(bounds), digest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    points, bounds, digest = merge_scans(args.source_dir, args.destination)
    print(f"RECOVERED_MAP={args.destination}")
    print(f"RECOVERED_POINT_COUNT={points}")
    print("RECOVERED_BBOX=" + ",".join(f"{value:.6f}" for value in bounds))
    print(f"RECOVERED_MAP_SHA256={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
