"""PCD point cloud reader for grain sampling robot mapping.

Parses PCD files written by the S-FAST_LIO ``save_map`` service (located under
``~/fastlio2_ws/src/S-FAST_LIO/PCD/``) and returns the raw x/y/z coordinates
as a list of dicts.  Both the ASCII and binary variants of the PCD format are
supported.

Typical usage::

    from grain_sampling_pointcloud.pcd_reader import read_pcd_points

    points = read_pcd_points("/home/user/fastlio2_ws/src/S-FAST_LIO/PCD/map.pcd")
    print(len(points), points[0])  # e.g. 123456 {'x': 0.94, 'y': 0.34, 'z': 0.0}
"""

from __future__ import annotations

import logging
import struct
from typing import Dict, List

logger = logging.getLogger(__name__)

__all__ = ["read_pcd_points"]

# PCD ``TYPE`` letter → :mod:`struct` format char.  Native widths of these
# format chars match the PCD ``SIZE`` values (4 → float32, 8 → float64, ...).
_PCD_TYPE_TO_FMT = {
    "F": "f",  # float32
    "D": "d",  # float64
    "I": "i",  # int32
    "U": "I",  # uint32
}


def read_pcd_points(path: str) -> List[Dict[str, float]]:
    """Read x/y/z coordinates from an ASCII or binary PCD file.

    Both the ``DATA ascii`` and ``DATA binary`` variants are supported.  Header
    fields ``VERSION``, ``FIELDS``, ``SIZE``, ``TYPE``, ``COUNT``, ``WIDTH``,
    ``HEIGHT``, ``POINTS`` and ``DATA`` are scanned; the column indices of
    ``x``/``y``/``z`` in ``FIELDS`` are recorded.  ASCII payloads are parsed
    line-by-line; binary payloads are decoded with :mod:`struct` using the
    field layout declared by ``SIZE``/``TYPE`` (``TYPE F`` + ``SIZE 4`` →
    ``float32``).  The binary blob follows the ``DATA binary`` line directly.

    Parameters
    ----------
    path : str
        Filesystem path to the PCD file.

    Returns
    -------
    list[dict]
        Points as ``[{"x": 1.0, "y": 2.0, "z": 3.0}, ...]``.  Empty list if
        the file declares zero points or contains no data rows.

    Raises
    ------
    FileNotFoundError
        If ``path`` does not exist or cannot be opened.
    ValueError
        If the ``DATA`` header declares an unsupported encoding (neither
        ``ascii`` nor ``binary``), or if the ``FIELDS`` header lacks an ``x``,
        ``y`` or ``z`` column.
    """
    fields: List[str] = []
    field_sizes: List[int] = []
    field_types: List[str] = []
    data_mode: str | None = None
    points: List[Dict[str, float]] = []
    idx_x = idx_y = idx_z = -1

    # 二进制模式打开：HEADER 逐行 decode 为 ASCII，DATA 段按需原样读取
    with open(path, "rb") as fh:
        # ── header section ────────────────────────────────────────────────
        while data_mode is None:
            raw_line = fh.readline()
            if not raw_line:
                raise ValueError("PCD 文件缺少 DATA 头字段")
            line = raw_line.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            if key == "FIELDS":
                fields = value.split()
            elif key == "SIZE":
                field_sizes = [int(s) for s in value.split()]
            elif key == "TYPE":
                field_types = value.split()
            elif key == "DATA":
                data_mode = value.strip()

        # FIELDS 先于 DATA 出现（PCD 规范）；此时列索引已确定
        for required in ("x", "y", "z"):
            if required not in fields:
                raise ValueError("点云数据缺少坐标字段")
        idx_x = fields.index("x")
        idx_y = fields.index("y")
        idx_z = fields.index("z")

        if data_mode == "binary":
            # ── binary data section ───────────────────────────────────────
            raw = fh.read()
            point_bytes = sum(field_sizes)
            if point_bytes <= 0 or len(field_sizes) != len(fields):
                raise ValueError("PCD 文件 SIZE 头字段缺失或与 FIELDS 不匹配")
            fmt_parts: List[str] = []
            for ftype, fsize in zip(field_types, field_sizes):
                fmt_char = _PCD_TYPE_TO_FMT.get(ftype.upper())
                if fmt_char is None:
                    raise ValueError(f"不支持的点云字段 TYPE: {ftype!r}")
                if struct.calcsize(fmt_char) != fsize:
                    raise ValueError(
                        f"点云字段 TYPE {ftype!r} 与 SIZE {fsize} 不匹配"
                    )
                fmt_parts.append(fmt_char)
            fmt_string = "<" + "".join(fmt_parts)

            count = len(raw) // point_bytes
            for i in range(count):
                values = struct.unpack_from(fmt_string, raw, i * point_bytes)
                points.append(
                    {
                        "x": values[idx_x],
                        "y": values[idx_y],
                        "z": values[idx_z],
                    }
                )
            logger.debug(
                "Parsed %d points (%s binary, %d bytes/point) from %s",
                len(points), fmt_string, point_bytes, path,
            )
        elif data_mode == "ascii":
            # ── ascii data section ────────────────────────────────────────
            for raw_line in fh:
                line = raw_line.decode("utf-8").strip()
                if not line or line.startswith("#"):
                    continue
                tokens = line.split()
                try:
                    points.append(
                        {
                            "x": float(tokens[idx_x]),
                            "y": float(tokens[idx_y]),
                            "z": float(tokens[idx_z]),
                        }
                    )
                except (IndexError, ValueError) as exc:
                    raise ValueError(
                        f"PCD 数据行无法解析: {line!r}"
                    ) from exc
            logger.debug("Parsed %d points (ascii) from %s", len(points), path)
        else:
            raise ValueError(f"仅支持 ASCII 或 binary 格式，收到: {data_mode!r}")

    return points
