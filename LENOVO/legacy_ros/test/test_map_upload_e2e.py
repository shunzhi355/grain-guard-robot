"""端到端测试：mock cloud server + CloudHttpClient + MapDataUploader 全链路。

在测试线程中启动真实 ``mock_cloud_server``（HTTP），构造 PCD 文件，
全链路 ``build_request → upload → 验证 mock server 落盘 + 归一化正确``。
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import threading
import time
from http.server import HTTPServer
from pathlib import Path
from unittest.mock import patch

import pytest

# ── 确保 mock_cloud_server 可导入 ─────────────────────────────────────
# conftest.py 已把 src/ 加入 sys.path，但 mock_cloud_server.py 在项目根目录。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import mock_cloud_server
from mock_cloud_server import MockCloudHandler

from grain_sampling_cloud.http_client import CloudHttpClient
from grain_sampling_pointcloud.map_data_uploader import MapDataUploader


# ═══════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════

def _ascii_pcd(points: list[tuple[float, float, float]]) -> str:
    """Build an ASCII PCD file body from ``(x, y, z)`` tuples."""
    lines = [
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    lines += [f"{x} {y} {z}" for x, y, z in points]
    return "\n".join(lines) + "\n"


# 廒间四壁点（z=1.0 层）+ 其他高度杂点
_AOJIAN_POINTS = [
    # z=1.0 层 — 四壁角点 (形成 2m×2m 正方形)
    (1.0, 1.0, 1.0),
    (3.0, 1.0, 1.0),
    (3.0, 3.0, 1.0),
    (1.0, 3.0, 1.0),
    # z=1.0 内部点
    (2.0, 2.0, 1.0),
    # 杂点 — 其他高度
    (100.0, 100.0, 0.5),
    (-50.0, -20.0, 5.0),
]


# ═══════════════════════════════════════════════════════════════════════
# Fixtures
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="module")
def mock_server():
    """启动 mock_cloud_server 为后台线程（随机端口），teardown 关闭 + 清理。"""
    # ── 重定向 _DATA_DIR 到临时目录（避免污染 mock_server_data/） ───────
    tmpdir = tempfile.TemporaryDirectory()
    original_data_dir = mock_cloud_server._DATA_DIR
    mock_cloud_server._DATA_DIR = tmpdir.name

    server = HTTPServer(("127.0.0.1", 0), MockCloudHandler)
    port = server.server_port
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    time.sleep(0.15)  # 等待 server 就绪

    yield port

    server.shutdown()
    t.join(timeout=2)
    mock_cloud_server._DATA_DIR = original_data_dir
    tmpdir.cleanup()


@pytest.fixture(scope="module")
def _server_port():
    """Autouse fixture: ensure nothing else runs until server is ready."""
    pass


@pytest.fixture
def pcd_file():
    """创建临时 ASCII PCD 文件（含廒间四壁点 + 杂点）。"""
    fd, pcd_path = tempfile.mkstemp(suffix=".pcd", text=True)
    with os.fdopen(fd, "w") as f:
        f.write(_ascii_pcd(_AOJIAN_POINTS))
    yield pcd_path
    try:
        os.unlink(pcd_path)
    except OSError:
        pass


@pytest.fixture
def cloud_client(mock_server):
    """构造指向 mock server 的 CloudHttpClient（绕过 from_app_config 占位符检查）。"""
    return CloudHttpClient({"base_url": f"http://127.0.0.1:{mock_server}"})


# ═══════════════════════════════════════════════════════════════════════
# Test
# ═══════════════════════════════════════════════════════════════════════

def test_e2e_upload_boundary(mock_server, pcd_file, cloud_client):
    """全链路：build_request → upload → 验证 mock server 落盘 + 归一化正确。"""
    mac = "AA:BB:CC:DD:EE:FF"
    aojian_id = "aojian-test-01"
    aojian = "测试廒间"

    # ── 1. build_request ─────────────────────────────────────────────────
    uploader = MapDataUploader()
    request = uploader.build_request(
        pcd_file, height=1.0, mac=mac, aojian_id=aojian_id, aojian=aojian,
    )
    assert request is not None, "build_request 应返回非 None（非空切面）"
    assert len(request.boundary) > 0, "应有边界点"

    # ── 2. upload（注入 client 替代 from_app_config） ──────────────────────
    with patch.object(CloudHttpClient, "from_app_config", return_value=cloud_client):
        result = uploader.upload(request)
        assert result is True, "upload 应返回 True（上传成功）"

    # ── 3. 验证 mock 落盘文件 ────────────────────────────────────────────
    data_dir = mock_cloud_server._DATA_DIR
    map_files = sorted(
        (f for f in os.listdir(data_dir) if f.startswith("map_") and f.endswith(".json")),
        reverse=True,
    )
    assert len(map_files) > 0, "应有至少一个 map_*.json 落盘文件"

    filepath = os.path.join(data_dir, map_files[0])
    with open(filepath, "r", encoding="utf-8") as f:
        saved = json.load(f)

    # ── 4. 验证字段完整性 ───────────────────────────────────────────────
    assert "boundary" in saved, "保存的数据应包含 boundary 字段"
    boundary = saved["boundary"]
    assert isinstance(boundary, list) and len(boundary) > 0, "boundary 应为非空列表"

    assert "map_bounds" in saved, "保存的数据应包含 map_bounds 字段"
    map_bounds = saved["map_bounds"]

    # ── 5. 验证归一化：所有坐标 x >= 0, y >= 0 ───────────────────────────
    for pt in boundary:
        assert pt["x"] >= 0, f"归一化后所有 x >= 0，但得到 x={pt['x']}"
        assert pt["y"] >= 0, f"归一化后所有 y >= 0，但得到 y={pt['y']}"

    # ── 6. 验证 map_bounds 正确 ──────────────────────────────────────────
    assert map_bounds["xmin"] == 0.0, f"xmin 应为 0，实际 {map_bounds['xmin']}"
    assert map_bounds["ymin"] == 0.0, f"ymin 应为 0，实际 {map_bounds['ymin']}"
    assert map_bounds["xmax"] > map_bounds["xmin"], "xmax > xmin"
    assert map_bounds["ymax"] > map_bounds["ymin"], "ymax > ymin"

    # ── 7. 验证其他字段 ─────────────────────────────────────────────────
    assert saved.get("mac") == mac, f"mac 应匹配，实际 {saved.get('mac')}"
    assert saved.get("aojian_id") == aojian_id, f"aojian_id 应匹配，实际 {saved.get('aojian_id')}"
    assert saved.get("aojian") == aojian, f"aojian 应匹配，实际 {saved.get('aojian')}"
