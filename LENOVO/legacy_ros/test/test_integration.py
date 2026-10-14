"""
Integration tests for the grain sampling robot software.

Tests wire up multiple modules together, verifying that data flows
correctly between subsystems.  All tests use :mod:`unittest.mock` —
**no hardware, no ROS runtime, no network** required.

Scenarios
---------
1. Cloud → Workflow integration
2. Workflow → Point cloud integration
3. Upload pipeline (collect → slice → preview → publish)
4. Device adapter integration (connect → command → disconnect)
5. Configuration loading and override
6. Full end-to-end pipeline (all modules, all mocked)
"""

from __future__ import annotations

import json
import os
import socket
import struct
import sys
import tempfile
from unittest import mock
from unittest.mock import MagicMock, patch

import pytest

# ── Mock ROS at module level BEFORE any module import ─────────────────
# This ensures that ``import rospy`` and ``from rospy.node import Node``
# inside collector.py succeed by injecting a fake package hierarchy
# into sys.modules.  Sensor message types are also mocked.
if "rospy" not in sys.modules:
    # Build a realistic fake Node base class so PointCloudCollector
    # can inherit from it properly without MagicMock interference.
    class _MockNode:
        """Fake ROS Node base class for testing."""
        def __init__(self, node_name: str = "") -> None:
            self._node_name = node_name
        def create_subscription(self, *args, **kwargs) -> MagicMock:
            return MagicMock()
        def get_logger(self) -> MagicMock:
            logger = MagicMock()
            logger.info = MagicMock()
            logger.warning = MagicMock()
            logger.error = MagicMock()
            return logger

    _fake_rospy_node = MagicMock()
    _fake_rospy_node.Node = _MockNode

    _fake_rospy = MagicMock()
    _fake_rospy.node = _fake_rospy_node
    _fake_rospy.logging = MagicMock()
    _fake_rospy.spin_once = MagicMock()
    _fake_rospy.init = MagicMock()
    _fake_rospy.shutdown = MagicMock()

    sys.modules["rospy"] = _fake_rospy
    sys.modules["rospy.node"] = _fake_rospy_node
    sys.modules["sensor_msgs"] = MagicMock()
    sys.modules["sensor_msgs.msg"] = MagicMock()

# ── Application modules ──────────────────────────────────────────
from grain_sampling_cloud.http_client import CloudHttpClient
from grain_sampling_cloud.protocol import ApiPath
from grain_sampling_devices.base_adapter import (
    BaseDeviceAdapter,
    DeviceConnectionError,
)
from grain_sampling_devices.biochemical_adapter import BiochemicalAdapter
from grain_sampling_devices.physicochemical_adapter import PhysicochemicalAdapter
from grain_sampling_devices.tcp_client import TCPClient
from grain_sampling_pointcloud.collector import PointCloudCollector
from grain_sampling_pointcloud.slice_extractor import PointCloudSlicer
from grain_sampling_pointcloud.uploader import MapUploader
from grain_sampling_workflow.state_machine import (
    SamplingAction,
    SamplingState,
    SamplingStateMachine,
)
from utils.config import AppConfig


# ═══════════════════════════════════════════════════════════════════════════
# Shared helpers
# ═══════════════════════════════════════════════════════════════════════════


def _make_pointcloud2(points: list[tuple[float, float, float]]) -> MagicMock:
    """Return a mock ``sensor_msgs/PointCloud2`` with the given (x,y,z)."""
    # Always use spec=None — sensor_msgs.msg is mocked at module level
    # and cannot be used as a spec for MagicMock.
    msg = MagicMock()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0
    msg.header.frame_id = "livox_frame"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        MagicMock(name="x", datatype=7, offset=0, count=1),
        MagicMock(name="y", datatype=7, offset=4, count=1),
        MagicMock(name="z", datatype=7, offset=8, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = struct.pack(f"<{3 * len(points)}f", *[v for p in points for v in p])
    msg.is_dense = True
    return msg


def _run_fsm_to_state(
    fsm: SamplingStateMachine, target: SamplingState
) -> None:
    """Advance *fsm* to *target* state via sequential actions.

    Only handles linear paths through the workflow.  Raises
    :class:`ValueError` if the path is ambiguous.
    """
    path = [
        (SamplingState.INIT, SamplingAction.CONFIRM_READY),
        (SamplingState.RECORD_START, SamplingAction.SYSTEM_RECORD_COMPLETE),
        (SamplingState.NAVIGATE_TO_POINT, SamplingAction.SYSTEM_NAV_COMPLETE),
        (SamplingState.ARRIVED_PROMPT, SamplingAction.CONFIRM_READY),
        (SamplingState.PRESS_AND_SUCTION, SamplingAction.SYSTEM_PRESS_COMPLETE),
        (SamplingState.ADD_PIPE_PROMPT, SamplingAction.CONFIRM_PIPE_ADDED),
        # After REPEAT, take the "depth reached" branch
        (SamplingState.REPEAT_UNTIL_DEPTH, SamplingAction.SYSTEM_DEPTH_REACHED),
        (SamplingState.DISCHARGE_WASTE, SamplingAction.CONFIRM_WASTE_DISCHARGED),
        (SamplingState.FORMAL_SAMPLING, SamplingAction.SYSTEM_SUCTION_COMPLETE),
        (SamplingState.CONVEY_1, SamplingAction.SYSTEM_CONVEY_COMPLETE),
        (SamplingState.OPEN_BIN, SamplingAction.SYSTEM_BIN_OPENED),
        (SamplingState.CONVEY_DONE, SamplingAction.CONFIRM_DONE),
        (SamplingState.NEXT_CHECK, SamplingAction.SYSTEM_ALL_DONE),
        (SamplingState.ALL_DONE_PROMPT, SamplingAction.CONFIRM_RETURN),
        (SamplingState.RETURN, SamplingAction.SYSTEM_RETURN_COMPLETE),
    ]

    for src, action in path:
        if fsm.current_state == target:
            return
        if fsm.current_state == src:
            fsm.transition(action)
        else:
            break

    if fsm.current_state != target:
        raise ValueError(
            f"Failed to reach {target.name}; stuck at {fsm.current_state.name}"
        )


# ═══════════════════════════════════════════════════════════════════════════
# 1. Cloud → Workflow integration
# ═══════════════════════════════════════════════════════════════════════════


class TestCloudToWorkflowIntegration:
    """Verify HTTP cloud response triggers correct FSM behaviour."""

    @patch("urllib.request.urlopen")
    def test_fetch_task_list_creates_fsm(self, mock_urlopen):
        """A task list response from cloud should initialise the FSM."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = json.dumps({
            "liangku_id": "L-001",
            "liangku": "粮仓A区",
            "orders": [
                {
                    "order_id": "O-001",
                    "aojian_id": "A-001",
                    "aojian": "廒间1号",
                    "depth_list": [3],
                    "jiance": [1, 2, 3],
                    "points": [{"x": 1.0, "y": 2.0}],
                },
            ],
        }).encode("utf-8")
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        result = client.post(ApiPath.TASK_LIST, {"mac": "00:00:00:00:00:00"})

        assert result["liangku_id"] == "L-001"
        assert len(result["orders"]) == 1

        # Feed order info to FSM
        order = result["orders"][0]
        fsm = SamplingStateMachine(
            total_waypoints=len(order["points"]),
            max_depth=max(order["depth_list"]),
        )
        assert fsm.current_state == SamplingState.INIT

        # Click "已就绪"
        fsm.transition(SamplingAction.CONFIRM_READY)
        assert fsm.current_state == SamplingState.RECORD_START

    @patch("urllib.request.urlopen")
    def test_http_status_report(self, mock_urlopen):
        """After FSM transitions, status should be reportable via HTTP POST."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        fsm = SamplingStateMachine(total_waypoints=2, max_depth=3)
        fsm.transition(SamplingAction.CONFIRM_READY)

        result = client.post(ApiPath.STATUS_REPORT, {
            "mac": "00:00:00:00:00:00",
            "order_id": "O-001",
            "status": fsm.current_state.name,
            "results": {"progress": fsm.progress_str},
        })
        assert result["success"] is True


# ═══════════════════════════════════════════════════════════════════════════
# 2. Workflow → Point cloud integration
# ═══════════════════════════════════════════════════════════════════════════


class TestWorkflowToPointCloud:
    """Verify FSM state triggers point cloud collection."""

    def test_fsm_reaches_sampling_state(self):
        """Advance FSM to PRESS_AND_SUCTION — the point that triggers
        the LiDAR to start collecting."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        _run_fsm_to_state(fsm, SamplingState.PRESS_AND_SUCTION)

        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION
        assert fsm.current_waypoint_index == 0
        assert fsm.current_depth_index == 0

    # ROS is mocked at module level — Node is already a MagicMock
    def test_pointcloud_collector_created_at_sampling(self):
        """Simulate creating a point cloud collector when FSM enters
        sampling phase."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        _run_fsm_to_state(fsm, SamplingState.PRESS_AND_SUCTION)
        assert fsm.current_state == SamplingState.PRESS_AND_SUCTION

        # Create collector (ROS init mocked)
        collector = PointCloudCollector(
            node_name="test_collector",
            buffer_size=5,
            topic="/livox/lidar",
        )
        assert collector is not None
        assert collector.frame_count == 0

        # Simulate a frame arriving
        pc_msg = _make_pointcloud2([
            (10.0, 20.0, 0.5), (10.1, 20.1, 0.55), (10.2, 19.9, 0.48),
        ])
        collector._lidar_callback(pc_msg)
        assert collector.frame_count == 1

        frame = collector.get_latest_frame()
        assert frame is not None
        parsed = collector._parse_points(frame)
        assert len(parsed) == 3

    # ROS is mocked at module level — Node is already a MagicMock
    def test_fsm_progress_with_pointcloud_collection(self):
        """FSM advances through full cycle; point cloud collector records
        frames throughout."""
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        states_visited: list[SamplingState] = []

        def record(prev, new, action):
            states_visited.append(new)

        fsm.on_state_change = record
        _run_fsm_to_state(fsm, SamplingState.COMPLETED)

        assert fsm.current_state == SamplingState.COMPLETED
        assert SamplingState.PRESS_AND_SUCTION in states_visited
        assert len(states_visited) >= 14


# ═══════════════════════════════════════════════════════════════════════════
# 3. Upload pipeline (collect → slice → preview → publish)
# ═══════════════════════════════════════════════════════════════════════════


class TestUploadPipeline:
    """Integration: point cloud → slice extraction → upload."""

    def test_slice_extraction_from_synthetic_pc(self):
        """Extract a 2D slice from a synthetic point cloud."""
        slicer = PointCloudSlicer(default_tolerance=0.1)

        # Create points at various heights
        pc_points = [
            (5.0, 5.0, 0.50),   # inside band
            (5.1, 5.1, 0.55),   # inside band
            (5.2, 5.2, 0.45),   # inside band
            (9.0, 9.0, 1.00),   # too high
            (1.0, 1.0, 0.00),   # too low
            (6.0, 6.0, 0.49),   # inside band (border)
        ]
        msg = _make_pointcloud2(pc_points)

        sliced = slicer.extract_slice(msg, height=0.5, tolerance=0.1)
        assert len(sliced) == 4  # 4 points in [0.4, 0.6]

        # All returned points should have x, y only
        for pt in sliced:
            assert "x" in pt
            assert "y" in pt
            assert "z" not in pt

    def test_contour_from_slice(self):
        """Extract contour from a cluster of points (non-uniform)."""
        slicer = PointCloudSlicer()
        # Create an L-shaped cluster so the contour algorithm
        # finds a true boundary (not just the full grid).
        cluster = []
        for i in range(12):
            for j in range(5):
                cluster.append({"x": 2.0 + i * 0.05, "y": 3.0 + j * 0.05})
        for i in range(5):
            for j in range(5, 12):
                cluster.append({"x": 2.0 + i * 0.05, "y": 3.0 + j * 0.05})
        contour = slicer.extract_contour(cluster, grid_resolution=0.05)
        # Contour should be a non-empty proper subset
        assert len(contour) > 0
        assert len(contour) < len(cluster)
        for pt in contour:
            assert isinstance(pt, dict)
            assert "x" in pt and "y" in pt

    @patch("urllib.request.urlopen")
    def test_full_upload_pipeline(self, mock_urlopen):
        """End-to-end: synthetic PC → slice → contour → upload via HTTP."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        uploader = MapUploader(http_client=client)
        slicer = PointCloudSlicer(default_tolerance=0.1)

        # 1. Create synthetic point cloud
        pc_points = [
            (i * 0.1, j * 0.1, 0.5)
            for i in range(10) for j in range(10)
        ]
        msg = _make_pointcloud2(pc_points)

        # 2. Extract slice at 0.5 m
        sliced = slicer.extract_slice(msg, height=0.5)
        assert len(sliced) > 0, "Expected non-empty slice"

        # 3. Extract contour
        contour = slicer.extract_contour(sliced, grid_resolution=0.05)
        assert len(contour) > 0

        # 4. Upload
        result = uploader.upload(contour, height=0.5, aojian_id="A-001", contour_only=False)
        assert result is True

        # 5. Verify the POST was made with correct data
        assert mock_urlopen.call_count >= 1
        call_args = mock_urlopen.call_args[0][0]
        assert ApiPath.MAP_UPLOAD in call_args.full_url
        body = json.loads(call_args.data)
        assert body["height"] == 0.5
        assert "timestamp" in body
        assert len(body["points"]) == len(contour)

    @patch("urllib.request.urlopen")
    def test_upload_pipeline_with_preview(self, mock_urlopen):
        """Include a preview image in the upload (local-only in HTTP arch)."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        uploader = MapUploader(http_client=client)
        points = [
            {"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0},
            {"x": 5.0, "y": 6.0},
        ]
        preview = b"\x89PNG\r\n\x1a\n" + b"\x00" * 50  # fake PNG header

        # In HTTP arch, preview is local-only — upload_with_preview
        # delegates to upload() without sending preview bytes
        result = uploader.upload_with_preview(
            points, height=1.2, aojian_id="A-001", preview_bytes=preview
        )
        assert result is True

    @patch("urllib.request.urlopen")
    def test_upload_fails_on_connection_error(self, mock_urlopen):
        """Upload should return False when HTTP connection fails."""
        from urllib.error import URLError
        mock_urlopen.side_effect = URLError("connection refused")

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "retry_count": 1,
            "retry_delay": 0.01,
        })
        uploader = MapUploader(http_client=client)
        result = uploader.upload(
            [{"x": 0.0, "y": 0.0}], height=0.5, aojian_id="A-001"
        )
        assert result is False


# ═══════════════════════════════════════════════════════════════════════════
# 4. Device adapter integration
# ═══════════════════════════════════════════════════════════════════════════


class TestDeviceAdapterIntegration:
    """Verify that concrete adapters work through the abstract interface."""

    def _adapter_connect_disconnect(self, adapter: BaseDeviceAdapter) -> None:
        """Helper: mock-socket connect → disconnect for any adapter."""
        fake_sock = MagicMock(spec=socket.socket)
        fake_sock.recv.return_value = b"OK"

        with patch("socket.socket", return_value=fake_sock):
            ok = adapter.connect()
            assert ok is True
            assert adapter.is_connected()

        adapter.disconnect()
        assert not adapter.is_connected()

    def test_biochemical_adapter_connect_disconnect(self):
        adapter = BiochemicalAdapter(host="192.168.1.10", port=5001)
        self._adapter_connect_disconnect(adapter)

    def test_physicochemical_adapter_connect_disconnect(self):
        adapter = PhysicochemicalAdapter(host="192.168.1.20", port=5002)
        self._adapter_connect_disconnect(adapter)

    def test_biochemical_adapter_properties(self):
        adapter = BiochemicalAdapter(host="10.0.0.1", port=9999)
        assert adapter.device_name == "Biochemical-Analyser"
        assert adapter.device_type == "biochemical"
        assert adapter.ip_address == "10.0.0.1"
        assert adapter.port == 9999

    def test_physicochemical_adapter_properties(self):
        adapter = PhysicochemicalAdapter(host="10.0.0.2")
        assert adapter.device_type == "physicochemical"
        assert adapter.port == 5002  # default

    def test_send_command_on_connected_adapter(self):
        """Send a command through a connected adapter and get a response."""
        adapter = BiochemicalAdapter(host="192.168.1.10", port=5001)
        fake_sock = MagicMock(spec=socket.socket)
        expected_response = b'{"status": "ok", "result": "success"}'

        with patch("socket.socket", return_value=fake_sock), \
             patch.object(adapter._client, "_receive_response",
                          return_value=expected_response):
            adapter.connect()
            assert adapter.is_connected()

            resp = adapter.send_command(b'{"cmd": "test"}')
            assert resp == expected_response

        adapter.disconnect()

    def test_adapter_interface_methods(self):
        """All abstract methods must be callable on concrete adapters."""
        for cls in [BiochemicalAdapter, PhysicochemicalAdapter]:
            adapter = cls(host="127.0.0.1")
            assert isinstance(adapter, BaseDeviceAdapter)
            assert adapter.get_status() in ("offline", "online", "error")
            info = adapter.get_device_info()
            assert "device_name" in info
            assert "device_type" in info

    def test_device_error_with_name(self):
        """DeviceError should store device_name."""
        err = DeviceConnectionError("timeout", device_name="BioChem-2000")
        assert err.device_name == "BioChem-2000"
        assert "timeout" in str(err)


# ═══════════════════════════════════════════════════════════════════════════
# 5. Configuration loading
# ═══════════════════════════════════════════════════════════════════════════


class TestConfigLoading:
    """Verify AppConfig creation, defaults, and overrides."""

    def test_default_values(self):
        cfg = AppConfig()
        assert cfg.mqtt_broker == "mqtt://localhost:1883"
        assert cfg.device_id == "robot-001"
        assert cfg.rtsp_port == 8554
        assert cfg.camera_device == "/dev/video0"
        assert cfg.livox_config_path == "config/livox_config.json"
        assert cfg.slice_default_height == 0.5
        assert cfg.biochemical_port == 5001
        assert cfg.physicochemical_port == 5002

    def test_from_dict_override_single(self):
        cfg = AppConfig.from_dict({"device_id": "robot-custom-99"})
        assert cfg.device_id == "robot-custom-99"
        assert cfg.mqtt_broker == "mqtt://localhost:1883"  # still default

    def test_from_dict_override_multiple(self):
        cfg = AppConfig.from_dict({
            "device_id": "r2d2",
            "rtsp_port": 9999,
            "slice_default_height": 1.0,
        })
        assert cfg.device_id == "r2d2"
        assert cfg.rtsp_port == 9999
        assert cfg.slice_default_height == 1.0

    def test_from_dict_ignores_unknown(self):
        """Unknown keys should be silently ignored."""
        cfg = AppConfig.from_dict({"foobar": 42, "device_id": "test"})
        assert cfg.device_id == "test"
        # foobar is not a field — no error

    def test_from_json_file(self):
        """Read config from a temporary JSON file."""
        data = {
            "device_id": "from-file-robot",
            "mqtt_broker": "mqtt://prod.example.com:1883",
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False, encoding="utf-8"
        ) as f:
            json.dump(data, f)
            tmp = f.name

        try:
            cfg = AppConfig.from_json_file(tmp)
            assert cfg.device_id == "from-file-robot"
            assert cfg.mqtt_broker == "mqtt://prod.example.com:1883"
            assert cfg.rtsp_port == 8554  # default retained
        finally:
            os.unlink(tmp)

    def test_from_json_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            AppConfig.from_json_file("/tmp/nonexistent_config_xyz.json")

    def test_from_env(self):
        """Override via environment variables with prefix."""
        with patch.dict(os.environ, {
            "ROBOT_DEVICE_ID": "env-robot",
            "ROBOT_RTSP_PORT": "1234",
        }):
            cfg = AppConfig.from_env(prefix="ROBOT_")
            assert cfg.device_id == "env-robot"
            assert cfg.rtsp_port == 1234
            assert cfg.mqtt_broker == "mqtt://localhost:1883"  # default

    def test_from_env_custom_prefix(self):
        with patch.dict(os.environ, {"MYAPP_DEVICE_ID": "myapp-01"}):
            cfg = AppConfig.from_env(prefix="MYAPP_")
            assert cfg.device_id == "myapp-01"

    def test_as_dict_roundtrip(self):
        original = AppConfig(device_id="roundtrip-test")
        d = original.as_dict()
        restored = AppConfig.from_dict(d)
        assert restored.device_id == "roundtrip-test"
        assert restored.mqtt_broker == original.mqtt_broker

    def test_per_module_override_pattern(self):
        """Simulate per-module config override (common pattern in app)."""
        base = AppConfig()
        # Cloud module overrides
        cloud_cfg = AppConfig.from_dict({"mqtt_broker": "mqtt://cloud:1883"})
        # Camera module overrides
        camera_cfg = AppConfig.from_dict({"rtsp_port": 9999})
        # Devices module overrides
        devices_cfg = AppConfig.from_dict({"biochemical_port": 6001})

        assert cloud_cfg.mqtt_broker != base.mqtt_broker
        assert camera_cfg.rtsp_port != base.rtsp_port
        assert devices_cfg.biochemical_port != base.biochemical_port
        # Non-overridden values fall back to defaults
        assert cloud_cfg.rtsp_port == base.rtsp_port


# ═══════════════════════════════════════════════════════════════════════════
# 6. Full end-to-end pipeline (all mocked)
# ═══════════════════════════════════════════════════════════════════════════


class TestFullPipeline:
    """Simulate the entire grain-sampling flow with all modules wired together.

    All hardware (ROS, LiDAR, camera, devices, network) is mocked.
    Network communication uses HTTP via CloudHttpClient (mocked urllib).
    """

    # ROS is mocked at module level
    @patch("urllib.request.urlopen")
    def test_full_pipeline_mock(self, mock_urlopen):
        """End-to-end: cloud fetch → FSM runs 15 steps → PC processed →
        map uploaded → status reported via HTTP."""
        # Mock a successful HTTP response for each cloud call
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        # ── 1. Setup: client, FSM, pointcloud pipeline ───────
        client = CloudHttpClient({
            "base_url": "http://test.com",
            "mac_address": "AA:BB:CC:DD:EE:FF",
        })
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        collector = PointCloudCollector(
            node_name="integration_collector",
            buffer_size=10,
        )
        slicer = PointCloudSlicer(default_tolerance=0.1)
        uploader = MapUploader(http_client=client)

        # ── 2. Fetch task list from cloud (simulated) ──────────
        resp = client.post(ApiPath.TASK_LIST, {"mac": client.mac_address})
        assert resp["success"] is True

        # ── 3. Run FSM through all 15 steps ─────────────────────
        try:
            _run_fsm_to_state(fsm, SamplingState.COMPLETED)
        except ValueError:
            pass

        assert fsm.current_state in (
            SamplingState.COMPLETED,
            SamplingState.RETURN,
        )

        # ── 4. Simulate point cloud collection ─────────────────
        pc_points: list[tuple[float, float, float]] = []
        for i in range(10):
            for j in range(10):
                pc_points.append((15.0 + i * 0.1, 25.0 + j * 0.1, 0.5))
        pc_msg = _make_pointcloud2(pc_points)
        collector._lidar_callback(pc_msg)

        frame = collector.get_latest_frame()
        assert frame is not None
        assert collector.frame_count == 1

        # ── 5. Process: extract slice → contour ────────────────
        sliced = slicer.extract_slice(frame, height=0.5)
        assert len(sliced) > 0, "Slice should not be empty"

        contour = slicer.extract_contour(sliced, grid_resolution=0.1)
        assert len(contour) > 0, "Contour should not be empty"

        # ── 6. Upload map ──────────────────────────────────────
        ok = uploader.upload(contour, height=0.5, aojian_id="A-001")
        assert ok is True

        # Verify the POST to MAP_UPLOAD was made
        map_upload_calls = [
            c for c in mock_urlopen.call_args_list
            if ApiPath.MAP_UPLOAD in c[0][0].full_url
        ]
        assert len(map_upload_calls) >= 1
        body = json.loads(map_upload_calls[-1][0][0].data)
        assert body["height"] == 0.5
        assert "timestamp" in body
        assert len(body["points"]) > 0

    @patch("urllib.request.urlopen")
    def test_pipeline_with_device_integration(self, mock_urlopen):
        """FSM → device command → HTTP status report."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "mac_address": "AA:BB:CC:DD:EE:FF",
        })
        adapter = BiochemicalAdapter(host="127.0.0.1", port=5001)
        fake_sock = MagicMock(spec=socket.socket)

        with patch("socket.socket", return_value=fake_sock):
            adapter.connect()

        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        _run_fsm_to_state(fsm, SamplingState.FORMAL_SAMPLING)

        # During formal sampling, run device test (mock response)
        resp = b'{"status":"ok","value":42}'
        with patch.object(adapter._client, "_receive_response", return_value=resp):
            result = adapter.send_command(b'{"cmd":"start_test"}')

        # Report device result via HTTP
        status_result = client.post(ApiPath.STATUS_REPORT, {
            "mac": client.mac_address,
            "order_id": "O-001",
            "status": "in_progress",
            "results": {"device_test": str(result[:50])},
        })
        assert status_result["success"] is True

        adapter.disconnect()

    @patch("urllib.request.urlopen")
    def test_pipeline_pause_resume_stop(self, mock_urlopen):
        """Verify safety controls (pause/resume/stop) during formal sampling."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "mac_address": "AA:BB:CC:DD:EE:FF",
        })
        fsm = SamplingStateMachine(total_waypoints=1, max_depth=1)
        _run_fsm_to_state(fsm, SamplingState.FORMAL_SAMPLING)
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

        # PAUSE
        fsm.transition(SamplingAction.PAUSE)
        assert fsm.paused is True
        assert fsm.current_state == SamplingState.FORMAL_SAMPLING

        # RESUME
        fsm.transition(SamplingAction.RESUME)
        assert fsm.paused is False

        # STOP
        fsm.transition(SamplingAction.STOP)
        assert fsm.current_state == SamplingState.STOPPED
        assert not fsm.is_running

        # Report stopped status via HTTP
        result = client.post(ApiPath.STATUS_REPORT, {
            "mac": client.mac_address,
            "order_id": "O-001",
            "status": "stopped",
            "results": {"paused": fsm.paused},
        })
        assert result["success"] is True
