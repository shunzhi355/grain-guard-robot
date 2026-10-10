"""
Unit tests for the HTTP cloud client module.

Tests cover:
    - MAC detection
    - Client configuration and initialization
    - POST request/response (with mocked urllib)
    - GET request/response
    - Retry logic on transient failures
    - Error handling (auth, connection, protocol)
    - Timeout handling
    - is_connected health check
"""

import json
from unittest.mock import MagicMock, patch

import pytest

from grain_sampling_cloud.http_client import (
    CloudHttpClient,
    CloudConnectionError,
    CloudAuthError,
    CloudProtocolError,
    detect_mac,
)
from grain_sampling_cloud.protocol import (
    TaskListRequest,
    TaskListResponse,
    OrderInfo,
    AcceptTaskRequest,
    AcceptTaskResponse,
    StatusReportRequest,
    StatusReportResponse,
    DetectionResult,
    RoomListRequest,
    MapUploadRequest,
    ApiPath,
    CloudErrorCode,
    ReportStatus,
)


# ======================================================================
# detect_mac
# ======================================================================


class TestMacDetection:
    """Tests for MAC address detection."""

    def test_detect_mac_returns_string(self) -> None:
        mac = detect_mac()
        assert isinstance(mac, str)
        assert len(mac) > 0

    def test_detect_mac_format(self) -> None:
        mac = detect_mac()
        # Should be colon-separated hex pairs, or fallback "00:00:00:00:00:00"
        parts = mac.split(":")
        assert len(parts) == 6
        for part in parts:
            assert len(part) == 2
            int(part, 16)  # should not raise


# ======================================================================
# CloudHttpClient
# ======================================================================


class TestCloudHttpClient:
    """Tests for CloudHttpClient configuration and basic operations."""

    def test_default_config(self) -> None:
        """Should use DEFAULT base_url when no config is provided."""
        client = CloudHttpClient()
        assert client.base_url == "http://cloud-api.xxx.com"
        assert isinstance(client.mac_address, str)
        assert len(client.mac_address) > 0

    def test_custom_config(self) -> None:
        """Should apply custom config values."""
        client = CloudHttpClient({
            "base_url": "http://mycloud.example.com",
            "timeout": 5.0,
            "retry_count": 2,
            "mac_address": "AA:BB:CC:DD:EE:FF",
        })
        assert client.base_url == "http://mycloud.example.com"
        assert client.mac_address == "AA:BB:CC:DD:EE:FF"

    @patch("urllib.request.urlopen")
    def test_post_success(self, mock_urlopen: MagicMock) -> None:
        """Successful POST should return parsed JSON."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"liangku_id": "e01", "liangku": "test"}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        result = client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})
        assert result["liangku_id"] == "e01"
        assert result["liangku"] == "test"

    @patch("urllib.request.urlopen")
    def test_post_injects_mac(self, mock_urlopen: MagicMock) -> None:
        """POST should auto-inject mac if not provided in data."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "mac_address": "11:22:33:44:55:66",
        })
        result = client.post("/api/test", {"key": "val"})
        assert isinstance(result, dict)
        # Verify mac was injected
        call_data = json.loads(mock_urlopen.call_args[0][0].data)
        assert call_data["mac"] == "11:22:33:44:55:66"
        assert call_data["key"] == "val"

    @patch("urllib.request.urlopen")
    def test_post_auth_error(self, mock_urlopen: MagicMock) -> None:
        """401 should raise CloudAuthError."""
        from urllib.error import HTTPError
        mock_urlopen.side_effect = HTTPError(
            "http://test.com/api", 401, "Unauthorized", {}, None
        )

        client = CloudHttpClient({"base_url": "http://test.com", "retry_count": 0})
        with pytest.raises(CloudAuthError, match="Authentication failed"):
            client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})

    @patch("urllib.request.urlopen")
    def test_post_retry_on_timeout(self, mock_urlopen: MagicMock) -> None:
        """Transient failures should trigger retry."""
        from urllib.error import URLError

        # Fail twice, succeed on third attempt
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"success": true}'
        mock_resp.getcode.return_value = 200

        # urlopen returns a context manager; __enter__ must give back mock_resp
        mock_context = MagicMock()
        mock_context.__enter__.return_value = mock_resp

        mock_urlopen.side_effect = [
            URLError("timeout"),
            URLError("timeout"),
            mock_context,
        ]

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "retry_count": 3,
            "retry_delay": 0.01,
        })
        result = client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})
        assert isinstance(result, dict)
        assert mock_urlopen.call_count == 3

    @patch("urllib.request.urlopen")
    def test_post_all_retries_exhausted(self, mock_urlopen: MagicMock) -> None:
        """When all retries fail, CloudConnectionError should be raised."""
        from urllib.error import URLError

        mock_urlopen.side_effect = URLError("network unreachable")

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "retry_count": 2,
            "retry_delay": 0.01,
        })
        with pytest.raises(CloudConnectionError, match="failed after 3 attempts"):
            client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})
        assert mock_urlopen.call_count == 3  # initial + 2 retries

    @patch("urllib.request.urlopen")
    def test_post_bad_json_raises_protocol_error(
        self, mock_urlopen: MagicMock
    ) -> None:
        """Non-JSON response should raise CloudProtocolError."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b"not-json"
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com", "retry_count": 0})
        with pytest.raises(CloudProtocolError, match="Invalid JSON"):
            client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})

    @patch("urllib.request.urlopen")
    def test_get_success(self, mock_urlopen: MagicMock) -> None:
        """Successful GET should return parsed JSON."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"items": [1, 2, 3]}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        result = client.get("/api/items")
        assert result["items"] == [1, 2, 3]

    @patch("urllib.request.urlopen")
    def test_get_with_params(self, mock_urlopen: MagicMock) -> None:
        """GET with query params should append them to the URL."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"ok": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        client.get("/api/search", params={"q": "hello", "page": "1"})

        # Check that the URL includes query string
        called_url = mock_urlopen.call_args[0][0].full_url
        assert "q=hello" in called_url
        assert "page=1" in called_url

    @patch("urllib.request.urlopen")
    def test_is_connected_success(self, mock_urlopen: MagicMock) -> None:
        """is_connected should return True on 200."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"code": 0, "data": {}}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        assert client.is_connected() is True

    @patch("urllib.request.urlopen")
    def test_is_connected_failure(self, mock_urlopen: MagicMock) -> None:
        """is_connected should return False on exception."""
        from urllib.error import URLError
        mock_urlopen.side_effect = URLError("no route to host")

        client = CloudHttpClient({"base_url": "http://test.com"})
        assert client.is_connected() is False

    @patch("urllib.request.urlopen")
    def test_timeout_respected(self, mock_urlopen: MagicMock) -> None:
        """The timeout config should be passed to urlopen."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"ok": true}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({
            "base_url": "http://test.com",
            "timeout": 3.0,
            "retry_count": 0,
        })
        client.post("/api/tasks", {"mac": "00:00:00:00:00:00"})
        # The timeout keyword argument should be 3.0
        _call_kwargs = mock_urlopen.call_args[1]
        assert _call_kwargs["timeout"] == 3.0

    # ── _unwrap_common_result tests ────────────────────────────

    @patch("urllib.request.urlopen")
    def test_unwrap_common_result_success(self, mock_urlopen: MagicMock) -> None:
        """Unwrap CommonResult: code==0 returns data payload."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = (
            b'{"code":0,"data":{"orders":[]},"msg":""}'
        )
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        result = client.post("/api/test", {"mac": "00:00:00:00:00:00"})
        assert result == {"orders": []}

    @patch("urllib.request.urlopen")
    def test_unwrap_common_result_error(self, mock_urlopen: MagicMock) -> None:
        """Unwrap CommonResult: code!=0 raises CloudProtocolError."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = (
            b'{"code":1070700008,"data":null,"msg":"the task has been accepted by another mobile platform"}'
        )
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com", "retry_count": 0})
        with pytest.raises(CloudProtocolError, match="1070700008"):
            client.post("/api/test", {"mac": "00:00:00:00:00:00"})

    @patch("urllib.request.urlopen")
    def test_unwrap_common_result_no_code(self, mock_urlopen: MagicMock) -> None:
        """Response without 'code' key is returned as-is (defensive compat)."""
        mock_resp = MagicMock()
        mock_resp.read.return_value = b'{"foo":"bar","baz":42}'
        mock_resp.getcode.return_value = 200
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        client = CloudHttpClient({"base_url": "http://test.com"})
        result = client.post("/api/test", {"mac": "00:00:00:00:00:00"})
        assert result == {"foo": "bar", "baz": 42}

    def test_unwrap_placeholder_disabled(self) -> None:
        """Default CloudHttpClient (placeholder URL) returns {} without request."""
        client = CloudHttpClient()
        assert "xxx" in client.base_url
        assert client.post("/any/path", {"mac": "00:00:00:00:00:00"}) == {}
        assert client.get("/any/path") == {}


# ======================================================================
# Data model validation
# ======================================================================


class TestDataModels:
    """Tests for protocol data model serialization."""

    def test_task_list_request(self) -> None:
        req = TaskListRequest(mac="AA:BB:CC:DD:EE:FF")
        d = req.to_dict()
        assert d["mac"] == "AA:BB:CC:DD:EE:FF"

    def test_task_list_response(self) -> None:
        order = OrderInfo(
            order_id="O-001",
            aojian_id="A-001",
            aojian="廒间1号",
            depth_list=[1.0, 2.0],
            jiance=[1, 2, 3],
            points=[{"x": 1.0, "y": 2.0}],
        )
        resp = TaskListResponse(
            liangku_id="L-001",
            liangku="粮仓A区",
            orders=[order],
        )
        assert resp.liangku_id == "L-001"
        assert resp.orders[0].order_id == "O-001"

    def test_order_info_pinzhong_fields(self) -> None:
        # With pinzhong fields (API v2 cloud response)
        order = OrderInfo(
            order_id="O-002",
            aojian_id="A-002",
            aojian="廒间2号",
            depth_list=[2.0],
            jiance=[1, 2],
            points=[{"x": 3.0, "y": 4.0}],
            pinzhong_code="XM",
            pinzhong="小麦",
        )
        assert order.pinzhong == "小麦"
        assert order.pinzhong_code == "XM"

    def test_order_info_pinzhong_defaults_empty(self) -> None:
        # Without pinzhong fields (local task construction) → default empty string
        order = OrderInfo(
            order_id="O-003",
            aojian_id="A-003",
            aojian="廒间3号",
            depth_list=[1.5],
            jiance=[1],
            points=[{"x": 5.0, "y": 6.0}],
        )
        assert order.pinzhong == ""
        assert order.pinzhong_code == ""

    def test_accept_task_request(self) -> None:
        req = AcceptTaskRequest(mac="AA:BB:CC:DD:EE:FF", order_id="O-001")
        assert req.mac == "AA:BB:CC:DD:EE:FF"
        assert req.order_id == "O-001"

    def test_status_report_request(self) -> None:
        req = StatusReportRequest(
            mac="AA:BB:CC:DD:EE:FF",
            order_id="O-001",
            status=ReportStatus.COMPLETE,
            jiance_results=[DetectionResult(indicator_id=1, value=12.5)],
        )
        assert req.status == ReportStatus.COMPLETE
        assert req.jiance_results[0].indicator_id == 1
        assert req.jiance_results[0].value == 12.5

    def test_room_list_request(self) -> None:
        req = RoomListRequest(mac="AA:BB:CC:DD:EE:FF")
        d = req.to_dict()
        assert d["mac"] == "AA:BB:CC:DD:EE:FF"

    def test_map_upload_request_timestamp(self) -> None:
        req = MapUploadRequest(
            mac="AA:BB:CC:DD:EE:FF",
            aojian_id="A-001",
            height=0.5,
            points=[{"x": 1.0, "y": 2.0}],
        )
        assert req.mac == "AA:BB:CC:DD:EE:FF"
        assert req.aojian_id == "A-001"
        assert req.timestamp != ""  # should be auto-generated

    def test_api_path_constants(self) -> None:
        """ApiPath 为相对路径（拼到 base_url 后；base_url 含 /admin-api 前缀）。"""
        assert ApiPath.TASK_LIST == "/task-list"
        assert ApiPath.ACCEPT_TASK == "/accept"
        assert ApiPath.STATUS_REPORT == "/report"
        assert ApiPath.WAREHOUSE_LIST == "/warehouse-list"
        assert ApiPath.ROOM_LIST == "/api/rooms"
        assert ApiPath.MAP_UPLOAD == "/api/maps"
