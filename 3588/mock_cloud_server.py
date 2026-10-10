#!/usr/bin/env python3
"""Mock cloud server for end-to-end testing of CloudHttpClient.

Implements the cloud API endpoints defined in grain-sampler-api.md
(protocol v2, relative paths under base URL), backed by
pre-seeded test data. Responses follow the CommonResult envelope:
{"code": 0, "data": {...}, "msg": ""}.

Also keeps `POST /api/maps` from the old protocol for uploader testing
(API #5 map parameter upload is not implemented on the backend yet).

Usage:
    python mock_cloud_server.py --port 8889
    python mock_cloud_server.py --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import traceback
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse


# ─── Pre-seeded mock data ───────────────────────────────────────────

_MOCK_ORDERS = [
    {
        "order_id": "RW20260715001",
        "aojian_id": 1,
        "aojian": "1",
        "depth_list": [0.5, 1.0, 1.5],
        "points": [{"x": 10, "y": 20}, {"x": 15, "y": 25}],
        "jiance": [1, 2],
    },
    {
        "order_id": "RW20260715002",
        "aojian_id": 2,
        "aojian": "2",
        "depth_list": [1.0, 2.0],
        "points": [{"x": 5, "y": 1}, {"x": 7.5, "y": 3}],
        "jiance": [1, 3],
    },
]

_MOCK_ROOMS = [
    {"aojian_id": 1, "aojian": "1"},
    {"aojian_id": 2, "aojian": "2"},
    {"aojian_id": 3, "aojian": "3"},
]

_MOCK_LIANGKU_ID = 1  # Long
_MOCK_LIANGKU = "江阴库"

# Error codes referenced by grain-sampler-api.md
_CLOUD_ERRORS = {
    1070700001: "任务不存在",
    1070700003: "设备正在验收中，无法承接新任务",
    1070700008: "该任务已被其他移动平台承接",
}


def _ok(data):
    """Build a successful CommonResult envelope."""
    return {"code": 0, "data": data, "msg": ""}


def _err(code, msg):
    """Build a failed CommonResult envelope (data is null)."""
    return {"code": code, "data": None, "msg": msg}


# ─── Route table — path → handler method name ───────────────────────
# GET routes receive the parsed query string (dict of lists);
# POST routes receive the parsed JSON body (dict).

_ROUTES_GET = {
    "/task-list": "_handle_task_list",
    "/warehouse-list": "_handle_warehouse_list",
}

_ROUTES_POST = {
    "/accept": "_handle_accept",
    "/report": "_handle_report",
    "/api/maps": "_handle_maps",  # kept for uploader testing
    "/map-upload": "_handle_map_upload",
}

# ─── Data directory ─────────────────────────────────────────────────

_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock_server_data")


# ─── Handler ────────────────────────────────────────────────────────

class MockCloudHandler(BaseHTTPRequestHandler):
    """HTTP request handler routing GET/POST requests to mock endpoints."""

    # Silence per-request log to stderr (we log our own to stdout)
    def log_message(self, fmt, *args):
        return

    # ── HTTP method dispatch ──────────────────────────────────────

    def do_GET(self):
        """Route GET requests by path."""
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        qs = parse_qs(parsed.query)

        if self._dispatch_error_simulation(qs):
            return

        handler_name = _ROUTES_GET.get(path)
        if handler_name is None:
            self._log("%s %s -> 404 (unknown path)", self.command, self.path)
            self._send_json(_err(-1, "not found"), status=404)
            return

        try:
            handler = getattr(self, handler_name)
            result, status = handler(qs)
            self._log("%s %s -> %d", self.command, self.path, status)
            self._send_json(result, status=status)
        except Exception:
            self._log("%s %s -> 500 (exception)", self.command, self.path)
            traceback.print_exc(file=sys.stderr)
            self._send_json(_err(-1, "internal server error"), status=500)

    def do_POST(self):
        """Route POST requests by path."""
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        qs = parse_qs(parsed.query)

        if self._dispatch_error_simulation(qs):
            return

        handler_name = _ROUTES_POST.get(path)
        if handler_name is None:
            self._log("%s %s -> 404 (unknown path)", self.command, self.path)
            self._send_json(_err(-1, "not found"), status=404)
            return

        try:
            body = self._read_json()
            handler = getattr(self, handler_name)
            result, status = handler(body)
            self._log("%s %s -> %d", self.command, self.path, status)
            self._send_json(result, status=status)
        except Exception:
            self._log("%s %s -> 500 (exception)", self.command, self.path)
            traceback.print_exc(file=sys.stderr)
            self._send_json(_err(-1, "internal server error"), status=500)

    def do_OPTIONS(self):
        """Handle CORS preflight (allow any origin)."""
        self.send_response(204)
        self._cors_headers()
        self.end_headers()

    # ── Helpers ────────────────────────────────────────────────────

    def _dispatch_error_simulation(self, qs):
        """Handle simulated error query params. Returns True if handled."""
        # Simulated HTTP 500 (kept from old protocol)
        if "simulate_error" in qs:
            self._log("%s %s -> 500 (simulated)", self.command, self.path)
            self._send_json(_err(-1, "simulated server error"), status=500)
            return True
        # Simulated cloud business error: ?simulate_cloud_error=<error_code>
        if "simulate_cloud_error" in qs:
            code = int(qs["simulate_cloud_error"][0])
            msg = _CLOUD_ERRORS.get(code, "业务处理失败")
            self._log("%s %s -> %d (simulated cloud error)", self.command, self.path, code)
            self._send_json(_err(code, msg), status=200)
            return True
        return False

    def _send_json(self, data, status=200):
        """Send a JSON response."""
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors_headers()
        self.end_headers()
        self.wfile.write(body)

    def _cors_headers(self):
        """Add permissive CORS headers."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _read_json(self):
        """Read and parse JSON request body."""
        content_length = int(self.headers.get("Content-Length", 0))
        if content_length == 0:
            return {}
        raw = self.rfile.read(content_length)
        return json.loads(raw.decode("utf-8"))

    def _log(self, fmt, *args):
        """Log to stdout with a timestamp."""
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        msg = fmt % args if args else fmt
        print(f"[{ts}] {msg}", flush=True)

    # ── GET endpoint handlers ──────────────────────────────────────

    def _handle_task_list(self, qs):
        """GET /task-list — liangku info + pending orders."""
        mac = qs.get("mac", [""])[0]  # device match is lenient in the mock
        response = {
            "liangku_id": _MOCK_LIANGKU_ID,
            "liangku": _MOCK_LIANGKU,
            "orders": _MOCK_ORDERS,
        }
        return _ok(response), 200

    def _handle_warehouse_list(self, qs):
        """GET /warehouse-list — liangku info + room list."""
        mac = qs.get("mac", [""])[0]  # device match is lenient in the mock
        response = {
            "liangku_id": _MOCK_LIANGKU_ID,
            "liangku": _MOCK_LIANGKU,
            "aojian_list": _MOCK_ROOMS,
        }
        return _ok(response), 200

    # ── POST endpoint handlers ─────────────────────────────────────

    def _handle_accept(self, body):
        """POST /accept — bind device to a task."""
        order_id = body.get("order_id", "")
        found = any(o["order_id"] == order_id for o in _MOCK_ORDERS)
        if found:
            return _ok(True), 200
        return _err(1070700001, _CLOUD_ERRORS[1070700001]), 404

    def _handle_report(self, body):
        """POST /report — report completion/abandonment."""
        self._append_status_log(body)
        return _ok(True), 200

    def _handle_maps(self, body):
        """POST /api/maps — save map data to file (legacy, uploader testing)."""
        self._save_map_data(body)
        return _ok(True), 200

    def _handle_map_upload(self, body):
        """POST /map-upload — 保存廒间切面地图数据。"""
        required = ("mac", "aojian_id", "boundary", "map_bounds")
        if not all(k in body for k in required):
            return _err(1070700009, "地图数据不完整"), 200
        self._save_map_data(body)
        return _ok(True), 200

    # ── File I/O ───────────────────────────────────────────────────

    def _append_status_log(self, data):
        """Append a JSON line to status_log.jsonl."""
        filepath = os.path.join(_DATA_DIR, "status_log.jsonl")
        os.makedirs(_DATA_DIR, exist_ok=True)
        entry = {
            "timestamp": datetime.datetime.now().isoformat(),
            "mac": data.get("mac", ""),
            "order_id": data.get("order_id", ""),
            "status": data.get("status", ""),
            "jiance_results": data.get("jiance_results", None),
        }
        with open(filepath, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def _save_map_data(self, data):
        """Save map upload body to a timestamped JSON file."""
        os.makedirs(_DATA_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(_DATA_DIR, f"map_{ts}.json")
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)


# ─── Main ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Mock cloud server for grain sampling robot testing"
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8888, help="Bind port (default: 8888)")
    args = parser.parse_args()

    # Ensure data directory exists
    os.makedirs(_DATA_DIR, exist_ok=True)

    server = HTTPServer((args.host, args.port), MockCloudHandler)
    print(f"[mock-cloud-server] Listening on http://{args.host}:{args.port}")
    print(f"[mock-cloud-server] Data directory: {_DATA_DIR}")
    print(f"[mock-cloud-server] Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[mock-cloud-server] Shutting down...")
        server.shutdown()
        print("[mock-cloud-server] Stopped.")


if __name__ == "__main__":
    main()
