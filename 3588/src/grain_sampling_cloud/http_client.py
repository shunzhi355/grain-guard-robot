"""HTTP REST client for grain sampling robot cloud communication.

Provides a thread-safe HTTP client with JSON serialization,
MAC-based authentication, auto-retry, and configurable settings.

Replaces the old MQTT-based cloud communication.
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from typing import Any, Dict, Optional
from urllib import request as urllib_request
from urllib.error import URLError, HTTPError

from utils.config import AppConfig

logger = logging.getLogger(__name__)


class CloudConnectionError(Exception):
    """Raised when cloud communication fails after all retries."""
    pass


class CloudAuthError(Exception):
    """Raised on 401/403 responses from the cloud."""
    pass


class CloudProtocolError(Exception):
    """Raised on unexpected response format."""
    pass


def detect_mac() -> str:
    """Detect the device's MAC address.

    Returns a formatted MAC string like ``"00:1A:2B:3C:4D:5E"``,
    or ``"00:00:00:00:00:00"`` as fallback.
    """
    try:
        mac = uuid.getnode()
        if (mac >> 40) % 2 == 0:
            return ':'.join(
                f'{(mac >> (8 * i)) & 0xFF:02X}' for i in range(5, -1, -1)
            )
    except Exception:
        pass
    return "00:00:00:00:00:00"


_DEFAULT_CONFIG: Dict[str, Any] = {
    "base_url": "http://cloud-api.xxx.com",
    "timeout": 10.0,
    "retry_count": 3,
    "retry_delay": 1.0,
    "retry_max_delay": 30.0,
}


class CloudHttpClient:
    """Thread-safe HTTP client for cloud API communication.

    Parameters
    ----------
    config : dict, optional
        Configuration dictionary. Keys:
        - ``base_url``: Cloud API base URL (default ``http://cloud-api.xxx.com``)
        - ``timeout``: Request timeout in seconds (default ``10.0``)
        - ``retry_count``: Max retry attempts (default ``3``)
        - ``retry_delay``: Initial retry delay in seconds (default ``1.0``)
        - ``retry_max_delay``: Maximum retry delay (default ``30.0``)
        - ``mac_address``: Override auto-detected MAC (optional)
    """

    @classmethod
    def from_app_config(cls, app_config: AppConfig) -> "CloudHttpClient":
        """Create client from AppConfig."""
        return cls({
            "base_url": app_config.cloud_base_url,
            "mac_address": app_config.device_mac or detect_mac(),
        })

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        cfg = {**_DEFAULT_CONFIG, **(config or {})}
        self._base_url = cfg["base_url"].rstrip("/")
        self._timeout = float(cfg["timeout"])
        self._retry_count = int(cfg["retry_count"])
        self._retry_delay = float(cfg["retry_delay"])
        self._retry_max_delay = float(cfg["retry_max_delay"])
        self._mac = str(cfg.get("mac_address", detect_mac()))

    # ── Properties ────────────────────────────────────────────

    @property
    def base_url(self) -> str:
        return self._base_url

    @property
    def mac_address(self) -> str:
        return self._mac

    # ── Public API ────────────────────────────────────────────

    def post(self, path: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """Send a POST request to ``{base_url}{path}``.

        Automatically injects the ``mac`` field into the payload if not present.

        The ``CommonResult`` envelope is unwrapped: on success the
        returned dict is the ``data`` payload.

        Parameters
        ----------
        path : str
            API path (e.g. ``/api/tasks``).
        data : dict
            JSON-serialisable request body.

        Returns
        -------
        dict
            Unwrapped business payload (``CommonResult.data``).

        Raises
        ------
        CloudConnectionError
            If all retries fail or network is unreachable.
        CloudAuthError
            On 401/403 responses.
        CloudProtocolError
            On unexpected response format or non-zero error code.
        """
        if "xxx" in self._base_url:
            logger.debug("Cloud API disabled (placeholder URL: %s)", self._base_url)
            return {}

        if "mac" not in data:
            data["mac"] = self._mac

        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        url = f"{self._base_url}{path}"

        last_error: Optional[Exception] = None
        delay = self._retry_delay

        for attempt in range(self._retry_count + 1):
            try:
                req = urllib_request.Request(
                    url,
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib_request.urlopen(req, timeout=self._timeout) as resp:
                    raw = resp.read().decode("utf-8")
                    result: Dict[str, Any] = json.loads(raw)
                    return self._unwrap_common_result(result)

            except HTTPError as e:
                if e.code in (401, 403):
                    raise CloudAuthError(
                        f"Authentication failed (HTTP {e.code}): {e.reason}"
                    ) from e
                last_error = CloudConnectionError(
                    f"HTTP {e.code}: {e.reason}"
                )
            except (URLError, TimeoutError, OSError) as e:
                last_error = CloudConnectionError(
                    f"Request failed: {e}"
                )
            except json.JSONDecodeError as e:
                raise CloudProtocolError(
                    f"Invalid JSON response: {e}"
                ) from e

            if attempt < self._retry_count:
                logger.warning(
                    "Cloud request to %s failed (attempt %d/%d), "
                    "retrying in %.1fs...",
                    url, attempt + 1, self._retry_count + 1, delay,
                )
                time.sleep(delay)
                delay = min(delay * 2, self._retry_max_delay)

        raise CloudConnectionError(
            f"Cloud request to {url} failed after "
            f"{self._retry_count + 1} attempts"
        ) from last_error

    def get(self, path: str, params: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
        """Send a GET request to ``{base_url}{path}``.

        Parameters are added as query string.

        The ``CommonResult`` envelope is unwrapped: on success the
        returned dict is the ``data`` payload.

        Returns
        -------
        dict
            Unwrapped business payload (``CommonResult.data``).

        Raises
        ------
        CloudConnectionError
            If all retries fail or network is unreachable.
        CloudAuthError
            On 401/403 responses.
        CloudProtocolError
            On unexpected response format or non-zero error code.
        """
        if "xxx" in self._base_url:
            logger.debug("Cloud API disabled (placeholder URL: %s)", self._base_url)
            return {}

        url = f"{self._base_url}{path}"
        if params:
            qs = "&".join(f"{k}={v}" for k, v in params.items())
            url = f"{url}?{qs}"

        last_error: Optional[Exception] = None
        delay = self._retry_delay

        for attempt in range(self._retry_count + 1):
            try:
                req = urllib_request.Request(url, method="GET")
                with urllib_request.urlopen(req, timeout=self._timeout) as resp:
                    raw = resp.read().decode("utf-8")
                    return self._unwrap_common_result(json.loads(raw))

            except HTTPError as e:
                if e.code in (401, 403):
                    raise CloudAuthError(
                        f"Authentication failed (HTTP {e.code})"
                    ) from e
                last_error = CloudConnectionError(
                    f"HTTP {e.code}: {e.reason}"
                )
            except (URLError, TimeoutError, OSError) as e:
                last_error = CloudConnectionError(
                    f"Request failed: {e}"
                )
            except json.JSONDecodeError as e:
                raise CloudProtocolError(
                    f"Invalid JSON response: {e}"
                ) from e

            if attempt < self._retry_count:
                time.sleep(delay)
                delay = min(delay * 2, self._retry_max_delay)

        raise CloudConnectionError(
            f"Cloud GET {url} failed after {self._retry_count + 1} attempts"
        ) from last_error

    # ── Response unwrapping ──────────────────────────────────

    def _unwrap_common_result(self, result: Dict[str, Any]) -> Dict[str, Any]:
        """Unwrap a ``CommonResult`` envelope into its ``data`` payload.

        All cloud API responses follow ``{"code": 0, "data": {...}, "msg": ""}``.

        - ``code == 0``: return ``data`` (the business payload).
        - ``code != 0``: raise :class:`CloudProtocolError` carrying the code.
        - No ``code`` key (or non-dict input): return as-is, defensively.

        Parameters
        ----------
        result : dict
            The parsed JSON response.

        Returns
        -------
        dict
            Unwrapped ``data`` payload, or the original dict if it is
            not a ``CommonResult`` envelope.

        Raises
        ------
        CloudProtocolError
            When ``code != 0``.
        """
        if not isinstance(result, dict) or "code" not in result:
            return result
        code = result["code"]
        if code == 0:
            return result.get("data", {})
        msg = result.get("msg", "")
        raise CloudProtocolError(f"cloud error {code}: {msg}")

    def is_connected(self) -> bool:
        """Check if the cloud server is reachable.

        Sends a GET to ``/task-list`` with the device MAC.  Returns
        ``True`` if the server responds (even with a business error
        like 1070700000 "device not found").
        """
        try:
            resp = self.get("/task-list", {"mac": self.mac_address})
            return isinstance(resp, dict)
        except Exception:
            return False
