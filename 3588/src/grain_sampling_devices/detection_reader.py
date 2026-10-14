"""Detection result reader for the biochemical / physicochemical instruments.

⚠️ **STUB IMPLEMENTATION** — The read logic assumes a simple JSON-over-TCP
protocol on the biochemical (5001) and physicochemical (5002) instruments.
The real wire protocol must be provided by the equipment supplier's API
documentation.  Replace every ``# STUB:`` marker once the API docs become
available.

Behaviour contract
------------------
- When a device is not connected / unreachable the reader degrades
  gracefully and returns ``None`` instead of raising exceptions.
- ``read_all`` skips failed items and returns a (possibly empty) list of
  successful reads.
"""

import json
import logging
from typing import Dict, List, Optional

from grain_sampling_devices.base_adapter import (
    DeviceConnectionError,
    DeviceError,
    DeviceTimeoutError,
)
from grain_sampling_devices.biochemical_adapter import BiochemicalAdapter
from grain_sampling_devices.physicochemical_adapter import PhysicochemicalAdapter

logger = logging.getLogger(__name__)


class DetectionResultReader:
    """Read detection values from the biochemical / physicochemical instruments.

    Parameters
    ----------
    biochemical_host : str
        IPv4 address of the biochemical analyser.  Default ``127.0.0.1``.
    biochemical_port : int
        TCP port of the biochemical analyser.  Default *5001*.
    physicochemical_host : str
        IPv4 address of the physicochemical tester.  Default ``127.0.0.1``.
    physicochemical_port : int
        TCP port of the physicochemical tester.  Default *5002*.

    ⚠️ **STUB IMPLEMENTATION** — The real wire protocol is unknown; the
    supplier's API documentation is required to replace the STUB markers.
    """

    def __init__(
        self,
        biochemical_host: str = "127.0.0.1",
        biochemical_port: int = 5001,
        physicochemical_host: str = "127.0.0.1",
        physicochemical_port: int = 5002,
    ) -> None:
        self._biochemical = BiochemicalAdapter(host=biochemical_host, port=biochemical_port)
        self._physicochemical = PhysicochemicalAdapter(
            host=physicochemical_host, port=physicochemical_port
        )

    # ── Public API ──────────────────────────────────────────────────

    def read_result(self, indicator_id: int, depth: Optional[float] = None) -> Optional[float]:
        """Read a single detection value for *indicator_id*.

        Routing (STUB, no mapping table available): odd ``indicator_id``
        targets the biochemical analyser first, even targets the
        physicochemical tester first; if that adapter fails the other one
        is tried as a fallback.

        Parameters
        ----------
        indicator_id : int
            Indicator identifier.  No official mapping exists yet.
        depth : float, optional
            Sampling depth in metres.  Not used by the STUB wire protocol
            but passed along for forward compatibility.

        Returns
        -------
        float or None
            The measured value, or ``None`` when the device is unreachable
            or the read fails.  Never raises.
        """
        # STUB routing rule — replace with the official mapping table when
        # the supplier's API documentation is provided.
        if indicator_id % 2 == 0:
            adapters = [self._physicochemical, self._biochemical]
        else:
            adapters = [self._biochemical, self._physicochemical]

        for adapter in adapters:
            value = self._read_from_adapter(adapter, indicator_id, depth)
            if value is not None:
                return value
        return None

    def read_all(self, jiance: List[int], depths: List[float]) -> List[Dict]:
        """Read detection values for a batch of indicators.

        Parameters
        ----------
        jiance : list of int
            Indicator identifiers to read.
        depths : list of float
            Sampling depths, one per indicator (may be shorter/longer than
            *jiance*; missing depths become ``None``).

        Returns
        -------
        list of dict
            Successful reads as ``{"indicator_id": id, "depth": depth,
            "value": value}``.  Failed reads are skipped, so the result
            may be empty when no device is connected.
        """
        results: List[Dict] = []
        for index, indicator_id in enumerate(jiance):
            depth: Optional[float] = None
            if index < len(depths):
                depth = depths[index]
            value = self.read_result(indicator_id, depth)
            if value is not None:
                results.append(
                    {
                        "indicator_id": indicator_id,
                        "depth": depth,
                        "value": value,
                    }
                )
        return results

    # ── Internal helpers ────────────────────────────────────────────

    def _read_from_adapter(self, adapter: object, indicator_id: int, depth: Optional[float]) -> Optional[float]:
        """Connect to *adapter*, issue the STUB read command, parse the value.

        Returns ``None`` on any failure (never raises).
        """
        try:
            if not adapter.connect():
                logger.warning(
                    "Detection read: %s (%s:%d) not reachable",
                    adapter.device_type,
                    adapter.ip_address,
                    adapter.port,
                )
                return None

            # STUB: Replace the command and response format with the real
            # protocol once the supplier's API documentation is available.
            payload = {"cmd": "read", "indicator_id": indicator_id}
            if depth is not None:
                payload["depth"] = depth
            cmd_bytes = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")

            response = adapter.send_command(cmd_bytes)
            # STUB: Assumes the device responds with JSON {"value": <float>}.
            resp_data: Dict = json.loads(response.decode("utf-8").strip())
            raw_value = resp_data.get("value")
            if raw_value is None:
                logger.warning(
                    "Detection read: %s returned no value field",
                    adapter.device_type,
                )
                return None
            return float(raw_value)
        except (DeviceConnectionError, DeviceTimeoutError) as exc:
            logger.warning("Detection read via %s failed: %s", adapter.device_type, exc)
            return None
        except DeviceError as exc:
            logger.warning("Detection read via %s failed: %s", adapter.device_type, exc)
            return None
        except (OSError, json.JSONDecodeError, UnicodeDecodeError, ValueError, TypeError) as exc:
            logger.warning(
                "Detection read via %s failed to parse response: %s",
                adapter.device_type,
                exc,
            )
            return None
        finally:
            try:
                adapter.disconnect()
            except Exception:  # noqa: BLE001 - cleanup must never raise
                pass
