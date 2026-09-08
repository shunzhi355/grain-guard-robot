"""RC receiver GPIO pulse-width measurement.

Reads the three PWM channels of an hobby-grade RC receiver
(CH1 = forward/reverse, CH3 = steering, CH5 = two-position mode switch)
by measuring the high-level pulse width on each GPIO input.

Signal characteristics (50 Hz, 1000~2000 us per channel) are standard for
aircraft/hobby RC receivers; the pin mapping below was measured on the
target board on 2026-08-16::

    rc_pins = {"CH1": 34, "CH3": 40, "CH5": 111}
      - CH1 -> physical Pin 15 (gpio-34,  GPIO1_A2,  gpiochip1)
      - CH3 -> physical Pin 22 (gpio-40,  GPIO1_B0,  gpiochip1)
      - CH5 -> physical Pin 40 (gpio-111, GPIO3_B7,  gpiochip3)  # mode switch, was Pin24/SPI0_CS0

Measurement strategy
--------------------
Pure sysfs GPIO polling with ``time.perf_counter()`` edge timestamps.
(``perf_counter`` is monotonic on every supported platform and keeps
microsecond resolution on Windows, unlike ``time.monotonic()`` which is
quantised to ~15.6 ms ticks there.)  Edge interrupts are more precise but
the sysfs ``poll()`` path has known timing jitter on this board, so the
module follows the verified approach: a background thread polls every
channel every ``poll_interval`` (~0.2 ms) and timestamps every 0->1 /
1->0 transition.

Anti-jitter / safety
--------------------
- A per-channel median filter over the last ``debounce_samples`` valid
  samples suppresses single-sample glitches (instant spikes are never
  reported).
- A reported value only changes when the filtered median moves by at
  least ``debounce_threshold`` us.
- Samples outside ``valid_range`` (default 700~2300 us) are dropped and
  counted; they never become control values.
- ``read()`` returns ``None`` for a channel whose data has gone stale
  (no valid sample within ``stale_timeout``), so stale/absent signals
  cannot produce a bogus control value.

No GPIO environment (Windows dev box) uses :class:`MockRCReceiver`, which
implements the same debounce/validation logic but records injected samples
instead of touching hardware.  Use :func:`create_rc_receiver` to pick the
right implementation automatically.
"""

from __future__ import annotations

import glob
import logging
import os
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Mapping, Optional, Tuple, Union

from grain_sampling_devices.base_adapter import DeviceError
from utils.sampling_params import RC_PINS

logger = logging.getLogger(__name__)

# ── Constants ───────────────────────────────────────────────────────────

#: sysfs GPIO root directory (Linux).
SYSFS_GPIO_DIR = "/sys/class/gpio"

#: Board-measured pin map (2026-08-16): CH1/CH3/CH5 -> sysfs GPIO numbers.
#: 统一参数源：utils.sampling_params.RC_PINS。
DEFAULT_RC_PINS: Dict[str, int] = dict(RC_PINS)

#: Jetson Orin NX 40-pin mapping (libgpiod backend).
#:
#: On Jetson the main GPIO controller ``tegra234-gpio`` (``/dev/gpiochip0``)
#: uses *line offsets* equal to the legacy sysfs/global GPIO numbers, so the
#: integer values here are used directly as libgpiod line offsets::
#:
#:     CH1 -> physical Pin 15 (GPIO12, PN.01,  line 85)
#:     CH3 -> physical Pin 31 (GPIO11, PQ.06,  line 106)
#:     CH5 -> physical Pin 7  (GPIO09, PAC.06, line 144)
#:
#: The accompanying ``JETSON_RC_PIN_NAMES`` map holds the stable Tegra port
#: names (``PN.01`` etc.) that ``gpioinfo`` reports as line names; they are
#: used for a runtime (chip, offset) probe that is robust across kernel
#: versions and chip ordering.
JETSON_RC_PINS: Dict[str, int] = {"CH1": 85, "CH3": 106, "CH5": 144}

#: Jetson Orin NX channel -> Tegra port name (line name in ``gpioinfo``).
JETSON_RC_PIN_NAMES: Dict[str, str] = {
    "CH1": "PN.01",
    "CH3": "PQ.06",
    "CH5": "PAC.06",
}

#: Jetson main GPIO controller label (``tegra234-gpio``).
JETSON_GPIOCHIP_LABEL = "tegra234-gpio"

#: RC frame period at 50 Hz, in seconds.
RC_FRAME_PERIOD_S = 0.020

#: Accepted pulse width range in us; anything outside is an anomaly.
VALID_PULSE_MIN_US = 700.0
VALID_PULSE_MAX_US = 2300.0

#: Sanity bound for a single measured pulse (one 50 Hz frame) in us.
MAX_PULSE_WIDTH_US = 25_000.0

ChannelName = str


# ── sysfs GPIO helpers ──────────────────────────────────────────────────


def _gpio_value_path(gpio: int, sysfs_dir: str = SYSFS_GPIO_DIR) -> str:
    return os.path.join(sysfs_dir, f"gpio{gpio}", "value")


def _read_gpio_value(gpio: int, sysfs_dir: str = SYSFS_GPIO_DIR) -> int:
    """Read the current logic level (0/1) of a sysfs GPIO."""
    with open(_gpio_value_path(gpio, sysfs_dir), "r", encoding="ascii") as fh:
        return int(fh.read().strip())


def _export_gpio(gpio: int, sysfs_dir: str = SYSFS_GPIO_DIR) -> bool:
    """Export *gpio* via sysfs; returns ``True`` if the export succeeded."""
    gpio_dir = os.path.join(sysfs_dir, f"gpio{gpio}")
    if os.path.isdir(gpio_dir):
        return True  # already exported (possibly by another process)
    try:
        with open(os.path.join(sysfs_dir, "export"), "w", encoding="ascii") as fh:
            fh.write(str(gpio))
    except OSError as exc:
        logger.warning("Failed to export GPIO %d: %s", gpio, exc)
        return False
    # sysfs export is asynchronous; wait for the gpioN directory to appear.
    for _ in range(50):
        if os.path.isdir(gpio_dir):
            return True
        time.sleep(0.01)
    logger.warning("GPIO %d exported but gpioN directory never appeared", gpio)
    return False


def _set_gpio_direction(gpio: int, direction: str, sysfs_dir: str = SYSFS_GPIO_DIR) -> None:
    with open(
        os.path.join(sysfs_dir, f"gpio{gpio}", "direction"),
        "w",
        encoding="ascii",
    ) as fh:
        fh.write(direction)


def _open_gpio_perms(gpio: int, sysfs_dir: str = SYSFS_GPIO_DIR) -> None:
    """尽力开放已导出 GPIO 的 direction/value 写权限（0666）。

    sysfs 动态创建的 gpioN 目录默认仅 root 可写；若本进程以 root 运行
    （或 systemd 已预授权）则 chmod 成功，普通用户进程则静默失败并
    依赖开机预授权（rc-gpio-prep.service / udev 规则）兜底。
    """
    for name in ("direction", "value"):
        path = os.path.join(sysfs_dir, f"gpio{gpio}", name)
        try:
            os.chmod(path, 0o666)
        except OSError:
            pass


def _unexport_gpio(gpio: int, sysfs_dir: str = SYSFS_GPIO_DIR) -> None:
    with open(os.path.join(sysfs_dir, "unexport"), "w", encoding="ascii") as fh:
        fh.write(str(gpio))


# ── libgpiod helpers ────────────────────────────────────────────────────


def _load_gpiod() -> Optional[object]:
    """Import the ``gpiod`` Python binding (v1 or v2); ``None`` if absent.

    Importing is lazy so the module stays importable on Windows / any host
    without libgpiod installed (mirrors the defensive sysfs handling).
    """
    try:
        import gpiod  # type: ignore[import-not-found]
    except ImportError:
        logger.debug("python 'gpiod' module not installed - libgpiod unavailable")
        return None
    return gpiod


def _gpiod_api_version(gpiod: object) -> int:
    """Return ``2`` for the v2 API (``LineSettings``), else ``1``."""
    return 2 if hasattr(gpiod, "LineSettings") else 1


def _iter_gpiochip_paths() -> List[str]:
    """Sorted list of ``/dev/gpiochip*`` character devices (empty elsewhere)."""
    return sorted(glob.glob("/dev/gpiochip*"))


def _libgpiod_available() -> bool:
    """Whether a libgpiod character device and Python binding are usable."""
    if not _iter_gpiochip_paths():
        return False
    return _load_gpiod() is not None


def _find_line_offset(gpiod: object, version: int, chip_path: str, name: str) -> Optional[int]:
    """Return the line offset for *name* on *chip_path*, or ``None``.

    Uses the stable Tegra port name (e.g. ``"PN.01"``) reported by gpioinfo
    as the line name.  On libgpiod v2 this is a direct lookup; on v1 the
    line list is scanned.
    """
    if not name:
        return None
    try:
        if version == 2:
            chip = gpiod.Chip(chip_path)
            try:
                return chip.get_line_offset_from_name(name)
            except (LookupError, KeyError, OSError):
                return None
            finally:
                chip.close()
        # v1 API
        chip = gpiod.Chip(chip_path, gpiod.Chip.OPEN_BY_PATH)
        try:
            for offset in range(chip.num_lines()):
                line = chip.get_line(offset)
                try:
                    line_name = line.name()
                except OSError:
                    continue
                if line_name == name:
                    return offset
        finally:
            chip.close()
    except Exception:  # noqa: BLE001 - any lookup failure -> fall back to offsets
        logger.debug("libgpiod line-name lookup failed on %s for %s", chip_path, name)
    return None


def _resolve_libgpiod_lines(
    rc_pins: Mapping[str, int],
    pin_names: Mapping[str, str],
    gpiod: object,
    version: int,
    gpiochip_paths: Optional[List[str]] = None,
) -> Dict[str, Tuple[str, int]]:
    """Resolve each channel to a ``(chip_path, line_offset)`` pair.

    Resolution order per channel:

    1. match the Tegra port name (``pin_names``) against the line names of
       every chip — robust across kernel versions and chip ordering;
    2. fall back to using the integer pin number directly as a line offset
       on the main ``tegra234-gpio`` chip (true on Orin NX, where the line
       offset equals the legacy sysfs number).
    """
    paths = list(gpiochip_paths) if gpiochip_paths else _iter_gpiochip_paths()
    resolved: Dict[str, Tuple[str, int]] = {}
    for channel, gpio in rc_pins.items():
        want_name = pin_names.get(channel)
        found: Optional[Tuple[str, int]] = None
        for chip_path in paths:
            offset = _find_line_offset(gpiod, version, chip_path, want_name)
            if offset is not None:
                found = (chip_path, offset)
                break
        if found is None:
            # On Jetson Orin NX the line offset equals the legacy sysfs
            # number on the main controller; try it on every chip and also
            # keep the offset within the chip's reported line count.
            found = (paths[0], int(gpio))
            logger.debug(
                "libgpiod fallback: %s -> %s line %d",
                channel,
                paths[0],
                int(gpio),
            )
        resolved[channel] = found
    return resolved


def _open_input_line(
    gpiod: object, version: int, chip_path: str, offset: int
) -> Tuple[Callable[[], int], Callable[[], None]]:
    """Request *offset* as an input line; return ``(reader, releaser)``.

    ``reader`` samples the current logic level (0/1), ``releaser`` returns
    the line to the kernel.  Supports both the v1 (``line.request``) and v2
    (``chip.request_lines``) Python APIs.
    """
    if version == 2:
        chip = gpiod.Chip(chip_path)
        request = chip.request_lines(
            config={offset: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT)},
            consumer="rc_receiver",
        )

        def reader_v2() -> int:
            return int(request.get_value(offset))

        def release_v2() -> None:
            for method in ("release", "close"):
                closer = getattr(request, method, None)
                if closer is not None:
                    try:
                        closer()
                        break
                    except Exception:  # noqa: BLE001
                        pass
            chip.close()

        return reader_v2, release_v2

    # v1 API
    chip = gpiod.Chip(chip_path, gpiod.Chip.OPEN_BY_PATH)
    line = chip.get_line(offset)
    line.request(consumer="rc_receiver", type=gpiod.LINE_REQ_DIR_IN)

    def reader_v1() -> int:
        return int(line.get_value())

    def release_v1() -> None:
        try:
            line.release()
        finally:
            chip.close()

    return reader_v1, release_v1


# ── Debounce helper ─────────────────────────────────────────────────────


def _median(values: List[float]) -> Optional[float]:
    """Median of *values* (``None`` for an empty sequence)."""
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class _PulseDebouncer:
    """Median-filter debounce over a rolling window of raw samples.

    A sample is *reported* only when the filtered median moves away from
    the previously reported value by at least ``threshold`` us.
    """

    def __init__(self, window: int, threshold: float) -> None:
        self.window: Deque[float] = deque(maxlen=max(1, window))
        self.threshold = threshold
        self.last_reported: Optional[float] = None

    def push(self, value: float) -> bool:
        """Feed one raw sample; returns ``True`` when the report changes."""
        self.window.append(value)
        median = _median(list(self.window))
        if median is None:
            return False
        if self.last_reported is None or abs(median - self.last_reported) >= self.threshold:
            self.last_reported = median
            return True
        return False


# ── Base receiver ───────────────────────────────────────────────────────


class _BaseRCReceiver:
    """Shared debounce / validation / history logic for all receivers.

    Parameters
    ----------
    rc_pins : Mapping[str, int], optional
        Per-channel sysfs GPIO numbers; defaults to the board-measured
        ``DEFAULT_RC_PINS`` map.
    debounce_samples : int
        Median-filter window size (consecutive samples).
    debounce_threshold : float
        Minimum pulse-width change (us) required to report a new value.
    valid_range : tuple[float, float], optional
        Accepted pulse width range in us (default 700~2300).
    stale_timeout : float
        A channel that receives no valid sample within this many seconds
        reports ``None`` (default 0.5 s).
    on_change : Callable[[str, float], None], optional
        Called with ``(channel, new_value)`` whenever a value is reported.
    clock : Callable[[], float], optional
        Monotonic clock used for edge timing and staleness checks
        (default ``time.perf_counter`` — high resolution on all OSes).
    """

    CHANNELS: tuple = ("CH1", "CH3", "CH5")
    DEFAULT_PINS: Mapping[str, int] = DEFAULT_RC_PINS

    def __init__(
        self,
        rc_pins: Optional[Mapping[str, int]] = None,
        debounce_samples: int = 5,
        debounce_threshold: float = 50.0,
        valid_range: Optional[tuple] = None,
        stale_timeout: float = 0.5,
        on_change: Optional[Callable[[str, float], None]] = None,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        pins = dict(self.DEFAULT_PINS)
        if rc_pins:
            pins.update(rc_pins)
        self.rc_pins: Dict[str, int] = pins

        lo, hi = valid_range or (VALID_PULSE_MIN_US, VALID_PULSE_MAX_US)
        self._valid_min = float(lo)
        self._valid_max = float(hi)
        self._stale_timeout = float(stale_timeout)
        self._on_change = on_change
        self._clock = clock or time.perf_counter

        self._lock = threading.Lock()
        self._debouncers: Dict[str, _PulseDebouncer] = {
            name: _PulseDebouncer(debounce_samples, float(debounce_threshold))
            for name in self.CHANNELS
        }
        self._histories: Dict[str, Deque[float]] = {
            name: deque(maxlen=200) for name in self.CHANNELS
        }
        self._reported: Dict[str, Optional[float]] = {
            name: None for name in self.CHANNELS
        }
        self._last_raw: Dict[str, Optional[float]] = {
            name: None for name in self.CHANNELS
        }
        self._last_valid_time: Dict[str, float] = {name: 0.0 for name in self.CHANNELS}
        self._out_of_range: Dict[str, int] = {name: 0 for name in self.CHANNELS}

    # ── Sample ingestion ────────────────────────────────────────────

    def _push_sample(self, channel: ChannelName, width_us: float) -> None:
        """Validate and ingest one raw pulse-width sample.

        Out-of-range samples are dropped and counted; valid samples feed
        the debounce window and update the reported value.
        """
        if channel not in self._debouncers:
            return
        changed = False
        value: Optional[float] = None
        with self._lock:
            self._last_raw[channel] = float(width_us)
            if width_us < self._valid_min or width_us > self._valid_max:
                self._out_of_range[channel] += 1
                logger.debug(
                    "RC %s sample out of range: %.0fus (accepted %s~%s)",
                    channel,
                    width_us,
                    self._valid_min,
                    self._valid_max,
                )
                return
            self._histories[channel].append(float(width_us))
            self._last_valid_time[channel] = self._clock()
            changed = self._debouncers[channel].push(float(width_us))
            value = self._debouncers[channel].last_reported
            self._reported[channel] = value
        if changed and value is not None and self._on_change is not None:
            try:
                self._on_change(channel, value)
            except Exception:  # noqa: BLE001 - callback must never kill the sampler
                logger.exception("RC change callback failed for channel %s", channel)

    # ── Public read API ─────────────────────────────────────────────

    def read(
        self, channel: Optional[ChannelName] = None
    ) -> Union[Dict[str, Optional[float]], Optional[float]]:
        """Read the latest debounced pulse width(s) in us.

        With no argument returns a dict of all channels.  A channel
        without valid recent data (or in the initial state) returns
        ``None`` — never a stale/illegal value.
        """
        with self._lock:
            if channel is None:
                return {name: self._read_locked(name) for name in self.CHANNELS}
            return self._read_locked(channel)

    def _read_locked(self, channel: ChannelName) -> Optional[float]:
        if channel not in self._reported:
            return None
        if (
            self._stale_timeout > 0
            and self._clock() - self._last_valid_time[channel] > self._stale_timeout
        ):
            return None
        return self._reported[channel]

    @property
    def latest(self) -> Dict[str, Optional[float]]:
        """Latest debounced pulse widths (us) for every channel."""
        result = self.read()
        return result if isinstance(result, dict) else {}

    @property
    def raw_values(self) -> Dict[str, Optional[float]]:
        """Most recently sampled raw pulse width per channel (us)."""
        with self._lock:
            return dict(self._last_raw)

    @property
    def out_of_range_counts(self) -> Dict[str, int]:
        """Number of dropped out-of-range samples per channel."""
        with self._lock:
            return dict(self._out_of_range)

    @property
    def history(self) -> Dict[str, List[float]]:
        """Valid raw pulse-width history per channel (us)."""
        with self._lock:
            return {name: list(self._histories[name]) for name in self.CHANNELS}

    @property
    def last_value(self) -> Optional[float]:
        """Latest debounced CH1 value (us); convenience for mock usage."""
        value = self.read("CH1")
        return value if isinstance(value, float) else None

    def reset(self) -> None:
        """Clear all sampled data and reported values."""
        with self._lock:
            for name in self.CHANNELS:
                self._histories[name].clear()
                self._debouncers[name].window.clear()
                self._debouncers[name].last_reported = None
                self._reported[name] = None
                self._last_raw[name] = None
                self._last_valid_time[name] = 0.0
                self._out_of_range[name] = 0

    def stop(self) -> None:
        """Stop sampling and release resources (no-op for mock receivers)."""

    def close(self) -> None:
        """Alias for :meth:`stop`."""
        self.stop()


# ── Polling receiver base ──────────────────────────────────────────────


class _PollingRCReceiver(_BaseRCReceiver):
    """Shared polling / edge-detection machinery for hardware receivers.

    A daemon background thread polls every channel every ``poll_interval``
    seconds and computes high-pulse widths from ``time.perf_counter()``
    edge timestamps.  Concrete subclasses only supply the GPIO backend via
    :meth:`_setup_gpios` / :meth:`_default_value_reader` /
    :meth:`_release_gpios`; all debounce / validation / reporting logic is
    inherited from :class:`_BaseRCReceiver`.
    """

    def __init__(
        self,
        rc_pins: Optional[Mapping[str, int]] = None,
        poll_interval: float = 0.0002,
        value_reader: Optional[Callable[[int], int]] = None,
        auto_start: bool = True,
        **kwargs: object,
    ) -> None:
        super().__init__(rc_pins=rc_pins, **kwargs)
        self._poll_interval = float(poll_interval)
        if value_reader is None:
            self._value_reader: Callable[[int], int] = self._default_value_reader
        else:
            self._value_reader = value_reader
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_level: Dict[str, Optional[int]] = {
            name: None for name in self.CHANNELS
        }
        self._rising: Dict[str, Optional[float]] = {name: None for name in self.CHANNELS}
        self._available = False
        self._setup_gpios()
        if auto_start:
            self.start()

    # ── Backend hooks (overridden by concrete receivers) ──────────────

    def _default_value_reader(self, gpio: int) -> int:  # pragma: no cover
        raise NotImplementedError

    def _setup_gpios(self) -> None:  # pragma: no cover
        raise NotImplementedError

    def _release_gpios(self) -> None:  # pragma: no cover
        pass

    # ── Lifecycle ─────────────────────────────────────────────────────

    @property
    def is_available(self) -> bool:
        """Whether the GPIO backend was successfully configured."""
        return self._available

    def start(self) -> None:
        """Start the background sampling thread (idempotent)."""
        if not self._available:
            logger.warning("RC receiver unavailable (no GPIO backend) - not starting")
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._last_level = {name: None for name in self.CHANNELS}
        self._rising = {name: None for name in self.CHANNELS}
        self._thread = threading.Thread(
            target=self._sample_loop,
            name="rc-receiver-sampler",
            daemon=True,
        )
        self._thread.start()
        logger.info("RC receiver sampling thread started")

    def stop(self, unexport: bool = True) -> None:  # type: ignore[override]
        """Stop the sampling thread and (optionally) release the GPIOs."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if unexport:
            self._release_gpios()

    # ── Sampling loop ─────────────────────────────────────────────────

    def _sample_loop(self) -> None:
        """Poll every channel at the configured cadence until stopped.

        The cadence is kept with a busy-wait on the monotonic clock:
        ``sleep()`` granularity (>=1 ms, coarser on Windows) is too large
        to resolve a 1.5 ms pulse reliably, so the thread spins between
        scans instead.  A scan cycle completes in ~0.2 ms on the target
        board, which comfortably catches the 1000~2000 us pulses.
        """
        while not self._stop_event.is_set():
            self._poll_once()
            deadline = self._clock() + self._poll_interval
            while not self._stop_event.is_set() and self._clock() < deadline:
                pass  # busy-wait keeps sub-ms cadence on every platform

    def _poll_once(
        self,
        last_level: Optional[Dict[str, Optional[int]]] = None,
        rising: Optional[Dict[str, Optional[float]]] = None,
    ) -> None:
        """Scan every channel once and measure pulse edges (testable hook).

        On a rising edge the monotonic timestamp is recorded; on the next
        falling edge the high-pulse width is computed and ingested.
        Edge state persists across calls on ``self``.
        """
        if last_level is None:
            last_level = self._last_level
        if rising is None:
            rising = self._rising

        for name in self.CHANNELS:
            gpio = self.rc_pins[name]
            try:
                level = int(self._value_reader(gpio))
            except (OSError, ValueError, IOError):
                logger.debug("Failed to read GPIO %d for %s", gpio, name)
                continue

            now = self._clock()
            prev = last_level[name]
            if prev == 0 and level == 1:
                rising[name] = now
            elif prev == 1 and level == 0:
                t0 = rising[name]
                if t0 is not None:
                    width_us = (now - t0) * 1e6
                    if 0.0 < width_us < MAX_PULSE_WIDTH_US:
                        self._push_sample(name, width_us)
                rising[name] = None
            last_level[name] = level


# ── Real sysfs receiver ────────────────────────────────────────────────


class RCReceiver(_PollingRCReceiver):
    """Poll three sysfs GPIO inputs and measure RC PWM pulse widths.

    A daemon background thread polls every channel every ``poll_interval``
    seconds and computes high-pulse widths from ``time.perf_counter()``
    edge timestamps.  GPIO access happens only here, inside ``__init__``,
    so importing the module is safe on non-Linux hosts.

    Parameters
    ----------
    rc_pins : Mapping[str, int], optional
        sysfs GPIO number per channel (defaults to ``DEFAULT_RC_PINS``).
    poll_interval : float
        Seconds between channel scans (default 0.0002 s ≈ 0.2 ms).
    value_reader : Callable[[int], int], optional
        Overridable GPIO level reader (used by tests to simulate PWM).
    sysfs_dir : str, optional
        sysfs GPIO root directory (default ``/sys/class/gpio``; used by
        tests to emulate a fake GPIO tree).
    auto_start : bool
        Start the sampling thread in ``__init__`` (default ``True``).

    Any extra keyword arguments are forwarded to the base receiver
    (debounce / validation options).

    Raises
    ------
    DeviceError
        If the sysfs GPIOs cannot be exported / configured.
    """

    def __init__(
        self,
        rc_pins: Optional[Mapping[str, int]] = None,
        poll_interval: float = 0.0002,
        value_reader: Optional[Callable[[int], int]] = None,
        sysfs_dir: str = SYSFS_GPIO_DIR,
        auto_start: bool = True,
        **kwargs: object,
    ) -> None:
        self._sysfs_dir = sysfs_dir
        self._exported: List[int] = []
        super().__init__(
            rc_pins=rc_pins,
            poll_interval=poll_interval,
            value_reader=value_reader,
            auto_start=auto_start,
            **kwargs,
        )

    # ── Lifecycle ───────────────────────────────────────────────────

    def _default_value_reader(self, gpio: int) -> int:
        return _read_gpio_value(gpio, self._sysfs_dir)

    def _setup_gpios(self) -> None:
        """Export and configure the three input GPIOs; no-op without sysfs."""
        if not os.path.isdir(self._sysfs_dir):
            logger.warning(
                "sysfs GPIO not found at %s - RCReceiver disabled; "
                "use MockRCReceiver on non-Linux hosts",
                self._sysfs_dir,
            )
            return
        try:
            for name in self.CHANNELS:
                gpio = self.rc_pins[name]
                if not _export_gpio(gpio, self._sysfs_dir):
                    raise DeviceError(f"cannot export GPIO {gpio} for {name}")
                _open_gpio_perms(gpio, self._sysfs_dir)
                _set_gpio_direction(gpio, "in", self._sysfs_dir)
                self._exported.append(gpio)
        except (OSError, DeviceError) as exc:
            logger.error("RC receiver GPIO setup failed: %s", exc)
            for gpio in self._exported:
                try:
                    _unexport_gpio(gpio, self._sysfs_dir)
                except OSError:
                    pass
            self._exported = []
            return
        self._available = True
        logger.info("RC receiver GPIO ready: %s", self.rc_pins)

    def _release_gpios(self) -> None:
        for gpio in self._exported:
            try:
                _unexport_gpio(gpio, self._sysfs_dir)
            except OSError:
                pass
        self._exported = []


# ── libgpiod receiver (Jetson Orin NX) ────────────────────────────────


class LibgpiodRCReceiver(_PollingRCReceiver):
    """Measure RC PWM pulse widths via libgpiod (Jetson Orin NX).

    Uses the ``gpiod`` Python binding (v1 or v2 API) to request the three
    channel GPIO lines as inputs and poll their levels from the same edge
    detector as :class:`RCReceiver`.  Line offsets default to
    ``JETSON_RC_PINS``; on Jetson Orin NX the main ``tegra234-gpio`` chip
    numbers lines so the offset equals the legacy sysfs number (85/106/144).

    Parameters
    ----------
    rc_pins : Mapping[str, int], optional
        libgpiod line offset per channel (defaults to ``JETSON_RC_PINS``).
    pin_names : Mapping[str, str], optional
        Tegra port name per channel (defaults to ``JETSON_RC_PIN_NAMES``),
        used for runtime ``(chip, offset)`` probing via gpioinfo line names.
    gpiochip_paths : List[str], optional
        Explicit ``/dev/gpiochip*`` list (defaults to autodiscovery).
    poll_interval : float
        Seconds between channel scans (default 0.0002 s ≈ 0.2 ms).
    value_reader : Callable[[int], int], optional
        Overridable GPIO level reader (used by tests to simulate PWM).
    auto_start : bool
        Start the sampling thread in ``__init__`` (default ``True``).
    """

    DEFAULT_PINS: Mapping[str, int] = JETSON_RC_PINS

    def __init__(
        self,
        rc_pins: Optional[Mapping[str, int]] = None,
        pin_names: Optional[Mapping[str, str]] = None,
        gpiochip_paths: Optional[List[str]] = None,
        poll_interval: float = 0.0002,
        value_reader: Optional[Callable[[int], int]] = None,
        auto_start: bool = True,
        **kwargs: object,
    ) -> None:
        self._pin_names: Dict[str, str] = dict(pin_names or JETSON_RC_PIN_NAMES)
        self._gpiochip_paths = gpiochip_paths
        self._readers: Dict[int, Callable[[], int]] = {}
        self._releasers: Dict[int, Callable[[], None]] = {}
        super().__init__(
            rc_pins=rc_pins,
            poll_interval=poll_interval,
            value_reader=value_reader,
            auto_start=auto_start,
            **kwargs,
        )

    # ── Lifecycle ───────────────────────────────────────────────────

    def _default_value_reader(self, gpio: int) -> int:
        reader = self._readers.get(gpio)
        if reader is None:
            raise OSError(f"no libgpiod reader for line offset {gpio}")
        return reader()

    def _setup_gpios(self) -> None:
        """Request the three channel lines as inputs via libgpiod."""
        gpiod = _load_gpiod()
        if gpiod is None:
            logger.warning(
                "python 'gpiod' module not installed - LibgpiodRCReceiver "
                "disabled; install gpiod==1.6.3 or gpiod>=2 on the Jetson"
            )
            return
        version = _gpiod_api_version(gpiod)
        paths = (
            list(self._gpiochip_paths) if self._gpiochip_paths else _iter_gpiochip_paths()
        )
        if not paths:
            logger.warning(
                "no /dev/gpiochip* devices found - LibgpiodRCReceiver disabled"
            )
            return
        resolved = _resolve_libgpiod_lines(
            self.rc_pins, self._pin_names, gpiod, version, paths
        )
        try:
            for name in self.CHANNELS:
                chip_path, offset = resolved[name]
                reader, release = _open_input_line(gpiod, version, chip_path, offset)
                self._readers[offset] = reader
                self._releasers[offset] = release
        except (KeyError, OSError) as exc:
            logger.error("libgpiod setup failed: %s", exc)
            self._release_gpios()
            return
        self._available = True
        logger.info("RC receiver libgpiod ready: %s", resolved)

    def _release_gpios(self) -> None:
        """Release requested lines and close their chips."""
        for release in list(self._releasers.values()):
            try:
                release()
            except Exception:  # noqa: BLE001
                pass
        self._readers = {}
        self._releasers = {}


# ── Mock receiver (Windows / tests) ─────────────────────────────────────


class MockRCReceiver(_BaseRCReceiver):
    """Records injected pulse widths; no GPIO access.

    Implements the identical debounce / validation / reporting logic as
    :class:`RCReceiver` but samples are injected programmatically via
    :meth:`update` / :meth:`update_all` instead of being measured from
    hardware.  Safe to use on Windows and in unit tests.
    """

    def update(self, channel: object = "CH1", width_us: Optional[float] = None) -> Optional[float]:
        """Inject one raw pulse-width sample.

        Two calling forms are supported::

            receiver.update("CH1", 1500)   # explicit channel + value
            receiver.update(1500)          # shorthand for CH1

        Returns the debounced value of *channel* after the update.
        """
        if width_us is None:
            # Single-argument shorthand targets CH1.
            channel, width_us = "CH1", float(channel)  # type: ignore[arg-type]
        name = str(channel)
        if name not in self.CHANNELS:
            raise ValueError(
                f"unknown channel {name!r}; expected one of {self.CHANNELS}"
            )
        self._push_sample(name, float(width_us))
        return self.read(name)  # type: ignore[return-value]

    def update_all(self, widths: Mapping[str, float]) -> None:
        """Inject one sample per channel from a ``{channel: width_us}`` map."""
        for channel, width_us in widths.items():
            self.update(str(channel), float(width_us))


# ── Factory ─────────────────────────────────────────────────────────────

#: Keyword arguments that only the hardware receivers understand (never
#: forwarded to :class:`MockRCReceiver`).
_RECEIVER_ONLY_KWARGS = ("poll_interval", "value_reader", "sysfs_dir", "gpiochip_paths", "pin_names")


def create_rc_receiver(
    rc_pins: Optional[Mapping[str, int]] = None,
    auto_start: bool = True,
    **kwargs: object,
) -> _BaseRCReceiver:
    """Return the best available RC receiver for the current host.

    Backend selection order:

    1. :class:`RCReceiver` when sysfs GPIOs are present (RK3588 and other
       classic Linux boards with ``/sys/class/gpio``);
    2. :class:`LibgpiodRCReceiver` when libgpiod character devices and the
       ``gpiod`` Python binding are available (Jetson Orin NX, where sysfs
       GPIO has been removed);
    3. :class:`MockRCReceiver` otherwise, so the application keeps running
       (mirrors the placeholder-URL defensive pattern in ``http_client``).
    """
    sysfs_pins = dict(DEFAULT_RC_PINS)
    if rc_pins:
        sysfs_pins.update(rc_pins)
    common_kwargs = {k: v for k, v in kwargs.items() if k not in _RECEIVER_ONLY_KWARGS}

    # 1) sysfs GPIO (RK3588 / classic Linux).
    if os.path.isdir(SYSFS_GPIO_DIR):
        try:
            receiver = RCReceiver(rc_pins=sysfs_pins, auto_start=False, **kwargs)
        except Exception as exc:  # noqa: BLE001 - fall back on any init failure
            logger.warning("RCReceiver init failed (%s) - trying libgpiod", exc)
        else:
            if receiver.is_available:
                if auto_start:
                    receiver.start()
                return receiver
            receiver.stop(unexport=False)

    # 2) libgpiod (Jetson Orin NX / modern kernels without sysfs GPIO).
    if _libgpiod_available():
        jetson_pins = dict(JETSON_RC_PINS)
        if rc_pins:
            jetson_pins.update(rc_pins)
        libgpiod_kwargs = {k: v for k, v in kwargs.items() if k != "sysfs_dir"}
        try:
            receiver = LibgpiodRCReceiver(
                rc_pins=jetson_pins, auto_start=False, **libgpiod_kwargs
            )
        except Exception as exc:  # noqa: BLE001 - fall back on any init failure
            logger.warning("LibgpiodRCReceiver init failed (%s) - using MockRCReceiver", exc)
        else:
            if receiver.is_available:
                if auto_start:
                    receiver.start()
                return receiver
            receiver.stop(unexport=False)

    # 3) No real GPIO at all (Windows dev box / CI).
    logger.warning("no GPIO backend available - using MockRCReceiver (no real GPIO)")
    return MockRCReceiver(rc_pins=sysfs_pins, **common_kwargs)


#: Descriptive alias for the sysfs backend (RK3588 / classic Linux).
SysfsRCReceiver = RCReceiver


__all__ = [
    "DEFAULT_RC_PINS",
    "JETSON_RC_PINS",
    "JETSON_RC_PIN_NAMES",
    "LibgpiodRCReceiver",
    "MockRCReceiver",
    "RCReceiver",
    "SysfsRCReceiver",
    "create_rc_receiver",
]
