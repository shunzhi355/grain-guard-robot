"""Preview PNG generator for 2D point cloud slice contours.

Renders contour points, grid overlay, scale bar, and labels on a
dark-themed 800×600 canvas suitable for operator UI display.
"""

from __future__ import annotations

import io
import math
from typing import List, Dict, Optional, Tuple

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = None  # type: ignore[assignment,misc]
    ImageDraw = None  # type: ignore[assignment,misc]
    ImageFont = None  # type: ignore[assignment,misc]


class PreviewGenerator:
    """Generates preview PNG images from 2D contour point data.

    Parameters
    ----------
    width : int
        Image width in pixels (default ``800``).
    height : int
        Image height in pixels (default ``600``).
    background : str
        Background colour as a hex string (default ``"#0D1117"``, GitHub dark).
    margin : int
        Plot area margin in pixels (default ``60``).
    """

    # ── colour palette ──────────────────────────────────────────────────

    COLOUR_BG = "#0D1117"
    COLOUR_GRID = "#21262D"
    COLOUR_POINT = "#FFFFFF"
    COLOUR_AXIS = "#8B949E"
    COLOUR_SCALE = "#58A6FF"

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        background: str = COLOUR_BG,
        margin: int = 60,
    ) -> None:
        self._width = width
        self._height = height
        self._background = background or self.COLOUR_BG
        self._margin = margin

        if Image is None:
            raise ImportError(
                "Pillow (PIL) is required for PreviewGenerator. "
                "Install with: pip install Pillow"
            )

    # ── public API ──────────────────────────────────────────────────────

    def generate_preview(
        self,
        contour_points: List[Dict[str, float]],
        map_bounds: Tuple[float, float, float, float],
    ) -> bytes:
        """Render a PNG preview of contour points with grid and scale bar.

        Parameters
        ----------
        contour_points : list[dict]
            Contour points as ``[{"x": ..., "y": ...}, ...]``.
        map_bounds : tuple
            Map extent ``(xmin, xmax, ymin, ymax)`` in metres.

        Returns
        -------
        bytes
            PNG image bytes (ready for file save or UI display).
        """
        if not contour_points:
            return self._generate_placeholder()

        img = Image.new("RGB", (self._width, self._height), self._background)
        draw = ImageDraw.Draw(img)

        xmin, xmax, ymin, ymax = map_bounds
        plot_w, plot_h = self._plot_dims()

        # Ensure non-zero extents
        if xmax == xmin:
            xmax = xmin + 1.0
        if ymax == ymin:
            ymax = ymin + 1.0

        def _world_to_pixel(wx: float, wy: float) -> Tuple[int, int]:
            px = self._margin + int((wx - xmin) / (xmax - xmin) * plot_w)
            py = self._height - self._margin - int((wy - ymin) / (ymax - ymin) * plot_h)
            return px, py

        # ── grid lines ───────────────────────────────────────────────
        self._draw_grid(draw, xmin, xmax, ymin, ymax, _world_to_pixel)

        # ── contour points ───────────────────────────────────────────
        for pt in contour_points:
            px, py = _world_to_pixel(pt["x"], pt["y"])
            draw.ellipse(
                [px - 2, py - 2, px + 2, py + 2],
                fill=self.COLOUR_POINT,
            )

        # ── bounding box ─────────────────────────────────────────────
        bx0, by0 = _world_to_pixel(xmin, ymin)
        bx1, by1 = _world_to_pixel(xmax, ymax)
        draw.rectangle([bx0, by1, bx1, by0], outline=self.COLOUR_AXIS, width=1)

        # ── axis labels ──────────────────────────────────────────────
        self._draw_axis_labels(draw, xmin, xmax, ymin, ymax)

        # ── scale bar ────────────────────────────────────────────────
        self._draw_scale_bar(draw, xmin, xmax, ymin, ymax)

        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()

    # ── placeholder ─────────────────────────────────────────────────────

    def _generate_placeholder(self) -> bytes:
        """Return a placeholder image with '无数据' (no data) text."""
        img = Image.new("RGB", (self._width, self._height), self._background)
        draw = ImageDraw.Draw(img)
        text = "\u65e0\u6570\u636e"  # 无数据
        try:
            font = self._get_font(48)
        except Exception:
            font = None
        bbox = draw.textbbox((0, 0), text, font=font)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]
        x = (self._width - tw) // 2
        y = (self._height - th) // 2
        draw.text((x, y), text, fill=self.COLOUR_AXIS, font=font)
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()

    # ── drawing helpers ─────────────────────────────────────────────────

    def _plot_dims(self) -> Tuple[int, int]:
        return (
            self._width - 2 * self._margin,
            self._height - 2 * self._margin,
        )

    def _draw_grid(
        self,
        draw: ImageDraw.ImageDraw,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
        w2p: callable,
        grid_lines: int = 10,
    ) -> None:
        """Draw light grid lines inside the plot area."""
        for i in range(grid_lines + 1):
            frac = i / grid_lines
            # vertical
            wx = xmin + frac * (xmax - xmin)
            px, py0 = w2p(wx, ymin)
            _px, py1 = w2p(wx, ymax)
            draw.line([px, py0, px, py1], fill=self.COLOUR_GRID, width=1)
            # horizontal
            wy = ymin + frac * (ymax - ymin)
            px0, py = w2p(xmin, wy)
            px1, _py = w2p(xmax, wy)
            draw.line([px0, py, px1, py], fill=self.COLOUR_GRID, width=1)

    def _draw_axis_labels(
        self,
        draw: ImageDraw.ImageDraw,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
    ) -> None:
        """Draw numeric labels along the plot border."""
        try:
            font = self._get_font(12)
        except Exception:
            font = None

        plot_w, plot_h = self._plot_dims()

        # X-axis labels (bottom)
        for i in range(6):
            frac = i / 5.0
            val = xmin + frac * (xmax - xmin)
            label = f"{val:.1f}"
            px = self._margin + int(frac * plot_w)
            py = self._height - self._margin + 8
            bbox = draw.textbbox((0, 0), label, font=font)
            tw = bbox[2] - bbox[0]
            draw.text(
                (px - tw // 2, py), label, fill=self.COLOUR_AXIS, font=font
            )

        # Y-axis labels (left)
        for i in range(6):
            frac = i / 5.0
            val = ymin + frac * (ymax - ymin)
            label = f"{val:.1f}"
            py = self._height - self._margin - int(frac * plot_h)
            bbox = draw.textbbox((0, 0), label, font=font)
            tw = bbox[2] - bbox[0]
            th = bbox[3] - bbox[1]
            draw.text(
                (self._margin - tw - 8, py - th // 2),
                label,
                fill=self.COLOUR_AXIS,
                font=font,
            )

    def _draw_scale_bar(
        self,
        draw: ImageDraw.ImageDraw,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
    ) -> None:
        """Draw a scale bar in the bottom-left corner of the plot."""
        extent = xmax - xmin
        # Pick a "nice" scale length
        nice = self._nice_scale_length(extent)
        if nice <= 0:
            return

        plot_w, _plot_h = self._plot_dims()
        bar_px = int(nice / extent * plot_w)
        x0 = self._margin + 10
        y0 = self._height - self._margin - 20
        x1 = x0 + bar_px

        # Bar
        draw.line([x0, y0, x1, y0], fill=self.COLOUR_SCALE, width=4)
        draw.line([x0, y0 - 4, x0, y0 + 4], fill=self.COLOUR_SCALE, width=2)
        draw.line([x1, y0 - 4, x1, y0 + 4], fill=self.COLOUR_SCALE, width=2)

        # Label
        try:
            font = self._get_font(11)
        except Exception:
            font = None
        label = f"{nice:.2f} m"
        bbox = draw.textbbox((0, 0), label, font=font)
        tw = bbox[2] - bbox[0]
        draw.text(
            (x0 + (bar_px - tw) // 2, y0 + 6),
            label,
            fill=self.COLOUR_SCALE,
            font=font,
        )

    # ── utilities ───────────────────────────────────────────────────────

    @staticmethod
    def _nice_scale_length(extent: float) -> float:
        """Return a "nice" scale bar length ≈ 25-35% of extent."""
        target = extent * 0.3
        if target <= 0:
            return 0.0
        exp = math.floor(math.log10(target))
        mantissa = target / (10 ** exp)
        # Pick nice mantissa: 1, 2, 2.5, 5
        nice_m = 1.0
        for candidate in (1.0, 2.0, 2.5, 5.0, 10.0):
            if candidate >= mantissa:
                nice_m = candidate
                break
        return nice_m * (10 ** exp)

    @staticmethod
    def _get_font(size: int) -> Optional[ImageFont.FreeTypeFont]:
        """Try to load a system font; fall back to default."""
        try:
            # Common Windows / Linux / macOS paths
            for path in (
                "C:/Windows/Fonts/msyh.ttc",       # Microsoft YaHei
                "C:/Windows/Fonts/simsun.ttc",      # SimSun
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "/System/Library/Fonts/Helvetica.ttc",
            ):
                try:
                    return ImageFont.truetype(path, size)
                except (OSError, IOError):
                    continue
        except Exception:
            pass
        return None
