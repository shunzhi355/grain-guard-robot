"""Map widget — centre area rendering warehouse map with robot pose overlay.

Renders an occupancy grid, robot position (blue triangle + direction arrow),
numbered waypoint markers, and a green dashed navigation path.

Supports mouse pan and wheel zoom.  Displays world coordinates in a
semi-transparent overlay label.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from PySide2.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PySide2.QtGui import (
    QBrush,
    QColor,
    QFont,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PySide2.QtWidgets import QFrame, QLabel, QSizePolicy, QVBoxLayout, QWidget


# ── Constants ─────────────────────────────────────────────┬
MAP_BG = QColor("#0D1117")
GRID_COLOR = QColor("#1C2333")
OCCUPIED_COLOR = QColor("#30363D")
FREE_COLOR = QColor("#161B22")
UNKNOWN_COLOR = QColor("#0D1117")
PATH_COLOR = QColor("#2EA043")
ROBOT_COLOR = QColor("#58A6FF")
WAYPOINT_COLOR = QColor("#D4A72C")
WAYPOINT_TEXT_COLOR = QColor("#0D1117")
AXIS_COLOR = QColor("#21262D")

CELL_SIZE_PX = 20  # base cell size at zoom=1.0
MIN_ZOOM = 0.1
MAX_ZOOM = 8.0
ROBOT_TRIANGLE_SIZE = 18  # px


# ── Data containers ────────────────────────────────────────


@dataclass
class OccupancyGrid:
    """2-D occupancy grid received from the ROS /map topic."""

    width: int = 100
    height: int = 100
    resolution: float = 0.05  # m/cell
    origin_x: float = -2.5  # world x of cell (0,0)
    origin_y: float = -2.5  # world y of cell (0,0)
    data: list[int] = field(default_factory=list)  # flat, row-major; -1 unknown

    @classmethod
    def default_grid(cls) -> OccupancyGrid:
        """Return a 5 m x 5 m empty grid at 0.05 m resolution."""
        g = cls(width=100, height=100, resolution=0.05, origin_x=-2.5, origin_y=-2.5)
        g.data = [-1] * (g.width * g.height)  # all unknown
        return g


# ── Widget ─────────────────────────────────────────────────


class MapWidget(QWidget):
    """Interactive map display for the grain sampling robot.

    Renders occupancy grid, robot pose, waypoints, and navigation path.
    Supports pan (left-drag) and zoom (scroll wheel).
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("map_widget")
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self.setMinimumSize(400, 300)

        # ── Internal state ──────────────────────────────────
        self._occupancy_grid = OccupancyGrid.default_grid()
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._waypoints: list[tuple[float, float, str]] = []  # (x, y, label)
        self._nav_path: list[tuple[float, float]] = []
        self._cloud_img: QImage | None = None  # 2D point cloud projection

        # ── View transform ──────────────────────────────────
        self._zoom: float = 1.0
        self._offset_x: float = 0.0  # world coords at centre of widget
        self._offset_y: float = 0.0
        self._panning: bool = False
        self._last_pan_pos: QPoint | None = None

        # ── UI ──────────────────────────────────────────────
        self._coord_label: QLabel | None = None
        self._setup_ui()

        # Enable mouse tracking for coordinate display
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # ── UI construction ─────────────────────────────────────

    def _setup_ui(self) -> None:
        """Create the coordinate overlay label."""
        self._coord_label = QLabel("X: ---  Y: ---", self)
        self._coord_label.setObjectName("map_coord_label")
        self._coord_label.setStyleSheet(
            "QLabel#map_coord_label {"
            "  background-color: rgba(13, 17, 23, 180);"
            "  color: #58A6FF;"
            "  font-size: 11pt;"
            "  font-family: 'Consolas', 'Courier New', monospace;"
            "  padding: 4px 10px;"
            "  border: 1px solid #30363D;"
            "  border-radius: 4px;"
            "}"
        )
        self._coord_label.adjustSize()
        self._coord_label.move(8, 8)

    # ── Public data slots ───────────────────────────────────

    def update_odometry(self, x: float, y: float, yaw: float) -> None:
        """Slot for ROS /odometry/filtered data."""
        self._robot_x = x
        self._robot_y = y
        self._robot_yaw = yaw
        self.update()

    def update_map(self, data: dict) -> None:
        """Slot for ROS /map (JSON) data.

        Expected keys in *data*:
            width, height, resolution, origin_x, origin_y, cells
        """
        try:
            self._occupancy_grid = OccupancyGrid(
                width=data.get("width", 100),
                height=data.get("height", 100),
                resolution=data.get("resolution", 0.05),
                origin_x=data.get("origin_x", -2.5),
                origin_y=data.get("origin_y", -2.5),
                data=data.get("cells", []),
            )
        except (TypeError, KeyError, ValueError):
            # Keep existing grid on malformed data
            pass
        self.update()

    def update_waypoints(
        self, waypoints: list[tuple[float, float, str]]
    ) -> None:
        """Set sample-point waypoints."""
        self._waypoints = waypoints
        self.update()

    def update_nav_path(self, path: list[tuple[float, float]]) -> None:
        """Set the navigation path for the robot."""
        self._nav_path = path
        self.update()

    def reset_view(self) -> None:
        """Reset zoom/pan to default centred on origin."""
        self._zoom = 1.0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self.update()

    def set_pointcloud_image(self, img: QImage) -> None:
        """Set a 2D point cloud projection image to render as background."""
        self._cloud_img = img
        self.update()

    # ── Coordinate transforms ───────────────────────────────

    def _world_to_widget(self, wx: float, wy: float) -> QPointF:
        """Convert world coordinates (metres) → widget pixel position."""
        cx, cy = self._widget_centre()
        px = (wx - self._offset_x) * CELL_SIZE_PX * self._zoom + cx
        # y-axis flips: positive world-y goes up, positive widget-y goes down
        py = -(wy - self._offset_y) * CELL_SIZE_PX * self._zoom + cy
        return QPointF(px, py)

    def _widget_to_world(self, px: float, py: float) -> tuple[float, float]:
        """Convert widget pixel position → world coordinates (metres)."""
        cx, cy = self._widget_centre()
        wx = (px - cx) / (CELL_SIZE_PX * self._zoom) + self._offset_x
        wy = -(py - cy) / (CELL_SIZE_PX * self._zoom) + self._offset_y
        return wx, wy

    def _widget_centre(self) -> tuple[float, float]:
        """Return the widget centre in pixels."""
        return self.width() / 2.0, self.height() / 2.0

    # ── Paint ───────────────────────────────────────────────

    def paintEvent(self, _event: Any) -> None:  # noqa: D401
        """Render map, robot, waypoints, and path."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), MAP_BG)

        # Point cloud 2D projection (background layer)
        if self._cloud_img is not None and not self._cloud_img.isNull():
            painter.drawImage(self.rect(), self._cloud_img)

        self._draw_grid(painter)
        self._draw_occupancy(painter)
        self._draw_axes(painter)
        self._draw_nav_path(painter)
        self._draw_waypoints(painter)
        self._draw_robot(painter)

        painter.end()

    def _draw_grid(self, painter: QPainter) -> None:
        """Draw a light coordinate grid."""
        pen = QPen(GRID_COLOR, 1, Qt.PenStyle.SolidLine)
        painter.setPen(pen)

        w, h = self.width(), self.height()
        step = int(CELL_SIZE_PX * self._zoom * 5)  # grid every 5 cells
        if step < 5:
            return  # too dense to render

        for x in range(0, w, step):
            painter.drawLine(x, 0, x, h)
        for y in range(0, h, step):
            painter.drawLine(0, y, w, y)

    def _draw_occupancy(self, painter: QPainter) -> None:
        """Draw occupancy grid cells."""
        grid = self._occupancy_grid
        cell_px = CELL_SIZE_PX * self._zoom
        if cell_px < 1.5:
            return  # skip when zoomed too far out

        cell_w = max(1, int(cell_px))

        for row in range(grid.height):
            for col in range(grid.width):
                idx = row * grid.width + col
                if idx >= len(grid.data):
                    continue
                val = grid.data[idx]

                if val == -1:
                    continue  # unknown → already bg colour
                elif val > 50:
                    c = OCCUPIED_COLOR
                else:
                    c = FREE_COLOR

                wx = grid.origin_x + col * grid.resolution
                wy = grid.origin_y + row * grid.resolution
                pt = self._world_to_widget(wx, wy)

                painter.fillRect(
                    QRectF(pt.x(), pt.y(), cell_w, cell_w),
                    QBrush(c),
                )

    @staticmethod
    def _draw_axes(painter: QPainter) -> None:
        """Draw X/Y axis markers in bottom-left corner."""
        # Not drawn via paintEvent's painter — we just skip for simplicity
        # in initial implementation; coordinates are shown in the overlay.
        pass

    def _draw_nav_path(self, painter: QPainter) -> None:
        """Draw navigation path as a green dashed polyline."""
        if len(self._nav_path) < 2:
            return

        pen = QPen(PATH_COLOR, 2, Qt.PenStyle.DashLine)
        pen.setDashPattern([6, 4])
        painter.setPen(pen)

        path = QPainterPath()
        first = True
        for wx, wy in self._nav_path:
            pt = self._world_to_widget(wx, wy)
            if first:
                path.moveTo(pt)
                first = False
            else:
                path.lineTo(pt)
        painter.drawPath(path)

    def _draw_waypoints(self, painter: QPainter) -> None:
        """Draw waypoint markers as numbered circles."""
        radius = max(6, int(10 * self._zoom))
        font = QFont("Microsoft YaHei", max(9, int(10 * self._zoom)))
        font.setBold(True)

        for i, (wx, wy, label) in enumerate(self._waypoints):
            pt = self._world_to_widget(wx, wy)

            # Circle
            painter.setPen(QPen(WAYPOINT_COLOR, 2))
            painter.setBrush(QBrush(WAYPOINT_COLOR))
            painter.drawEllipse(pt, radius, radius)

            # Number
            painter.setPen(QPen(WAYPOINT_TEXT_COLOR))
            painter.setFont(font)
            display = label if label else str(i + 1)
            r = QRectF(
                pt.x() - radius, pt.y() - radius,
                radius * 2, radius * 2,
            )
            painter.drawText(r, Qt.AlignmentFlag.AlignCenter, display)

    def _draw_robot(self, painter: QPainter) -> None:
        """Draw the robot as a blue triangle with a directional arrow."""
        pt = self._world_to_widget(self._robot_x, self._robot_y)

        # Triangle vertices (pointing up when yaw=0, i.e. toward +x in world)
        half = ROBOT_TRIANGLE_SIZE / 2.0
        # base triangle in local space (pointing right = 0 rad in math convention)
        tri_local = QPolygonF([
            QPointF(half, 0),                          # nose
            QPointF(-half, -half * 0.65),              # bottom-left
            QPointF(-half, half * 0.65),               # top-left
        ])

        # Rotate around origin and translate
        transformed = QPolygonF()
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        for i in range(tri_local.size()):
            lx = tri_local[i].x()
            ly = tri_local[i].y()
            rx = lx * cos_yaw - ly * sin_yaw
            ry = lx * sin_yaw + ly * cos_yaw
            transformed.append(QPointF(pt.x() + rx, pt.y() - ry))

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(ROBOT_COLOR))
        painter.drawPolygon(transformed)

        # Direction line
        painter.setPen(QPen(ROBOT_COLOR.lighter(150), 2))
        dir_len = ROBOT_TRIANGLE_SIZE * 1.6
        tip = QPointF(
            pt.x() + dir_len * cos_yaw,
            pt.y() - dir_len * sin_yaw,
        )
        painter.drawLine(pt, tip)

    # ── Mouse / touch interaction ───────────────────────────

    def mousePressEvent(self, event: Any) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()

    def mouseMoveEvent(self, event: Any) -> None:
        if self._panning and self._last_pan_pos is not None:
            delta = event.pos() - self._last_pan_pos
            self._offset_x -= delta.x() / (CELL_SIZE_PX * self._zoom)
            self._offset_y += delta.y() / (CELL_SIZE_PX * self._zoom)
            self._last_pan_pos = event.pos()
            self.update()

        # Update coordinate overlay
        wx, wy = self._widget_to_world(event.pos().x(), event.pos().y())
        if self._coord_label:
            self._coord_label.setText(f"X: {wx:.2f}  Y: {wy:.2f}")

        event.accept()

    def mouseReleaseEvent(self, event: Any) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self._panning = False
            self._last_pan_pos = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()

    def wheelEvent(self, event: Any) -> None:
        """Zoom in/out centred on cursor position."""
        old_zoom = self._zoom
        delta = event.angleDelta().y()
        factor = 1.15 if delta > 0 else 1.0 / 1.15
        new_zoom = max(MIN_ZOOM, min(MAX_ZOOM, old_zoom * factor))

        # Zoom toward cursor
        wx, wy = self._widget_to_world(
            event.pos().x(), event.pos().y(),
        )
        self._zoom = new_zoom
        # Adjust offset to keep cursor point stationary
        ratio = old_zoom / new_zoom
        self._offset_x = wx - (wx - self._offset_x) * ratio
        self._offset_y = wy - (wy - self._offset_y) * ratio

        self.update()
        event.accept()

    # ── Resize ──────────────────────────────────────────────

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        # Keep coordinate label at top-left
        if self._coord_label:
            self._coord_label.move(8, 8)
