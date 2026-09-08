"""Grain Sampling — Point Cloud Processing

Provides LiDAR point cloud collection (Livox Mid-360), 2D slice extraction,
preview image generation, and cloud upload capabilities.
"""

from .slice_extractor import PointCloudSlicer
from .preview_generator import PreviewGenerator
from .uploader import MapUploader

# collector depends on rospy (ROS) — not always available on dev machines
try:
    from .collector import PointCloudCollector  # noqa: F401
except ImportError:
    PointCloudCollector = None  # type: ignore[assignment,misc]

__all__ = [
    "PointCloudCollector",
    "PointCloudSlicer",
    "PreviewGenerator",
    "MapUploader",
]
