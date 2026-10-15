"""carla_canvas: swap CARLA's camera and LiDAR rendering behind the CARLA API.

A typesafe_carla program runs unchanged; with ``CARLA_CANVAS_CONFIG`` naming a
configuration file, the sensors it lists are rendered by a renderer service
(3D Gaussian splatting, a reference renderer, or your own) over gRPC instead
of the CARLA server, and their ROS 2 topics are published by carla_canvas.

Renderer authors subclass :class:`carla_canvas.Renderer`; see docs/renderers.md.
"""

__version__ = "0.2.0"

from .config import CanvasConfig, load_config  # noqa: E402
from .geometry import Transform  # noqa: E402
from .renderer import (  # noqa: E402
    CameraFrame,
    CameraView,
    LidarView,
    RayHits,
    Renderer,
    WorldInfo,
)
from .scene import ActorInfo, Scene  # noqa: E402

__all__ = [
    "ActorInfo",
    "CameraFrame",
    "CameraView",
    "CanvasConfig",
    "LidarView",
    "RayHits",
    "Renderer",
    "Scene",
    "Transform",
    "WorldInfo",
    "load_config",
]
