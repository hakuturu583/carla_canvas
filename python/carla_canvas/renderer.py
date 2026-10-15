"""The interface a renderer implements.

A renderer draws what a camera at a pose sees (:meth:`Renderer.render_camera`)
and, optionally, answers ray queries (:meth:`Renderer.cast_rays`). Everything
CARLA-specific is done by carla_canvas around it: sensor attributes, CARLA's
image encodings (depth, segmentation), the LiDAR model (scan pattern, partial
sweeps per tick, intensity, drop-off, noise), the measurement layouts, the
gRPC protocol and the ROS 2 topics. A renderer without ray queries still
gets LiDARs: carla_canvas samples depth images instead.

Coordinates are CARLA's (meters, Unreal's left-handed x forward / y right /
z up; see :mod:`carla_canvas.geometry`).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import PinholeCamera
from .scene import Scene


@dataclass
class WorldInfo:
    map_name: str  # e.g. "Carla/Maps/Town10HD_Opt"
    episode_id: int
    config: dict[str, Any]  # the configuration's `renderer` section
    base_dir: Path | None = None  # where relative asset paths start

    @property
    def map_basename(self) -> str:
        return self.map_name.rsplit("/", 1)[-1]

    def resolve(self, path: str) -> Path:
        p = Path(path).expanduser()
        if not p.is_absolute() and self.base_dir is not None:
            p = self.base_dir / p
        return p


@dataclass
class CameraView:
    camera: PinholeCamera  # its pose is the sensor's world transform
    scene: Scene
    outputs: frozenset[str]  # subset of {"rgb", "depth", "semantic", "instance"}
    sensor_id: int = 0
    attributes: dict[str, str] = field(default_factory=dict)  # the sensor blueprint's
    max_depth: float = 1000.0
    #: actors not to draw (a LiDAR does not see its own vehicle)
    ignore_actors: frozenset[int] = frozenset()


@dataclass
class CameraFrame:
    """What render_camera returns; arrays are (height, width, ...)."""

    rgb: np.ndarray | None = None  # uint8 (H, W, 3), RGB order
    depth: np.ndarray | None = None  # float32 (H, W): distance along the camera's x, meters; inf: nothing
    semantic: np.ndarray | None = None  # uint8 (H, W): carla.CityObjectLabel
    actor_id: np.ndarray | None = None  # uint32 (H, W): CARLA actor id, 0: none


@dataclass
class RayHits:
    """What cast_rays returns for N rays."""

    distance: np.ndarray  # float (N,): along the ray, inf: no hit
    normal: np.ndarray | None = None  # float (N, 3) world, facing the ray
    semantic: np.ndarray | None = None  # uint8 (N,)
    actor_id: np.ndarray | None = None  # uint32 (N,)


class Renderer(ABC):
    """Subclass and register it (``renderer.backend`` in the configuration:
    a built-in name or ``"package.module:Class"``)."""

    #: shown by GetInfo and in the client's log
    name = "renderer"
    version = "0"
    #: what render_camera can produce
    camera_outputs: frozenset[str] = frozenset({"rgb"})

    def __init__(self, options: dict[str, Any] | None = None) -> None:
        self.options = dict(options or {})

    @abstractmethod
    def load_world(self, world: WorldInfo) -> None:
        """Load the map's assets (called again after load_world on the client)."""

    @abstractmethod
    def render_camera(self, view: CameraView) -> CameraFrame:
        """Render the outputs view.outputs asks for (a subset of camera_outputs)."""

    def cast_rays(self, scene: Scene, origins: np.ndarray, directions: np.ndarray,
                  max_distance: float, ignore_actors: frozenset[int] = frozenset()) -> RayHits:
        """Optional: first hits of N rays (world origins and unit directions,
        (N, 3)). LiDARs use it when supports_rays is true."""
        raise NotImplementedError

    @property
    def supports_rays(self) -> bool:
        return type(self).cast_rays is not Renderer.cast_rays

    def close(self) -> None:
        """Release resources (the service is shutting down)."""
