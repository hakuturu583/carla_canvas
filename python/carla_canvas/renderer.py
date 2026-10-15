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

import os
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .geometry import PinholeCamera, Transform
from .scene import Scene


@dataclass
class WorldInfo:
    map_name: str  # e.g. "Carla/Maps/Town10HD_Opt"
    episode_id: int
    config: dict[str, Any]  # the configuration's `renderer` section
    base_dir: Path | None = None  # where relative asset paths start
    opendrive: str = ""  # carla.Map.to_opendrive(): road network and geo-reference

    @property
    def map_basename(self) -> str:
        return self.map_name.rsplit("/", 1)[-1]

    def resolve(self, path: str) -> Path:
        """An asset path: ~ and $VARS expanded, relative to the configuration file."""
        p = Path(os.path.expandvars(os.path.expanduser(str(path))))
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
    #: float (N,) in [0, 1]: the renderer's own return intensity (e.g. a learned
    #: reflectance); replaces CARLA's exp(-attenuation * distance) when given
    intensity: np.ndarray | None = None


@dataclass
class LidarView:
    """One LiDAR measurement's rays, laid out as CARLA's ray-cast LiDAR fires
    them (carla_canvas.lidar): what :meth:`Renderer.cast_lidar_rays` answers."""

    scene: Scene
    pose: Transform  # the sensor's world transform
    directions: np.ndarray  # (N, 3) unit, sensor frame (x forward, y right, z up)
    channel: np.ndarray  # (N,) int: the laser of each ray (0: upper_fov)
    elevations: np.ndarray  # (channels,) degrees: laser i's pitch, descending
    azimuths: np.ndarray  # (N,) degrees: each ray's yaw (right positive)
    max_distance: float
    semantic: bool = False  # a semantic LiDAR: fill RayHits.semantic / actor_id / normal
    sensor_id: int = 0
    attributes: dict[str, str] = field(default_factory=dict)  # the blueprint's
    ignore_actors: frozenset[int] = frozenset()  # the LiDAR's own vehicle


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

    #: whether an overridden cast_lidar_rays (or cast_rays) fills the semantic
    #: LiDAR's fields (RayHits.semantic, actor_id, normal)
    semantic_lidar = False

    def cast_rays(self, scene: Scene, origins: np.ndarray, directions: np.ndarray,
                  max_distance: float, ignore_actors: frozenset[int] = frozenset()) -> RayHits:
        """Optional: first hits of N rays (world origins and unit directions,
        (N, 3)). LiDARs use it when supports_rays is true."""
        raise NotImplementedError

    def cast_lidar_rays(self, view: LidarView) -> RayHits:
        """A LiDAR measurement's hits. Override it to render LiDARs natively
        (e.g. a LiDAR-specific rasterizer); carla_canvas applies CARLA's LiDAR
        model around it either way. By default: :meth:`cast_rays` when the
        renderer has it, else depth images (carla_canvas.lidar)."""
        if self.supports_rays:
            world = view.directions @ view.pose.rotation_matrix().T
            origins = np.broadcast_to(np.asarray(view.pose.location, float), world.shape)
            return self.cast_rays(view.scene, origins, world, view.max_distance, view.ignore_actors)
        if "depth" not in self.camera_outputs:
            raise NotImplementedError(f"renderer '{self.name}' cannot render LiDARs")
        from .lidar import cast_rays_from_depth

        return cast_rays_from_depth(
            self, view.scene, view.pose, view.directions, view.max_distance, view.sensor_id,
            semantic=view.semantic, ignore_actors=view.ignore_actors,
            pixels_per_degree=float(self.options.get("lidar_pixels_per_degree", 8.0)))

    @property
    def supports_rays(self) -> bool:
        return type(self).cast_rays is not Renderer.cast_rays

    @property
    def lidar_types(self) -> list[str]:
        """The LiDAR blueprints this renderer can render."""
        native = type(self).cast_lidar_rays is not Renderer.cast_lidar_rays
        if not (native or self.supports_rays or "depth" in self.camera_outputs):
            return []
        semantic = (self.semantic_lidar if native or self.supports_rays
                    else "semantic" in self.camera_outputs)
        return ["sensor.lidar.ray_cast"] + (["sensor.lidar.ray_cast_semantic"] if semantic else [])

    def close(self) -> None:
        """Release resources (the service is shutting down)."""
