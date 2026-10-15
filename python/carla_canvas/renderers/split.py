"""One backend for cameras and another for LiDARs (``renderer.camera`` /
``renderer.lidar`` in the configuration): e.g. splatsim's Gaussian splatting
for cameras and a ray-traced mesh renderer for LiDARs. Each backend loads the
world with its own section of the configuration (its assets and options)."""

from __future__ import annotations

from ..config import merge, renderer_section
from ..renderer import CameraFrame, CameraView, LidarView, RayHits, Renderer, WorldInfo


class SplitRenderer(Renderer):
    """Cameras from `camera`, LiDARs from `lidar` (through its own
    cast_lidar_rays: its LiDAR rasterizer, ray casts or depth images)."""

    def __init__(self, camera: Renderer, lidar: Renderer) -> None:
        super().__init__({})
        self.camera = camera
        self.lidar = lidar
        self.name = f"camera: {camera.name}, lidar: {lidar.name}"
        self.version = f"{camera.version}, {lidar.version}"
        self.camera_outputs = camera.camera_outputs

    def load_world(self, world: WorldInfo) -> None:
        for kind, renderer in (("camera", self.camera), ("lidar", self.lidar)):
            section = renderer_section(world.config, kind)
            renderer.options = merge(renderer.options, section.get("options") or {})
            renderer.load_world(WorldInfo(world.map_name, world.episode_id, section,
                                          world.base_dir, world.opendrive))

    def render_camera(self, view: CameraView) -> CameraFrame:
        return self.camera.render_camera(view)

    def cast_lidar_rays(self, view: LidarView) -> RayHits:
        return self.lidar.cast_lidar_rays(view)

    @property
    def lidar_types(self) -> list[str]:
        return self.lidar.lidar_types

    def close(self) -> None:
        self.camera.close()
        self.lidar.close()
