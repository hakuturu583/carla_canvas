"""A reference renderer: the ground plane and every actor's bounding box,
ray traced with numpy.

It needs no assets, renders every output (RGB, depth, semantic and instance
segmentation) and answers ray queries, so it exercises the whole pipeline
(cameras, both LiDARs, ROS 2) and is the template for a new renderer.

Options (``renderer.options``):

* ``ground_z`` (0.0): height of the ground plane, meters; ``null``: none;
* ``sky_color`` ([135, 180, 235]), ``ground_color`` ([90, 90, 90]);
* ``light_direction`` ([0.3, -0.4, -1.0]): direction the light travels.
"""

from __future__ import annotations

import zlib

import numpy as np

from ..geometry import Transform
from ..renderer import CameraFrame, CameraView, RayHits, Renderer, WorldInfo
from ..scene import CityObjectLabel, Scene


class PrimitiveRenderer(Renderer):
    name = "primitives"
    version = "1"
    camera_outputs = frozenset({"rgb", "depth", "semantic", "instance"})

    def __init__(self, options=None):
        super().__init__(options)
        self._read_options()

    def _read_options(self) -> None:
        o = self.options
        self.ground_z = o.get("ground_z", 0.0)
        self.sky = np.array(o.get("sky_color", [135, 180, 235]), float)
        self.ground = np.array(o.get("ground_color", [90, 90, 90]), float)
        light = np.array(o.get("light_direction", [0.3, -0.4, -1.0]), float)
        self.light = light / np.linalg.norm(light)

    def load_world(self, world: WorldInfo) -> None:
        self._read_options()  # the service resolved this world's options

    # --- ray casting ---------------------------------------------------------------

    def cast_rays(self, scene: Scene, origins: np.ndarray, directions: np.ndarray,
                  max_distance: float, ignore_actors: frozenset[int] = frozenset()) -> RayHits:
        n = len(directions)
        dist = np.full(n, np.inf)
        normal = np.zeros((n, 3))
        semantic = np.zeros(n, np.uint8)
        actor = np.zeros(n, np.uint32)
        origins = np.asarray(origins, float)
        directions = np.asarray(directions, float)

        if self.ground_z is not None:
            dz = directions[:, 2]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (self.ground_z - origins[:, 2]) / dz
            hit = (dz < -1e-9) & (t > 0) & (t < dist)
            dist[hit] = t[hit]
            normal[hit] = (0.0, 0.0, 1.0)
            semantic[hit] = CityObjectLabel.Roads

        for info, transform in scene.bodies(exclude=set(ignore_actors)):
            b = info.bounding_box
            box = Transform(b.location, b.rotation)
            # into the box's frame
            world_to_box = box.inverse_matrix() @ transform.inverse_matrix()
            o = origins @ world_to_box[:3, :3].T + world_to_box[:3, 3]
            d = directions @ world_to_box[:3, :3].T
            ext = np.asarray(b.extent, float)
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / d
                t1 = (-ext - o) * inv
                t2 = (ext - o) * inv
            tmin = np.nanmax(np.minimum(t1, t2), axis=1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 1e-6) & (tmin < dist)
            if not hit.any():
                continue
            dist[hit] = tmin[hit]
            # the face entered: the axis whose slab gave tmin
            idx = np.nonzero(hit)[0]
            near = np.minimum(t1[idx], t2[idx])
            face = np.argmax(np.where(np.isfinite(near), near, -np.inf), axis=1)
            local_n = np.zeros((len(idx), 3))
            local_n[np.arange(len(idx)), face] = -np.sign(d[idx, face])
            box_to_world = transform.rotation_matrix() @ box.rotation_matrix()
            normal[idx] = local_n @ box_to_world.T
            semantic[hit] = info.semantic_tag
            actor[hit] = info.id

        far = dist > max_distance
        dist[far] = np.inf
        return RayHits(dist, normal, semantic, actor)

    # --- cameras ---------------------------------------------------------------------

    def render_camera(self, view: CameraView) -> CameraFrame:
        cam = view.camera
        local = cam.ray_directions().reshape(-1, 3)
        world = local @ cam.pose.rotation_matrix().T
        origins = np.broadcast_to(np.asarray(cam.pose.location, float), world.shape)
        hits = self.cast_rays(view.scene, origins, world, view.max_depth,
                              view.ignore_actors | {view.sensor_id})
        h, w = cam.height, cam.width
        frame = CameraFrame()
        hit = np.isfinite(hits.distance)
        if "rgb" in view.outputs:
            shade = np.clip(-(hits.normal @ self.light), 0.0, 1.0) * 0.7 + 0.3
            color = np.tile(self.sky, (len(local), 1))
            ground = hit & (hits.actor_id == 0)
            points = origins + world * np.where(hit, hits.distance, 0.0)[:, None]
            checker = (np.floor(points[:, 0]) + np.floor(points[:, 1])) % 2
            color[ground] = self.ground * (0.85 + 0.15 * checker[ground, None])
            bodies = hit & (hits.actor_id != 0)
            color[bodies] = _actor_colors(hits.actor_id[bodies])
            color[hit] *= shade[hit, None]
            frame.rgb = np.clip(color, 0, 255).astype(np.uint8).reshape(h, w, 3)
        if "depth" in view.outputs:
            # z-depth: the distance along the camera's forward axis
            frame.depth = np.where(hit, hits.distance * local[:, 0], np.inf).astype(np.float32).reshape(h, w)
        if view.outputs & {"semantic", "instance"}:
            frame.semantic = np.where(hit, hits.semantic, CityObjectLabel.Sky).astype(np.uint8).reshape(h, w)
        if "instance" in view.outputs:
            frame.actor_id = hits.actor_id.reshape(h, w)
        return frame


def _actor_colors(ids: np.ndarray) -> np.ndarray:
    """A stable color per actor id."""
    unique, inverse = np.unique(ids, return_inverse=True)
    table = np.empty((len(unique), 3))
    for i, actor_id in enumerate(unique.tolist()):
        h = zlib.crc32(str(actor_id).encode())
        table[i] = (80 + h % 160, 80 + (h >> 8) % 160, 80 + (h >> 16) % 160)
    return table[inverse]
