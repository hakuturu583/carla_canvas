"""CARLA's ray-cast LiDAR model on top of any renderer.

Mirrors the CARLA server (RayCastLidar / RayCastSemanticLidar): each tick a
sensor sweeps ``rotation_frequency * horizontal_fov * delta_seconds`` degrees
with ``points_per_second * delta_seconds / channels`` rays per channel, from
where the previous tick stopped; channel i points at ``upper_fov - i *
(upper_fov - lower_fov) / (channels - 1)``. The ray-cast LiDAR then applies
the general drop-off, noise along the ray, intensity ``exp(-attenuation *
distance)`` and the intensity drop-off; the semantic LiDAR reports the
incidence cosine, the actor id and its CityObjectLabel instead.

Hits come from :meth:`Renderer.cast_rays` when the renderer has it, otherwise
from depth images (:func:`cast_rays_from_depth`), e.g. for Gaussian splatting.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import PinholeCamera, Transform, direction, rotation_matrix, transform_from_matrix
from .renderer import CameraView, RayHits, Renderer
from .scene import Scene
from .sensors import LidarSpec


@dataclass
class LidarScan:
    horizontal_angle: float  # radians, where the sweep stopped
    points_per_channel: list[int]
    raw_data: bytes  # carla.LidarMeasurement / SemanticLidarMeasurement layout
    points: np.ndarray  # (N, 4) float32 x, y, z, intensity (or cos_inc_angle), sensor frame
    semantic: np.ndarray | None = None  # (N, 2) uint32 object_idx, object_tag


MAX_HALF_SIZE = 2048  # pixels, of the depth images LiDARs read

SEMANTIC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos_inc_angle", "<f4"),
                           ("object_idx", "<u4"), ("object_tag", "<u4")])


class LidarSimulator:
    """One LiDAR sensor's state (where its sweep is) and model."""

    def __init__(self, spec: LidarSpec, sensor_id: int, depth_pixels_per_degree: float = 8.0):
        self.spec = spec
        self.sensor_id = sensor_id
        self.current_angle = 0.0  # degrees
        self.rng = np.random.default_rng(spec.noise_seed or sensor_id)
        self.depth_pixels_per_degree = depth_pixels_per_degree

    def laser_angles(self) -> np.ndarray:
        s = self.spec
        if s.channels == 1:
            return np.array([s.upper_fov], float)
        delta = (s.upper_fov - s.lower_fov) / (s.channels - 1)
        return s.upper_fov - np.arange(s.channels) * delta

    def scan(self, renderer: Renderer, scene: Scene, pose: Transform, delta_seconds: float,
             ignore_actors: frozenset[int] = frozenset()) -> LidarScan:
        s = self.spec
        points_per_laser = int(s.points_per_second * max(delta_seconds, 0.0) / s.channels)
        tick_angle = s.rotation_frequency * s.horizontal_fov * max(delta_seconds, 0.0)
        if points_per_laser <= 0:
            return self._finish(np.zeros((0, 3)), np.zeros(0, int), None, None, tick_angle, None)
        step = tick_angle / points_per_laser
        horizontal = np.mod(self.current_angle + step * np.arange(points_per_laser),
                            s.horizontal_fov) - s.horizontal_fov / 2.0
        vertical = self.laser_angles()
        vv, hh = np.meshgrid(vertical, horizontal, indexing="ij")  # (channels, points)
        channel = np.repeat(np.arange(s.channels), points_per_laser)
        local = direction(vv.ravel(), hh.ravel())  # sensor frame
        keep = np.ones(len(local), bool)
        if not s.semantic and s.dropoff_general_rate > 0:
            keep = self.rng.uniform(size=len(local)) > s.dropoff_general_rate
        local, channel = local[keep], channel[keep]

        world_dirs = local @ pose.rotation_matrix().T
        origins = np.broadcast_to(np.asarray(pose.location, float), world_dirs.shape)
        if renderer.supports_rays:
            hits = renderer.cast_rays(scene, origins, world_dirs, s.range, ignore_actors)
        else:
            hits = cast_rays_from_depth(renderer, scene, pose, local, s.range, self.sensor_id,
                                        semantic=s.semantic, ignore_actors=ignore_actors,
                                        pixels_per_degree=self.depth_pixels_per_degree)
        return self._finish(local, channel, hits, world_dirs, tick_angle, ignore_actors)

    def _finish(self, local, channel, hits: RayHits | None, world_dirs, tick_angle, _ignore):
        s = self.spec
        self.current_angle = math.fmod(self.current_angle + tick_angle, s.horizontal_fov)
        angle = math.radians(self.current_angle)
        if hits is None or len(local) == 0:
            return self._pack(angle, s.channels, np.zeros(0, int), np.zeros((0, 4), np.float32), None)
        distance = np.asarray(hits.distance, float)
        valid = np.isfinite(distance) & (distance <= s.range)
        if not s.semantic and s.noise_stddev > 0:
            distance = distance + self.rng.normal(0.0, s.noise_stddev, size=distance.shape)
        local, channel, distance = local[valid], channel[valid], distance[valid]
        xyz = local * distance[:, None]
        if s.semantic:
            n = len(xyz)
            cos_inc = np.ones(n)
            if hits.normal is not None:
                cos_inc = np.clip(-np.einsum("ij,ij->i", hits.normal[valid], world_dirs[valid]), 0, 1)
            idx = (hits.actor_id[valid] if hits.actor_id is not None else np.zeros(n)).astype(np.uint32)
            tag = (hits.semantic[valid] if hits.semantic is not None else np.zeros(n)).astype(np.uint32)
            pts = np.concatenate([xyz, cos_inc[:, None]], axis=1).astype(np.float32)
            return self._pack(angle, s.channels, channel, pts, np.stack([idx, tag], axis=1))
        intensity = np.exp(-s.atmosphere_attenuation_rate * distance)
        if s.dropoff_intensity_limit > 0:
            beta = 1.0 - s.dropoff_zero_intensity
            alpha = s.dropoff_zero_intensity / s.dropoff_intensity_limit
            keep = (intensity > s.dropoff_intensity_limit) | (
                self.rng.uniform(size=len(intensity)) < alpha * intensity + beta)
            xyz, channel, intensity = xyz[keep], channel[keep], intensity[keep]
        pts = np.concatenate([xyz, intensity[:, None]], axis=1).astype(np.float32)
        return self._pack(angle, s.channels, channel, pts, None)

    @staticmethod
    def _pack(angle, channels, channel, pts, semantic) -> LidarScan:
        order = np.argsort(channel, kind="stable")  # grouped by channel, in sweep order
        pts = pts[order]
        counts = np.bincount(channel, minlength=channels).astype(int).tolist() if len(channel) \
            else [0] * channels
        if semantic is not None:
            semantic = semantic[order]
            rec = np.empty(len(pts), SEMANTIC_DTYPE)
            for i, name in enumerate(("x", "y", "z", "cos_inc_angle")):
                rec[name] = pts[:, i]
            rec["object_idx"], rec["object_tag"] = semantic[:, 0], semantic[:, 1]
            raw = rec.tobytes()
        else:
            raw = pts.astype("<f4").tobytes()
        return LidarScan(angle, counts, raw, pts, semantic)


def cast_rays_from_depth(renderer: Renderer, scene: Scene, pose: Transform, local_dirs: np.ndarray,
                         max_distance: float, sensor_id: int = 0, semantic: bool = False,
                         pixels_per_degree: float = 8.0,
                         ignore_actors: frozenset[int] = frozenset()) -> RayHits:
    """Ray hits from depth images: the rays (unit, sensor frame) are split
    among up to four virtual pinhole cameras at the sensor (yaw 0, 90, 180,
    -90 degrees), each rendered with the depth (and, for a semantic LiDAR,
    segmentation) output; each ray reads its nearest pixel. Normals come from
    the depth image's gradient."""
    n = len(local_dirs)
    distance = np.full(n, np.inf)
    normal = np.zeros((n, 3))
    labels = np.zeros(n, np.uint8)
    actors = np.zeros(n, np.uint32)
    if n == 0:
        return RayHits(distance, normal, labels, actors)
    az = np.degrees(np.arctan2(local_dirs[:, 1], local_dirs[:, 0]))
    sector = np.mod(np.round(az / 90.0).astype(int), 4)  # 0: front, 1: right, 2: back, 3: left
    outputs = {"depth"}
    if semantic:
        outputs |= {"semantic", "instance"} & set(renderer.camera_outputs)
    f_max = pixels_per_degree * 180.0 / math.pi
    pose_m = pose.matrix()
    for k in range(4):
        sel = np.nonzero(sector == k)[0]
        if len(sel) == 0:
            continue
        yaw_r = rotation_matrix(0.0, 90.0 * k, 0.0)
        d = local_dirs[sel] @ yaw_r  # in the virtual camera's frame
        u_t = d[:, 1] / d[:, 0]
        v_t = d[:, 2] / d[:, 0]
        # cover every ray (|azimuth| <= 45 degrees: |u_t| <= 1), at most
        # MAX_HALF_SIZE pixels from the center
        span_u = max(float(np.abs(u_t).max()), 1e-3)
        span_v = max(float(np.abs(v_t).max()), 1e-3)
        f = min(f_max, MAX_HALF_SIZE / max(span_u, span_v))
        half_w = int(math.ceil(f * span_u)) + 1
        half_h = int(math.ceil(f * span_v)) + 1
        width, height = 2 * half_w, 2 * half_h
        fov = math.degrees(2.0 * math.atan(half_w / f))
        cam_m = pose_m.copy()
        cam_m[:3, :3] = pose_m[:3, :3] @ yaw_r
        camera = PinholeCamera(width, height, fov, transform_from_matrix(cam_m))
        frame = renderer.render_camera(CameraView(camera, scene, frozenset(outputs), sensor_id,
                                                  max_depth=max_distance,
                                                  ignore_actors=ignore_actors))
        depth = frame.depth
        fc = camera.focal
        col = np.clip(np.floor(width / 2.0 + fc * u_t).astype(int), 0, width - 1)
        row = np.clip(np.floor(height / 2.0 - fc * v_t).astype(int), 0, height - 1)
        z = depth[row, col]
        hit = np.isfinite(z) & (z > 0)
        dist = np.where(hit, z / d[:, 0], np.inf)
        distance[sel] = dist
        cam_normals = _depth_normals(depth, fc)
        normal[sel] = cam_normals[row, col] @ cam_m[:3, :3].T
        if frame.semantic is not None:
            labels[sel] = frame.semantic[row, col]
        if frame.actor_id is not None:
            actors[sel] = frame.actor_id[row, col]
    return RayHits(distance, normal, labels, actors)


def _depth_normals(depth: np.ndarray, focal: float) -> np.ndarray:
    """(H, W, 3) unit normals in the camera's sensor frame, facing it."""
    h, w = depth.shape
    z = np.where(np.isfinite(depth), depth, 0.0)
    u = (np.arange(w) + 0.5 - w / 2.0) / focal
    v = (np.arange(h) + 0.5 - h / 2.0) / focal
    uu, vv = np.meshgrid(u, v)
    pts = np.stack([z, z * uu, -z * vv], axis=-1)
    du = np.gradient(pts, axis=1)
    dv = np.gradient(pts, axis=0)
    n = np.cross(du, dv)
    norm = np.linalg.norm(n, axis=-1, keepdims=True)
    n = np.where(norm > 1e-12, n / np.maximum(norm, 1e-12), np.array([-1.0, 0.0, 0.0]))
    facing = np.einsum("hwc,hwc->hw", n, pts) > 0  # points away from the camera: flip
    n[facing] *= -1
    return n
