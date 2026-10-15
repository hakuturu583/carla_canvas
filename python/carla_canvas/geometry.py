"""CARLA's (Unreal's) frames and transforms, with numpy.

CARLA uses Unreal's left-handed frame: x forward, y right, z up, in meters;
rotations in degrees: pitch about y (nose up positive), yaw about z (turning
right positive), roll about x. :meth:`Transform.matrix` is Unreal's
FRotationMatrix, so it agrees with what the CARLA server renders whatever
convention a LibCarla version uses for ``Rotation.get_forward_vector``.
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Transform:
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # pitch, yaw, roll (degrees)

    @staticmethod
    def from_proto(t) -> "Transform":
        return Transform((t.location.x, t.location.y, t.location.z),
                         (t.rotation.pitch, t.rotation.yaw, t.rotation.roll))

    def to_proto(self, out) -> None:
        out.location.x, out.location.y, out.location.z = self.location
        out.rotation.pitch, out.rotation.yaw, out.rotation.roll = self.rotation

    def rotation_matrix(self) -> np.ndarray:
        """3x3: column i is the actor's axis i (forward, right, up) in world coordinates."""
        return rotation_matrix(*self.rotation)

    def matrix(self) -> np.ndarray:
        """4x4 local-to-world matrix."""
        m = np.eye(4)
        m[:3, :3] = self.rotation_matrix()
        m[:3, 3] = self.location
        return m

    def inverse_matrix(self) -> np.ndarray:
        """4x4 world-to-local matrix."""
        r = self.rotation_matrix()
        m = np.eye(4)
        m[:3, :3] = r.T
        m[:3, 3] = -r.T @ np.asarray(self.location, dtype=float)
        return m

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Local (N, 3) points to world coordinates."""
        return np.asarray(points, dtype=float) @ self.rotation_matrix().T + np.asarray(self.location)

    def inverse_transform_points(self, points: np.ndarray) -> np.ndarray:
        """World (N, 3) points to local coordinates."""
        return (np.asarray(points, dtype=float) - np.asarray(self.location)) @ self.rotation_matrix()


def rotation_matrix(pitch: float, yaw: float, roll: float) -> np.ndarray:
    """Unreal's FRotationMatrix, transposed so columns are the rotated axes."""
    p, y, r = (math.radians(a) for a in (pitch, yaw, roll))
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    sr, cr = math.sin(r), math.cos(r)
    forward = (cp * cy, cp * sy, sp)
    right = (sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp)
    up = (-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp)
    return np.array([forward, right, up], dtype=float).T


def relative(parent: Transform, child_world: Transform) -> tuple[np.ndarray, np.ndarray]:
    """(translation, 3x3 rotation) of a world transform in the parent's frame."""
    rp = parent.rotation_matrix()
    rc = child_world.rotation_matrix()
    t = rp.T @ (np.asarray(child_world.location) - np.asarray(parent.location))
    return t, rp.T @ rc


@dataclass
class PinholeCamera:
    """CARLA's camera model: horizontal fov (degrees), square pixels, principal
    point at the image center, looking along the sensor's +x."""

    width: int
    height: int
    fov: float
    pose: Transform = field(default_factory=Transform)

    @property
    def focal(self) -> float:
        return self.width / (2.0 * math.tan(math.radians(self.fov) / 2.0))

    def intrinsics(self) -> np.ndarray:
        f = self.focal
        return np.array([[f, 0.0, self.width / 2.0], [0.0, f, self.height / 2.0], [0.0, 0.0, 1.0]])

    def pixel_rays(self) -> np.ndarray:
        """(H, W, 3) directions through pixel centers with x = 1, in the sensor
        frame (x forward, y right, z up): a z-depth times one is the point."""
        return _pixel_rays(self.width, self.height, self.focal)

    def ray_directions(self) -> np.ndarray:
        """pixel_rays(), normalized."""
        d = self.pixel_rays()
        return d / np.linalg.norm(d, axis=-1, keepdims=True)

    def project(self, directions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(column, row) indices of the pixels sensor-frame directions (x > 0)
        pass through, clipped to the image."""
        f = self.focal
        col = np.floor(self.width / 2.0 + f * directions[:, 1] / directions[:, 0]).astype(int)
        row = np.floor(self.height / 2.0 - f * directions[:, 2] / directions[:, 0]).astype(int)
        return np.clip(col, 0, self.width - 1), np.clip(row, 0, self.height - 1)

    def world_to_camera_cv(self) -> np.ndarray:
        """4x4 world-to-camera matrix in the OpenCV convention (x right, y down,
        z forward), what most splatting rasterizers take as the view matrix."""
        sensor_to_cv = np.array([[0.0, 1.0, 0.0, 0.0],   # cv x = sensor y (right)
                                 [0.0, 0.0, -1.0, 0.0],  # cv y = -sensor z (down)
                                 [1.0, 0.0, 0.0, 0.0],   # cv z = sensor x (forward)
                                 [0.0, 0.0, 0.0, 1.0]])
        return sensor_to_cv @ self.pose.inverse_matrix()


@functools.lru_cache(maxsize=32)
def _pixel_rays(width: int, height: int, focal: float) -> np.ndarray:
    u = (np.arange(width) + 0.5 - width / 2.0) / focal
    v = (np.arange(height) + 0.5 - height / 2.0) / focal
    uu, vv = np.meshgrid(u, v)
    d = np.stack([np.ones_like(uu), uu, -vv], axis=-1)
    d.flags.writeable = False  # shared by every camera of this size
    return d


def rotation_from_matrix(r: np.ndarray) -> tuple[float, float, float]:
    """(pitch, yaw, roll) in degrees of a rotation matrix whose columns are the
    forward, right and up axes (the inverse of rotation_matrix)."""
    forward, right, up = r[:, 0], r[:, 1], r[:, 2]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, forward[2]))))
    yaw = math.degrees(math.atan2(forward[1], forward[0]))
    roll = math.degrees(math.atan2(-right[2], up[2]))
    return pitch, yaw, roll


def transform_from_matrix(m: np.ndarray) -> Transform:
    return Transform(tuple(float(v) for v in m[:3, 3]), rotation_from_matrix(m[:3, :3]))


def direction(pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Unit forward vectors of rotations (degrees), (..., 3)."""
    p, y = np.radians(pitch), np.radians(yaw)
    return np.stack([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), np.sin(p)], axis=-1)
