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

#: The coordinate frames a renderer can work in (``Renderer.coordinate_frame``):
#:
#: * ``carla``: CARLA's, Unreal's left-handed frame (world and bodies x forward,
#:   y right, z up);
#: * ``right_handed``: the same with y negated (y left), world and bodies: the
#:   frame of OpenDRIVE, ROS and CARLA's ROS publisher (TransformFromCarlaRotation);
#:   body axes x forward, y left, z up (FLU).
COORDINATE_FRAMES = ("carla", "right_handed")

#: Unreal's y right <-> y left; on both sides of a pose it is the change of frame.
FLIP_Y = np.diag([1.0, -1.0, 1.0])

#: OpenCV camera axes (x right, y down, z forward) in a sensor's body axes: row
#: i is camera axis i; per coordinate frame (body y right, or y left).
CV_FROM_BODY = {
    "carla": np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]),
    "right_handed": np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]),
}


def check_frame(frame: str) -> str:
    if frame not in COORDINATE_FRAMES:
        raise ValueError(f"coordinate frame {frame!r}: one of {', '.join(COORDINATE_FRAMES)}")
    return frame


def to_frame(pose: np.ndarray, frame: str) -> np.ndarray:
    """A 4x4 CARLA pose in ``frame`` (and back: the change is an involution)."""
    if check_frame(frame) == "carla":
        return np.asarray(pose, dtype=float)
    m = np.asarray(pose, dtype=float)
    out = np.eye(4)
    out[:3, :3] = FLIP_Y @ m[:3, :3] @ FLIP_Y
    out[:3, 3] = FLIP_Y @ m[:3, 3]
    return out


def vectors_to_frame(v: np.ndarray, frame: str) -> np.ndarray:
    """(..., 3) points or directions of CARLA's frame in ``frame`` (and back)."""
    v = np.asarray(v, dtype=float)
    return v if check_frame(frame) == "carla" else v * (1.0, -1.0, 1.0)


def view_matrix_cv(pose: np.ndarray, frame: str = "carla") -> np.ndarray:
    """4x4 world-to-camera (OpenCV: x right, y down, z forward) of a camera
    looking along its body x, given its 4x4 body-to-world pose in ``frame``'s
    conventions. The world may be any one reached from that frame by a proper
    rigid map (a renderer's scene frame): only the body axes' convention counts."""
    m = np.asarray(pose, dtype=float)
    r = CV_FROM_BODY[check_frame(frame)] @ m[:3, :3].T
    view = np.eye(4)
    view[:3, :3] = r
    view[:3, 3] = -r @ m[:3, 3]
    return view


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

    def matrix(self, frame: str = "carla") -> np.ndarray:
        """4x4 local-to-world matrix, in CARLA's frame or another coordinate
        frame (:data:`COORDINATE_FRAMES`, :func:`to_frame`)."""
        m = np.eye(4)
        m[:3, :3] = self.rotation_matrix()
        m[:3, 3] = self.location
        return to_frame(m, frame)

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
        return view_matrix_cv(self.pose.matrix())


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


def quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """``(x, y, z, w)`` of a proper rotation matrix (a ``right_handed`` pose's;
    CARLA's rotations are proper too, but quaternions are seldom wanted in a
    left-handed frame), with ``w >= 0``."""
    r = np.asarray(rotation, dtype=float)
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    w, x, y, z = (float(v) for v in q)
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return x, y, z, w


def transform_from_matrix(m: np.ndarray) -> Transform:
    return Transform(tuple(float(v) for v in m[:3, 3]), rotation_from_matrix(m[:3, :3]))


def direction(pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Unit forward vectors of rotations (degrees), (..., 3)."""
    p, y = np.radians(pitch), np.radians(yaw)
    return np.stack([np.cos(p) * np.cos(y), np.cos(p) * np.sin(y), np.sin(p)], axis=-1)
