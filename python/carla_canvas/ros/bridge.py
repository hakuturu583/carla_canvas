"""CARLA's native ROS 2 interface, published by carla_canvas for the sensors it
renders: the same topics, types, frames and conversions as the CARLA server's
publishers (LibCarla/source/carla/ros2), so ROS 2 nodes cannot tell the
difference and no renderer implements any of it.

For a sensor with ROS enabled (``sensor.enable_for_ros()``):

* name: the blueprint's ``ros_name`` attribute, or CARLA's ``<prefix><id>``
  for the placeholder ``<prefix>__`` (CARLA numbers by stream id, carla_canvas
  by actor id); prefixes ``rgb``, ``depth``, ``semantic_segmentation``,
  ``instance_segmentation``, ``ray_cast``, ``ray_cast_semantic``;
* topic base: ``/carla/[<parent ros_name>/...]<name>`` (parents that have a
  ``ros_name``); frame_id: the name;
* cameras: ``<base>/image`` (sensor_msgs/Image, ``bgra8``) and
  ``<base>/camera_info`` (plumb_bob, no distortion, fx = fy = width / (2
  tan(fov / 2)), principal point at the center);
* LiDARs: ``<base>/point_cloud`` (sensor_msgs/PointCloud2; x, y, z, intensity
  float32, or x, y, z, cos_inc_angle float32 + object_idx, object_tag uint32;
  y negated into ROS's right-handed frame; height 1, is_dense false);
* ``/tf``: the sensor's transform in its parent's frame (``map`` when the
  parent has no ROS name), y and the rotation converted as CARLA does;
* ``/clock``: the simulation time, once per frame.

Stamps are the simulation time (elapsed_seconds).
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np

from ..geometry import PinholeCamera, Transform, relative, rotation_from_matrix
from ..lidar import SEMANTIC_DTYPE
from ..scene import ActorInfo, Scene
from . import messages as m
from .transports import Transport

LIDAR_FIELDS = [m.PointField("x", 0, m.PointField.FLOAT32), m.PointField("y", 4, m.PointField.FLOAT32),
                m.PointField("z", 8, m.PointField.FLOAT32),
                m.PointField("intensity", 12, m.PointField.FLOAT32)]
SEMANTIC_LIDAR_FIELDS = [
    m.PointField("x", 0, m.PointField.FLOAT32), m.PointField("y", 4, m.PointField.FLOAT32),
    m.PointField("z", 8, m.PointField.FLOAT32), m.PointField("cos_inc_angle", 12, m.PointField.FLOAT32),
    m.PointField("object_idx", 16, m.PointField.UINT32),
    m.PointField("object_tag", 20, m.PointField.UINT32)]


def explicit_ros_name(actor: ActorInfo | None) -> str:
    """The actor's ros_name attribute, "" when unset or CARLA's "<prefix>__"
    placeholder."""
    name = actor.attributes.get("ros_name", "") if actor is not None else ""
    return "" if name.endswith("__") else name


def ros_name(actor: ActorInfo) -> str:
    """The explicit name, else CARLA's "<prefix><id>" (the prefix is the
    blueprint's last part: rgb, depth, ray_cast, ...)."""
    return explicit_ros_name(actor) or f"{actor.type_id.rsplit('.', 1)[-1]}{actor.id}"


def parent_chain(scene: Scene, actor: ActorInfo) -> str:
    """"a/b" of the ancestors that have a ROS name (outermost first)."""
    names = [explicit_ros_name(scene.actors.get(p)) for p in scene.ancestors(actor.id)]
    return "/".join(reversed([n for n in names if n]))


def carla_to_ros_transform(location, rotation) -> m.TransformMsg:
    """CARLA's TransformFromCarlaRotation (LibCarla ros2/publishers/TransformQuaternion.h)."""
    tx, ty, tz = location
    pitch, yaw, roll = rotation
    rx, ry, rz = -math.radians(pitch), -math.radians(yaw), math.radians(roll)
    cr, sr = math.cos(rz * 0.5), math.sin(rz * 0.5)
    cp, sp = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cy, sy = math.cos(ry * 0.5), math.sin(ry * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return m.TransformMsg(m.Vector3(tx, -ty, tz), m.Quaternion(x, y, z, w))


def camera_info(header: m.Header, width: int, height: int, fov: float) -> m.CameraInfo:
    """The model the image was rendered with (PinholeCamera)."""
    k = PinholeCamera(width, height, fov).intrinsics()
    return m.CameraInfo(
        header=header, height=height, width=width, distortion_model="plumb_bob",
        d=[0.0] * 5, k=k.ravel().tolist(), r=np.eye(3).ravel().tolist(),
        p=np.hstack([k, np.zeros((3, 1))]).ravel().tolist(),
        roi=m.RegionOfInterest(0, 0, height, width, True))


def point_cloud(header: m.Header, raw: bytes, semantic: bool) -> m.PointCloud2:
    step = SEMANTIC_DTYPE.itemsize if semantic else 16
    n = len(raw) // step
    if semantic:
        rec = np.frombuffer(raw, SEMANTIC_DTYPE, n).copy()
        rec["y"] *= -1.0
        data = rec.tobytes()
    else:
        pts = np.frombuffer(raw, "<f4", n * 4).reshape(n, 4).copy()
        pts[:, 1] *= -1.0
        data = pts.tobytes()
    return m.PointCloud2(header=header, height=1, width=n,
                         fields=SEMANTIC_LIDAR_FIELDS if semantic else LIDAR_FIELDS,
                         is_bigendian=False, point_step=step, row_step=n * step, data=data,
                         is_dense=False)


@dataclass
class _Published:
    base: str
    frame_id: str
    parent_frame: str


class RosBridge:
    """Publishes the measurements of the ROS-enabled sensors."""

    def __init__(self, transport: Transport, publish_tf: bool = True, publish_clock: bool = True):
        self.transport = transport
        self.publish_tf = publish_tf
        self.publish_clock = publish_clock
        self._enabled: set[int] = set()  # names are resolved when publishing
        self._publishers: dict[str, object] = {}
        self._last_clock_frame = -1
        self._lock = threading.Lock()

    def is_enabled(self, sensor_id: int) -> bool:
        return sensor_id in self._enabled

    def set_enabled(self, sensor_id: int, enabled: bool) -> None:
        with self._lock:
            if enabled:
                self._enabled.add(sensor_id)
            else:
                self._enabled.discard(sensor_id)

    @staticmethod
    def names(scene: Scene, sensor: ActorInfo) -> _Published:
        """Topic base, frame id and TF parent frame (the parents must be known:
        they come with the frame's snapshot, after enable_for_ros)."""
        chain = parent_chain(scene, sensor)
        name = ros_name(sensor)
        base = "/carla/" + (chain + "/" if chain else "") + name
        return _Published(base, name, chain.rsplit("/", 1)[-1] if chain else "map")

    def _publisher(self, topic: str, msg_type):
        pub = self._publishers.get(topic)
        if pub is None:
            pub = self.transport.publisher(topic, msg_type)
            self._publishers[topic] = pub
        return pub

    def publish(self, scene: Scene, sensor: ActorInfo, data) -> None:
        """`data`: a carla_canvas.v1.SensorData of the sensor."""
        with self._lock:
            if sensor.id not in self._enabled:
                return
            p = self.names(scene, sensor)
            header = m.Header(m.Time.from_seconds(data.timestamp), p.frame_id)
            kind = data.WhichOneof("data")
            if kind == "image":
                img = data.image
                self._publisher(p.base + "/image", m.Image).publish(m.Image(
                    header=header, height=img.height, width=img.width, encoding="bgra8",
                    is_bigendian=0, step=img.width * 4, data=bytes(img.raw_data)))
                self._publisher(p.base + "/camera_info", m.CameraInfo).publish(
                    camera_info(header, img.width, img.height, img.fov))
            elif kind in ("lidar", "semantic_lidar"):
                lidar = getattr(data, kind)
                self._publisher(p.base + "/point_cloud", m.PointCloud2).publish(
                    point_cloud(header, bytes(lidar.raw_data), kind == "semantic_lidar"))
            if self.publish_tf:
                self._publish_tf(scene, sensor, p, header, Transform.from_proto(data.transform))
            if self.publish_clock and scene.frame != self._last_clock_frame:
                self._last_clock_frame = scene.frame
                self._publisher("/clock", m.Clock).publish(m.Clock(header.stamp))

    def _publish_tf(self, scene: Scene, sensor: ActorInfo, p: _Published, header: m.Header,
                    world: Transform) -> None:
        location, rotation = world.location, world.rotation
        if p.parent_frame != "map" and sensor.parent_id in scene.states:
            t, r = relative(scene.transform(sensor.parent_id), world)
            location, rotation = tuple(t), rotation_from_matrix(r)
        ts = m.TransformStamped(m.Header(header.stamp, p.parent_frame), p.frame_id,
                                carla_to_ros_transform(location, rotation))
        self._publisher("/tf", m.TFMessage).publish(m.TFMessage([ts]))

    def close(self) -> None:
        self.transport.close()

