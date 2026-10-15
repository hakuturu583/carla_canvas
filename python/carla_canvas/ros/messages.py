"""The ROS 2 messages CARLA's native ROS 2 interface publishes for cameras and
LiDARs, as plain dataclasses. A transport (carla_canvas.ros.transports) maps
them onto its middleware's types: CycloneDDS IDL types with ROS 2's type
names, or rclpy's message classes."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Time:  # builtin_interfaces/Time
    sec: int = 0
    nanosec: int = 0

    @staticmethod
    def from_seconds(seconds: float) -> "Time":
        sec = int(seconds // 1)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec, nanosec = sec + 1, nanosec - 1_000_000_000
        return Time(sec, nanosec)


@dataclass
class Header:  # std_msgs/Header
    stamp: Time = field(default_factory=Time)
    frame_id: str = ""


@dataclass
class Image:  # sensor_msgs/Image
    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    encoding: str = ""
    is_bigendian: int = 0
    step: int = 0
    data: bytes = b""


@dataclass
class RegionOfInterest:  # sensor_msgs/RegionOfInterest
    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False


@dataclass
class CameraInfo:  # sensor_msgs/CameraInfo
    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    distortion_model: str = ""
    d: list[float] = field(default_factory=list)
    k: list[float] = field(default_factory=lambda: [0.0] * 9)
    r: list[float] = field(default_factory=lambda: [0.0] * 9)
    p: list[float] = field(default_factory=lambda: [0.0] * 12)
    binning_x: int = 0
    binning_y: int = 0
    roi: RegionOfInterest = field(default_factory=RegionOfInterest)


@dataclass
class PointField:  # sensor_msgs/PointField
    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)
    name: str = ""
    offset: int = 0
    datatype: int = 0
    count: int = 1


@dataclass
class PointCloud2:  # sensor_msgs/PointCloud2
    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    fields: list[PointField] = field(default_factory=list)
    is_bigendian: bool = False
    point_step: int = 0
    row_step: int = 0
    data: bytes = b""
    is_dense: bool = False


@dataclass
class Vector3:  # geometry_msgs/Vector3
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:  # geometry_msgs/Quaternion
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class TransformMsg:  # geometry_msgs/Transform
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:  # geometry_msgs/TransformStamped
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: TransformMsg = field(default_factory=TransformMsg)


@dataclass
class TFMessage:  # tf2_msgs/TFMessage
    transforms: list[TransformStamped] = field(default_factory=list)


@dataclass
class Clock:  # rosgraph_msgs/Clock
    clock: Time = field(default_factory=Time)


# message class -> ROS 2 type name ("package/msg/Name")
ROS_TYPES = {
    Image: "sensor_msgs/msg/Image",
    CameraInfo: "sensor_msgs/msg/CameraInfo",
    PointCloud2: "sensor_msgs/msg/PointCloud2",
    TFMessage: "tf2_msgs/msg/TFMessage",
    Clock: "rosgraph_msgs/msg/Clock",
}
