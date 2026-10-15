"""ROS 2 message types as CycloneDDS IDL types, with ROS 2's DDS type names
(<package>::msg::dds_::<Name>_), so ROS 2 nodes receive them. Imported only
by the cyclonedds transport (it needs the cyclonedds package).

No `from __future__ import annotations` here: cyclonedds resolves the field
types from this module's globals.
"""

from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float64, int32, sequence, uint8, uint32


@dataclass
class Time(IdlStruct, typename="builtin_interfaces::msg::dds_::Time_"):
    sec: int32
    nanosec: uint32


@dataclass
class Header(IdlStruct, typename="std_msgs::msg::dds_::Header_"):
    stamp: Time
    frame_id: str


@dataclass
class Image(IdlStruct, typename="sensor_msgs::msg::dds_::Image_"):
    header: Header
    height: uint32
    width: uint32
    encoding: str
    is_bigendian: uint8
    step: uint32
    data: sequence[uint8]


@dataclass
class RegionOfInterest(IdlStruct, typename="sensor_msgs::msg::dds_::RegionOfInterest_"):
    x_offset: uint32
    y_offset: uint32
    height: uint32
    width: uint32
    do_rectify: bool


@dataclass
class CameraInfo(IdlStruct, typename="sensor_msgs::msg::dds_::CameraInfo_"):
    header: Header
    height: uint32
    width: uint32
    distortion_model: str
    d: sequence[float64]
    k: array[float64, 9]
    r: array[float64, 9]
    p: array[float64, 12]
    binning_x: uint32
    binning_y: uint32
    roi: RegionOfInterest


@dataclass
class PointField(IdlStruct, typename="sensor_msgs::msg::dds_::PointField_"):
    name: str
    offset: uint32
    datatype: uint8
    count: uint32


@dataclass
class PointCloud2(IdlStruct, typename="sensor_msgs::msg::dds_::PointCloud2_"):
    header: Header
    height: uint32
    width: uint32
    fields: sequence[PointField]
    is_bigendian: bool
    point_step: uint32
    row_step: uint32
    data: sequence[uint8]
    is_dense: bool


@dataclass
class Vector3(IdlStruct, typename="geometry_msgs::msg::dds_::Vector3_"):
    x: float64
    y: float64
    z: float64


@dataclass
class Quaternion(IdlStruct, typename="geometry_msgs::msg::dds_::Quaternion_"):
    x: float64
    y: float64
    z: float64
    w: float64


@dataclass
class TransformMsg(IdlStruct, typename="geometry_msgs::msg::dds_::Transform_"):
    translation: Vector3
    rotation: Quaternion


@dataclass
class TransformStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TransformStamped_"):
    header: Header
    child_frame_id: str
    transform: TransformMsg


@dataclass
class TFMessage(IdlStruct, typename="tf2_msgs::msg::dds_::TFMessage_"):
    transforms: sequence[TransformStamped]


@dataclass
class Clock(IdlStruct, typename="rosgraph_msgs::msg::dds_::Clock_"):
    clock: Time
