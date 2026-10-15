from google.protobuf import struct_pb2 as _struct_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Vector3D(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class Rotation(_message.Message):
    __slots__ = ("pitch", "yaw", "roll")
    PITCH_FIELD_NUMBER: _ClassVar[int]
    YAW_FIELD_NUMBER: _ClassVar[int]
    ROLL_FIELD_NUMBER: _ClassVar[int]
    pitch: float
    yaw: float
    roll: float
    def __init__(self, pitch: _Optional[float] = ..., yaw: _Optional[float] = ..., roll: _Optional[float] = ...) -> None: ...

class Transform(_message.Message):
    __slots__ = ("location", "rotation")
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    location: Vector3D
    rotation: Rotation
    def __init__(self, location: _Optional[_Union[Vector3D, _Mapping]] = ..., rotation: _Optional[_Union[Rotation, _Mapping]] = ...) -> None: ...

class BoundingBox(_message.Message):
    __slots__ = ("location", "extent", "rotation")
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    EXTENT_FIELD_NUMBER: _ClassVar[int]
    ROTATION_FIELD_NUMBER: _ClassVar[int]
    location: Vector3D
    extent: Vector3D
    rotation: Rotation
    def __init__(self, location: _Optional[_Union[Vector3D, _Mapping]] = ..., extent: _Optional[_Union[Vector3D, _Mapping]] = ..., rotation: _Optional[_Union[Rotation, _Mapping]] = ...) -> None: ...

class ActorAttribute(_message.Message):
    __slots__ = ("id", "value")
    ID_FIELD_NUMBER: _ClassVar[int]
    VALUE_FIELD_NUMBER: _ClassVar[int]
    id: str
    value: str
    def __init__(self, id: _Optional[str] = ..., value: _Optional[str] = ...) -> None: ...

class Actor(_message.Message):
    __slots__ = ("id", "type_id", "parent_id", "attributes", "bounding_box")
    ID_FIELD_NUMBER: _ClassVar[int]
    TYPE_ID_FIELD_NUMBER: _ClassVar[int]
    PARENT_ID_FIELD_NUMBER: _ClassVar[int]
    ATTRIBUTES_FIELD_NUMBER: _ClassVar[int]
    BOUNDING_BOX_FIELD_NUMBER: _ClassVar[int]
    id: int
    type_id: str
    parent_id: int
    attributes: _containers.RepeatedCompositeFieldContainer[ActorAttribute]
    bounding_box: BoundingBox
    def __init__(self, id: _Optional[int] = ..., type_id: _Optional[str] = ..., parent_id: _Optional[int] = ..., attributes: _Optional[_Iterable[_Union[ActorAttribute, _Mapping]]] = ..., bounding_box: _Optional[_Union[BoundingBox, _Mapping]] = ...) -> None: ...

class ActorSnapshot(_message.Message):
    __slots__ = ("id", "transform", "velocity", "angular_velocity", "acceleration")
    ID_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    ANGULAR_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    id: int
    transform: Transform
    velocity: Vector3D
    angular_velocity: Vector3D
    acceleration: Vector3D
    def __init__(self, id: _Optional[int] = ..., transform: _Optional[_Union[Transform, _Mapping]] = ..., velocity: _Optional[_Union[Vector3D, _Mapping]] = ..., angular_velocity: _Optional[_Union[Vector3D, _Mapping]] = ..., acceleration: _Optional[_Union[Vector3D, _Mapping]] = ...) -> None: ...

class Timestamp(_message.Message):
    __slots__ = ("frame", "elapsed_seconds", "delta_seconds", "platform_timestamp")
    FRAME_FIELD_NUMBER: _ClassVar[int]
    ELAPSED_SECONDS_FIELD_NUMBER: _ClassVar[int]
    DELTA_SECONDS_FIELD_NUMBER: _ClassVar[int]
    PLATFORM_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    frame: int
    elapsed_seconds: float
    delta_seconds: float
    platform_timestamp: float
    def __init__(self, frame: _Optional[int] = ..., elapsed_seconds: _Optional[float] = ..., delta_seconds: _Optional[float] = ..., platform_timestamp: _Optional[float] = ...) -> None: ...

class WorldSnapshot(_message.Message):
    __slots__ = ("id", "timestamp", "actors")
    ID_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ACTORS_FIELD_NUMBER: _ClassVar[int]
    id: int
    timestamp: Timestamp
    actors: _containers.RepeatedCompositeFieldContainer[ActorSnapshot]
    def __init__(self, id: _Optional[int] = ..., timestamp: _Optional[_Union[Timestamp, _Mapping]] = ..., actors: _Optional[_Iterable[_Union[ActorSnapshot, _Mapping]]] = ...) -> None: ...

class Image(_message.Message):
    __slots__ = ("width", "height", "fov", "raw_data")
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    FOV_FIELD_NUMBER: _ClassVar[int]
    RAW_DATA_FIELD_NUMBER: _ClassVar[int]
    width: int
    height: int
    fov: float
    raw_data: bytes
    def __init__(self, width: _Optional[int] = ..., height: _Optional[int] = ..., fov: _Optional[float] = ..., raw_data: _Optional[bytes] = ...) -> None: ...

class LidarMeasurement(_message.Message):
    __slots__ = ("horizontal_angle", "points_per_channel", "raw_data")
    HORIZONTAL_ANGLE_FIELD_NUMBER: _ClassVar[int]
    POINTS_PER_CHANNEL_FIELD_NUMBER: _ClassVar[int]
    RAW_DATA_FIELD_NUMBER: _ClassVar[int]
    horizontal_angle: float
    points_per_channel: _containers.RepeatedScalarFieldContainer[int]
    raw_data: bytes
    def __init__(self, horizontal_angle: _Optional[float] = ..., points_per_channel: _Optional[_Iterable[int]] = ..., raw_data: _Optional[bytes] = ...) -> None: ...

class SemanticLidarMeasurement(_message.Message):
    __slots__ = ("horizontal_angle", "points_per_channel", "raw_data")
    HORIZONTAL_ANGLE_FIELD_NUMBER: _ClassVar[int]
    POINTS_PER_CHANNEL_FIELD_NUMBER: _ClassVar[int]
    RAW_DATA_FIELD_NUMBER: _ClassVar[int]
    horizontal_angle: float
    points_per_channel: _containers.RepeatedScalarFieldContainer[int]
    raw_data: bytes
    def __init__(self, horizontal_angle: _Optional[float] = ..., points_per_channel: _Optional[_Iterable[int]] = ..., raw_data: _Optional[bytes] = ...) -> None: ...

class SensorData(_message.Message):
    __slots__ = ("sensor_id", "frame", "timestamp", "transform", "image", "lidar", "semantic_lidar")
    SENSOR_ID_FIELD_NUMBER: _ClassVar[int]
    FRAME_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    TRANSFORM_FIELD_NUMBER: _ClassVar[int]
    IMAGE_FIELD_NUMBER: _ClassVar[int]
    LIDAR_FIELD_NUMBER: _ClassVar[int]
    SEMANTIC_LIDAR_FIELD_NUMBER: _ClassVar[int]
    sensor_id: int
    frame: int
    timestamp: float
    transform: Transform
    image: Image
    lidar: LidarMeasurement
    semantic_lidar: SemanticLidarMeasurement
    def __init__(self, sensor_id: _Optional[int] = ..., frame: _Optional[int] = ..., timestamp: _Optional[float] = ..., transform: _Optional[_Union[Transform, _Mapping]] = ..., image: _Optional[_Union[Image, _Mapping]] = ..., lidar: _Optional[_Union[LidarMeasurement, _Mapping]] = ..., semantic_lidar: _Optional[_Union[SemanticLidarMeasurement, _Mapping]] = ...) -> None: ...

class GetInfoRequest(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class RendererInfo(_message.Message):
    __slots__ = ("name", "version", "protocol_version", "sensor_types")
    NAME_FIELD_NUMBER: _ClassVar[int]
    VERSION_FIELD_NUMBER: _ClassVar[int]
    PROTOCOL_VERSION_FIELD_NUMBER: _ClassVar[int]
    SENSOR_TYPES_FIELD_NUMBER: _ClassVar[int]
    name: str
    version: str
    protocol_version: int
    sensor_types: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, name: _Optional[str] = ..., version: _Optional[str] = ..., protocol_version: _Optional[int] = ..., sensor_types: _Optional[_Iterable[str]] = ...) -> None: ...

class LoadWorldRequest(_message.Message):
    __slots__ = ("map_name", "episode_id", "renderer_config", "opendrive")
    MAP_NAME_FIELD_NUMBER: _ClassVar[int]
    EPISODE_ID_FIELD_NUMBER: _ClassVar[int]
    RENDERER_CONFIG_FIELD_NUMBER: _ClassVar[int]
    OPENDRIVE_FIELD_NUMBER: _ClassVar[int]
    map_name: str
    episode_id: int
    renderer_config: _struct_pb2.Struct
    opendrive: str
    def __init__(self, map_name: _Optional[str] = ..., episode_id: _Optional[int] = ..., renderer_config: _Optional[_Union[_struct_pb2.Struct, _Mapping]] = ..., opendrive: _Optional[str] = ...) -> None: ...

class LoadWorldResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SpawnSensorRequest(_message.Message):
    __slots__ = ("sensor",)
    SENSOR_FIELD_NUMBER: _ClassVar[int]
    sensor: Actor
    def __init__(self, sensor: _Optional[_Union[Actor, _Mapping]] = ...) -> None: ...

class SpawnSensorResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class DestroySensorRequest(_message.Message):
    __slots__ = ("sensor_id",)
    SENSOR_ID_FIELD_NUMBER: _ClassVar[int]
    sensor_id: int
    def __init__(self, sensor_id: _Optional[int] = ...) -> None: ...

class DestroySensorResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class EnableForRosRequest(_message.Message):
    __slots__ = ("sensor_id", "enabled")
    SENSOR_ID_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    sensor_id: int
    enabled: bool
    def __init__(self, sensor_id: _Optional[int] = ..., enabled: _Optional[bool] = ...) -> None: ...

class EnableForRosResponse(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class SensorRequest(_message.Message):
    __slots__ = ("sensor_id", "deliver")
    SENSOR_ID_FIELD_NUMBER: _ClassVar[int]
    DELIVER_FIELD_NUMBER: _ClassVar[int]
    sensor_id: int
    deliver: bool
    def __init__(self, sensor_id: _Optional[int] = ..., deliver: _Optional[bool] = ...) -> None: ...

class RenderRequest(_message.Message):
    __slots__ = ("snapshot", "new_actors", "removed_actors", "sensors")
    SNAPSHOT_FIELD_NUMBER: _ClassVar[int]
    NEW_ACTORS_FIELD_NUMBER: _ClassVar[int]
    REMOVED_ACTORS_FIELD_NUMBER: _ClassVar[int]
    SENSORS_FIELD_NUMBER: _ClassVar[int]
    snapshot: WorldSnapshot
    new_actors: _containers.RepeatedCompositeFieldContainer[Actor]
    removed_actors: _containers.RepeatedScalarFieldContainer[int]
    sensors: _containers.RepeatedCompositeFieldContainer[SensorRequest]
    def __init__(self, snapshot: _Optional[_Union[WorldSnapshot, _Mapping]] = ..., new_actors: _Optional[_Iterable[_Union[Actor, _Mapping]]] = ..., removed_actors: _Optional[_Iterable[int]] = ..., sensors: _Optional[_Iterable[_Union[SensorRequest, _Mapping]]] = ...) -> None: ...

class RenderResponse(_message.Message):
    __slots__ = ("measurements",)
    MEASUREMENTS_FIELD_NUMBER: _ClassVar[int]
    measurements: _containers.RepeatedCompositeFieldContainer[SensorData]
    def __init__(self, measurements: _Optional[_Iterable[_Union[SensorData, _Mapping]]] = ...) -> None: ...
