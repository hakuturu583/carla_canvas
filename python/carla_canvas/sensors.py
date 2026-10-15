"""CARLA's sensor blueprints: their attributes (with the blueprint library's
defaults) and which renderer outputs each needs."""

from __future__ import annotations

from dataclasses import dataclass

from .scene import ActorInfo

CAMERA_TYPES = {
    "sensor.camera.rgb": "rgb",
    "sensor.camera.depth": "depth",
    "sensor.camera.semantic_segmentation": "semantic",
    "sensor.camera.instance_segmentation": "instance",
}


def _float(attrs: dict[str, str], key: str, default: float) -> float:
    value = attrs.get(key, "")
    try:
        return float(value) if value != "" else default
    except ValueError:
        raise ValueError(f"attribute {key}={value!r} is not a number") from None


def _int(attrs: dict[str, str], key: str, default: int) -> int:
    return int(round(_float(attrs, key, default)))


@dataclass
class CameraSpec:
    kind: str  # rgb | depth | semantic | instance
    width: int = 800
    height: int = 600
    fov: float = 90.0

    @staticmethod
    def from_actor(actor: ActorInfo) -> "CameraSpec":
        a = actor.attributes
        spec = CameraSpec(CAMERA_TYPES[actor.type_id], _int(a, "image_size_x", 800),
                          _int(a, "image_size_y", 600), _float(a, "fov", 90.0))
        if spec.width <= 0 or spec.height <= 0 or not 0 < spec.fov < 180:
            raise ValueError(f"camera {actor.id}: bad size or fov ({spec})")
        return spec


@dataclass
class LidarSpec:
    semantic: bool = False
    channels: int = 32
    range: float = 10.0
    points_per_second: int = 56000
    rotation_frequency: float = 10.0
    upper_fov: float = 10.0
    lower_fov: float = -30.0
    horizontal_fov: float = 360.0
    atmosphere_attenuation_rate: float = 0.004
    dropoff_general_rate: float = 0.45
    dropoff_intensity_limit: float = 0.8
    dropoff_zero_intensity: float = 0.4
    noise_stddev: float = 0.0
    noise_seed: int = 0

    @staticmethod
    def from_actor(actor: ActorInfo) -> "LidarSpec":
        a = actor.attributes
        semantic = actor.type_id == "sensor.lidar.ray_cast_semantic"
        spec = LidarSpec(
            semantic=semantic,
            channels=_int(a, "channels", 32),
            range=_float(a, "range", 10.0),
            points_per_second=_int(a, "points_per_second", 56000),
            rotation_frequency=_float(a, "rotation_frequency", 10.0),
            upper_fov=_float(a, "upper_fov", 10.0),
            lower_fov=_float(a, "lower_fov", -30.0),
            horizontal_fov=_float(a, "horizontal_fov", 360.0),
        )
        if not semantic:  # the semantic LiDAR has no intensity model
            spec.atmosphere_attenuation_rate = _float(a, "atmosphere_attenuation_rate", 0.004)
            spec.dropoff_general_rate = _float(a, "dropoff_general_rate", 0.45)
            spec.dropoff_intensity_limit = _float(a, "dropoff_intensity_limit", 0.8)
            spec.dropoff_zero_intensity = _float(a, "dropoff_zero_intensity", 0.4)
            spec.noise_stddev = _float(a, "noise_stddev", 0.0)
            spec.noise_seed = _int(a, "noise_seed", 0)
        if spec.channels <= 0 or spec.range <= 0 or spec.points_per_second < 0:
            raise ValueError(f"LiDAR {actor.id}: bad channels, range or points_per_second")
        return spec

