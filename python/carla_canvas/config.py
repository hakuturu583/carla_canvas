"""The carla_canvas configuration file (YAML), named by ``CARLA_CANVAS_CONFIG``.

One file serves both sides: the client plugin reads ``renderer.endpoint``,
``renderer.command`` and ``sensors``; the renderer service reads
``renderer`` (which backend, its assets and options; ``renderer.camera`` and
``renderer.lidar`` may name another backend for cameras or LiDARs) and
``ros``. See docs/configuration.md for every key.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ENV_CONFIG = "CARLA_CANVAS_CONFIG"
DEFAULT_ENDPOINT = "localhost:50051"
# The sensors carla_canvas renders; the other cameras and LiDARs (DVS, optical
# flow, normals, HSS) stay CARLA's.
DEFAULT_SENSORS = ("sensor.camera.rgb", "sensor.camera.depth",
                   "sensor.camera.semantic_segmentation", "sensor.camera.instance_segmentation",
                   "sensor.lidar.ray_cast", "sensor.lidar.ray_cast_semantic")


# Sensor kinds whose backend `renderer.<kind>` can choose.
SENSOR_KINDS = ("camera", "lidar")
_KIND_KEYS = {"backend", "assets", "options"}
DEFAULT_BACKEND = "primitives"


class ConfigError(ValueError):
    pass


def merge(base: dict, over: dict) -> dict:
    """Deep merge: `over`'s keys win, mappings merge."""
    out = dict(base)
    for k, v in over.items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def renderer_section(renderer: dict, kind: str) -> dict:
    """The `renderer` section as `kind`'s backend sees it: `renderer.<kind>`
    over the rest (its backend and assets replace, its options merge)."""
    base = {k: v for k, v in renderer.items() if k not in SENSOR_KINDS}
    override = renderer.get(kind) or {}
    section = merge(base, override)
    if "assets" in override:
        section["assets"] = override["assets"]
    return section


@dataclass
class SensorRule:
    type: str
    role_name: str = ""  # empty: any


@dataclass
class RosConfig:
    backend: str = "auto"  # auto | cyclonedds | rclpy | none | memory (tests)
    domain_id: int | None = None
    publish_tf: bool = True
    publish_clock: bool = True


@dataclass
class CanvasConfig:
    path: Path | None = None
    endpoint: str = DEFAULT_ENDPOINT
    command: list[str] = field(default_factory=list)
    timeout: float = 30.0
    startup_timeout: float = 60.0
    async_render: bool = True
    backend: str = DEFAULT_BACKEND
    #: each sensor kind's backend (renderer.camera / renderer.lidar, else backend)
    backends: dict[str, str] = field(default_factory=dict)
    options: dict[str, Any] = field(default_factory=dict)
    assets: dict[str, Any] = field(default_factory=dict)
    renderer: dict[str, Any] = field(default_factory=dict)  # the raw `renderer` section
    sensors: list[SensorRule] = field(default_factory=list)
    ros: RosConfig = field(default_factory=RosConfig)



def _section(data: dict, key: str) -> dict:
    value = data.get(key)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ConfigError(f"`{key}` must be a mapping")
    return value


def parse_config(data: Any, path: Path | None = None) -> CanvasConfig:
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError("the top level must be a mapping")
    renderer = _section(data, "renderer")
    config = CanvasConfig(path=path, renderer=renderer)
    config.endpoint = str(renderer.get("endpoint", DEFAULT_ENDPOINT))
    command = renderer.get("command", [])
    if not isinstance(command, list):
        raise ConfigError("`renderer.command` must be a list of arguments")
    config.command = [str(a) for a in command]
    config.timeout = float(renderer.get("timeout", config.timeout))
    config.startup_timeout = float(renderer.get("startup_timeout", config.startup_timeout))
    config.async_render = bool(renderer.get("async", True))
    config.backend = str(renderer.get("backend", config.backend))
    for kind in SENSOR_KINDS:
        section = _section(renderer, kind)
        unknown = set(section) - _KIND_KEYS
        if unknown:
            raise ConfigError(f"`renderer.{kind}` takes {', '.join(sorted(_KIND_KEYS))}, "
                              f"not {', '.join(sorted(unknown))}")
        for key in ("assets", "options"):
            if not isinstance(section.get(key) or {}, dict):
                raise ConfigError(f"`renderer.{kind}.{key}` must be a mapping")
        config.backends[kind] = str(section.get("backend", config.backend))
    config.options = _section(renderer, "options")
    assets = renderer.get("assets") or {}
    if not isinstance(assets, dict):
        raise ConfigError("`renderer.assets` must be a mapping")
    config.assets = assets
    if config.timeout <= 0 or config.startup_timeout <= 0:
        raise ConfigError("timeouts must be positive")

    sensors = data.get("sensors")
    if sensors is None:
        config.sensors = [SensorRule(t) for t in DEFAULT_SENSORS]
    elif not isinstance(sensors, list):
        raise ConfigError("`sensors` must be a list of {type, role_name} rules")
    else:
        for rule in sensors:
            if isinstance(rule, str):
                config.sensors.append(SensorRule(rule))
            elif isinstance(rule, dict) and "type" in rule:
                config.sensors.append(SensorRule(str(rule["type"]), str(rule.get("role_name", ""))))
            else:
                raise ConfigError("each `sensors` rule needs a `type`")

    ros = _section(data, "ros")
    config.ros = RosConfig(
        backend=str(ros.get("backend", "auto")),
        domain_id=None if ros.get("domain_id") is None else int(ros["domain_id"]),
        publish_tf=bool(ros.get("publish_tf", True)),
        publish_clock=bool(ros.get("publish_clock", True)),
    )
    if config.ros.backend not in ("auto", "cyclonedds", "rclpy", "none", "memory"):
        raise ConfigError(f"unknown ros.backend {config.ros.backend!r}")
    return config


def load_config(path: str | os.PathLike | None = None) -> CanvasConfig:
    """Reads the file at `path` (default: ``$CARLA_CANVAS_CONFIG``)."""
    if path is None:
        path = os.environ.get(ENV_CONFIG)
        if not path:
            raise ConfigError(f"{ENV_CONFIG} is not set")
    p = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(p.read_text())
    except OSError as e:
        raise ConfigError(f"carla_canvas config '{p}': {e}") from e
    except yaml.YAMLError as e:
        raise ConfigError(f"carla_canvas config '{p}': {e}") from e
    try:
        return parse_config(data, p)
    except ConfigError as e:
        raise ConfigError(f"carla_canvas config '{p}': {e}") from e
