"""The renderer service: carla_canvas.v1.Renderer over a :class:`Renderer`.

It keeps the scene (actors and transforms) from the client's calls, turns
each due sensor into camera renders or LiDAR ray casts, encodes them as
CARLA's measurements, publishes the ROS 2 topics of ROS-enabled sensors and
returns the measurements the client listens to.
"""

from __future__ import annotations

import functools
import logging
import threading
from dataclasses import dataclass

import grpc
import numpy as np
from google.protobuf import json_format

from .. import __version__, encodings
from ..config import CanvasConfig
from ..geometry import PinholeCamera
from ..lidar import LidarSimulator
from ..renderer import CameraView, Renderer, WorldInfo
from ..ros.bridge import RosBridge
from ..scene import ActorInfo, Scene
from ..sensors import CAMERA_TYPES, CameraSpec, LidarSpec
from ..v1 import renderer_pb2 as pb
from ..v1 import renderer_pb2_grpc as pb_grpc

PROTOCOL_VERSION = 2
log = logging.getLogger("carla_canvas")


@dataclass
class SensorRuntime:
    info: ActorInfo
    camera: CameraSpec | None = None
    lidar: LidarSimulator | None = None
    #: its last measurement, answered again when the client retries that frame
    last: pb.SensorData | None = None


def merge(base: dict, over: dict) -> dict:
    """Deep merge: `over`'s keys win, mappings merge."""
    out = dict(base)
    for k, v in over.items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


class RenderError(Exception):
    def __init__(self, code: grpc.StatusCode, message: str):
        super().__init__(message)
        self.code = code


def _rpc(method):
    """One call at a time (the scene is shared); errors become gRPC statuses."""

    @functools.wraps(method)
    def wrapper(self, request, context):
        with self._lock:
            try:
                return method(self, request)
            except RenderError as e:
                context.abort(e.code, str(e))
            except (KeyError, ValueError) as e:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(e))
            except Exception as e:  # noqa: BLE001 - reported to the client
                log.exception("renderer failure")
                context.abort(grpc.StatusCode.INTERNAL, f"{type(e).__name__}: {e}")

    return wrapper


class RendererService(pb_grpc.RendererServicer):
    def __init__(self, renderer: Renderer, config: CanvasConfig | None = None,
                 ros: RosBridge | None = None):
        self.renderer = renderer
        self.config = config or CanvasConfig()
        self.ros = ros
        self.scene = Scene()
        self.sensors: dict[int, SensorRuntime] = {}
        self.world_loaded = False
        self._lock = threading.Lock()

    def sensor_types(self) -> list[str]:
        outputs = self.renderer.camera_outputs
        return [t for t, kind in CAMERA_TYPES.items() if kind in outputs] + self.renderer.lidar_types

    # --- RPCs ---------------------------------------------------------------------

    def GetInfo(self, request, context):
        return pb.RendererInfo(name=self.renderer.name,
                               version=f"{self.renderer.version} (carla_canvas {__version__})",
                               protocol_version=PROTOCOL_VERSION, sensor_types=self.sensor_types())

    @_rpc
    def LoadWorld(self, request):
        sent = json_format.MessageToDict(request.renderer_config) \
            if request.HasField("renderer_config") else {}
        # The service's own configuration, with what the client's file says on top
        # (usually the same file). Options are resolved here, once: renderers read
        # self.options.
        config = merge(self.config.renderer, sent)
        self.renderer.options = merge(self.renderer.options, config.get("options") or {})
        base = self.config.path.parent if self.config.path else None
        self.renderer.load_world(WorldInfo(request.map_name, request.episode_id, config, base,
                                           request.opendrive))
        self.scene.reset(request.map_name, request.episode_id)
        self.sensors.clear()
        self.world_loaded = True
        log.info("world %s (episode %d)", request.map_name, request.episode_id)
        return pb.LoadWorldResponse()

    @_rpc
    def SpawnSensor(self, request):
        info = ActorInfo.from_proto(request.sensor)
        if info.type_id not in self.sensor_types():
            raise RenderError(grpc.StatusCode.UNIMPLEMENTED,
                              f"renderer '{self.renderer.name}' does not render "
                              f"{info.type_id} (it renders {', '.join(self.sensor_types())})")
        runtime = SensorRuntime(info)
        if info.type_id in CAMERA_TYPES:
            runtime.camera = CameraSpec.from_actor(info)
        else:
            runtime.lidar = LidarSimulator(LidarSpec.from_actor(info), info.id, info.attributes)
        self.sensors[info.id] = runtime
        self.scene.add_actor(info)
        log.info("sensor %d: %s", info.id, info.type_id)
        return pb.SpawnSensorResponse()

    @_rpc
    def DestroySensor(self, request):
        self.sensors.pop(request.sensor_id, None)
        if self.ros is not None:
            self.ros.set_enabled(request.sensor_id, False)
        return pb.DestroySensorResponse()

    @_rpc
    def EnableForRos(self, request):
        self._sensor(request.sensor_id)
        if self.ros is not None:
            self.ros.set_enabled(request.sensor_id, request.enabled)
        elif request.enabled:
            raise RenderError(grpc.StatusCode.FAILED_PRECONDITION,
                              "ROS 2 output is off (ros.backend: none)")
        return pb.EnableForRosResponse()

    @_rpc
    def Render(self, request):
        return self.render(request)

    # --- rendering -------------------------------------------------------------------

    def _sensor(self, sensor_id: int) -> SensorRuntime:
        sensor = self.sensors.get(sensor_id)
        if sensor is None:
            raise RenderError(grpc.StatusCode.NOT_FOUND, f"no sensor {sensor_id} (SpawnSensor first)")
        return sensor

    def render(self, request) -> pb.RenderResponse:
        if not self.world_loaded:
            raise RenderError(grpc.StatusCode.FAILED_PRECONDITION, "LoadWorld first")
        self.scene.apply(request)
        response = pb.RenderResponse()
        for r in request.sensors:
            sensor = self._sensor(r.sensor_id)
            last = sensor.last
            if last is not None and (last.frame, last.timestamp) == (self.scene.frame,
                                                                     self.scene.elapsed_seconds):
                # a retried frame (after a failure or a client deadline): the
                # same measurement, published once
                if r.deliver:
                    response.measurements.add().CopyFrom(last)
                continue
            # built in place: a measurement holds a whole image
            data = response.measurements.add() if r.deliver else pb.SensorData()
            self.render_sensor(sensor, data)
            sensor.last = data
            if self.ros is not None and self.ros.is_enabled(sensor.info.id):
                self.ros.publish(self.scene, sensor.info, data)
        return response

    def render_sensor(self, sensor: SensorRuntime, data: pb.SensorData) -> None:
        scene = self.scene
        pose = scene.transform(sensor.info.id)
        data.sensor_id, data.frame, data.timestamp = sensor.info.id, scene.frame, scene.elapsed_seconds
        pose.to_proto(data.transform)
        if sensor.camera is not None:
            spec = sensor.camera
            outputs = {"rgb": {"rgb"}, "depth": {"depth"}, "semantic": {"semantic"},
                       "instance": {"semantic", "instance"}}[spec.kind]
            frame = self.renderer.render_camera(CameraView(
                PinholeCamera(spec.width, spec.height, spec.fov, pose), scene, frozenset(outputs),
                sensor.info.id, sensor.info.attributes))
            data.image.width, data.image.height, data.image.fov = spec.width, spec.height, spec.fov
            data.image.raw_data = self._encode_camera(spec, frame).tobytes()
        else:
            # The sweep covers the time since the sensor's last measurement (more
            # than one frame with sensor_tick), as CARLA's LiDAR does; at most a
            # revolution (after a stop(): CARLA's LiDAR kept sweeping meanwhile).
            revolution = 1.0 / max(sensor.lidar.spec.rotation_frequency, 1e-6)
            dt = scene.delta_seconds if sensor.last is None \
                else min(scene.elapsed_seconds - sensor.last.timestamp,
                         max(revolution, scene.delta_seconds))
            ignore = frozenset([sensor.info.id] + scene.ancestors(sensor.info.id))
            scan = sensor.lidar.scan(self.renderer, scene, pose, dt, ignore)
            target = data.semantic_lidar if sensor.lidar.spec.semantic else data.lidar
            target.horizontal_angle = scan.horizontal_angle
            target.points_per_channel.extend(scan.points_per_channel)
            target.raw_data = scan.raw_data

    @staticmethod
    def _encode_camera(spec: CameraSpec, frame) -> np.ndarray:
        shape = (spec.height, spec.width)

        def need(array, what):
            if array is None:
                raise RenderError(grpc.StatusCode.INTERNAL, f"the renderer returned no {what}")
            if array.shape[:2] != shape:
                raise RenderError(grpc.StatusCode.INTERNAL,
                                  f"the renderer's {what} is {array.shape[:2]}, not {shape}")
            return array

        if spec.kind == "rgb":
            return encodings.bgra(np.asarray(need(frame.rgb, "rgb image"), np.uint8))
        if spec.kind == "depth":
            return encodings.depth_bgra(np.asarray(need(frame.depth, "depth"), np.float64))
        if spec.kind == "semantic":
            return encodings.semantic_bgra(need(frame.semantic, "segmentation"))
        return encodings.instance_bgra(need(frame.semantic, "segmentation"),
                                       need(frame.actor_id, "actor ids"))
