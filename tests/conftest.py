from __future__ import annotations

import socket

import grpc
import pytest

from carla_canvas.config import CanvasConfig, RosConfig
from carla_canvas.geometry import Transform
from carla_canvas.server import start
from carla_canvas.v1 import renderer_pb2 as pb
from carla_canvas.v1 import renderer_pb2_grpc as pb_grpc


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def actor(actor_id, type_id, parent_id=0, extent=(0, 0, 0), **attributes) -> pb.Actor:
    a = pb.Actor(id=actor_id, type_id=type_id, parent_id=parent_id)
    a.bounding_box.extent.x, a.bounding_box.extent.y, a.bounding_box.extent.z = extent
    for k, v in attributes.items():
        a.attributes.add(id=k, value=str(v))
    return a


def snapshot_request(frame, actors: dict[int, Transform], new=(), removed=(), sensors=(),
                     delta=0.05) -> pb.RenderRequest:
    r = pb.RenderRequest()
    r.snapshot.id = 1
    r.snapshot.timestamp.frame = frame
    r.snapshot.timestamp.elapsed_seconds = frame * delta
    r.snapshot.timestamp.delta_seconds = delta
    for actor_id, t in actors.items():
        s = r.snapshot.actors.add(id=actor_id)
        t.to_proto(s.transform)
    r.new_actors.extend(new)
    r.removed_actors.extend(removed)
    for sensor_id, deliver in sensors:
        r.sensors.add(sensor_id=sensor_id, deliver=deliver)
    return r


@pytest.fixture
def make_server():
    """make_server(renderer, ros_backend="memory") -> (stub, service)."""
    started = []

    def make(renderer, ros_backend="memory", renderer_config=None):
        endpoint = f"127.0.0.1:{free_port()}"
        config = CanvasConfig(endpoint=endpoint, renderer=renderer_config or {},
                              ros=RosConfig(backend=ros_backend))
        server, service, _ = start(renderer, endpoint, config)
        channel = grpc.insecure_channel(endpoint)
        started.append((server, channel))
        return pb_grpc.RendererStub(channel), service

    yield make
    for server, channel in started:
        channel.close()
        server.stop(None)
