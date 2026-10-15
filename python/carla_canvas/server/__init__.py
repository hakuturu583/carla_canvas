"""Running a renderer behind the carla_canvas protocol.

    from carla_canvas.server import serve
    serve(MyRenderer(), "localhost:50051")     # blocks

or ``carla-canvas serve --config canvas.yaml`` for a configured backend.
"""

from __future__ import annotations

import importlib
import logging
from concurrent import futures

import grpc

from ..config import CanvasConfig
from ..renderer import Renderer
from ..ros.bridge import RosBridge
from ..ros.transports import make_transport
from ..v1 import renderer_pb2_grpc as pb_grpc
from .service import RendererService

BUILTIN_BACKENDS = {
    "primitives": "carla_canvas.renderers.primitives:PrimitiveRenderer",
    "gaussian_splatting": "carla_canvas.renderers.gaussian_splatting:GaussianSplattingRenderer",
}

log = logging.getLogger("carla_canvas")


def load_backend(name: str, options: dict | None = None) -> Renderer:
    """A built-in backend name or "package.module:Class"."""
    target = BUILTIN_BACKENDS.get(name, name)
    if ":" not in target:
        raise ValueError(f"unknown renderer backend {name!r} (built in: "
                         f"{', '.join(BUILTIN_BACKENDS)}; or 'package.module:Class')")
    module, _, cls = target.partition(":")
    renderer_cls = getattr(importlib.import_module(module), cls)
    if not (isinstance(renderer_cls, type) and issubclass(renderer_cls, Renderer)):
        raise TypeError(f"{target} is not a carla_canvas.Renderer")
    return renderer_cls(options or {})


def make_service(renderer: Renderer, config: CanvasConfig | None = None) -> RendererService:
    config = config or CanvasConfig()
    transport = make_transport(config.ros.backend, config.ros.domain_id)
    ros = None if transport is None else RosBridge(transport, config.ros.publish_tf,
                                                   config.ros.publish_clock)
    if ros is not None:
        log.info("ROS 2 output through %s", transport.name)
    return RendererService(renderer, config, ros)


def start(renderer: Renderer, endpoint: str = "localhost:50051",
          config: CanvasConfig | None = None, max_message_mb: int = 512):
    """Starts a gRPC server; returns (server, service, port)."""
    service = make_service(renderer, config)
    options = [("grpc.max_send_message_length", max_message_mb << 20),
               ("grpc.max_receive_message_length", max_message_mb << 20)]
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4), options=options)
    pb_grpc.add_RendererServicer_to_server(service, server)
    port = server.add_insecure_port(endpoint)
    if port == 0:
        raise RuntimeError(f"cannot listen at {endpoint}")
    server.start()
    return server, service, port


def serve(renderer: Renderer, endpoint: str = "localhost:50051",
          config: CanvasConfig | None = None) -> None:
    server, service, port = start(renderer, endpoint, config)
    log.info("carla_canvas renderer '%s' listening at %s", renderer.name, endpoint)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop(grace=1.0)
        renderer.close()
        if service.ros is not None:
            service.ros.close()


__all__ = ["BUILTIN_BACKENDS", "RendererService", "load_backend", "make_service", "serve", "start"]
