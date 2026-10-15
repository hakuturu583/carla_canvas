"""Running a renderer behind the carla_canvas protocol.

    from carla_canvas.server import serve
    serve(MyRenderer(), "localhost:50051")     # blocks

or ``carla-canvas serve --config canvas.yaml`` for a configured backend.
"""

from __future__ import annotations

import logging
import os
import pkgutil
import threading
import time
from concurrent import futures

import grpc

from ..config import SENSOR_KINDS, CanvasConfig, renderer_section
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
    renderer_cls = pkgutil.resolve_name(target)
    if not (isinstance(renderer_cls, type) and issubclass(renderer_cls, Renderer)):
        raise TypeError(f"{target} is not a carla_canvas.Renderer")
    return renderer_cls(options or {})


def load_renderer(config: CanvasConfig, backend: str | None = None) -> Renderer:
    """The configured renderer: `renderer.backend` (or `backend`), or a
    SplitRenderer when `renderer.camera` / `renderer.lidar` choose another
    backend, assets or options for cameras or LiDARs."""
    base = dict(config.renderer)
    if backend:
        base["backend"] = backend
    sections = {kind: renderer_section(base, kind) for kind in SENSOR_KINDS}

    def make(section: dict) -> Renderer:
        return load_backend(str(section.get("backend", config.backend)), section.get("options"))

    if sections["camera"] == sections["lidar"]:
        return make(sections["camera"])
    from ..renderers.split import SplitRenderer

    return SplitRenderer(make(sections["camera"]), make(sections["lidar"]))


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
    _watch_parent(server)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop(grace=1.0)
        renderer.close()
        if service.ros is not None:
            service.ros.close()


def _watch_parent(server) -> None:
    """Started by the client plugin (CARLA_CANVAS_PARENT_PID): stop when that
    program is gone, even if it could not stop us (killed, crashed)."""
    pid = os.environ.get("CARLA_CANVAS_PARENT_PID")
    if not pid:
        return

    def alive(pid: int) -> bool:  # not getppid(): the command may be a wrapper (bash -c, uv run)
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            pass
        return True

    def watch() -> None:
        while alive(int(pid)):
            time.sleep(1.0)
        log.info("the program that started this renderer is gone: stopping")
        server.stop(grace=0)

    threading.Thread(target=watch, name="carla_canvas-parent", daemon=True).start()


__all__ = ["BUILTIN_BACKENDS", "RendererService", "load_backend", "load_renderer", "make_service",
           "serve", "start"]
