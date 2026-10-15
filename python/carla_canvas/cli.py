"""``carla-canvas``: run a renderer, and run programs with it.

    carla-canvas serve [--config canvas.yaml] [--endpoint HOST:PORT] [--backend NAME]
        Runs the renderer service the configuration describes (renderer.backend:
        primitives, gaussian_splatting or package.module:Class).
    carla-canvas run [--config canvas.yaml] [typesafe-codon args...]
        typesafe-codon with CARLA_CANVAS_CONFIG set: `carla-canvas run run main.py`
        is `CARLA_CANVAS_CONFIG=canvas.yaml typesafe-codon run main.py`.
    carla-canvas env [--config canvas.yaml]
        Shell exports for running a built executable (or anything else) with
        carla_canvas: eval "$(carla-canvas env --config canvas.yaml)"; ./main
    carla-canvas info [--config canvas.yaml]
        The plugin, the configuration, and whether the renderer answers.

--config defaults to $CARLA_CANVAS_CONFIG.
"""

from __future__ import annotations

import argparse
import logging
import os
import shlex
import shutil
import sys
from pathlib import Path

from . import __version__
from .config import ENV_CONFIG, ConfigError, load_config
from .plugin import PluginNotFound, plugin_path

ENV_SENSOR_PLUGIN = "TYPESAFE_CARLA_SENSOR_PLUGIN"


def _config_path(arg: str | None) -> str:
    path = arg or os.environ.get(ENV_CONFIG)
    if not path:
        raise SystemExit(f"carla-canvas: no configuration (--config or {ENV_CONFIG})")
    return str(Path(path).expanduser().resolve())


def _serve(args) -> int:
    from .server import load_backend, serve

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="[carla_canvas] %(message)s")
    config = load_config(_config_path(args.config))
    endpoint = args.endpoint or config.endpoint
    backend = args.backend or config.backend
    renderer = load_backend(backend, config.options)
    serve(renderer, endpoint, config)
    return 0


def _env(config: str) -> dict[str, str]:
    return {ENV_CONFIG: config, ENV_SENSOR_PLUGIN: str(plugin_path())}


def _run(args, rest: list[str]) -> int:
    env = os.environ.copy()
    env.update(_env(_config_path(args.config)))
    launcher = shutil.which("typesafe-codon")
    if launcher is None:
        raise SystemExit("carla-canvas: typesafe-codon not found (install typesafe-carla)")
    os.execve(launcher, [launcher] + rest, env)
    return 127


def _info(args) -> int:
    print(f"carla_canvas       {__version__}")
    try:
        print(f"plugin             {plugin_path()}")
    except PluginNotFound as e:
        print(f"plugin             NOT FOUND: {e}")
    path = args.config or os.environ.get(ENV_CONFIG)
    if not path:
        print(f"config             none ({ENV_CONFIG} unset: CARLA renders every sensor)")
        return 0
    try:
        config = load_config(path)
    except ConfigError as e:
        print(f"config             INVALID: {e}")
        return 1
    print(f"config             {config.path}")
    print(f"renderer           {config.backend} at {config.endpoint}"
          + (f" (started with: {shlex.join(config.command)})" if config.command else ""))
    print(f"sensors            {', '.join(r.type + (f' [{r.role_name}]' if r.role_name else '') for r in config.sensors)}")
    print(f"ros                {config.ros.backend}")
    try:
        import grpc

        from .v1 import renderer_pb2 as pb
        from .v1 import renderer_pb2_grpc as pb_grpc

        with grpc.insecure_channel(config.endpoint) as channel:
            info = pb_grpc.RendererStub(channel).GetInfo(pb.GetInfoRequest(), timeout=2.0)
        print(f"renderer answers   {info.name} {info.version}: {', '.join(info.sensor_types)}")
    except Exception as e:  # noqa: BLE001
        print(f"renderer answers   no ({type(e).__name__})")
    return 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(__doc__.strip())
        return 0
    command, rest = argv[0], argv[1:]
    parser = argparse.ArgumentParser(prog=f"carla-canvas {command}")
    parser.add_argument("--config")
    try:
        if command == "serve":
            parser.add_argument("--endpoint")
            parser.add_argument("--backend")
            parser.add_argument("-v", "--verbose", action="store_true")
            return _serve(parser.parse_args(rest))
        if command == "run":
            args, passthrough = parser.parse_known_args(rest)
            return _run(args, passthrough)
        if command == "env":
            for key, value in _env(_config_path(parser.parse_args(rest).config)).items():
                print(f"export {key}={shlex.quote(value)}")
            return 0
        if command == "plugin-path":
            print(plugin_path())
            return 0
        if command == "info":
            return _info(parser.parse_args(rest))
    except (ConfigError, PluginNotFound) as e:
        print(f"carla-canvas: {e}", file=sys.stderr)
        return 2
    print(f"carla-canvas: unknown command {command!r} (see carla-canvas --help)", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
