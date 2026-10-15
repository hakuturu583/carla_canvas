"""Where the client plugin (libcarla_canvas_plugin.so) is, and the
``typesafe_carla.sensor_plugins`` entry point: typesafe_carla's launcher (and
``import typesafe_carla.carla``) loads the plugin when CARLA_CANVAS_CONFIG
names a configuration, so the program itself does not change."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from .config import ENV_CONFIG

PLUGIN_NAME = "libcarla_canvas_plugin.so"
ENV_PLUGIN = "CARLA_CANVAS_PLUGIN"  # overrides the search below
_PACKAGE_DIR = Path(__file__).resolve().parent


class PluginNotFound(RuntimeError):
    pass


def _candidates() -> list[Path]:
    out = [_PACKAGE_DIR / "_native" / PLUGIN_NAME]
    for entry in sys.path:  # editable installs put CMake output in site-packages
        p = Path(entry or ".") / "carla_canvas" / "_native" / PLUGIN_NAME
        if p not in out:
            out.append(p)
    root = _PACKAGE_DIR.parent.parent  # a source checkout's build directory
    if (root / "CMakeLists.txt").is_file():
        out.append(root / "build" / PLUGIN_NAME)
    return out


def plugin_path() -> Path:
    explicit = os.environ.get(ENV_PLUGIN)
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            raise PluginNotFound(f"{ENV_PLUGIN}={explicit} does not exist")
        return p.resolve()
    for p in _candidates():
        if p.is_file():
            return p.resolve()
    raise PluginNotFound(f"{PLUGIN_NAME} not found (the installation is incomplete; "
                         f"in a checkout: cmake -S . -B build && cmake --build build)")


def sensor_plugin() -> str | None:
    """The entry point: the plugin's path when carla_canvas is configured."""
    if not os.environ.get(ENV_CONFIG):
        return None
    return str(plugin_path())
