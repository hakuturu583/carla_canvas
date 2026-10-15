"""Checks an installed carla-canvas wheel as a user gets it: the sensor plugin is
there, exports only its entry point, and needs no gRPC, protobuf or yaml-cpp
from the system (they are linked in statically).

    pip install --no-deps dist/carla_canvas-*.whl
    python tools/check_wheel.py      # run outside the source tree
"""

from __future__ import annotations

import re
import subprocess
import sys

SYSTEM_LIBS = re.compile(r"lib(grpc|gpr|protobuf|yaml-cpp|absl|ssl|crypto|cares|re2|upb)")


def main() -> int:
    import carla_canvas
    from carla_canvas.plugin import plugin_path

    plugin = plugin_path()
    if "site-packages" not in str(plugin) and "dist-packages" not in str(plugin):
        print(f"{plugin} is not the installed wheel's plugin", file=sys.stderr)
        return 1
    print(f"carla_canvas {carla_canvas.__version__}: {plugin}")

    symbols = subprocess.run(["nm", "-D", "--defined-only", str(plugin)], check=True,
                             capture_output=True, text=True).stdout.split("\n")
    exported = {line.split()[-1] for line in symbols
                if line.strip() and line.split()[-2] in ("T", "W", "D", "B")}
    if exported != {"tsc_sensor_plugin_init"}:
        print(f"exports {sorted(exported)[:10]}, expected only tsc_sensor_plugin_init",
              file=sys.stderr)
        return 1

    needed = subprocess.run(["readelf", "-d", str(plugin)], check=True, capture_output=True,
                            text=True).stdout
    libs = re.findall(r"\(NEEDED\).*\[(.+)\]", needed)
    print("needs:", " ".join(libs))
    bad = [lib for lib in libs if SYSTEM_LIBS.match(lib)]
    if bad:
        print(f"links against system libraries: {bad}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
