# carla_canvas

Swaps CARLA's camera/LiDAR rendering (3D Gaussian splatting or any renderer)
behind the CARLA API for typesafe_carla programs: a typesafe_carla sensor
plugin (C++, `native/`) ↔ gRPC (`proto/carla_canvas/v1/renderer.proto`) ↔
renderer service (Python, `python/carla_canvas/`). Design: `docs/design.md`.

## Commands

```sh
uv venv && TSC_BACKEND=mock uv pip install -e ../typesafe_carla   # typesafe_carla with ABI >= 5.3
uv pip install scikit-build-core grpcio grpcio-tools protobuf numpy pyyaml pytest cyclonedds
uv pip install --no-build-isolation -e .                 # builds libcarla_canvas_plugin.so
cmake -S . -B build && cmake --build build -j && ctest --test-dir build
PATH=$PWD/.venv/bin:$PATH .venv/bin/python -m pytest     # e2e tests run Codon programs (mock CARLA)
.venv/bin/python tools/gen_proto.py                      # after editing proto/ (commit the output)
CARLA_CANVAS_CONFIG=examples/primitives.yaml typesafe-codon run examples/camera_lidar.py
```

System packages for the plugin: `libgrpc++-dev protobuf-compiler-grpc
libprotobuf-dev protobuf-compiler libyaml-cpp-dev`.

## Rules

- The user's program never changes: everything is chosen by the file in
  `CARLA_CANVAS_CONFIG`. Do not add carla_canvas imports to examples or tests
  of programs (`examples/camera_lidar.py` is typesafe_carla's example).
- The protocol mirrors the CARLA API (names, units, Unreal's left-handed
  frame, world transforms, CARLA's raw measurement layouts). A protocol
  change bumps `kProtocolVersion` (plugin.cpp) and `PROTOCOL_VERSION`
  (server/service.py) together, and regenerates `python/carla_canvas/v1/`.
- CARLA semantics live in the service, not in renderers: encodings
  (`encodings.py`), the LiDAR model (`lidar.py`, mirrors RayCastLidar), sensor
  attributes and defaults (`sensors.py`), ROS 2 (`ros/`). A renderer only
  implements `load_world` / `render_camera` (and optionally `cast_rays`).
- ROS 2 output mirrors CARLA's publishers (`LibCarla/source/carla/ros2`):
  topic names, types, frame ids, y negation, TransformFromCarlaRotation.
  Renderers never implement DDS.
- The plugin talks to typesafe_carla only through `get_proc` (`tsc_api.hpp`);
  it must not link against the library. It needs typesafe_carla's C ABI 5.3+.
- `ros/dds_types.py` must not use `from __future__ import annotations`
  (cyclonedds resolves field types from the module's globals).
