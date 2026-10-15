# carla_canvas

Swap CARLA's camera and LiDAR rendering for **3D Gaussian Splatting or any
other renderer**, behind the CARLA API, for
[typesafe_carla](https://github.com/hakuturu583/typesafe_carla) programs.

The program does not change. Point `CARLA_CANVAS_CONFIG` at a configuration
file and the sensors it names are rendered by a renderer service over gRPC
instead of the CARLA server; `listen()`, `poll()`, `wait_for_data()`,
`carla.Image` / `carla.LidarMeasurement` and `enable_for_ros()` keep working,
and the ROS 2 topics keep CARLA's names and types.

> **概要 (日本語)** — CARLA のセンサー描画 (カメラ / LiDAR) を 3D Gaussian Splatting
> などの別手法に差し替える互換ライブラリです。ユーザーのプログラムは変更不要で、
> 起動時の環境変数 `CARLA_CANVAS_CONFIG` で設定ファイル (どのレンダラーを使うか、
> 読み込むアセット、どのセンサーを差し替えるか) を指定します。クライアントと
> レンダラーは gRPC で CARLA API 互換のメッセージをやり取りし、計測値は LibCarla の
> `Image` / `LidarMeasurement` そのものとして届きます。`enable_for_ros()` された
> センサーの ROS 2 (DDS) 出力は carla_canvas が CARLA と同じトピック名・型で行うので、
> レンダラー実装側も利用者も通信バックエンドを意識する必要はありません。

```python
# examples/camera_lidar.py: typesafe_carla's camera example, unchanged
import typesafe_carla as carla
...
camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(1.5, 0.0, 2.0)),
                           attach_to=vehicle).as_sensor()
camera.listen(4)
world.tick()
image = camera.wait_for_data(5.0).as_image()      # rendered by carla_canvas
```

```sh
CARLA_CANVAS_CONFIG=examples/splatsim.yaml typesafe-codon run examples/camera_lidar.py
```

## How it works

```
 typesafe_carla program (Codon / Python)            unchanged
   │ listen / tick / enable_for_ros
   ▼
 libtypesafe_carla_ffi ── sensor source (C ABI 5.3) ──▶ libcarla_canvas_plugin.so
   │ unclaimed sensors, everything else                  │ claimed sensors: snapshot,
   ▼                                                     │ sensor attributes, actors
 CARLA server (LibCarla RPC + streams)                   ▼ gRPC (proto/carla_canvas/v1)
                                                   carla_canvas renderer service
                                                     ├─ Renderer: 3DGS / primitives / yours
                                                     ├─ CARLA's LiDAR model, encodings
                                                     └─ ROS 2 over DDS (CARLA's topics)
```

* typesafe_carla loads the plugin (its launcher finds it through an entry
  point when `CARLA_CANVAS_CONFIG` is set). The plugin claims the sensors the
  configuration lists; CARLA never streams them.
* Each `world.tick()` (and each tick in asynchronous mode) the plugin sends
  the world snapshot, new and removed actors, and the sensors due (by
  `sensor_tick`) in one `Render` call. Measurements come back in CARLA's raw
  layouts and are queued as LibCarla's own `Image` / `LidarMeasurement`
  objects: in synchronous mode a callback receives frame N inside the
  `tick()` that returned N.
* The renderer service does everything CARLA-specific: sensor blueprint
  attributes, depth / segmentation encodings, the ray-cast LiDAR model
  (sweep per tick, channels, drop-off, intensity, noise, semantic LiDAR), and
  the ROS 2 output. A renderer only draws a camera view (and optionally casts
  rays).

See [docs/design.md](docs/design.md).

## Install

```sh
pip install 'carla-canvas[client]'  # the client (where the typesafe_carla program runs) + typesafe-carla
pip install 'carla-canvas[ros]'     # ROS 2 output over DDS without a ROS installation
pip install carla-canvas            # a renderer host, e.g. next to splatsim on a GPU machine
pip install 'carla-canvas[gsplat]'  # the built-in Gaussian splatting on CUDA
```

The wheels (Linux x86_64 and aarch64, manylinux_2_28) carry the sensor plugin
with gRPC, protobuf and yaml-cpp linked in: no system packages. A source build
(another platform, or `--no-binary carla-canvas`) uses the system's gRPC C++,
protobuf and yaml-cpp when they are installed, or builds them from source:

```sh
pip install --no-binary carla-canvas 'carla-canvas[client]' \
  -C cmake.define.CARLA_CANVAS_FETCH_DEPS=ON      # needs a C++17 compiler and git; ~20 min
```

## Use

```sh
export CARLA_CANVAS_CONFIG=canvas.yaml
typesafe-codon run main.py                     # the plugin starts the renderer if needed
python main.py                                 # import typesafe_carla.carla: the same
carla-canvas serve --config canvas.yaml        # or run the renderer yourself (another host, GPU)
eval "$(carla-canvas env --config canvas.yaml)"; ./main   # a built executable
carla-canvas info                              # plugin, configuration, renderer reachable?
```

A configuration ([docs/configuration.md](docs/configuration.md)):

```yaml
renderer:
  backend: splatsim.carla_canvas:SplatsimRenderer   # or primitives, gaussian_splatting, package.module:Class
  endpoint: "localhost:50051"
  command: [carla-canvas, serve, --config, "{config}"]   # started when nothing answers
  assets:
    maps:
      "Town10HD*": {scene: scenes/town10.usdz}     # aligned through the map's geo-reference
    actors:
      "vehicle.*": sedan                           # the scene bundle's actor asset
  options: {device: cuda}
sensors:                               # rendered by carla_canvas; every other sensor stays CARLA's
  - sensor.camera.rgb
  - {type: sensor.lidar.ray_cast, role_name: "top*"}
ros:
  backend: auto                        # rclpy if available, else CycloneDDS
```

## Renderers

| backend | what it draws | outputs |
|---|---|---|
| `splatsim.carla_canvas:SplatsimRenderer` | **3D Gaussian Splatting for real use**: [splatsim](https://github.com/hakuturu583/splatsim)'s scene USDZ, actor bank, gsplat cameras and its LiDAR rasterizer (learned intensity, ray drop), geo-referenced to the CARLA map | RGB; LiDARs |
| `gaussian_splatting` | reference and test implementation: a 3DGS map (INRIA `.ply`) plus per-blueprint actor splats; a numpy EWA rasterizer (or gsplat) | RGB, depth, semantic and instance segmentation; LiDARs from depth |
| `primitives` | reference renderer: the ground plane and every actor's bounding box, ray traced (no assets) | everything, with exact ray casts |
| `package.module:Class` | yours: subclass `carla_canvas.Renderer` | what you declare |

Writing one takes two methods ([docs/renderers.md](docs/renderers.md)):

```python
from carla_canvas import CameraFrame, CameraView, Renderer, WorldInfo

class MyRenderer(Renderer):
    name = "mine"
    camera_outputs = frozenset({"rgb", "depth"})

    def load_world(self, world: WorldInfo) -> None: ...          # load world.map_name's assets
    def render_camera(self, view: CameraView) -> CameraFrame: ... # view.camera: pose, size, fov
```

## ROS 2

`sensor.enable_for_ros()` on a rendered sensor publishes, as the CARLA server
would: `/carla/[<parent>/]<ros_name>/image` + `/camera_info`, or
`/point_cloud`, plus `/tf` and `/clock`, stamped with simulation time
([docs/ros2.md](docs/ros2.md)). It is implemented once, in carla_canvas
(`carla_canvas.ros`), over CycloneDDS with ROS 2's wire types (no ROS
installation needed) or rclpy; renderers never deal with it.

## Development

```sh
uv venv && uv pip install -e ../typesafe_carla   # TSC_BACKEND=mock for the mock CARLA
uv pip install --no-build-isolation -e .         # builds the plugin
cmake -S . -B build && cmake --build build -j && ctest --test-dir build
uv run pytest                                    # unit, service, ROS, end to end (Codon + mock)
uv run python tools/gen_proto.py                 # after editing proto/
```

## Status

| | |
|---|---|
| client plugin (C++, gRPC) | ✅ synchronous and asynchronous rendering, renderer auto-start, `sensor_tick`, episodes (`load_world`) |
| sensors | ✅ `sensor.camera.rgb`, `.depth`, `.semantic_segmentation`, `.instance_segmentation`, `sensor.lidar.ray_cast`, `.ray_cast_semantic` |
| renderers | ✅ splatsim (hakuturu583/splatsim#9; tested with stand-ins, not yet on a GPU); reference: ✅ `primitives`, ✅ `gaussian_splatting` (numpy rasterizer tested; its gsplat path untested in CI) |
| ROS 2 | ✅ CycloneDDS (tested) and rclpy transports |
| verified | typesafe_carla mock backend end to end; the typesafe_carla side also against LibCarla ue5-dev (C ABI tests); not yet against a CARLA server |
