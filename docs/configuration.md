# Configuration

One YAML file, named by `CARLA_CANVAS_CONFIG`, read by both the client plugin
and the renderer service (`carla-canvas serve --config`). Relative paths are
relative to the file. When the plugin starts the renderer, it passes the same
file; a separately started renderer may use its own file: the service merges
the client's `renderer` section over its own (`LoadWorld`).

```yaml
renderer:
  backend: gaussian_splatting   # primitives | gaussian_splatting | package.module:Class
  endpoint: "localhost:50051"   # gRPC address of the renderer service
  command: [carla-canvas, serve, --config, "{config}"]
  startup_timeout: 60           # seconds to wait for a started renderer
  timeout: 30                   # seconds per renderer call
  async: true                   # also render ticks the program does not make
  assets: {}                    # backend-specific (below)
  options: {}                   # backend-specific (below)

sensors:                        # which sensors carla_canvas renders
  - sensor.camera.rgb           # a blueprint id glob
  - type: sensor.lidar.*        # or {type, role_name}
    role_name: "top*"

ros:
  backend: auto                 # auto | cyclonedds | rclpy | none
  domain_id: 0                  # default: ROS_DOMAIN_ID, else 0
  publish_tf: true
  publish_clock: true
```

## `renderer`

| key | default | meaning |
|---|---|---|
| `backend` | `primitives` | the renderer `carla-canvas serve` runs |
| `endpoint` | `localhost:50051` | where the plugin connects and the service listens |
| `command` | none | started by the plugin when nothing answers at `endpoint`; `{config}` is replaced by the file's path. It runs with the program's environment and ends with it |
| `startup_timeout` | 60 | seconds the plugin waits for a started renderer (also `LoadWorld`'s deadline) |
| `timeout` | 30 | deadline of every other call |
| `async` | true | the plugin's thread renders frames from `World.wait_for_tick` too (asynchronous mode) |
| `assets` | `{}` | passed to the backend (`LoadWorld`) |
| `options` | `{}` | the backend's `Renderer.options`: the service merges them (the client's file over its own) before `load_world` |

Common `options`: `lidar_pixels_per_degree` (8): resolution of the depth
images LiDARs read when the renderer has no ray casts.

### `gaussian_splatting` (reference)

The built-in reference renderer; splatsim's backend documents its own keys
(splatsim `docs/carla_canvas.md`).

```yaml
renderer:
  backend: gaussian_splatting
  assets:
    maps:                         # first glob matching the map's name wins
      "Town10HD*":                # (Carla/Maps/Town10HD_Opt: the last part or the whole)
        scene: town10/point_cloud.ply
        transform:                # model frame -> CARLA world
          location: [0, 0, 0]     # meters
          rotation: [0, 0, 0]     # pitch, yaw, roll, degrees
          scale: 1.0
          flip_y: true            # right-handed model (COLMAP, ROS, OpenCV) to CARLA
          # matrix: [[...], ...]  # or a 4x4 row-major matrix instead
        semantic_tag: 0           # label of Gaussians without a `semantic` property
    actors:                       # per blueprint glob, first match
      "vehicle.*":
        asset: cars/sedan.ply
        transform: {flip_y: true} # model frame -> actor frame
        semantic_tag: auto        # auto: the actor's (Car, Truck, Pedestrians, ...)
  options:
    rasterizer: auto              # auto (gsplat if usable) | gsplat | numpy
    device: auto                  # gsplat: cuda
    sh_degree: 3                  # spherical harmonics bands used
    background: [0, 0, 0]         # RGB 0-255 where nothing covers
    near: 0.1
    far: 1000.0
```

Models are INRIA-layout `.ply` files (3DGS, gsplat, nerfstudio splatfacto
exports). An optional `semantic` (or `label`) uint8 property per Gaussian
gives CityObjectLabels for segmentation cameras and the semantic LiDAR.

### `primitives`

| option | default | |
|---|---|---|
| `ground_z` | 0.0 | ground plane height (`null`: none) |
| `sky_color`, `ground_color` | `[135, 180, 235]`, `[90, 90, 90]` | RGB |
| `light_direction` | `[0.3, -0.4, -1.0]` | shading |

The service has no authentication and merges the client's `renderer` section
(asset paths included) over its own: keep the endpoint on localhost or a
trusted network. A renderer started by the plugin stops with the program
(`carla-canvas serve` watches it through `CARLA_CANVAS_PARENT_PID`).

## `sensors`

A sensor is rendered by carla_canvas when a rule matches its blueprint id
(and, if given, its `role_name` attribute). Without `sensors`: the rgb,
depth, semantic and instance segmentation cameras and both ray-cast LiDARs
(the others, e.g. DVS, optical flow, normals, HSS LiDAR, stay CARLA's). A rule naming a type the renderer
cannot draw makes that sensor's `listen()` fail.

## `ros`

`auto` uses rclpy when it imports (a sourced ROS 2 environment), else
CycloneDDS (`pip install 'carla-canvas[ros]'`), else none. `none` turns ROS
off: `enable_for_ros()` on a rendered sensor then fails.

## Environment

| variable | |
|---|---|
| `CARLA_CANVAS_CONFIG` | the configuration; unset: carla_canvas is off |
| `CARLA_CANVAS_PLUGIN` | use this plugin library instead of the installed one |
| `TYPESAFE_CARLA_SENSOR_PLUGIN` | what typesafe_carla loads; set by its launcher, or by `carla-canvas env` |
| `ROS_DOMAIN_ID` | the DDS domain when `ros.domain_id` is not set |
