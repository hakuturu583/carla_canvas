# carla_canvas design

## 1. Goal

Replace how CARLA *renders* cameras and LiDARs — with 3D Gaussian Splatting,
another neural or classical renderer — while the program, written against the
CARLA API through typesafe_carla, stays as it is. Three constraints shape it:

1. **The program does not change.** No new imports or calls. What to render
   with, which assets, and which sensors are chosen at start-up by a
   configuration file named in `CARLA_CANVAS_CONFIG`.
2. **CARLA-compatible gRPC.** The client and the renderer talk over gRPC with
   messages that mirror the CARLA API (`proto/carla_canvas/v1/renderer.proto`),
   so a renderer can live in another process, on another host or GPU, in any
   language.
3. **ROS 2 without per-renderer work.** CARLA can publish sensors on ROS 2
   (`enable_for_ros()`); carla_canvas publishes the sensors it renders with
   CARLA's topics and types, once, for every renderer.

## 2. Components

| component | where | role |
|---|---|---|
| sensor source C ABI | typesafe_carla (ABI 5.3) | lets a plugin take over sensors' streams and ROS calls; builds LibCarla `Image` / `LidarMeasurement` objects from bytes |
| client plugin | `native/src/plugin.cpp` → `libcarla_canvas_plugin.so` | reads the configuration, claims sensors, collects the world state, calls the renderer, queues measurements |
| protocol | `proto/carla_canvas/v1/renderer.proto` | CARLA-shaped messages and the `Renderer` service (protocol version 2: `LoadWorld` carries the map's OpenDRIVE) |
| renderer service | `carla_canvas.server` | the protocol over a `Renderer`: scene, sensor models, encodings, LiDAR, ROS |
| renderers | `carla_canvas.renderers` | `gaussian_splatting`, `primitives`, or a user class |
| ROS 2 bridge | `carla_canvas.ros` | CARLA's topics over CycloneDDS or rclpy |

## 3. Loading

typesafe_carla loads a sensor plugin from `TYPESAFE_CARLA_SENSOR_PLUGIN`. Its
launcher (`typesafe-codon run`, `env`, and `import typesafe_carla.carla`) fills
that variable from the `typesafe_carla.sensor_plugins` entry point when it is
unset; carla_canvas's entry point returns the plugin's path only when
`CARLA_CANVAS_CONFIG` is set. So `CARLA_CANVAS_CONFIG=canvas.yaml typesafe-codon
run main.py` is all it takes, and without the variable nothing changes. A
built executable reads `TYPESAFE_CARLA_SENSOR_PLUGIN` only
(`eval "$(carla-canvas env)"`).

The plugin resolves typesafe_carla's functions through `get_proc` (it does not
link against the library), so one build serves every CARLA ref's library of a
typesafe_carla installation. It refuses an ABI other than 5.x with x ≥ 3.

## 4. Claiming sensors

At a sensor's first `listen()` (or `enable_for_ros()`), typesafe_carla asks the
plugin whether it supplies the sensor; the plugin matches the blueprint id and
`role_name` against the configuration's `sensors` rules (globs). For a claimed
sensor:

* LibCarla's `Listen` is never called: the server does not stream it. (The
  actor still exists in CARLA, so its attachment, `get_transform()`,
  `destroy()` and the rest of the Actor API are CARLA's.)
* The plugin registers it with the renderer (`SpawnSensor`: the actor with its
  blueprint attributes) and connects first if needed: `GetInfo` on the
  endpoint; if nothing answers and `renderer.command` is set, it starts that
  command (killed with the program: `PR_SET_PDEATHSIG`) and waits up to
  `startup_timeout`. `LoadWorld` sends the map name and the configuration's
  `renderer` section and the map's OpenDRIVE (with its geo-reference, for
  geo-referenced assets) once per episode; a new episode (`load_world`) re-sends
  everything.
* `enable_for_ros()` / `disable_for_ros()` / `is_enabled_for_ros()` go to the
  plugin and the renderer (`EnableForRos`), not to the server.

A failure (no renderer, a sensor type the renderer cannot draw) fails the
`listen()` with `CarlaError` and the plugin's message.

## 5. Frames

```
program thread                       plugin                       renderer service
world.tick() ─▶ server ─▶ frame N
              world_ticked(N) ─────▶ snapshot of N, new/removed actors,
                                     sensors due (sensor_tick)  ─▶ Render ─▶ scene.apply
                                                                           per sensor: render
                                                                           ROS publish
                                     tsc_sensor_source_push ◀──── measurements (deliver)
              dispatch: callbacks get frame N
```

* **Synchronous mode**: typesafe_carla calls the plugin after `World.tick()`
  returns and before it dispatches callbacks; the plugin renders that frame
  then. Frame N's measurements are delivered by the tick that returned N,
  every time (CARLA itself can deliver them a tick later).
* **Asynchronous mode** (`renderer.async`, default on): a plugin thread waits
  for ticks (`World.wait_for_tick`) and renders each new frame; with
  `World.tick()` both paths see the same frames and each frame is rendered
  once.
* **Due sensors**: every frame unless the blueprint's `sensor_tick` has not
  elapsed since the sensor's last measurement.
* **deliver**: a sensor with ROS enabled but no `listen()` is rendered for ROS
  only; the renderer returns no measurement for it.
* A sensor whose handles are all released (no stream left) stops being
  delivered: `tsc_sensor_source_push` reports it.
* A sensor is rendered from the first snapshot that contains it; one that
  leaves the snapshots without `Actor.destroy()` (a batch `DestroyActor`,
  another client) is forgotten and removed from the renderer.

The actors' static data (`Actor`: type id, attributes, parent, bounding box)
crosses once, when an actor first appears in a snapshot; afterwards only
snapshots (transform, velocities) per frame.

## 6. Measurements

The renderer answers with `SensorData`: frame, timestamp, the sensor's world
transform, and CARLA's raw layout:

| sensor | message | `raw_data` |
|---|---|---|
| `sensor.camera.rgb` | `Image` | BGRA |
| `sensor.camera.depth` | `Image` | CARLA's depth encoding: R + G·256 + B·256², normalized, × 1000 m |
| `sensor.camera.semantic_segmentation` | `Image` | CityObjectLabel in R |
| `sensor.camera.instance_segmentation` | `Image` | label in R, actor id in G (low) and B (high) |
| `sensor.lidar.ray_cast` | `LidarMeasurement` | {x, y, z, intensity} float32 per point, grouped by channel |
| `sensor.lidar.ray_cast_semantic` | `SemanticLidarMeasurement` | {x, y, z, cos_inc_angle} float32 + {object_idx, object_tag} uint32 |

The plugin turns them into typesafe_carla measurements
(`tsc_sensor_data_new_image` / `_lidar` / `_semantic_lidar`), which with
LibCarla are deserialized from CARLA's own wire format: they *are*
`carla::sensor::data::Image` / `LidarMeasurement` objects (LibCarla makes
image alpha 255, as for the server's images). `Image.convert`,
`save_to_disk`, iteration and the zero-copy views work unchanged.

## 7. Coordinates

Everything crossing the protocol is CARLA's: Unreal's left-handed frame (x
forward, y right, z up), meters, degrees (pitch, yaw, roll), world transforms.
`carla_canvas.geometry` implements Unreal's `FRotationMatrix`, so a renderer's
camera matches what the server would render (independent of LibCarla
version differences in `get_forward_vector` conventions). Cameras are
pinhole with CARLA's model: horizontal `fov`, square pixels, principal point
at the center, looking along +x. `PinholeCamera.world_to_camera_cv()` gives
the OpenCV-convention view matrix most splatting code expects. Depth is
z-depth (along the camera's x), as CARLA's depth camera.

LiDAR points are in the sensor's frame (x forward, y right, z up), as CARLA
sends them; the ROS bridge negates y, as CARLA's publisher does.

## 8. LiDAR model

`carla_canvas.lidar` reproduces RayCastLidar / RayCastSemanticLidar:

* per measurement, `points_per_second · Δt / channels` rays per channel,
  sweeping `rotation_frequency · horizontal_fov · Δt` degrees from where the
  previous one stopped, Δt being the time since the sensor's previous
  measurement (more than a frame with `sensor_tick`); channel i at `upper_fov − i·(upper_fov − lower_fov)/(channels − 1)`;
  the measurement's `horizontal_angle` is where the sweep stopped;
* ray-cast LiDAR: general drop-off (`dropoff_general_rate`), noise along the
  ray (`noise_stddev`), intensity `exp(−atmosphere_attenuation_rate · d)`,
  intensity drop-off (`dropoff_intensity_limit`, `dropoff_zero_intensity`);
* semantic LiDAR: `cos_inc_angle` from the surface normal, `object_idx` (actor
  id, 0 for the map) and `object_tag` (CityObjectLabel);
* a LiDAR does not see its own vehicle (the sensor's ancestors are ignored).

Hits come from `Renderer.cast_lidar_rays(LidarView)`: the rays as CARLA
fires them (directions, laser index, elevations, azimuths, range, blueprint
attributes). A renderer with its own LiDAR rasterizer overrides it (splatsim
renders a panorama at exactly the lasers' elevations and samples it) and may
return its own intensities, which replace the atmospheric model. Otherwise
`Renderer.cast_rays` answers when the renderer has it, else depth images: up
to four virtual pinhole cameras at the sensor (front, right, back, left), each
ray reading its nearest pixel; normals from the depth gradient (semantic
LiDAR only). This is how a camera-only renderer gets LiDARs.

## 9. ROS 2

See [ros2.md](ros2.md). The bridge resolves topic names when publishing (the
parent vehicle's `ros_name` is known from the snapshot), publishes per
rendered frame of a ROS-enabled sensor, and uses the transport the
configuration names: CycloneDDS with ROS 2 type names (`sensor_msgs::msg::dds_::Image_`)
and topic names (`rt/carla/...`), interoperable with ROS 2 nodes on standard
DDS RMWs, without a ROS installation; or rclpy in a sourced ROS 2 environment.

## 10. Limitations

* Rendered sensors: the four camera kinds and both LiDARs. Other sensors
  (GNSS, IMU, radar, collision, ...) stay CARLA's (they are not rendering).
* A claimed sensor's actor still exists in CARLA; whether the server renders
  an RGB camera nobody listens to depends on the server. Claiming happens at
  `listen()`/`enable_for_ros()`, after the spawn.
* `Client.apply_batch_sync(do_tick=True)` does not trigger the synchronous
  render; the asynchronous path renders those frames (delivered at a later
  dispatch point, as CARLA's own data may be).
* ROS names of CARLA's auto-named sensors use the actor id where CARLA uses
  the stream id.
* G-buffer streams (`listen_to_gbuffer`) of a claimed camera are CARLA's.
