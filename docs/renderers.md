# Writing a renderer

A renderer subclasses `carla_canvas.Renderer` and draws camera views. The
service around it handles the protocol, the scene, the sensors' attributes
and encodings, the LiDAR model and ROS 2 — none of that is the renderer's job.

```python
import numpy as np
from carla_canvas import CameraFrame, CameraView, Renderer, WorldInfo


class FlatRenderer(Renderer):
    name = "flat"
    version = "1"
    camera_outputs = frozenset({"rgb", "depth"})   # what render_camera can produce

    def load_world(self, world: WorldInfo) -> None:
        # world.map_name ("Carla/Maps/Town10HD_Opt"), world.config (the `renderer`
        # section: assets, options), world.resolve(path) for relative asset paths
        self.color = world.config.get("options", {}).get("color", [128, 128, 128])

    def render_camera(self, view: CameraView) -> CameraFrame:
        cam = view.camera                  # width, height, fov, pose (world Transform)
        h, w = cam.height, cam.width
        frame = CameraFrame()
        if "rgb" in view.outputs:
            frame.rgb = np.full((h, w, 3), self.color, np.uint8)
        if "depth" in view.outputs:
            frame.depth = np.full((h, w), np.inf, np.float32)   # meters along the camera axis
        return frame
```

Run it: `renderer.backend: mypackage.flat:FlatRenderer`, or from Python:

```python
from carla_canvas.server import serve
serve(FlatRenderer(), "0.0.0.0:50051")
```

## The view

`CameraView`:

| field | |
|---|---|
| `camera` | `PinholeCamera`: `width`, `height`, `fov` (horizontal, degrees), `pose` (`Transform`, world), `focal`, `intrinsics()`, `ray_directions()` (sensor frame), `world_to_camera_cv()` (OpenCV view matrix) |
| `scene` | `Scene`: `actors` (`ActorInfo`: type id, attributes, parent, bounding box, `semantic_tag`), `states` (transforms, velocities), `frame`, `elapsed_seconds`, `map_name` |
| `outputs` | the subset of `camera_outputs` needed now |
| `sensor_id`, `attributes` | the sensor and its blueprint attributes |
| `max_depth` | ignore anything farther |
| `ignore_actors` | actors not to draw (a LiDAR's own vehicle) |

### Coordinate frame

A renderer says which frame it works in, and carla_canvas converts for it:

```python
class MyRenderer(Renderer):
    coordinate_frame = "right_handed"   # or "carla" (default)
```

| `coordinate_frame` | world | bodies (sensors, actors) |
|---|---|---|
| `carla` | Unreal's, left-handed: x forward, y right, z up | x forward, y right, z up |
| `right_handed` | the same with y negated: OpenDRIVE's, ROS's (CARLA's ROS publisher's transforms) | x forward, y left, z up (FLU) |

Poses are the CARLA world transforms of the tick's snapshot (a sensor's own
entry: no parent × mount arithmetic in a renderer). The views give them in
the renderer's frame:

| accessor | |
|---|---|
| `view.pose_matrix()` | 4x4 sensor-to-world |
| `view.world_to_camera_cv()` | 4x4 world-to-camera, OpenCV axes (x right, y down, z forward); `geometry.view_matrix_cv(pose, frame)` for a pose the renderer mapped further (its own scene frame) |
| `view.actor_matrix(id)`, `view.parent_matrix()` | an actor's / the sensor's vehicle's 4x4 body-to-world |
| `lidar_view.body_directions()`, `body_azimuths()` | ray directions in the sensor's body axes, and their atan2(y, x) in degrees |

`cast_rays` gets origins and directions in the renderer's frame and returns
normals in it. `view.scene` (states, `ActorInfo.bounding_box`) and the raw
fields (`camera.pose`, `directions`, `azimuths`) stay CARLA's;
`Transform.matrix(frame)` converts any of them. `geometry.quaternion_xyzw`
gives a rotation's quaternion.

`CameraFrame` (all `(height, width, …)`):

| output | array | |
|---|---|---|
| `rgb` | uint8 (H, W, 3), RGB | RGB camera |
| `depth` | float32 (H, W), meters along the camera's x; `inf`: nothing | depth camera, LiDARs |
| `semantic` | uint8 (H, W), `carla_canvas.scene.CityObjectLabel` | segmentation cameras, semantic LiDAR |
| `actor_id` | uint32 (H, W), 0: none | instance segmentation, semantic LiDAR |

The sensor types the service accepts follow from `camera_outputs`: `rgb` →
`sensor.camera.rgb`; `depth` → `sensor.camera.depth` (and LiDARs, from depth
images); `semantic` → segmentation camera (and the semantic LiDAR with
`depth`); `instance` → instance segmentation.

## LiDARs (optional)

carla_canvas fires the rays as CARLA's LiDAR does and hands them to
`cast_lidar_rays(view: LidarView) -> RayHits`:

| `LidarView` | |
|---|---|
| `pose` | the sensor's world `Transform` |
| `directions` | (N, 3) unit, sensor frame (x forward, y right, z up) |
| `channel`, `elevations`, `azimuths` | each ray's laser index, the lasers' pitches (descending), each ray's yaw (degrees, right positive) |
| `max_distance`, `semantic`, `attributes`, `ignore_actors` | range, semantic LiDAR, blueprint attributes, the LiDAR's own vehicle |

`RayHits`: `distance` (inf: none) and optionally `intensity` ([0, 1]: replaces
CARLA's `exp(-attenuation · d)`), `normal`, `semantic`, `actor_id`. Override
it for a native LiDAR (set `semantic_lidar = True` if it fills the semantic
fields); carla_canvas still applies CARLA's drop-off, noise and layout.
Without an override it uses `cast_rays(scene, origins, directions,
max_distance, ignore_actors)` when the renderer has exact ray casts (again
`semantic_lidar = True` if they fill the semantic fields), else depth images
(`renderer.options.lidar_pixels_per_degree`).

## splatsim

[splatsim](https://github.com/hakuturu583/splatsim) plugs in as a backend
(`renderer.backend: splatsim.carla_canvas:SplatsimRenderer`, its
`carla_canvas` extra): its gsplat renderer for cameras, its LiDAR panorama
rasterizer (learned intensity and ray drop) through `cast_lidar_rays`, its
scene USDZ and actor bank for assets, placed in CARLA through the map's
geo-reference. See splatsim's documentation.

## Gaussian splatting (reference)

The built-in `gaussian_splatting` backend is the reference and test
implementation (for real scenes, splatsim above).

`carla_canvas.renderers.gaussian_splatting` composes, per view, the map's
splats with one copy of an actor asset per matching actor, transformed by the
actor's CARLA transform (means and covariances; view-dependent colors are
evaluated in each model's own frame, so mirrored or scaled placements stay
correct). It rasterizes with gsplat on CUDA, or a numpy EWA reference
rasterizer (the 3DGS paper's algorithm, with its frustum-clamped Jacobian;
median depth). Segmentation comes from the dominant Gaussian per pixel.

`carla_canvas.renderers.demo_assets.write_demo_assets(dir)` writes a small
synthetic ground and a box-shaped car to try the pipeline without a trained
scene (`examples/gaussian_splatting_demo.yaml`).
