# ROS 2 output

CARLA UE5 publishes a sensor on ROS 2 after `sensor.enable_for_ros()`. For the
sensors carla_canvas renders, the server never sees that call: the renderer
service publishes the same topics, with the same message types, frames,
stamps and conversions as CARLA's publishers
(`LibCarla/source/carla/ros2/`). It is implemented once in `carla_canvas.ros`,
so no renderer implements DDS and nodes cannot tell the difference.

| CARLA sensor | topics | type |
|---|---|---|
| cameras | `<base>/image` | `sensor_msgs/Image`, `bgra8`, step = width·4 |
| | `<base>/camera_info` | `sensor_msgs/CameraInfo`: `plumb_bob`, D = 0, K = [fx 0 w/2; 0 fx h/2; 0 0 1] with fx = w / (2 tan(fov/2)), R = I, P = [K 0], ROI = image, do_rectify |
| `sensor.lidar.ray_cast` | `<base>/point_cloud` | `sensor_msgs/PointCloud2`: x, y, z, intensity (float32), 16 B per point |
| `sensor.lidar.ray_cast_semantic` | `<base>/point_cloud` | x, y, z, cos_inc_angle (float32), object_idx, object_tag (uint32), 24 B |
| every published sensor | `/tf` | `tf2_msgs/TFMessage`: parent frame → sensor frame |
| | `/clock` | `rosgraph_msgs/Clock`, once per frame |

* `<base>` = `/carla/` + the ancestors' `ros_name`s (those that have one) +
  the sensor's name. The name is the blueprint's `ros_name` attribute, or for
  CARLA's placeholder (`rgb__`, `ray_cast__`, ...) the prefix plus the actor
  id. The frame id is the name.
* Points and transforms are converted to ROS's right-handed frame as CARLA
  does: y negated; rotations with CARLA's `TransformFromCarlaRotation`. The TF
  is relative to the parent with a ROS name, or to `map`.
* Stamps are simulation time (`elapsed_seconds`).
* PointCloud2: height 1, little endian, `is_dense` false.

## Transports

| `ros.backend` | |
|---|---|
| `cyclonedds` | plain DDS (`pip install 'carla-canvas[ros]'`), no ROS installation: ROS 2 type names (`sensor_msgs::msg::dds_::Image_`), topic names `rt/carla/...`, QoS reliable / volatile / keep last 10. ROS 2 nodes on Cyclone DDS or Fast DDS receive them in the same `ROS_DOMAIN_ID` |
| `rclpy` | a node `carla_canvas` in a sourced ROS 2 environment |
| `auto` | rclpy if it imports, else cyclonedds |
| `none` | off: `enable_for_ros()` on a rendered sensor fails |

The transports take carla_canvas's own message dataclasses
(`carla_canvas.ros.messages`); adding another middleware is a `Transport`
subclass with one `publisher(topic, type)` method.
