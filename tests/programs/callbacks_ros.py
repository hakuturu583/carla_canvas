# A typesafe_carla program with nothing carla_canvas-specific: callbacks,
# enable_for_ros(), a depth camera and a semantic LiDAR, synchronous mode.
import typesafe_carla as carla

client = carla.Client("localhost", 2000)
world = client.get_world()
lib = world.get_blueprint_library()
vehicle = world.spawn_actor(lib.filter("vehicle.*")[0], world.get_map().get_spawn_points()[0])

cam_bp = lib.find("sensor.camera.depth")
cam_bp.set_attribute("image_size_x", "32")
cam_bp.set_attribute("image_size_y", "16")
camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(1.5, 0.0, 2.0)),
                           attach_to=vehicle).as_sensor()
lidar_bp = lib.find("sensor.lidar.ray_cast_semantic")
lidar_bp.set_attribute("range", "50")
lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(0.0, 0.0, 2.4)),
                          attach_to=vehicle).as_sensor()
gnss = world.spawn_actor(lib.find("sensor.other.gnss"), carla.Transform(),
                         attach_to=vehicle).as_sensor()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

image_frames = List[int]()
widths = List[int]()
lidar_frames = List[int]()
tags = List[int]()
gnss_frames = List[int]()

def on_image(data: carla.SensorData):
    image = data.as_image()
    image_frames.append(image.frame)
    widths.append(image.width)

def on_lidar(data: carla.SensorData):
    sweep = data.as_semantic_lidar()
    lidar_frames.append(sweep.frame)
    for p in sweep:
        tags.append(int(p.object_tag))

camera.listen(on_image)
lidar.listen(on_lidar)
gnss.listen(lambda d: gnss_frames.append(d.frame))
camera.enable_for_ros()
print("listening", camera.is_listening, "ros", camera.is_enabled_for_ros(),
      "gnss ros", gnss.is_enabled_for_ros())
for _ in range(3):
    frame = world.tick()
    # rendered synchronously: this tick's dispatch delivered this frame
    print("tick", frame, "image", image_frames[-1], "lidar", lidar_frames[-1])
print("widths", widths[0], "tagged", len(tags), "gnss", len(gnss_frames) > 0)
camera.stop()
print("stopped", camera.is_listening)
for a in (camera, lidar, gnss):
    a.destroy()
vehicle.destroy()
print("done")
