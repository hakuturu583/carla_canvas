# The typesafe_carla camera example, unchanged: run it with CARLA_CANVAS_CONFIG
# set and its camera and LiDAR come from the carla_canvas renderer.
import typesafe_carla as carla

client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
lib = world.get_blueprint_library()
vehicle = world.spawn_actor(lib.filter("vehicle.*")[0],
                            world.get_map().get_spawn_points()[0]).as_vehicle()

cam_bp = lib.find("sensor.camera.rgb")
cam_bp.set_attribute("image_size_x", "640")
cam_bp.set_attribute("image_size_y", "360")
camera = world.spawn_actor(cam_bp, carla.Transform(carla.Location(1.5, 0.0, 2.0)),
                           attach_to=vehicle).as_sensor()
lidar = world.spawn_actor(lib.find("sensor.lidar.ray_cast"),
                          carla.Transform(carla.Location(0.0, 0.0, 2.4)), attach_to=vehicle).as_sensor()

original = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
camera.listen(4)
lidar.listen(4)
try:
    vehicle.apply_control(carla.VehicleControl(throttle=0.4))
    for _ in range(10):
        world.tick()
        image = camera.wait_for_data(5.0).as_image()
        sweep = lidar.wait_for_data(5.0).as_lidar()
        # Mean brightness straight from the BGRA bytes (a zero-copy view).
        raw = image.raw_data
        total = 0
        for i in range(0, len(raw), 4):
            total += int(raw[i]) + int(raw[i + 1]) + int(raw[i + 2])
        nearest = 1e9
        for p in sweep:
            nearest = min(nearest, p.point.distance(carla.Location()))
        print("frame", image.frame, f"{image.width}x{image.height}",
              "brightness", total // (3 * image.width * image.height),
              "lidar points", len(sweep), "nearest", nearest)
finally:
    camera.stop()
    lidar.stop()
    world.apply_settings(original)
    for a in (camera, lidar):
        a.destroy()
    vehicle.destroy()
