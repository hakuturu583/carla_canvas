# A rendered camera destroyed by a batch command (not Actor.destroy()): the
# following ticks go on, and a new camera renders.
import typesafe_carla as carla

client = carla.Client("localhost", 2000)
world = client.get_world()
lib = world.get_blueprint_library()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

bp = lib.find("sensor.camera.rgb")
bp.set_attribute("image_size_x", "16")
bp.set_attribute("image_size_y", "8")
first = world.spawn_actor(bp, carla.Transform(carla.Location(0.0, 0.0, 2.0))).as_sensor()
first.listen(4)
world.tick()
print("first", first.wait_for_data(5.0).as_image().width)
responses = client.apply_batch_sync([carla.command.DestroyActor(first.id)])
print("batch error", responses[0].has_error())
world.tick()
world.tick()
second = world.spawn_actor(bp, carla.Transform(carla.Location(0.0, 0.0, 2.0))).as_sensor()
second.listen(4)
frame = world.tick()
print("second", second.wait_for_data(5.0).as_image().frame == frame)
second.destroy()
print("done")
