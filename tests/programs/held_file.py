# A program holding a file open when the plugin starts the renderer: the
# renderer gets none of the program's descriptors but stdin, stdout and stderr
# (a socket it inherited would keep its port bound after the program closed it).
import os
import typesafe_carla as carla

held = open(os.getenv("HELD_FILE", ""), "w")
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
camera = world.spawn_actor(bp, carla.Transform(carla.Location(0.0, 0.0, 2.0))).as_sensor()
camera.listen(4)
world.tick()
print("image", camera.wait_for_data(5.0).as_image().width)
camera.destroy()
held.close()
print("done")
