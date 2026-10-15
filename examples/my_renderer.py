"""A custom renderer in a few lines: run it with
    python examples/my_renderer.py                 # serves at localhost:50051
and point the configuration at it (renderer.endpoint), or name it as the
backend: `renderer.backend: my_renderer:SkyAndGround` (on PYTHONPATH).

It draws a sky/ground split by camera pitch; carla_canvas supplies everything
else, including LiDARs (from its depth) and ROS 2 output."""

import numpy as np

from carla_canvas import CameraFrame, CameraView, Renderer, WorldInfo


class SkyAndGround(Renderer):
    name = "sky_and_ground"
    camera_outputs = frozenset({"rgb", "depth"})

    def load_world(self, world: WorldInfo) -> None:
        print("loading", world.map_name)

    def render_camera(self, view: CameraView) -> CameraFrame:
        cam = view.camera
        dirs = cam.ray_directions() @ cam.pose.rotation_matrix().T  # world directions
        height = cam.pose.location[2]
        down = dirs[..., 2] < -1e-6
        frame = CameraFrame()
        if "rgb" in view.outputs:
            rgb = np.empty(dirs.shape, np.uint8)
            rgb[:] = (135, 180, 235)
            rgb[down] = (90, 90, 90)
            frame.rgb = rgb
        if "depth" in view.outputs:
            distance = np.where(down, height / -np.minimum(dirs[..., 2], -1e-6), np.inf)
            frame.depth = (distance * cam.ray_directions()[..., 0]).astype(np.float32)
        return frame


if __name__ == "__main__":
    import logging

    from carla_canvas.server import serve

    logging.basicConfig(level=logging.INFO)
    serve(SkyAndGround(), "localhost:50051")
