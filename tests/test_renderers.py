from __future__ import annotations

import numpy as np
import pytest

from carla_canvas.geometry import PinholeCamera, Transform
from carla_canvas.lidar import LidarSimulator, SEMANTIC_DTYPE, cast_rays_from_depth
from carla_canvas.renderer import CameraView, WorldInfo
from carla_canvas.renderers.demo_assets import box, ground, write_demo_assets
from carla_canvas.renderers.gaussian_splatting import (
    GaussianSplattingRenderer,
    NumpyRasterizer,
    placement_matrix,
)
from carla_canvas.renderers.gs_ply import GaussianModel, load_ply, save_ply, sh_to_rgb
from carla_canvas.renderers.primitives import PrimitiveRenderer
from carla_canvas.scene import ActorInfo, ActorState, BoundingBox, CityObjectLabel, Scene
from carla_canvas.sensors import LidarSpec


def scene_with_car(at=(10.0, 0.0, 0.0)) -> Scene:
    s = Scene()
    s.add_actor(ActorInfo(1, "vehicle.test.car", attributes={"base_type": "car"},
                          bounding_box=BoundingBox((0, 0, 0.75), (2.2, 1.0, 0.75))))
    s.states[1] = ActorState(Transform(at))
    return s


def view(scene, outputs, w=64, h=32, pose=Transform((0, 0, 1.0))):
    return CameraView(PinholeCamera(w, h, 90.0, pose), scene, frozenset(outputs))


def test_primitives_camera_sees_ground_car_and_sky():
    r = PrimitiveRenderer()
    f = r.render_camera(view(scene_with_car(), {"rgb", "depth", "semantic", "instance"}))
    assert f.rgb.shape == (32, 64, 3) and f.rgb.dtype == np.uint8
    center = (16, 32)
    assert f.semantic[center] == CityObjectLabel.Car and f.actor_id[center] == 1
    # the car's rear face is at x = 10 - 2.2: z-depth along the camera axis
    assert f.depth[center] == pytest.approx(7.8, abs=1e-6)
    assert f.semantic[0, 0] == CityObjectLabel.Sky and np.isinf(f.depth[0, 0])
    assert f.semantic[-1, 0] == CityObjectLabel.Roads


def test_primitives_rays_and_ignored_actors():
    r = PrimitiveRenderer()
    s = scene_with_car()
    o = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    d = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    hits = r.cast_rays(s, o, d, 100.0)
    np.testing.assert_allclose(hits.distance, [7.8, 1.0])
    np.testing.assert_allclose(hits.normal[0], [-1, 0, 0], atol=1e-12)
    assert hits.actor_id.tolist() == [1, 0]
    assert np.isinf(r.cast_rays(s, o, d, 100.0, frozenset({1})).distance[0])
    assert np.isinf(r.cast_rays(s, o, d, 5.0).distance[0])  # beyond max distance


def test_lidar_scan_pattern_and_layout():
    spec = LidarSpec(channels=4, range=50.0, points_per_second=4000, rotation_frequency=10.0,
                     upper_fov=0.0, lower_fov=-30.0, dropoff_general_rate=0.0,
                     dropoff_intensity_limit=0.0)
    sim = LidarSimulator(spec, sensor_id=9)
    scan = sim.scan(PrimitiveRenderer(), scene_with_car(), Transform((0, 0, 2.0)), 0.05)
    # 4000 pts/s * 0.05 s / 4 channels = 50 rays per channel over 180 degrees
    assert len(scan.points_per_channel) == 4 and sum(scan.points_per_channel) == len(scan.points)
    assert scan.points_per_channel[0] < 50  # channel 0 is horizontal: misses behind
    assert scan.points_per_channel[3] == 50  # -30 degrees: always the ground
    assert scan.horizontal_angle == pytest.approx(np.pi)
    assert len(scan.raw_data) == 16 * len(scan.points)
    pts = np.frombuffer(scan.raw_data, "<f4").reshape(-1, 4)
    ground_pts = pts[pts[:, 2] < -1.9]  # sensor frame: ground 2 m below
    assert len(ground_pts) > 0
    np.testing.assert_allclose(ground_pts[:, 2], -2.0, atol=1e-4)
    np.testing.assert_allclose(pts[:, 3], np.exp(-0.004 * np.linalg.norm(pts[:, :3], axis=1)),
                               rtol=1e-5)
    scan2 = sim.scan(PrimitiveRenderer(), scene_with_car(), Transform((0, 0, 2.0)), 0.05)
    assert scan2.horizontal_angle == pytest.approx(0.0, abs=1e-9)  # a full turn in two ticks


def test_semantic_lidar_and_depth_fallback_agree():
    spec = LidarSpec(semantic=True, channels=8, range=40.0, points_per_second=16000,
                     upper_fov=5.0, lower_fov=-25.0)
    scene = scene_with_car()
    pose = Transform((0, 0, 1.0))
    direct = LidarSimulator(spec, 3).scan(PrimitiveRenderer(), scene, pose, 0.05)
    assert len(direct.raw_data) == SEMANTIC_DTYPE.itemsize * len(direct.points)
    rec = np.frombuffer(direct.raw_data, SEMANTIC_DTYPE)
    assert set(rec["object_tag"].tolist()) <= {CityObjectLabel.Roads, CityObjectLabel.Car}
    assert (rec["object_idx"][rec["object_tag"] == CityObjectLabel.Car] == 1).all()
    assert (rec["cos_inc_angle"] >= 0).all() and (rec["cos_inc_angle"] <= 1).all()

    # the same rays through depth images (what a camera-only renderer gets)
    local = np.array([[1.0, 0.0, 0.0], [np.cos(0.3), 0.0, -np.sin(0.3)], [0.0, 1.0, -0.2]])
    local /= np.linalg.norm(local, axis=1, keepdims=True)
    via_depth = cast_rays_from_depth(PrimitiveRenderer(), scene, pose, local, 40.0, semantic=True,
                                     pixels_per_degree=20)
    world = local @ pose.rotation_matrix().T
    exact = PrimitiveRenderer().cast_rays(scene, np.tile(pose.location, (3, 1)), world, 40.0)
    np.testing.assert_allclose(via_depth.distance, exact.distance, rtol=0.02)
    assert via_depth.semantic.tolist() == exact.semantic.tolist()
    np.testing.assert_allclose(via_depth.normal[0], [-1, 0, 0], atol=0.05)


def test_ply_round_trip(tmp_path):
    m = box()
    m.sh = np.concatenate([m.sh, np.random.default_rng(0).normal(size=(len(m), 3, 3))], axis=1)
    m.labels = np.arange(len(m), dtype=np.uint8)
    save_ply(tmp_path / "m.ply", m)
    back = load_ply(tmp_path / "m.ply")
    np.testing.assert_allclose(back.means, m.means, atol=1e-6)
    np.testing.assert_allclose(back.scales, m.scales, rtol=1e-5)
    np.testing.assert_allclose(back.opacities, m.opacities, atol=1e-5)
    np.testing.assert_allclose(back.sh, m.sh, atol=1e-5)
    assert back.sh_degree == 1 and (back.labels == m.labels).all()


def test_sh_degree_zero_is_the_dc_color():
    from carla_canvas.renderers.gs_ply import rgb_to_sh0

    sh = rgb_to_sh0(np.array([[0.2, 0.5, 0.9]]))[:, None, :]
    np.testing.assert_allclose(sh_to_rgb(sh, np.array([[0.0, 0.0, 1.0]])), [[0.2, 0.5, 0.9]])


def test_numpy_rasterizer_single_gaussian():
    m = GaussianModel(np.array([[5.0, 0.0, 0.0]]), np.array([[1.0, 0, 0, 0]]),
                      np.array([[0.2, 0.2, 0.2]]), np.array([0.9]),
                      np.zeros((1, 1, 3)), np.array([7], np.uint8))
    from carla_canvas.renderers.gaussian_splatting import PlacedModel, Splats

    p = PlacedModel.place(m, np.eye(4), 0)
    s = Splats(p.means, p.covs, np.array([[1.0, 0.0, 0.0]]), m.opacities, p.labels, p.owners)
    cam = PinholeCamera(33, 33, 60.0, Transform())
    f = NumpyRasterizer().render(s, cam.world_to_camera_cv(), cam.intrinsics(), 33, 33, 0.1, 100,
                                 np.zeros(3), {"rgb", "depth", "semantic", "instance"})
    assert f.rgb[16, 16, 0] > 200 and f.rgb[16, 16, 1] == 0  # red, nearly opaque at the center
    assert f.rgb[0, 0].tolist() == [0, 0, 0]
    assert f.depth[16, 16] == pytest.approx(5.0)
    assert f.semantic[16, 16] == 7 and f.semantic[0, 0] == CityObjectLabel.Sky


def test_gaussian_splatting_renderer_with_map_and_actor_assets(tmp_path):
    write_demo_assets(tmp_path)
    r = GaussianSplattingRenderer({"rasterizer": "numpy"})
    r.load_world(WorldInfo("Carla/Maps/Town01", 1, {
        "assets": {"maps": {"Town0*": {"scene": "ground.ply"}},
                   "actors": {"vehicle.*": {"asset": "car.ply"}}}}, tmp_path))
    assert r.world is not None and len(r.actor_assets) == 1
    s = scene_with_car((8.0, 0.0, 0.0))
    f = r.render_camera(view(s, {"rgb", "depth", "semantic", "instance"}, w=48, h=24))
    center = (12, 24)
    assert f.semantic[center] == CityObjectLabel.Car and f.actor_id[center] == 1
    assert f.depth[center] == pytest.approx(8.0 - 2.2, abs=0.3)
    assert f.rgb[center][0] > 150  # the red box
    assert f.semantic[-1, 0] == CityObjectLabel.Roads
    assert f.semantic[0, 0] == CityObjectLabel.Sky
    # LiDAR through depth images
    spec = LidarSpec(channels=4, range=30.0, points_per_second=2000, upper_fov=0, lower_fov=-20,
                     dropoff_general_rate=0, dropoff_intensity_limit=0)
    r.options["lidar_pixels_per_degree"] = 2
    scan = LidarSimulator(spec, 2).scan(r, s, Transform((0, 0, 1.5)), 0.05)
    assert len(scan.points) > 0


def test_placement_flip_y_mirrors():
    m = placement_matrix({"flip_y": True, "location": [1, 2, 3], "scale": 2.0})
    np.testing.assert_allclose(m[:3, :3] @ [0, 1, 0], [0, -2, 0])
    np.testing.assert_allclose(m[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(placement_matrix({"matrix": np.eye(4).tolist()}), np.eye(4))
    with pytest.raises(ValueError):
        placement_matrix({"matrix": [[1, 0], [0, 1]]})


def test_ground_demo_model_is_flat():
    g = ground(size=4.0)
    assert (g.means[:, 2] == 0).all() and (g.labels == CityObjectLabel.Roads).all()


class RightHandedPrimitives(PrimitiveRenderer):
    """The primitives renderer working in the right_handed frame: its rays and
    normals are y-mirrored versions of CARLA's."""

    coordinate_frame = "right_handed"

    def cast_rays(self, scene, origins, directions, max_distance, ignore_actors=frozenset()):
        flip = np.array([1.0, -1.0, 1.0])
        hits = super().cast_rays(scene, origins * flip, directions * flip, max_distance,
                                 ignore_actors)
        hits.normal = hits.normal * flip
        return hits


def test_a_right_handed_renderer_gets_the_same_lidar():
    spec = LidarSpec(semantic=True, channels=8, range=40.0, points_per_second=16000,
                     upper_fov=5.0, lower_fov=-25.0)
    scene = scene_with_car(at=(10.0, 3.0, 0.0))
    pose = Transform((0, 0, 1.0), (0.0, 20.0, 0.0))
    a = LidarSimulator(spec, 3).scan(PrimitiveRenderer(), scene, pose, 0.05)
    b = LidarSimulator(spec, 3).scan(RightHandedPrimitives(), scene, pose, 0.05)
    assert a.raw_data == b.raw_data and len(a.points)
