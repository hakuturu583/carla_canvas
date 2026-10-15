from __future__ import annotations

import numpy as np
import pytest

from carla_canvas.config import ConfigError, load_config, parse_config
from carla_canvas.encodings import decode_depth, depth_bgra
from carla_canvas.geometry import (
    PinholeCamera,
    Transform,
    flu_axes,
    quaternion_xyzw,
    right_handed,
    rotation_from_matrix,
    rotation_matrix,
    transform_from_matrix,
    view_matrix_cv,
)
from carla_canvas.ros.bridge import carla_to_ros_transform
from carla_canvas.scene import ActorInfo, ActorState, Scene


def test_unreal_axes():
    # yaw 90: forward is +y (right); pitch 90: forward is up
    np.testing.assert_allclose(rotation_matrix(0, 90, 0)[:, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(rotation_matrix(90, 0, 0)[:, 0], [0, 0, 1], atol=1e-12)
    # roll 90: the right axis points down (Unreal's FRotationMatrix)
    np.testing.assert_allclose(rotation_matrix(0, 0, 90)[:, 1], [0, 0, -1], atol=1e-12)
    r = rotation_matrix(12, -40, 7)
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-12)


@pytest.mark.parametrize("rot", [(10, 20, 30), (-45, 170, -60), (0, -90, 0)])
def test_rotation_round_trip(rot):
    np.testing.assert_allclose(rotation_from_matrix(rotation_matrix(*rot)), rot, atol=1e-9)
    t = Transform((1, 2, 3), rot)
    back = transform_from_matrix(t.matrix())
    np.testing.assert_allclose(back.rotation, rot, atol=1e-9)
    p = np.array([[1.0, -2.0, 0.5]])
    np.testing.assert_allclose(t.inverse_transform_points(t.transform_points(p)), p, atol=1e-12)
    np.testing.assert_allclose(t.inverse_matrix() @ t.matrix(), np.eye(4), atol=1e-12)


def _quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize("rot", [(0, 0, 0), (10, 30, -5), (-20, 135, 40), (5, -90, 170)])
def test_right_handed_pose_is_carla_ros_transform(rot):
    """right_handed() is the frame CARLA's ROS publisher uses (y negated,
    TransformFromCarlaRotation), and quaternion_xyzw its quaternion."""
    t = Transform((12.0, -3.5, 1.2), rot)
    m = t.right_handed_matrix()
    ros = carla_to_ros_transform(t.location, t.rotation)
    np.testing.assert_allclose(m[:3, 3], (ros.translation.x, ros.translation.y, ros.translation.z))
    q = ros.rotation
    np.testing.assert_allclose(m[:3, :3], _quat_matrix(q.x, q.y, q.z, q.w), atol=1e-9)
    assert np.isclose(np.linalg.det(m[:3, :3]), 1.0)
    np.testing.assert_allclose(_quat_matrix(*quaternion_xyzw(m[:3, :3])), m[:3, :3], atol=1e-9)
    np.testing.assert_allclose(right_handed(m), t.matrix(), atol=1e-12)  # an involution
    # FLU axes: the body's left is CARLA's -right
    np.testing.assert_allclose(flu_axes(t.rotation_matrix())[:, 1], -t.rotation_matrix()[:, 1])


def test_view_matrix_cv():
    t = Transform((1.0, 2.0, 3.0), (0.0, 90.0, 0.0))  # looking along +y
    view = view_matrix_cv(t.location, t.rotation_matrix())
    np.testing.assert_allclose(view @ [1.0, 7.0, 3.0, 1.0], [0, 0, 5, 1], atol=1e-12)  # ahead
    np.testing.assert_allclose(view @ [0.0, 2.0, 3.0, 1.0], [1, 0, 0, 1], atol=1e-12)  # right
    np.testing.assert_allclose(view @ [1.0, 2.0, 2.0, 1.0], [0, 1, 0, 1], atol=1e-12)  # below
    np.testing.assert_allclose(PinholeCamera(4, 2, 90.0, t).world_to_camera_cv(), view)


def test_parent_transform():
    scene = Scene()
    scene.add_actor(ActorInfo(10, "vehicle.ego"))
    scene.add_actor(ActorInfo(11, "sensor.camera.rgb", parent_id=10))
    scene.add_actor(ActorInfo(12, "sensor.camera.rgb", parent_id=99))
    scene.states[10] = ActorState(Transform((5.0, 0.0, 0.0)))
    assert scene.parent_transform(11) == Transform((5.0, 0.0, 0.0))
    assert scene.parent_transform(10) is None and scene.parent_transform(12) is None


def test_camera_model():
    cam = PinholeCamera(4, 2, 90.0)
    assert cam.focal == pytest.approx(2.0)
    d = cam.ray_directions()
    assert d.shape == (2, 4, 3)
    assert (d[..., 0] > 0).all()
    assert d[0, 0, 1] < 0 and d[0, 0, 2] > 0  # top-left looks left and up
    # world_to_camera_cv: the sensor's forward is the OpenCV +z
    cam.pose = Transform((1, 2, 3), (0, 90, 0))
    v = cam.world_to_camera_cv()
    p = v @ np.array([1, 7, 3, 1.0])  # 5 m in front (yaw 90: +y)
    np.testing.assert_allclose(p[:3], [0, 0, 5], atol=1e-12)


def test_depth_encoding_round_trip():
    depth = np.array([[0.0, 1.0, 12.345], [999.0, np.inf, 0.001]])
    decoded = decode_depth(depth_bgra(depth))
    expected = np.where(np.isfinite(depth), depth, 1000.0)
    np.testing.assert_allclose(decoded, expected, atol=1000 / (256 ** 3))


def test_config(tmp_path):
    path = tmp_path / "canvas.yaml"
    path.write_text("""
renderer:
  backend: gaussian_splatting
  endpoint: "127.0.0.1:7000"
  command: [carla-canvas, serve, --config, "{config}"]
  timeout: 3
  options: {sh_degree: 1}
  assets: {maps: {"Town*": {scene: town.ply}}}
sensors:
  - {type: sensor.camera.rgb, role_name: front}
  - sensor.lidar.*
ros: {backend: cyclonedds, domain_id: 5}
""")
    c = load_config(path)
    assert c.backend == "gaussian_splatting" and c.endpoint == "127.0.0.1:7000"
    assert c.timeout == 3 and c.options == {"sh_degree": 1}
    assert [(r.type, r.role_name) for r in c.sensors] == [("sensor.camera.rgb", "front"),
                                                          ("sensor.lidar.*", "")]
    assert c.ros.backend == "cyclonedds" and c.ros.domain_id == 5
    d = parse_config({})
    assert "sensor.camera.rgb" in [r.type for r in d.sensors]
    assert not any(r.type.endswith(".dvs") or "*" in r.type for r in d.sensors)
    for bad in ({"sensors": {"type": "x"}}, {"renderer": []}, {"ros": {"backend": "zmq"}},
                {"renderer": {"timeout": 0}}):
        with pytest.raises(ConfigError):
            parse_config(bad)
    with pytest.raises(ConfigError, match="nope.yaml"):
        load_config(tmp_path / "nope.yaml")
