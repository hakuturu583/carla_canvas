from __future__ import annotations

import math
import time

import grpc
import numpy as np
import pytest

from carla_canvas.encodings import decode_depth
from carla_canvas.geometry import Transform
from carla_canvas.lidar import SEMANTIC_DTYPE
from carla_canvas.renderer import CameraFrame, Renderer
from carla_canvas.renderers.primitives import PrimitiveRenderer
from carla_canvas.ros import messages as m
from carla_canvas.ros.bridge import carla_to_ros_transform, ros_name
from carla_canvas.scene import ActorInfo, CityObjectLabel
from carla_canvas.v1 import renderer_pb2 as pb
from conftest import actor, snapshot_request

VEHICLE, CAMERA, LIDAR, SEM = 10, 11, 12, 13


def setup_world(stub):
    stub.LoadWorld(pb.LoadWorldRequest(map_name="Carla/Maps/Town01", episode_id=3))
    stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(
        CAMERA, "sensor.camera.depth", VEHICLE, image_size_x=40, image_size_y=20, fov=90,
        ros_name="front")))
    stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(
        LIDAR, "sensor.lidar.ray_cast", VEHICLE, channels=8, range=60, points_per_second=8000,
        upper_fov=0, lower_fov=-30, dropoff_general_rate=0, dropoff_intensity_limit=0)))
    stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(
        SEM, "sensor.lidar.ray_cast_semantic", VEHICLE, channels=4, range=60,
        points_per_second=4000, upper_fov=0, lower_fov=-20)))


def poses():
    return {VEHICLE: Transform((0, 0, 0)), CAMERA: Transform((1.5, 0, 0.5)),
            LIDAR: Transform((0, 0, 2.5)), SEM: Transform((0, 0, 2.5)),
            20: Transform((12, 0, 0))}


def new_actors():
    return [actor(VEHICLE, "vehicle.ego", extent=(2, 1, 0.8), ros_name="hero"),
            actor(20, "vehicle.other", extent=(2.2, 1.0, 0.75), base_type="truck")]


def test_info_and_errors(make_server):
    stub, _ = make_server(PrimitiveRenderer())
    info = stub.GetInfo(pb.GetInfoRequest())
    assert info.name == "primitives" and info.protocol_version == 1
    assert "sensor.lidar.ray_cast_semantic" in info.sensor_types
    with pytest.raises(grpc.RpcError) as e:
        stub.Render(snapshot_request(1, {}))
    assert e.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    stub.LoadWorld(pb.LoadWorldRequest(map_name="Carla/Maps/Town01", episode_id=1))
    with pytest.raises(grpc.RpcError) as e:
        stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(5, "sensor.other.gnss")))
    assert e.value.code() == grpc.StatusCode.UNIMPLEMENTED
    with pytest.raises(grpc.RpcError) as e:
        stub.Render(snapshot_request(1, {}, sensors=[(99, True)]))
    assert e.value.code() == grpc.StatusCode.NOT_FOUND
    with pytest.raises(grpc.RpcError) as e:
        stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(6, "sensor.camera.rgb", fov=0)))
    assert e.value.code() == grpc.StatusCode.INVALID_ARGUMENT


def test_render_measurements(make_server):
    stub, service = make_server(PrimitiveRenderer())
    setup_world(stub)
    r = stub.Render(snapshot_request(7, poses(), new=new_actors(),
                                     sensors=[(CAMERA, True), (LIDAR, True), (SEM, True)]))
    by_id = {d.sensor_id: d for d in r.measurements}
    cam = by_id[CAMERA]
    assert cam.frame == 7 and cam.timestamp == pytest.approx(0.35)
    assert cam.transform.location.x == 1.5 and cam.image.width == 40 and cam.image.height == 20
    depth = decode_depth(np.frombuffer(cam.image.raw_data, np.uint8).reshape(20, 40, 4))
    # the truck's rear face at x = 12 - 2.2, seen from x = 1.5 (from inside the ego's box,
    # which does not hide it)
    assert depth[10, 20] == pytest.approx(12 - 2.2 - 1.5, abs=0.01)
    lidar = by_id[LIDAR].lidar
    pts = np.frombuffer(lidar.raw_data, "<f4").reshape(-1, 4)
    assert sum(lidar.points_per_channel) == len(pts) and len(lidar.points_per_channel) == 8
    assert lidar.horizontal_angle == pytest.approx(math.pi)
    assert pts[:, 2].min() == pytest.approx(-2.5, abs=1e-4)  # ground, 2.5 m below
    sem = np.frombuffer(by_id[SEM].semantic_lidar.raw_data, SEMANTIC_DTYPE)
    tags = set(sem["object_tag"].tolist())
    assert CityObjectLabel.Truck in tags and CityObjectLabel.Roads in tags
    assert VEHICLE not in set(sem["object_idx"].tolist())  # a LiDAR ignores its own vehicle
    # deliver=False: rendered (for ROS) but not returned
    r = stub.Render(snapshot_request(8, poses(), sensors=[(CAMERA, False)]))
    assert len(r.measurements) == 0
    # removed actors leave the scene
    stub.Render(snapshot_request(9, {k: v for k, v in poses().items() if k != 20}, removed=[20]))
    assert 20 not in service.scene.actors


class _Counting(Renderer):
    name = "counting"
    camera_outputs = frozenset({"rgb", "depth"})

    def __init__(self, options=None):
        super().__init__(options)
        self.calls = 0

    def load_world(self, world):
        self.world = world

    def render_camera(self, view):
        self.calls += 1
        h, w = view.camera.height, view.camera.width
        return CameraFrame(rgb=np.full((h, w, 3), 7, np.uint8), depth=np.full((h, w), 5.0, np.float32))


def test_camera_only_renderer_gets_lidar_through_depth(make_server):
    renderer = _Counting()
    stub, _ = make_server(renderer, renderer_config={"options": {"x": 1}})
    info = stub.GetInfo(pb.GetInfoRequest())
    assert set(info.sensor_types) == {"sensor.camera.rgb", "sensor.camera.depth",
                                      "sensor.lidar.ray_cast"}
    stub.LoadWorld(pb.LoadWorldRequest(map_name="Carla/Maps/X", episode_id=1))
    assert renderer.world.config == {"options": {"x": 1}}  # from the service's configuration
    stub.SpawnSensor(pb.SpawnSensorRequest(sensor=actor(
        LIDAR, "sensor.lidar.ray_cast", channels=2, range=100, points_per_second=400,
        upper_fov=0, lower_fov=-1, dropoff_general_rate=0, dropoff_intensity_limit=0)))
    r = stub.Render(snapshot_request(1, {LIDAR: Transform()}, sensors=[(LIDAR, True)]))
    pts = np.frombuffer(r.measurements[0].lidar.raw_data, "<f4").reshape(-1, 4)
    assert len(pts) == 20 and renderer.calls >= 1
    # every virtual camera sees a wall at depth 5 along its axis (front, back, sides)
    np.testing.assert_allclose(np.abs(pts[:, :2]).max(axis=1), 5.0, rtol=1e-4)


def test_ros_topics_like_carla(make_server):
    stub, service = make_server(PrimitiveRenderer(), ros_backend="memory")
    setup_world(stub)
    stub.EnableForRos(pb.EnableForRosRequest(sensor_id=CAMERA, enabled=True))
    stub.EnableForRos(pb.EnableForRosRequest(sensor_id=LIDAR, enabled=True))
    stub.Render(snapshot_request(4, poses(), new=new_actors(),
                                 sensors=[(CAMERA, False), (LIDAR, False)]))
    msgs = service.ros.transport.messages
    image = msgs["/carla/hero/front/image"][0]
    assert image.encoding == "bgra8" and image.width == 40 and image.step == 160
    assert image.header.frame_id == "front" and image.header.stamp == m.Time(0, 200_000_000)
    info = msgs["/carla/hero/front/camera_info"][0]
    assert info.k[0] == pytest.approx(20.0) and info.k[2] == 20.0 and info.k[5] == 10.0
    assert info.distortion_model == "plumb_bob" and info.roi.do_rectify
    cloud = msgs[f"/carla/hero/ray_cast{LIDAR}/point_cloud"][0]
    assert cloud.point_step == 16 and [f.name for f in cloud.fields] == ["x", "y", "z", "intensity"]
    tf = [t for msg in msgs["/tf"] for t in msg.transforms]
    cam_tf = next(t for t in tf if t.child_frame_id == "front")
    assert cam_tf.header.frame_id == "hero"  # relative to the parent with a ROS name
    assert cam_tf.transform.translation.x == pytest.approx(1.5)
    assert cam_tf.transform.translation.z == pytest.approx(0.5)
    assert len(msgs["/clock"]) == 1
    stub.EnableForRos(pb.EnableForRosRequest(sensor_id=CAMERA, enabled=False))
    stub.Render(snapshot_request(5, poses(), sensors=[(CAMERA, False)]))
    assert len(msgs["/carla/hero/front/image"]) == 1


def test_ros_name_and_transform_conventions():
    a = ActorInfo(42, "sensor.camera.rgb")
    assert ros_name(a) == "rgb42"
    assert ros_name(ActorInfo(42, "sensor.camera.rgb", attributes={"ros_name": "rgb__"})) == "rgb42"
    assert ros_name(ActorInfo(1, "sensor.lidar.ray_cast", attributes={"ros_name": "top"})) == "top"
    t = carla_to_ros_transform((1, 2, 3), (0, 90, 0))
    assert (t.translation.x, t.translation.y, t.translation.z) == (1, -2, 3)
    # CARLA yaw +90 (clockwise seen from above) is ROS yaw -90
    q = t.rotation
    assert q.w == pytest.approx(math.cos(math.pi / 4)) and q.z == pytest.approx(-math.sin(math.pi / 4))


def test_cyclonedds_transport_round_trip():
    pytest.importorskip("cyclonedds")
    from carla_canvas.ros.transports import CycloneDdsTransport

    t = CycloneDdsTransport(domain_id=91)
    reader = t.subscribe("/carla/test/point_cloud", m.PointCloud2)
    pub = t.publisher("/carla/test/point_cloud", m.PointCloud2)
    time.sleep(0.3)
    from carla_canvas.ros.bridge import point_cloud

    pub.publish(point_cloud(m.Header(m.Time(3, 4), "test"),
                            np.array([1, 2, 3, 0.5], "<f4").tobytes(), False))
    got = []
    for _ in range(50):
        got += list(reader.take(10))
        if got:
            break
        time.sleep(0.05)
    assert got and got[0].header.frame_id == "test" and got[0].width == 1
    assert type(got[0]).__name__ == "PointCloud2"
    assert np.frombuffer(bytes(got[0].data), "<f4").tolist() == [1, -2, 3, 0.5]  # y into ROS
