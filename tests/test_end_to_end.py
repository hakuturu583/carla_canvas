"""A typesafe_carla program (Codon) against typesafe_carla's mock CARLA, with
the carla_canvas plugin and a renderer service: the program does not change.

Needs typesafe-codon (typesafe-carla installed with the mock backend:
TSC_BACKEND=mock) and the built plugin; skipped otherwise.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from carla_canvas.plugin import PluginNotFound, plugin_path
from carla_canvas.renderers.primitives import PrimitiveRenderer
from conftest import free_port

ROOT = Path(__file__).resolve().parent.parent
LAUNCHER = shutil.which("typesafe-codon") or str(Path(sys.executable).parent / "typesafe-codon")


def _mock_backend() -> bool:
    try:
        from typesafe_carla import paths

        return paths.native_info()["backend"] == "mock"
    except Exception:  # noqa: BLE001
        return False


pytestmark = pytest.mark.skipif(not Path(LAUNCHER).exists() or not _mock_backend(),
                                reason="needs typesafe-codon with the mock backend")


@pytest.fixture(scope="module")
def plugin() -> Path:
    try:
        return plugin_path()
    except PluginNotFound as e:
        pytest.skip(str(e))


def run_program(program: Path, config: Path, extra_env=None) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env.pop("TYPESAFE_CARLA_SENSOR_PLUGIN", None)  # found through the entry point
    env["CARLA_CANVAS_CONFIG"] = str(config)
    env.update(extra_env or {})
    return subprocess.run([LAUNCHER, "run", str(program)], env=env, capture_output=True,
                          text=True, timeout=900)


def test_unchanged_program_gets_rendered_sensors(tmp_path, plugin, make_server):
    stub, service = make_server(PrimitiveRenderer(), ros_backend="memory")
    endpoint = service.config.endpoint
    config = tmp_path / "canvas.yaml"
    config.write_text(f'renderer: {{endpoint: "{endpoint}", backend: primitives}}\n')
    result = run_program(ROOT / "tests" / "programs" / "callbacks_ros.py", config)
    assert result.returncode == 0, result.stdout + result.stderr
    out = result.stdout
    assert "listening True ros True gnss ros False" in out
    for frame in (1, 2, 3):  # each tick delivered its own frame's measurements
        assert f"tick {frame} image {frame} lidar {frame}" in out
    assert "widths 32" in out and "gnss True" in out and "stopped False" in out
    assert "tagged 0" not in out
    # ROS 2 through the renderer: the depth camera only (enable_for_ros)
    topics = set(service.ros.transport.messages)
    camera_topics = [t for t in topics if t.endswith("/image")]
    # under its vehicle, as CARLA names an unnamed one: vehicle<id>/depth<id>
    assert len(camera_topics) == 1
    assert re.fullmatch(r"/carla/vehicle\d+/depth\d+/image", camera_topics[0])
    assert len(service.ros.transport.messages[camera_topics[0]]) == 3
    # the program destroyed its sensors: the plugin told the renderer
    assert service.sensors == {}


def test_sensor_destroyed_by_a_batch_command(tmp_path, plugin, make_server):
    stub, service = make_server(PrimitiveRenderer(), ros_backend="none")
    config = tmp_path / "canvas.yaml"
    config.write_text(f'renderer: {{endpoint: "{service.config.endpoint}"}}\n')
    result = run_program(ROOT / "tests" / "programs" / "batch_destroy.py", config)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "first 16" in result.stdout and "batch error False" in result.stdout
    assert "second True" in result.stdout and "done" in result.stdout
    assert service.sensors == {}


def test_gaussian_splatting_end_to_end(tmp_path, plugin, make_server):
    from carla_canvas.renderers.demo_assets import write_demo_assets
    from carla_canvas.renderers.gaussian_splatting import GaussianSplattingRenderer

    write_demo_assets(tmp_path)
    renderer = GaussianSplattingRenderer()
    stub, service = make_server(renderer, ros_backend="memory", renderer_config={
        "backend": "gaussian_splatting",
        "assets": {"maps": {"*": {"scene": str(tmp_path / "ground.ply")}},
                   "actors": {"vehicle.*": {"asset": str(tmp_path / "car.ply")}}},
        "options": {"rasterizer": "numpy", "lidar_pixels_per_degree": 1}})
    config = tmp_path / "canvas.yaml"
    config.write_text(f'renderer: {{endpoint: "{service.config.endpoint}"}}\n')
    result = run_program(ROOT / "tests" / "programs" / "callbacks_ros.py", config)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "tick 3 image 3 lidar 3" in result.stdout
    assert renderer.world is not None and len(renderer.world.means) == 3600


def test_plugin_starts_the_renderer(tmp_path, plugin):
    port = free_port()
    config = tmp_path / "canvas.yaml"
    config.write_text(f"""
renderer:
  backend: primitives
  endpoint: "127.0.0.1:{port}"
  command: ["{sys.executable}", "-m", "carla_canvas.cli", "serve", "--config", "{{config}}"]
ros: {{backend: none}}
""")
    result = run_program(ROOT / "examples" / "camera_lidar.py", config)
    assert result.returncode == 0, result.stdout + result.stderr
    lines = [line for line in result.stdout.splitlines() if line.startswith("frame ")]
    assert len(lines) == 10 and all("640x360" in line for line in lines)
    assert "started the renderer" in result.stderr


def test_without_config_carla_renders(tmp_path, plugin):
    env = os.environ.copy()
    env.pop("CARLA_CANVAS_CONFIG", None)
    env.pop("TYPESAFE_CARLA_SENSOR_PLUGIN", None)
    result = subprocess.run([LAUNCHER, "run", str(ROOT / "examples" / "camera_lidar.py")],
                            env=env, capture_output=True, text=True, timeout=900)
    assert result.returncode == 0, result.stderr
    assert "carla_canvas" not in result.stderr


def test_unreachable_renderer_fails_listen(tmp_path, plugin):
    config = tmp_path / "canvas.yaml"
    config.write_text(f'renderer: {{endpoint: "127.0.0.1:{free_port()}", timeout: 1}}\n')
    result = run_program(ROOT / "examples" / "camera_lidar.py", config)
    assert result.returncode != 0
    assert "no renderer answers" in result.stdout + result.stderr
