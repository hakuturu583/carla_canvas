from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

import carla_canvas
from carla_canvas import plugin

ROOT = Path(__file__).resolve().parent.parent


def test_generated_protocol_is_up_to_date():
    pytest.importorskip("grpc_tools")
    rc = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_proto.py"), "--check"]).returncode
    assert rc == 0, "run tools/gen_proto.py"


def test_entry_point_only_when_configured(monkeypatch, tmp_path):
    monkeypatch.delenv("CARLA_CANVAS_CONFIG", raising=False)
    assert plugin.sensor_plugin() is None
    monkeypatch.setenv("CARLA_CANVAS_CONFIG", "canvas.yaml")
    fake = tmp_path / "libfake.so"
    fake.write_bytes(b"")
    monkeypatch.setenv(plugin.ENV_PLUGIN, str(fake))
    assert plugin.sensor_plugin() == str(fake.resolve())
    monkeypatch.setenv(plugin.ENV_PLUGIN, str(tmp_path / "missing.so"))
    with pytest.raises(plugin.PluginNotFound):
        plugin.sensor_plugin()


def test_protocol_version_matches_the_plugin():
    from carla_canvas.server.service import PROTOCOL_VERSION

    src = (ROOT / "native" / "src" / "plugin.cpp").read_text()
    assert f"kProtocolVersion = {PROTOCOL_VERSION};" in src


def test_version_is_in_one_place():
    text = (ROOT / "pyproject.toml").read_text()
    assert 'dynamic = ["version"]' in text
    assert re.fullmatch(r"\d+\.\d+\.\d+", carla_canvas.__version__)


def test_bump_version(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        import bump_version as bv
    finally:
        sys.path.pop(0)
    assert bv.current_version() == carla_canvas.__version__
    assert [bv.next_version("1.2.3", s) for s in ("patch", "minor", "major", "2.0.0")] == \
        ["1.2.4", "1.3.0", "2.0.0", "2.0.0"]
    with pytest.raises(SystemExit):
        bv.next_version("1.2.3", "huge")
    f = tmp_path / "__init__.py"
    f.write_text('"""doc"""\n\n__version__ = "0.1.0"\n')
    assert bv.bump("minor", f) == "0.2.0" and bv.current_version(f) == "0.2.0"
