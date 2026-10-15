"""Regenerates the Python protocol modules from proto/ (commit the result).

    uv run python tools/gen_proto.py          # write
    uv run python tools/gen_proto.py --check  # fail if they are out of date

The C++ side is generated at build time by CMake (protoc + grpc_cpp_plugin).
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROTO = ROOT / "proto"
OUT = ROOT / "python"
FILES = ["carla_canvas/v1/renderer.proto"]
GENERATED = ["renderer_pb2.py", "renderer_pb2.pyi", "renderer_pb2_grpc.py"]


def generate(out: Path) -> None:
    import grpc_tools
    from grpc_tools import protoc

    include = Path(grpc_tools.__file__).parent / "_proto"
    for f in FILES:
        rc = protoc.main(["protoc", f"-I{PROTO}", f"-I{include}", f"--python_out={out}",
                          f"--pyi_out={out}", f"--grpc_python_out={out}", str(PROTO / f)])
        if rc != 0:
            raise SystemExit(f"protoc failed on {f}")


def main(argv: list[str]) -> int:
    target = OUT / "carla_canvas" / "v1"
    if "--check" in argv:
        with tempfile.TemporaryDirectory() as tmp:
            generate(Path(tmp))
            stale = [n for n in GENERATED
                     if (Path(tmp) / "carla_canvas" / "v1" / n).read_text()
                     != (target / n).read_text()]
        if stale:
            print(f"out of date: {stale}; run tools/gen_proto.py", file=sys.stderr)
            return 1
        return 0
    generate(OUT)
    (target / "__init__.py").write_text('"""Generated from proto/carla_canvas/v1 (tools/gen_proto.py)."""\n')
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
