#!/usr/bin/env python3
"""Bumps the package version (python/carla_canvas/__init__.py, which
scikit-build-core reads for the wheel's metadata). The auto-release job of
.github/workflows/release.yml calls it for a merged PR's release label.

    tools/bump_version.py patch          # 0.1.0 -> 0.1.1
    tools/bump_version.py minor          # 0.1.0 -> 0.2.0
    tools/bump_version.py major          # 0.1.0 -> 1.0.0
    tools/bump_version.py 1.4.2          # set exactly
    tools/bump_version.py --current      # print the version, change nothing

The one line printed is the new (or current) version.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

VERSION_FILE = Path(__file__).resolve().parent.parent / "python" / "carla_canvas" / "__init__.py"
_LINE = re.compile(r'(?m)^(__version__ = )"([^"]+)"')
_SEMVER = re.compile(r"\d+\.\d+\.\d+")


def current_version(path: Path = VERSION_FILE) -> str:
    match = _LINE.search(path.read_text())
    if match is None:
        raise SystemExit(f"could not find __version__ in {path}")
    return match.group(2)


def next_version(current: str, spec: str) -> str:
    if spec not in {"major", "minor", "patch"}:
        if not _SEMVER.fullmatch(spec):
            raise SystemExit(f"expected major|minor|patch or an X.Y.Z version, got {spec!r}")
        return spec
    if not _SEMVER.fullmatch(current):
        raise SystemExit(f"cannot bump {current!r}: not an X.Y.Z version; give one explicitly")
    major, minor, patch = (int(part) for part in current.split("."))
    if spec == "major":
        return f"{major + 1}.0.0"
    if spec == "minor":
        return f"{major}.{minor + 1}.0"
    return f"{major}.{minor}.{patch + 1}"


def bump(spec: str, path: Path = VERSION_FILE) -> str:
    new = next_version(current_version(path), spec)
    path.write_text(_LINE.sub(rf'\g<1>"{new}"', path.read_text(), count=1))
    return new


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    print(current_version() if argv[0] == "--current" else bump(argv[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
