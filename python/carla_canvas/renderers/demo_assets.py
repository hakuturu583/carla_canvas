"""Small synthetic splat models (tests, examples, trying the pipeline without
a trained scene): a checkered ground and a box-shaped "car"."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..scene import CityObjectLabel
from .gs_ply import GaussianModel, rgb_to_sh0, save_ply


def _model(means, colors, scales, labels=None, opacity=0.95) -> GaussianModel:
    n = len(means)
    quats = np.zeros((n, 4))
    quats[:, 0] = 1.0
    sh = rgb_to_sh0(colors)[:, None, :]
    return GaussianModel(np.asarray(means, float), quats, np.asarray(scales, float),
                         np.full(n, opacity), sh, labels)


def ground(size: float = 60.0, spacing: float = 1.0) -> GaussianModel:
    """Flat Gaussians on z = 0 (Roads), a 1 m checkerboard."""
    xs = np.arange(-size / 2, size / 2, spacing) + spacing / 2
    gx, gy = np.meshgrid(xs, xs)
    means = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], 1)
    checker = ((np.floor(gx) + np.floor(gy)) % 2).ravel()
    colors = np.where(checker[:, None] > 0, [0.45, 0.45, 0.45], [0.30, 0.30, 0.30])
    scales = np.tile([spacing * 0.6, spacing * 0.6, 0.01], (len(means), 1))
    labels = np.full(len(means), CityObjectLabel.Roads, np.uint8)
    return _model(means, colors, scales, labels)


def box(extent=(2.2, 1.0, 0.75), spacing: float = 0.25, color=(0.8, 0.1, 0.1)) -> GaussianModel:
    """Gaussians on the surface of a box centered at (0, 0, extent z)."""
    ex, ey, ez = extent
    pts = []
    for axis, e in enumerate((ex, ey, ez)):
        others = [i for i in range(3) if i != axis]
        a = np.arange(-(ex, ey, ez)[others[0]], (ex, ey, ez)[others[0]] + 1e-9, spacing)
        b = np.arange(-(ex, ey, ez)[others[1]], (ex, ey, ez)[others[1]] + 1e-9, spacing)
        ga, gb = np.meshgrid(a, b)
        for sign in (-1.0, 1.0):
            p = np.zeros((ga.size, 3))
            p[:, axis] = sign * e
            p[:, others[0]] = ga.ravel()
            p[:, others[1]] = gb.ravel()
            pts.append(p)
    means = np.concatenate(pts) + [0.0, 0.0, ez]
    colors = np.tile(color, (len(means), 1))
    scales = np.full((len(means), 3), spacing * 0.6)
    return _model(means, colors, scales)


def write_demo_assets(directory: str | Path) -> dict[str, Path]:
    d = Path(directory)
    d.mkdir(parents=True, exist_ok=True)
    paths = {"ground": d / "ground.ply", "car": d / "car.ply"}
    save_ply(paths["ground"], ground())
    save_ply(paths["car"], box())
    return paths
