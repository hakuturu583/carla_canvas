"""3D Gaussian Splatting models in the INRIA PLY layout (what 3DGS training
code, gsplat, nerfstudio's splatfacto export, ... write):

    x y z [nx ny nz] f_dc_0..2 [f_rest_0..] opacity scale_0..2 rot_0..3 [semantic]

opacity is a logit, scales are logs, rot is a (w, x, y, z) quaternion, f_rest
holds the higher spherical-harmonics bands channel-major. An optional
``semantic`` (or ``label``) property gives a carla.CityObjectLabel per
Gaussian (for segmentation cameras and the semantic LiDAR).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_PLY_TYPES = {"char": "i1", "uchar": "u1", "int8": "i1", "uint8": "u1", "short": "i2",
              "ushort": "u2", "int16": "i2", "uint16": "u2", "int": "i4", "uint": "u4",
              "int32": "i4", "uint32": "u4", "float": "f4", "float32": "f4", "double": "f8",
              "float64": "f8"}


@dataclass
class GaussianModel:
    means: np.ndarray  # (N, 3)
    quats: np.ndarray  # (N, 4) w, x, y, z, normalized
    scales: np.ndarray  # (N, 3) (exp of the stored logs)
    opacities: np.ndarray  # (N,) in [0, 1]
    sh: np.ndarray  # (N, K, 3), K = (degree + 1)^2
    labels: np.ndarray | None = None  # (N,) uint8
    _covariances: np.ndarray | None = field(default=None, init=False, repr=False, compare=False)

    def __len__(self) -> int:
        return len(self.means)

    @property
    def sh_degree(self) -> int:
        return int(round(np.sqrt(self.sh.shape[1]))) - 1

    def covariances(self) -> np.ndarray:
        """(N, 3, 3) R S S^T R^T (computed once: placing an actor's model every
        frame reuses it)."""
        if self._covariances is None:
            rs = quat_to_matrix(self.quats) * self.scales[:, None, :]
            self._covariances = rs @ rs.transpose(0, 2, 1)
        return self._covariances


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], axis=1)


def _read_header(f) -> tuple[str, int, list[tuple[str, str]]]:
    if f.readline().strip() != b"ply":
        raise ValueError("not a PLY file")
    fmt, count, props, in_vertex = "", 0, [], False
    while True:
        line = f.readline()
        if not line:
            raise ValueError("truncated PLY header")
        parts = line.decode("ascii", "replace").split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                count = int(parts[2])
            elif count == 0:
                raise ValueError("the vertex element must come first")
        elif parts[0] == "property" and in_vertex:
            if parts[1] == "list":
                raise ValueError("list properties are not supported in the vertex element")
            props.append((parts[2], _PLY_TYPES[parts[1]]))
        elif parts[0] == "end_header":
            return fmt, count, props


def load_ply(path: str | Path) -> GaussianModel:
    with open(path, "rb") as f:
        fmt, count, props = _read_header(f)
        endian = {"binary_little_endian": "<", "binary_big_endian": ">"}.get(fmt)
        if endian is None:
            raise ValueError(f"{path}: only binary PLY is supported (got {fmt})")
        dtype = np.dtype([(name, endian + t) for name, t in props])
        v = np.fromfile(f, dtype=dtype, count=count)
    if len(v) != count:
        raise ValueError(f"{path}: {len(v)} of {count} vertices")
    names = set(v.dtype.names)
    need = {"x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2", "opacity", "scale_0", "scale_1",
            "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"}
    missing = need - names
    if missing:
        raise ValueError(f"{path}: not a Gaussian splatting PLY (missing {sorted(missing)})")
    means = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    quats = np.stack([v[f"rot_{i}"] for i in range(4)], 1).astype(np.float64)
    quats /= np.maximum(np.linalg.norm(quats, axis=1, keepdims=True), 1e-12)
    scales = np.exp(np.stack([v[f"scale_{i}"] for i in range(3)], 1).astype(np.float64))
    opacities = 1.0 / (1.0 + np.exp(-v["opacity"].astype(np.float64)))
    dc = np.stack([v[f"f_dc_{i}"] for i in range(3)], 1).astype(np.float64)[:, None, :]
    rest_names = sorted((n for n in names if n.startswith("f_rest_")), key=lambda n: int(n[7:]))
    if rest_names:
        rest = np.stack([v[n] for n in rest_names], 1).astype(np.float64)
        k1 = len(rest_names) // 3
        rest = rest[:, :3 * k1].reshape(count, 3, k1).transpose(0, 2, 1)
        sh = np.concatenate([dc, rest], axis=1)
    else:
        sh = dc
    labels = None
    for name in ("semantic", "label"):
        if name in names:
            labels = v[name].astype(np.uint8)
    return GaussianModel(means, quats, scales, opacities, sh, labels)


def save_ply(path: str | Path, model: GaussianModel) -> None:
    """Writes the INRIA layout (for tests and tools)."""
    n, k, _ = model.sh.shape
    fields = ["x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2"]
    fields += [f"f_rest_{i}" for i in range(3 * (k - 1))]
    fields += ["opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"]
    dtype = [(f, "<f4") for f in fields]
    if model.labels is not None:
        dtype.append(("semantic", "u1"))
    v = np.empty(n, dtype=dtype)
    for i, c in enumerate("xyz"):
        v[c] = model.means[:, i]
    for i in range(3):
        v[f"f_dc_{i}"] = model.sh[:, 0, i]
    rest = model.sh[:, 1:, :].transpose(0, 2, 1).reshape(n, -1)
    for i in range(rest.shape[1]):
        v[f"f_rest_{i}"] = rest[:, i]
    op = np.clip(model.opacities, 1e-6, 1 - 1e-6)
    v["opacity"] = np.log(op / (1 - op))
    for i in range(3):
        v[f"scale_{i}"] = np.log(model.scales[:, i])
    for i in range(4):
        v[f"rot_{i}"] = model.quats[:, i]
    if model.labels is not None:
        v["semantic"] = model.labels
    header = ["ply", "format binary_little_endian 1.0", f"element vertex {n}"]
    header += [f"property {'uchar' if t == 'u1' else 'float'} {name}" for name, t in dtype]
    header.append("end_header")
    with open(path, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        f.write(v.tobytes())


SH_C0 = 0.28209479177387814
SH_C1 = 0.4886025119029199
SH_C2 = (1.0925484305920792, -1.0925484305920792, 0.31539156525252005, -1.0925484305920792,
         0.5462742152960396)
SH_C3 = (-0.5900435899266435, 2.890611442640554, -0.4570457994644658, 0.3731763325901154,
         -0.4570457994644658, 1.445305721320277, -0.5900435899266435)


def sh_to_rgb(sh: np.ndarray, dirs: np.ndarray, degree: int | None = None) -> np.ndarray:
    """Colors in [0, 1] of (N, K, 3) coefficients seen along unit (N, 3)
    directions (from the camera to the Gaussian, in the model's frame)."""
    k = sh.shape[1]
    deg = int(round(np.sqrt(k))) - 1 if degree is None else min(degree, int(round(np.sqrt(k))) - 1)
    result = SH_C0 * sh[:, 0]
    if deg > 0:
        x, y, z = (dirs[:, i:i + 1] for i in range(3))
        result = result - SH_C1 * y * sh[:, 1] + SH_C1 * z * sh[:, 2] - SH_C1 * x * sh[:, 3]
        if deg > 1:
            xx, yy, zz, xy, yz, xz = x * x, y * y, z * z, x * y, y * z, x * z
            result = (result + SH_C2[0] * xy * sh[:, 4] + SH_C2[1] * yz * sh[:, 5]
                      + SH_C2[2] * (2 * zz - xx - yy) * sh[:, 6] + SH_C2[3] * xz * sh[:, 7]
                      + SH_C2[4] * (xx - yy) * sh[:, 8])
            if deg > 2:
                result = (result + SH_C3[0] * y * (3 * xx - yy) * sh[:, 9]
                          + SH_C3[1] * xy * z * sh[:, 10] + SH_C3[2] * y * (4 * zz - xx - yy) * sh[:, 11]
                          + SH_C3[3] * z * (2 * zz - 3 * xx - 3 * yy) * sh[:, 12]
                          + SH_C3[4] * x * (4 * zz - xx - yy) * sh[:, 13]
                          + SH_C3[5] * z * (xx - yy) * sh[:, 14]
                          + SH_C3[6] * x * (xx - 3 * yy) * sh[:, 15])
    return np.clip(result + 0.5, 0.0, 1.0)


def rgb_to_sh0(rgb: np.ndarray) -> np.ndarray:
    """DC coefficients for colors in [0, 1] (for building models)."""
    return (np.asarray(rgb, float) - 0.5) / SH_C0
