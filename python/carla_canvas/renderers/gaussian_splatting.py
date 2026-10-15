"""3D Gaussian Splatting in place of CARLA's cameras and LiDARs.

The map is a splat model (a trained 3DGS ``.ply``); vehicles, walkers and
other actors can have their own splat models, placed at the actors' CARLA
transforms every frame (compositional splatting), so the rendered traffic
follows the simulation. Depth and segmentation come from the same splats;
LiDARs sample rendered depth (carla_canvas.lidar).

Configuration (``renderer`` section, docs/renderers.md):

.. code-block:: yaml

    renderer:
      backend: gaussian_splatting
      assets:
        maps:
          "Town10HD*":                 # glob on the map's name (Carla/Maps/<name>)
            scene: town10/point_cloud.ply
            transform:                 # model frame -> CARLA world
              location: [0, 0, 0]      # meters
              rotation: [0, 0, 0]      # pitch, yaw, roll (degrees)
              scale: 1.0
              flip_y: true             # right-handed model (COLMAP, ROS, OpenCV) -> CARLA
            semantic_tag: 0            # label of Gaussians without one (Unlabeled)
        actors:
          "vehicle.*":
            asset: cars/sedan.ply
            transform: {flip_y: true}  # model frame -> actor frame
      options:
        device: auto                   # auto | cuda | cpu (gsplat needs cuda)
        rasterizer: auto               # auto | gsplat | numpy
        background: [0, 0, 0]          # RGB 0-255 where no Gaussian covers
        sh_degree: 3
        near: 0.1
        far: 1000.0

``transform`` may instead be a 4x4 ``matrix`` (row-major). Depth is the
median depth (where the accumulated opacity crosses 0.5) with the numpy
rasterizer and gsplat's expected depth with gsplat. The numpy
rasterizer is a reference: exact but slow (a Python loop over Gaussians),
for small models and tests; use gsplat (``pip install carla-canvas[gsplat]``)
with a GPU for real scenes.
"""

from __future__ import annotations

import fnmatch
import logging
from dataclasses import dataclass

import numpy as np

from ..geometry import Transform
from ..renderer import CameraFrame, CameraView, Renderer, WorldInfo
from ..scene import CityObjectLabel, Scene
from .gs_ply import GaussianModel, load_ply, sh_to_rgb

log = logging.getLogger("carla_canvas")

FLIP_Y = np.diag([1.0, -1.0, 1.0])


def placement_matrix(spec: dict | None) -> np.ndarray:
    """4x4 model-to-target matrix of an asset's `transform`."""
    spec = spec or {}
    if "matrix" in spec:
        m = np.asarray(spec["matrix"], float)
        if m.shape != (4, 4):
            raise ValueError("transform.matrix must be 4x4")
        return m
    t = Transform(tuple(spec.get("location", (0.0, 0.0, 0.0))),
                  tuple(spec.get("rotation", (0.0, 0.0, 0.0))))
    linear = t.rotation_matrix() * float(spec.get("scale", 1.0))
    if spec.get("flip_y", False):
        linear = linear @ FLIP_Y
    m = np.eye(4)
    m[:3, :3] = linear
    m[:3, 3] = t.location
    return m


@dataclass
class PlacedModel:
    """A model in a target frame: means and covariances transformed, with what
    view-dependent colors need (the inverse linear part)."""

    model: GaussianModel
    means: np.ndarray
    covs: np.ndarray
    linear_inv: np.ndarray  # target -> model directions
    labels: np.ndarray  # uint8
    owners: np.ndarray  # uint32 actor ids (0: the map)

    @staticmethod
    def place(model: GaussianModel, matrix: np.ndarray, label: int, owner: int = 0) -> "PlacedModel":
        linear = matrix[:3, :3]
        means = model.means @ linear.T + matrix[:3, 3]
        covs = linear @ model.covariances() @ linear.T
        labels = model.labels if model.labels is not None else np.full(len(model), label, np.uint8)
        return PlacedModel(model, means, covs, np.linalg.inv(linear), labels.astype(np.uint8),
                           np.full(len(model), owner, np.uint32))

    def colors(self, camera_position: np.ndarray, degree: int) -> np.ndarray:
        dirs = (self.means - camera_position) @ self.linear_inv.T
        dirs /= np.maximum(np.linalg.norm(dirs, axis=1, keepdims=True), 1e-12)
        return sh_to_rgb(self.model.sh, dirs, degree)


@dataclass
class Splats:
    means: np.ndarray
    covs: np.ndarray
    colors: np.ndarray  # (N, 3) in [0, 1]
    opacities: np.ndarray
    labels: np.ndarray
    owners: np.ndarray


class GaussianSplattingRenderer(Renderer):
    name = "gaussian_splatting"
    version = "1"
    camera_outputs = frozenset({"rgb", "depth", "semantic", "instance"})

    def __init__(self, options=None):
        super().__init__(options)
        self.world: PlacedModel | None = None
        self.actor_assets: list[tuple[str, GaussianModel, np.ndarray, object]] = []
        self._actor_models: dict[str, GaussianModel] = {}
        self._rasterizer = None

    # --- assets --------------------------------------------------------------------

    def load_world(self, world: WorldInfo) -> None:
        config = world.config or {}
        self.options = {**self.options, **(config.get("options") or {})}
        assets = config.get("assets") or {}
        self.world = None
        maps = assets.get("maps") or {}
        for pattern, spec in maps.items():
            if fnmatch.fnmatchcase(world.map_basename, pattern) or \
                    fnmatch.fnmatchcase(world.map_name, pattern):
                path = world.resolve(spec["scene"])
                model = load_ply(path)
                label = int(spec.get("semantic_tag", CityObjectLabel.NONE))
                self.world = PlacedModel.place(model, placement_matrix(spec.get("transform")), label)
                log.info("map %s: %d Gaussians from %s", world.map_name, len(model), path)
                break
        else:
            log.warning("no splat model for map %s (renderer.assets.maps): only actors are drawn",
                        world.map_name)
        self.actor_assets = []
        for pattern, spec in (assets.get("actors") or {}).items():
            path = str(world.resolve(spec["asset"]))
            model = self._actor_models.get(path)
            if model is None:
                model = self._actor_models[path] = load_ply(path)
            self.actor_assets.append((pattern, model, placement_matrix(spec.get("transform")),
                                      spec.get("semantic_tag")))
        self._rasterizer = None

    def _splats(self, scene: Scene, camera_position: np.ndarray, exclude: set[int]) -> Splats:
        degree = int(self.options.get("sh_degree", 3))
        parts: list[tuple[PlacedModel, np.ndarray]] = []
        if self.world is not None:
            parts.append((self.world, self.world.colors(camera_position, degree)))
        if self.actor_assets:
            for actor_id, info in scene.actors.items():
                if actor_id in exclude or actor_id not in scene.states or not info.is_visible_body:
                    continue
                for pattern, model, local, tag in self.actor_assets:
                    if info.matches(pattern):
                        label = info.semantic_tag if tag in (None, "auto") else int(tag)
                        m = scene.transform(actor_id).matrix() @ local
                        placed = PlacedModel.place(model, m, label, actor_id)
                        parts.append((placed, placed.colors(camera_position, degree)))
                        break
        if not parts:
            z = np.zeros((0, 3))
            return Splats(z, np.zeros((0, 3, 3)), z, np.zeros(0), np.zeros(0, np.uint8),
                          np.zeros(0, np.uint32))
        return Splats(np.concatenate([p.means for p, _ in parts]),
                      np.concatenate([p.covs for p, _ in parts]),
                      np.concatenate([c for _, c in parts]),
                      np.concatenate([p.model.opacities for p, _ in parts]),
                      np.concatenate([p.labels for p, _ in parts]),
                      np.concatenate([p.owners for p, _ in parts]))

    # --- rendering ---------------------------------------------------------------------

    def rasterizer(self):
        if self._rasterizer is None:
            choice = str(self.options.get("rasterizer", "auto"))
            device = str(self.options.get("device", "auto"))
            if choice in ("auto", "gsplat"):
                try:
                    self._rasterizer = GsplatRasterizer(device)
                except Exception as e:  # noqa: BLE001
                    if choice == "gsplat":
                        raise RuntimeError(f"gsplat is not usable: {e}") from e
                    log.warning("gsplat unavailable (%s): using the numpy rasterizer (slow)", e)
            if self._rasterizer is None:
                self._rasterizer = NumpyRasterizer()
        return self._rasterizer

    def render_camera(self, view: CameraView) -> CameraFrame:
        cam = view.camera
        exclude = set(view.ignore_actors) | {view.sensor_id}
        splats = self._splats(view.scene, np.asarray(cam.pose.location, float), exclude)
        near = float(self.options.get("near", 0.1))
        far = min(float(self.options.get("far", 1000.0)), view.max_depth)
        background = np.asarray(self.options.get("background", [0, 0, 0]), float) / 255.0
        out = self.rasterizer().render(splats, cam.world_to_camera_cv(), cam.intrinsics(),
                                       cam.width, cam.height, near, far, background, view.outputs)
        return out


class NumpyRasterizer:
    """EWA splatting as in the 3DGS paper, front to back, on the CPU."""

    name = "numpy"
    alpha_min = 1.0 / 255.0
    transmittance_min = 1e-4

    def render(self, s: Splats, viewmat: np.ndarray, k: np.ndarray, width: int, height: int,
               near: float, far: float, background: np.ndarray, outputs) -> CameraFrame:
        rgb = np.zeros((height, width, 3))
        trans = np.ones((height, width))
        median = np.full((height, width), np.inf)
        best_w = np.zeros((height, width))
        best_i = np.full((height, width), -1, np.int64)
        w_rot, w_t = viewmat[:3, :3], viewmat[:3, 3]
        if len(s.means):
            p = s.means @ w_rot.T + w_t
            z = p[:, 2]
            vis = np.nonzero((z > near) & (z < far))[0]
            p, z = p[vis], z[vis]
            fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
            cov_c = w_rot @ s.covs[vis] @ w_rot.T
            # As the reference rasterizer: the Jacobian is taken at the point
            # clamped to 1.3x the view frustum, so Gaussians beside or near the
            # camera do not blow up across the image.
            lim_x, lim_y = 1.3 * (width / 2.0) / fx, 1.3 * (height / 2.0) / fy
            tx = np.clip(p[:, 0] / z, -lim_x, lim_x) * z
            ty = np.clip(p[:, 1] / z, -lim_y, lim_y) * z
            j = np.zeros((len(vis), 2, 3))
            j[:, 0, 0] = fx / z
            j[:, 0, 2] = -fx * tx / (z * z)
            j[:, 1, 1] = fy / z
            j[:, 1, 2] = -fy * ty / (z * z)
            cov2 = j @ cov_c @ j.transpose(0, 2, 1) + 0.3 * np.eye(2)
            a, b, c = cov2[:, 0, 0], cov2[:, 0, 1], cov2[:, 1, 1]
            det = a * c - b * b
            ok = det > 0
            mid = 0.5 * (a + c)
            lam = mid + np.sqrt(np.maximum(mid * mid - det, 0.1))
            radius = np.ceil(3.0 * np.sqrt(lam))
            u = fx * p[:, 0] / z + cx
            v = fy * p[:, 1] / z + cy
            ok &= (u + radius > 0) & (u - radius < width) & (v + radius > 0) & (v - radius < height)
            order = np.argsort(z[ok], kind="stable")
            idx = np.nonzero(ok)[0][order]
            for i in idx:
                x0, x1 = max(int(u[i] - radius[i]), 0), min(int(u[i] + radius[i]) + 1, width)
                y0, y1 = max(int(v[i] - radius[i]), 0), min(int(v[i] + radius[i]) + 1, height)
                if x0 >= x1 or y0 >= y1:
                    continue
                t_patch = trans[y0:y1, x0:x1]
                if t_patch.max() < self.transmittance_min:
                    continue
                inv_det = 1.0 / det[i]
                ca, cb, cc = c[i] * inv_det, -b[i] * inv_det, a[i] * inv_det
                dx = np.arange(x0, x1) + 0.5 - u[i]
                dy = (np.arange(y0, y1) + 0.5 - v[i])[:, None]
                power = -0.5 * (ca * dx * dx + cc * dy * dy) - cb * dx * dy
                alpha = np.minimum(0.99, s.opacities[vis[i]] * np.exp(np.minimum(power, 0.0)))
                alpha[alpha < self.alpha_min] = 0.0
                w = alpha * t_patch
                rgb[y0:y1, x0:x1] += w[..., None] * s.colors[vis[i]]
                # median depth: where the accumulated opacity crosses 0.5
                crossing = (t_patch >= 0.5) & (t_patch * (1.0 - alpha) < 0.5)
                median[y0:y1, x0:x1][crossing] = z[i]
                better = w > best_w[y0:y1, x0:x1]
                best_w[y0:y1, x0:x1][better] = w[better]
                best_i[y0:y1, x0:x1][better] = vis[i]
                trans[y0:y1, x0:x1] = t_patch * (1.0 - alpha)
        return _frame(rgb, trans, median, best_i, s, background, outputs)


def _frame(rgb, trans, median, best_i, s: Splats, background, outputs) -> CameraFrame:
    frame = CameraFrame()
    alpha = 1.0 - trans
    covered = alpha > 0.5
    if "rgb" in outputs:
        frame.rgb = np.clip((rgb + trans[..., None] * background) * 255.0 + 0.5, 0, 255).astype(np.uint8)
    if "depth" in outputs:
        frame.depth = np.where(covered, median, np.inf).astype(np.float32)
    if "semantic" in outputs or "instance" in outputs:
        has = covered & (best_i >= 0)
        labels = np.full(best_i.shape, CityObjectLabel.Sky, np.uint8)
        labels[has] = s.labels[best_i[has]]
        frame.semantic = labels
        if "instance" in outputs:
            ids = np.zeros(best_i.shape, np.uint32)
            ids[has] = s.owners[best_i[has]]
            frame.actor_id = ids
    return frame


class GsplatRasterizer:
    """gsplat's CUDA rasterizer (gsplat >= 1.3: covariances as input)."""

    name = "gsplat"

    def __init__(self, device: str = "auto"):
        import torch
        from gsplat import rasterization

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        if device != "cuda":
            raise RuntimeError("gsplat needs a CUDA device")
        self.torch, self.rasterization, self.device = torch, rasterization, device

    def render(self, s: Splats, viewmat, k, width, height, near, far, background, outputs) -> CameraFrame:
        torch = self.torch
        dev = self.device
        n = len(s.means)
        if n == 0:
            empty = Splats(*(np.zeros((0,) + a.shape[1:], a.dtype) for a in (
                s.means, s.covs, s.colors, s.opacities, s.labels, s.owners)))
            return NumpyRasterizer().render(empty, viewmat, k, width, height, near, far,
                                            background, outputs)

        def t(a, dtype=torch.float32):
            return torch.as_tensor(np.ascontiguousarray(a), dtype=dtype, device=dev)

        means, covs, opac = t(s.means), t(s.covs), t(s.opacities)
        quats = torch.zeros((n, 4), device=dev)
        quats[:, 0] = 1.0
        scales = torch.ones((n, 3), device=dev)
        common = dict(viewmats=t(viewmat)[None], Ks=t(k)[None], width=width, height=height,
                      near_plane=near, far_plane=far, covars=covs)
        image, alpha, _ = self.rasterization(means, quats, scales, opac, t(s.colors),
                                             render_mode="RGB+ED",
                                             backgrounds=t(background)[None], **common)
        image = image[0].detach().cpu().numpy()
        alpha = alpha[0, ..., 0].detach().cpu().numpy()
        frame = CameraFrame()
        covered = alpha > 0.5
        if "rgb" in outputs:
            frame.rgb = np.clip(image[..., :3] * 255.0 + 0.5, 0, 255).astype(np.uint8)
        if "depth" in outputs:
            frame.depth = np.where(covered, image[..., 3], np.inf).astype(np.float32)
        if "semantic" in outputs or "instance" in outputs:
            # one channel per class: the argmax is the dominant label
            classes, inverse = np.unique(s.labels, return_inverse=True)
            onehot = np.eye(len(classes), dtype=np.float32)[inverse]
            labels_img, _, _ = self.rasterization(means, quats, scales, opac, t(onehot), **common)
            best = labels_img[0].argmax(-1).cpu().numpy()
            frame.semantic = np.where(covered, classes[best], CityObjectLabel.Sky).astype(np.uint8)
            if "instance" in outputs:
                owners, inverse = np.unique(s.owners, return_inverse=True)
                onehot = np.eye(len(owners), dtype=np.float32)[inverse]
                owner_img, _, _ = self.rasterization(means, quats, scales, opac, t(onehot), **common)
                best = owner_img[0].argmax(-1).cpu().numpy()
                frame.actor_id = np.where(covered, owners[best], 0).astype(np.uint32)
        return frame

