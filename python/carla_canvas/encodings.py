"""CARLA's camera encodings (carla.Image.raw_data: BGRA, alpha 255)."""

from __future__ import annotations

import numpy as np

DEPTH_FAR = 1000.0  # meters, CARLA's depth camera range


def bgra(rgb: np.ndarray) -> np.ndarray:
    """(H, W, 3) RGB uint8 to (H, W, 4) BGRA."""
    h, w, _ = rgb.shape
    out = np.empty((h, w, 4), np.uint8)
    out[..., 0] = rgb[..., 2]
    out[..., 1] = rgb[..., 1]
    out[..., 2] = rgb[..., 0]
    out[..., 3] = 255
    return out


def depth_bgra(depth: np.ndarray) -> np.ndarray:
    """sensor.camera.depth: normalized = (R + G * 256 + B * 256^2) / (256^3 - 1),
    depth = 1000 m * normalized (what carla.ColorConverter.Depth decodes)."""
    d = np.where(np.isfinite(depth), depth, DEPTH_FAR)
    value = np.round(np.clip(d / DEPTH_FAR, 0.0, 1.0) * (256 ** 3 - 1)).astype(np.uint32)
    h, w = depth.shape
    out = np.empty((h, w, 4), np.uint8)
    out[..., 2] = value % 256  # R
    out[..., 1] = (value // 256) % 256  # G
    out[..., 0] = value // 65536  # B
    out[..., 3] = 255
    return out


def decode_depth(image_bgra: np.ndarray) -> np.ndarray:
    b, g, r = (image_bgra[..., i].astype(np.float64) for i in range(3))
    return (r + g * 256 + b * 65536) / (256 ** 3 - 1) * DEPTH_FAR


def semantic_bgra(labels: np.ndarray) -> np.ndarray:
    """sensor.camera.semantic_segmentation: the CityObjectLabel in red
    (carla.ColorConverter.CityScapesPalette colors it)."""
    h, w = labels.shape
    out = np.zeros((h, w, 4), np.uint8)
    out[..., 2] = labels
    out[..., 3] = 255
    return out


def instance_bgra(labels: np.ndarray, actor_ids: np.ndarray) -> np.ndarray:
    """sensor.camera.instance_segmentation: the label in red, the actor id's
    low 16 bits in green (low byte) and blue (high byte)."""
    out = semantic_bgra(labels)
    ids = actor_ids.astype(np.uint32)
    out[..., 1] = ids & 0xFF
    out[..., 0] = (ids >> 8) & 0xFF
    return out
