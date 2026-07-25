"""Point-grid and box geometry shared by assignment and inference."""

from __future__ import annotations

from collections.abc import Sequence

import torch
from torch import Tensor


def make_grid_points(
    feature_shapes: Sequence[tuple[int, int]],
    strides: Sequence[int],
    *,
    device: torch.device | str | None = None,
    dtype: torch.dtype = torch.float32,
) -> tuple[Tensor, tuple[slice, ...]]:
    """Create flattened (x, y) cell-center coordinates and per-level slices."""
    if len(feature_shapes) != len(strides):
        raise ValueError("feature_shapes and strides must have equal length")

    all_points: list[Tensor] = []
    level_slices: list[slice] = []
    offset = 0
    for (height, width), stride in zip(feature_shapes, strides, strict=True):
        y = (torch.arange(height, device=device, dtype=dtype) + 0.5) * stride
        x = (torch.arange(width, device=device, dtype=dtype) + 0.5) * stride
        grid_y, grid_x = torch.meshgrid(y, x, indexing="ij")
        points = torch.stack((grid_x.reshape(-1), grid_y.reshape(-1)), dim=1)
        all_points.append(points)
        level_slices.append(slice(offset, offset + points.shape[0]))
        offset += points.shape[0]
    return torch.cat(all_points, dim=0), tuple(level_slices)


def encode_ltrb(points: Tensor, boxes: Tensor) -> Tensor:
    """Encode paired points and xyxy boxes as left/top/right/bottom distances."""
    return torch.stack(
        (
            points[:, 0] - boxes[:, 0],
            points[:, 1] - boxes[:, 1],
            boxes[:, 2] - points[:, 0],
            boxes[:, 3] - points[:, 1],
        ),
        dim=1,
    )


def decode_ltrb(points: Tensor, distances: Tensor) -> Tensor:
    """Decode paired point-relative distances into xyxy boxes."""
    return torch.stack(
        (
            points[:, 0] - distances[:, 0],
            points[:, 1] - distances[:, 1],
            points[:, 0] + distances[:, 2],
            points[:, 1] + distances[:, 3],
        ),
        dim=1,
    )


def box_iou(boxes1: Tensor, boxes2: Tensor) -> Tensor:
    """Pairwise IoU for xyxy boxes."""
    top_left = torch.maximum(boxes1[:, None, :2], boxes2[None, :, :2])
    bottom_right = torch.minimum(boxes1[:, None, 2:], boxes2[None, :, 2:])
    intersection = (bottom_right - top_left).clamp(min=0).prod(dim=2)
    area1 = (boxes1[:, 2:] - boxes1[:, :2]).clamp(min=0).prod(dim=1)
    area2 = (boxes2[:, 2:] - boxes2[:, :2]).clamp(min=0).prod(dim=1)
    union = area1[:, None] + area2[None, :] - intersection
    return intersection / union.clamp(min=torch.finfo(intersection.dtype).eps)
