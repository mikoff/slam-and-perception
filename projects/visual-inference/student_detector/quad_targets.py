"""Dense training targets for the quadrilateral proposal head."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import torch
from torch import Tensor

from .geometry import make_grid_points
from .quad_assigner import QuadAssigner
from .quad_geometry import quad_from_bbox


@dataclass(frozen=True)
class QuadTrainingTargets:
    quality: Tensor
    corner_offsets: Tensor
    positive_mask: Tensor
    trusted_background_mask: Tensor
    weak_background_mask: Tensor
    weak_negative_weight: float
    valid_point_masks: tuple[Tensor, Tensor, Tensor]
    valid_gt_count: Tensor
    fallback_count: Tensor
    unrepresentable_count: Tensor
    positive_counts_per_level: Tensor
    matched_gt_indices: Tensor

    def to(self, device: torch.device, *, non_blocking: bool = False) -> "QuadTrainingTargets":
        return QuadTrainingTargets(
            self.quality.to(device, non_blocking=non_blocking),
            self.corner_offsets.to(device, non_blocking=non_blocking),
            self.positive_mask.to(device, non_blocking=non_blocking),
            self.trusted_background_mask.to(device, non_blocking=non_blocking),
            self.weak_background_mask.to(device, non_blocking=non_blocking),
            self.weak_negative_weight,
            tuple(mask.to(device, non_blocking=non_blocking) for mask in self.valid_point_masks),  # type: ignore[arg-type]
            self.valid_gt_count.to(device, non_blocking=non_blocking),
            self.fallback_count.to(device, non_blocking=non_blocking),
            self.unrepresentable_count.to(device, non_blocking=non_blocking),
            self.positive_counts_per_level.to(device, non_blocking=non_blocking),
            self.matched_gt_indices.to(device, non_blocking=non_blocking),
        )


def _sample_quads(sample: object, name: str, box_name: str) -> Tensor:
    value = getattr(sample, name, None)
    if value is not None:
        return torch.as_tensor(value, dtype=torch.float32)
    box_value = getattr(sample, box_name, None)
    if box_value is None:
        return torch.empty((0, 4, 2), dtype=torch.float32)
    boxes = torch.as_tensor(box_value, dtype=torch.float32)
    if boxes.numel() == 0:
        return boxes.new_empty((0, 4, 2))
    return torch.stack([quad_from_bbox(box) for box in boxes])


def points_inside_quads(points: Tensor, quads: Tensor) -> Tensor:
    if quads.numel() == 0:
        return torch.zeros(points.shape[0], dtype=torch.bool, device=points.device)
    # All loader quads are canonical convex geometry. Evaluate their four
    # half-planes in one broadcast instead of launching one GPU kernel per
    # trusted-background tile.
    edges = torch.roll(quads, -1, dims=1) - quads
    relative = points[:, None, None, :] - quads[None, :, :, :]
    turns = (
        edges[None, :, :, 0] * relative[..., 1]
        - edges[None, :, :, 1] * relative[..., 0]
    )
    inside = (turns >= -1e-6).all(dim=2) | (turns <= 1e-6).all(dim=2)
    return inside.any(dim=1)


def point_validity_from_pixel_mask(
    pixel_mask: Tensor,
    feature_shapes: Sequence[tuple[int, int]],
    strides: Sequence[int],
) -> tuple[Tensor, tuple[Tensor, ...]]:
    points, level_slices = make_grid_points(
        feature_shapes, strides, device=pixel_mask.device, dtype=torch.float32
    )
    height, width = pixel_mask.shape[-2:]
    x = points[:, 0].floor().long().clamp(0, width - 1)
    y = points[:, 1].floor().long().clamp(0, height - 1)
    flattened = pixel_mask[y, x].bool()
    levels = tuple(
        flattened[level_slice].reshape(shape)
        for level_slice, shape in zip(level_slices, feature_shapes, strict=True)
    )
    return flattened, levels


class QuadTargetBuilder:
    """Build fixed-shape quad targets outside the exported model graph."""

    def __init__(self, assigner: QuadAssigner, *, weak_negative_weight: float = 0.0) -> None:
        if not 0 <= weak_negative_weight <= 1:
            raise ValueError("weak_negative_weight must be in [0, 1]")
        self.assigner = assigner
        self.weak_negative_weight = weak_negative_weight

    def __call__(
        self,
        samples: Sequence[object],
        feature_shapes: Sequence[tuple[int, int]],
        *,
        device: torch.device,
    ) -> QuadTrainingTargets:
        points, _ = make_grid_points(
            feature_shapes, self.assigner.strides, device=device, dtype=torch.float32
        )
        quality, corner_offsets, positive_masks = [], [], []
        matched_gt_indices = []
        trusted_background_masks, weak_background_masks = [], []
        valid_levels_by_batch: list[tuple[Tensor, ...]] = []
        valid_gt_count = torch.zeros((), device=device)
        fallback_count = torch.zeros((), device=device)
        unrepresentable_count = torch.zeros((), device=device)
        positive_counts = torch.zeros(len(feature_shapes), dtype=torch.long, device=device)

        for sample in samples:
            quads = _sample_quads(sample, "quads", "boxes").to(device)
            ignore_quads = _sample_quads(sample, "ignore_quads", "ignore_boxes").to(device)
            valid_flat, valid_levels = point_validity_from_pixel_mask(
                torch.as_tensor(sample.valid_mask, device=device),
                feature_shapes,
                self.assigner.strides,
            )
            assignment = self.assigner.assign(
                quads,
                feature_shapes,
                (int(sample.image.shape[-2]), int(sample.image.shape[-1])),
                valid_flat,
            )
            if assignment.unrepresentable_gt_indices.numel():
                ignore_quads = torch.cat((ignore_quads, quads[assignment.unrepresentable_gt_indices]))
            ignored = points_inside_quads(points, ignore_quads)
            trusted_quads = _sample_quads(
                sample, "trusted_background_quads", "trusted_background_boxes"
            ).to(device)
            background = valid_flat & ~ignored & ~assignment.positive_mask
            quality.append(assignment.quality_targets)
            corner_offsets.append(assignment.corner_targets)
            positive_masks.append(assignment.positive_mask)
            matched_gt_indices.append(assignment.matched_gt_indices)
            trusted_background = background & points_inside_quads(points, trusted_quads)
            trusted_background_masks.append(trusted_background)
            weak_background_masks.append(background & ~trusted_background)
            valid_levels_by_batch.append(valid_levels)
            valid_gt_count += assignment.valid_gt_mask.sum()
            fallback_count += assignment.fallback_gt_indices.numel()
            unrepresentable_count += assignment.unrepresentable_gt_indices.numel()
            positive_counts += assignment.positive_counts_per_level

        level_masks = tuple(
            torch.stack([valid_levels_by_batch[index][level] for index in range(len(samples))])
            for level in range(len(feature_shapes))
        )
        return QuadTrainingTargets(
            torch.stack(quality),
            torch.stack(corner_offsets),
            torch.stack(positive_masks),
            torch.stack(trusted_background_masks),
            torch.stack(weak_background_masks),
            self.weak_negative_weight,
            level_masks,  # type: ignore[arg-type]
            valid_gt_count,
            fallback_count,
            unrepresentable_count,
            positive_counts,
            torch.stack(matched_gt_indices),
        )
