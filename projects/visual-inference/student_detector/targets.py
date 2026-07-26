"""Convert per-image annotations into flattened P3-P5 training targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import torch
from torch import Tensor

from .assigner import ATSSAssigner
from .data import ProposalSample
from .geometry import make_grid_points


@dataclass(frozen=True)
class TrainingTargets:
    objectness: Tensor
    objectness_mask: Tensor
    objectness_weights: Tensor
    box_distances: Tensor
    centerness: Tensor
    positive_mask: Tensor
    valid_point_masks: tuple[Tensor, Tensor, Tensor]
    valid_gt_count: Tensor
    fallback_count: Tensor
    unrepresentable_count: Tensor
    positive_counts_per_level: Tensor


def points_inside_boxes(points: Tensor, boxes: Tensor) -> Tensor:
    """Return a point mask covering the union of xyxy boxes."""
    if boxes.numel() == 0:
        return torch.zeros(points.shape[0], dtype=torch.bool, device=points.device)
    inside = (
        (points[:, None, 0] >= boxes[None, :, 0])
        & (points[:, None, 0] <= boxes[None, :, 2])
        & (points[:, None, 1] >= boxes[None, :, 1])
        & (points[:, None, 1] <= boxes[None, :, 3])
    )
    return inside.any(dim=1)


def point_validity_from_pixel_mask(
    pixel_mask: Tensor,
    feature_shapes: Sequence[tuple[int, int]],
    strides: Sequence[int],
) -> tuple[Tensor, tuple[Tensor, ...]]:
    """Sample the input validity mask at feature cell centers."""
    points, level_slices = make_grid_points(
        feature_shapes,
        strides,
        device=pixel_mask.device,
        dtype=torch.float32,
    )
    height, width = pixel_mask.shape[-2:]
    x = points[:, 0].floor().long().clamp(0, width - 1)
    y = points[:, 1].floor().long().clamp(0, height - 1)
    flattened = pixel_mask[y, x].bool()
    levels = tuple(
        flattened[level_slice].reshape(feature_shape)
        for level_slice, feature_shape in zip(
            level_slices, feature_shapes, strict=True
        )
    )
    return flattened, levels


class TargetBuilder:
    """Batch target construction kept outside the exported detector graph."""

    def __init__(
        self,
        assigner: ATSSAssigner,
        *,
        background_loss_weights: Mapping[str, float] | None = None,
    ) -> None:
        self.assigner = assigner
        self.background_loss_weights = dict(background_loss_weights or {})
        if any(
            weight < 0 or weight > 1
            for weight in self.background_loss_weights.values()
        ):
            raise ValueError("background loss weights must be in [0, 1]")

    def __call__(
        self,
        samples: Sequence[ProposalSample],
        feature_shapes: Sequence[tuple[int, int]],
        *,
        device: torch.device,
    ) -> TrainingTargets:
        points, _ = make_grid_points(
            feature_shapes,
            self.assigner.strides,
            device=device,
            dtype=torch.float32,
        )
        objectness = []
        objectness_masks = []
        objectness_weights = []
        distances = []
        centerness = []
        positive_masks = []
        valid_levels_by_batch: list[tuple[Tensor, ...]] = []
        fallback_count = torch.zeros((), device=device)
        valid_gt_count = torch.zeros((), device=device)
        unrepresentable_count = torch.zeros((), device=device)
        positive_counts_per_level = torch.zeros(
            len(feature_shapes), dtype=torch.long, device=device
        )

        for sample in samples:
            boxes = sample.boxes.to(device=device)
            ignore_boxes = sample.ignore_boxes.to(device=device)
            valid_flat, valid_levels = point_validity_from_pixel_mask(
                sample.valid_mask.to(device=device),
                feature_shapes,
                self.assigner.strides,
            )
            assignment = self.assigner.assign(
                boxes,
                feature_shapes,
                (sample.image.shape[-2], sample.image.shape[-1]),
                valid_flat,
            )
            # A geometrically valid but grid-unrepresentable GT is ignored rather
            # than forced into negative ReLU-constrained LTRB targets.
            if assignment.unrepresentable_gt_indices.numel():
                ignore_boxes = torch.cat((
                    ignore_boxes,
                    boxes[assignment.unrepresentable_gt_indices],
                ))
            ignored_points = points_inside_boxes(points, ignore_boxes)
            positive = assignment.positive_mask
            background_weight = (
                1.0
                if sample.background_supervision
                else self.background_loss_weights.get(
                    sample.source_dataset, 0.0
                )
            )
            weights = (
                (valid_flat & ~ignored_points).to(dtype=torch.float32)
                * background_weight
            )
            # Assigned positives are never weakened, even for a sparse source.
            weights[positive] = 1.0
            supervised = weights > 0

            objectness.append(positive.to(dtype=torch.float32))
            objectness_masks.append(supervised)
            objectness_weights.append(weights)
            distances.append(assignment.box_targets)
            centerness.append(assignment.centerness_targets)
            positive_masks.append(positive)
            valid_levels_by_batch.append(valid_levels)
            fallback_count += assignment.fallback_gt_indices.numel()
            valid_gt_count += assignment.valid_gt_mask.sum()
            unrepresentable_count += (
                assignment.unrepresentable_gt_indices.numel()
            )
            positive_counts_per_level += assignment.positive_counts_per_level

        level_masks = tuple(
            torch.stack([
                valid_levels_by_batch[batch_index][level]
                for batch_index in range(len(samples))
            ])
            for level in range(len(feature_shapes))
        )
        return TrainingTargets(
            torch.stack(objectness),
            torch.stack(objectness_masks),
            torch.stack(objectness_weights),
            torch.stack(distances),
            torch.stack(centerness),
            torch.stack(positive_masks),
            level_masks,  # type: ignore[arg-type]
            valid_gt_count,
            fallback_count,
            unrepresentable_count,
            positive_counts_per_level,
        )
