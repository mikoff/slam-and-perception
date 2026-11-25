"""Convert per-image annotations into flattened P3-P5 training targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import torch
from torch import Tensor

from .assigner import ATSSAssigner
from .data import ProposalSample
from .geometry import make_grid_points
from .head import DetectorOutput
from .task_aligned_assigner import TaskAlignedAssigner


@dataclass(frozen=True)
class TrainingTargets:
    objectness: Tensor
    objectness_mask: Tensor
    objectness_weights: Tensor
    box_distances: Tensor
    centerness: Tensor
    positive_mask: Tensor
    trusted_background_mask: Tensor
    hard_negative_focus_mask: Tensor
    weak_background_mask: Tensor
    ignore_mask: Tensor
    valid_point_masks: tuple[Tensor, ...]
    valid_gt_count: Tensor
    fallback_count: Tensor
    unrepresentable_count: Tensor
    positive_counts_per_level: Tensor
    source_datasets: tuple[str, ...]
    domains: tuple[str, ...]
    matched_gt_indices: Tensor | None = None

    def to(
        self, device: torch.device, *, non_blocking: bool = False
    ) -> TrainingTargets:
        """Move the fixed-shape target batch to an accelerator.

        ATSS intentionally runs on the CPU for XLA training because assignment
        contains variable-length candidate selection and Python control flow.
        The resulting dense tensors are safe to transfer to either CUDA or XLA.
        """
        return TrainingTargets(
            objectness=self.objectness.to(device, non_blocking=non_blocking),
            objectness_mask=self.objectness_mask.to(device, non_blocking=non_blocking),
            objectness_weights=self.objectness_weights.to(
                device, non_blocking=non_blocking
            ),
            box_distances=self.box_distances.to(device, non_blocking=non_blocking),
            centerness=self.centerness.to(device, non_blocking=non_blocking),
            positive_mask=self.positive_mask.to(device, non_blocking=non_blocking),
            trusted_background_mask=self.trusted_background_mask.to(
                device, non_blocking=non_blocking
            ),
            hard_negative_focus_mask=self.hard_negative_focus_mask.to(
                device, non_blocking=non_blocking
            ),
            weak_background_mask=self.weak_background_mask.to(
                device, non_blocking=non_blocking
            ),
            ignore_mask=self.ignore_mask.to(device, non_blocking=non_blocking),
            valid_point_masks=tuple(
                mask.to(device, non_blocking=non_blocking)
                for mask in self.valid_point_masks
            ),  # type: ignore[arg-type]
            valid_gt_count=self.valid_gt_count.to(device, non_blocking=non_blocking),
            fallback_count=self.fallback_count.to(device, non_blocking=non_blocking),
            unrepresentable_count=self.unrepresentable_count.to(
                device, non_blocking=non_blocking
            ),
            positive_counts_per_level=self.positive_counts_per_level.to(
                device, non_blocking=non_blocking
            ),
            source_datasets=self.source_datasets,
            domains=self.domains,
            matched_gt_indices=(
                self.matched_gt_indices.to(device, non_blocking=non_blocking)
                if self.matched_gt_indices is not None
                else None
            ),
        )


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
    points: Tensor | None = None,
    level_slices: Sequence[slice] | None = None,
) -> tuple[Tensor, tuple[Tensor, ...]]:
    """Sample the input validity mask at feature cell centers."""
    if points is None or level_slices is None:
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
        for level_slice, feature_shape in zip(level_slices, feature_shapes, strict=True)
    )
    return flattened, levels


class TargetBuilder:
    """Batch target construction kept outside the exported detector graph."""

    def __init__(
        self,
        assigner: ATSSAssigner | TaskAlignedAssigner,
        *,
        background_loss_weights: Mapping[str, float] | None = None,
        default_background_weight: float = 0.0,
        hard_negative_focus_weight: float = 1.0,
        ignore_unmatched_gt_interior: bool = False,
        max_one_to_one_interior_area_frac: float = 0.90,
    ) -> None:
        self.assigner = assigner
        self.background_loss_weights = dict(background_loss_weights or {})
        if not (0.0 <= default_background_weight <= 1.0):
            raise ValueError("default background weight must be in [0, 1]")
        self.default_background_weight = default_background_weight
        if any(
            weight < 0 or weight > 1 for weight in self.background_loss_weights.values()
        ):
            raise ValueError("background loss weights must be in [0, 1]")
        if hard_negative_focus_weight < 1:
            raise ValueError("hard-negative focus weight must be at least 1")
        self.hard_negative_focus_weight = hard_negative_focus_weight
        self.ignore_unmatched_gt_interior = ignore_unmatched_gt_interior
        self.max_one_to_one_interior_area_frac = max_one_to_one_interior_area_frac
        self._grid_cache: dict[
            tuple[tuple[tuple[int, int], ...], torch.device],
            tuple[Tensor, Sequence[slice]],
        ] = {}

    def __call__(
        self,
        samples: Sequence[ProposalSample],
        feature_shapes: Sequence[tuple[int, int]],
        *,
        device: torch.device,
        pred_output: DetectorOutput | None = None,
    ) -> TrainingTargets:
        cache_key = (tuple(feature_shapes), device)
        if cache_key not in self._grid_cache:
            self._grid_cache[cache_key] = make_grid_points(
                feature_shapes,
                self.assigner.strides,
                device=device,
                dtype=torch.float32,
            )
        points, level_slices = self._grid_cache[cache_key]
        pred_scores_b: Tensor | None = None
        pred_distances_b: Tensor | None = None
        if pred_output is not None:
            from .losses import flatten_detector_output

            obj_flat, dist_flat, _ = flatten_detector_output(pred_output)
            pred_scores_b = obj_flat.detach().float()
            pred_distances_b = dist_flat.detach().float()

        objectness = []
        objectness_masks = []
        objectness_weights = []
        distances = []
        centerness = []
        positive_masks = []
        matched_gt_indices = []
        trusted_background_masks = []
        hard_negative_focus_masks = []
        weak_background_masks = []
        ignore_masks = []
        valid_levels_by_batch: list[tuple[Tensor, ...]] = []
        fallback_count = torch.zeros((), device=device)
        valid_gt_count = torch.zeros((), device=device)
        unrepresentable_count = torch.zeros((), device=device)
        positive_counts_per_level = torch.zeros(
            len(feature_shapes), dtype=torch.long, device=device
        )

        for i, sample in enumerate(samples):
            boxes = sample.boxes.to(device=device)
            ignore_boxes = sample.ignore_boxes.to(device=device)
            valid_flat, valid_levels = point_validity_from_pixel_mask(
                sample.valid_mask.to(device=device),
                feature_shapes,
                self.assigner.strides,
                points=points,
                level_slices=level_slices,
            )
            scores_i = pred_scores_b[i] if pred_scores_b is not None else None
            dists_i = pred_distances_b[i] if pred_distances_b is not None else None
            if isinstance(self.assigner, TaskAlignedAssigner):
                assignment = self.assigner.assign(
                    boxes,
                    feature_shapes,
                    (sample.image.shape[-2], sample.image.shape[-1]),
                    valid_flat,
                    pred_scores=scores_i,
                    pred_distances=dists_i,
                )
            else:
                assignment = self.assigner.assign(
                    boxes,
                    feature_shapes,
                    (sample.image.shape[-2], sample.image.shape[-1]),
                    valid_flat,
                )
            # A geometrically valid but grid-unrepresentable GT is ignored rather
            # than forced into negative ReLU-constrained LTRB targets.
            if assignment.unrepresentable_gt_indices.numel():
                ignore_boxes = torch.cat(
                    (
                        ignore_boxes,
                        boxes[assignment.unrepresentable_gt_indices],
                    )
                )
            ignored_points = points_inside_boxes(points, ignore_boxes)
            positive = assignment.positive_mask
            is_one_to_one = (
                getattr(self.assigner, "assignment_mode", None) == "one_to_one"
            )
            non_giant_boxes = boxes.new_empty((0, 4))
            if boxes.numel() > 0:
                box_areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
                img_area = float(sample.image.shape[-2] * sample.image.shape[-1])
                non_giant_boxes = boxes[box_areas < 0.90 * img_area]
            if self.ignore_unmatched_gt_interior and non_giant_boxes.numel() > 0:
                ignored_points = ignored_points | (
                    points_inside_boxes(points, non_giant_boxes) & ~positive
                )
            trusted_boxes = sample.trusted_background_boxes
            trusted_points = points_inside_boxes(
                points,
                trusted_boxes.to(device=device)
                if trusted_boxes is not None
                else boxes.new_empty((0, 4)),
            )
            if (
                is_one_to_one
                and not self.ignore_unmatched_gt_interior
                and non_giant_boxes.numel() > 0
            ):
                # In 1-to-1 assignment, unmatched interior anchors of medium/large GTs
                # receive full negative suppression to eliminate duplicate clusters,
                # while small/slender GTs (min_side < 12 or area < 256) remain weak
                # negatives so 3x3 conv smoothing does not crush their objectness.
                ng_w = non_giant_boxes[:, 2] - non_giant_boxes[:, 0]
                ng_h = non_giant_boxes[:, 3] - non_giant_boxes[:, 1]
                ng_area = ng_w * ng_h
                large_enough = (
                    (torch.minimum(ng_w, ng_h) >= 12.0)
                    & (ng_area >= 256.0)
                    & (ng_area < self.max_one_to_one_interior_area_frac * img_area)
                )
                if large_enough.any():
                    trusted_points = trusted_points | points_inside_boxes(
                        points, non_giant_boxes[large_enough]
                    )
            available = valid_flat & ~ignored_points & ~positive
            trusted_background = available & trusted_points
            focus_boxes = sample.hard_negative_focus_boxes
            focus_points = points_inside_boxes(
                points,
                focus_boxes.to(device=device)
                if focus_boxes is not None
                else boxes.new_empty((0, 4)),
            )
            hard_negative_focus = trusted_background & focus_points
            weak_background = available & ~trusted_points
            ignored = valid_flat & ignored_points & ~positive
            weak_weight = self.background_loss_weights.get(
                sample.source_dataset, self.default_background_weight
            )
            weights = torch.zeros_like(valid_flat, dtype=torch.float32)
            weights[trusted_background] = 1.0
            weights[hard_negative_focus] = self.hard_negative_focus_weight
            weights[weak_background] = weak_weight
            # Assigned positives are never weakened, even for a sparse source.
            weights[positive] = 1.0
            supervised = weights > 0

            objectness.append(positive.to(dtype=torch.float32))
            objectness_masks.append(supervised)
            objectness_weights.append(weights)
            distances.append(assignment.box_targets)
            centerness.append(assignment.centerness_targets)
            positive_masks.append(positive)
            matched_gt_indices.append(assignment.matched_gt_indices)
            trusted_background_masks.append(trusted_background)
            hard_negative_focus_masks.append(hard_negative_focus)
            weak_background_masks.append(weak_background)
            ignore_masks.append(ignored)
            valid_levels_by_batch.append(valid_levels)
            fallback_count += assignment.fallback_gt_indices.numel()
            valid_gt_count += assignment.valid_gt_mask.sum()
            unrepresentable_count += assignment.unrepresentable_gt_indices.numel()
            positive_counts_per_level += assignment.positive_counts_per_level

        level_masks = tuple(
            torch.stack(
                [
                    valid_levels_by_batch[batch_index][level]
                    for batch_index in range(len(samples))
                ]
            )
            for level in range(len(feature_shapes))
        )
        return TrainingTargets(
            torch.stack(objectness),
            torch.stack(objectness_masks),
            torch.stack(objectness_weights),
            torch.stack(distances),
            torch.stack(centerness),
            torch.stack(positive_masks),
            torch.stack(trusted_background_masks),
            torch.stack(hard_negative_focus_masks),
            torch.stack(weak_background_masks),
            torch.stack(ignore_masks),
            level_masks,  # type: ignore[arg-type]
            valid_gt_count,
            fallback_count,
            unrepresentable_count,
            positive_counts_per_level,
            tuple(sample.source_dataset for sample in samples),
            tuple(sample.domain for sample in samples),
            torch.stack(matched_gt_indices),
        )
