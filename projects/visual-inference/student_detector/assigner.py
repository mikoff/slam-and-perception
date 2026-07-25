"""Adaptive Training Sample Selection (ATSS) for one prior per location."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import torch
from torch import Tensor

from .geometry import box_iou, encode_ltrb, make_grid_points


@dataclass(frozen=True)
class Assignment:
    """Flattened P3-P5 targets for one image."""

    matched_gt_indices: Tensor
    box_targets: Tensor
    centerness_targets: Tensor
    positive_mask: Tensor
    valid_gt_mask: Tensor
    unrepresentable_gt_indices: Tensor


class ATSSAssigner:
    """ATSS with square virtual priors and FCOS-style distance targets."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        prior_sizes: tuple[int, int, int] = (32, 64, 128),
        top_k: int = 9,
    ) -> None:
        if len(strides) != len(prior_sizes):
            raise ValueError("strides and prior_sizes must have equal length")
        if top_k < 1:
            raise ValueError("top_k must be positive")
        self.strides = strides
        self.prior_sizes = prior_sizes
        self.top_k = top_k

    def _priors(self, points: Tensor, level_slices: Sequence[slice]) -> Tensor:
        priors = torch.empty((points.shape[0], 4), device=points.device, dtype=points.dtype)
        for level_slice, side in zip(level_slices, self.prior_sizes, strict=True):
            half_side = side / 2.0
            priors[level_slice, :2] = points[level_slice] - half_side
            priors[level_slice, 2:] = points[level_slice] + half_side
        return priors

    def assign(
        self,
        gt_boxes: Tensor,
        feature_shapes: Sequence[tuple[int, int]],
        image_size: tuple[int, int],
    ) -> Assignment:
        """Assign one image's xyxy ground truths to flattened feature locations.

        A ground truth is geometrically representable only when at least one grid
        point lies inside it. Boxes without such a point are returned in
        ``unrepresentable_gt_indices`` instead of receiving impossible negative
        distance targets.
        """
        if gt_boxes.ndim != 2 or gt_boxes.shape[1] != 4:
            raise ValueError("gt_boxes must have shape [num_boxes, 4]")
        height, width = image_size
        points, level_slices = make_grid_points(
            feature_shapes,
            self.strides,
            device=gt_boxes.device,
            dtype=gt_boxes.dtype,
        )
        num_points = points.shape[0]
        matched = torch.full((num_points,), -1, dtype=torch.long, device=gt_boxes.device)
        box_targets = torch.zeros((num_points, 4), dtype=gt_boxes.dtype, device=gt_boxes.device)
        centerness = torch.zeros((num_points,), dtype=gt_boxes.dtype, device=gt_boxes.device)

        valid_gt_mask = (
            (gt_boxes[:, 2] > gt_boxes[:, 0])
            & (gt_boxes[:, 3] > gt_boxes[:, 1])
            & (gt_boxes[:, 0] < width)
            & (gt_boxes[:, 1] < height)
            & (gt_boxes[:, 2] > 0)
            & (gt_boxes[:, 3] > 0)
        )
        valid_indices = torch.where(valid_gt_mask)[0]
        if valid_indices.numel() == 0:
            positive = matched >= 0
            return Assignment(
                matched, box_targets, centerness, positive, valid_gt_mask, valid_indices
            )

        boxes = gt_boxes[valid_indices]
        priors = self._priors(points, level_slices)
        prior_ious = box_iou(priors, boxes)
        candidate_quality = torch.full_like(prior_ious, -1.0)

        gt_centers = (boxes[:, :2] + boxes[:, 2:]) * 0.5
        squared_distances = ((points[:, None, :] - gt_centers[None, :, :]) ** 2).sum(dim=2)
        for gt_index in range(boxes.shape[0]):
            candidate_parts: list[Tensor] = []
            for level_slice in level_slices:
                level_distances = squared_distances[level_slice, gt_index]
                count = min(self.top_k, level_distances.numel())
                relative = torch.topk(level_distances, k=count, largest=False).indices
                candidate_parts.append(relative + int(level_slice.start or 0))
            candidates = torch.cat(candidate_parts)
            candidate_ious = prior_ious[candidates, gt_index]
            threshold = candidate_ious.mean() + candidate_ious.std(unbiased=False)

            candidate_points = points[candidates]
            distances = encode_ltrb(
                candidate_points,
                boxes[gt_index].unsqueeze(0).expand(candidates.shape[0], -1),
            )
            inside = distances.min(dim=1).values >= 0
            positive_candidates = candidates[(candidate_ious >= threshold) & inside]
            candidate_quality[positive_candidates, gt_index] = prior_ious[
                positive_candidates, gt_index
            ]

        best_quality, best_local_gt = candidate_quality.max(dim=1)
        normally_positive = best_quality >= 0
        matched[normally_positive] = valid_indices[best_local_gt[normally_positive]]

        # A threshold can reject every candidate even when an inside point exists.
        # Fill those holes with the nearest currently-free inside point. We do not
        # assign outside points: their signed l/t/r/b targets cannot be emitted by
        # the ReLU-constrained regression head.
        unrepresentable: list[Tensor] = []
        for local_gt, original_gt in enumerate(valid_indices):
            if torch.any(matched == original_gt):
                continue
            all_distances = encode_ltrb(
                points, boxes[local_gt].unsqueeze(0).expand(num_points, -1)
            )
            inside = all_distances.min(dim=1).values >= 0
            free_inside = inside & (matched < 0)
            if not torch.any(free_inside):
                unrepresentable.append(original_gt)
                continue
            distances = squared_distances[:, local_gt].clone()
            distances[~free_inside] = torch.inf
            fallback_point = distances.argmin()
            matched[fallback_point] = original_gt

        positive = matched >= 0
        if torch.any(positive):
            positive_points = points[positive]
            positive_boxes = gt_boxes[matched[positive]]
            targets = encode_ltrb(positive_points, positive_boxes)
            box_targets[positive] = targets
            left, top, right, bottom = targets.unbind(dim=1)
            horizontal = torch.minimum(left, right) / torch.maximum(left, right).clamp(min=1e-7)
            vertical = torch.minimum(top, bottom) / torch.maximum(top, bottom).clamp(min=1e-7)
            centerness[positive] = torch.sqrt((horizontal * vertical).clamp(min=0))

        if unrepresentable:
            unrepresentable_indices = torch.stack(unrepresentable)
        else:
            unrepresentable_indices = torch.empty(
                (0,), dtype=torch.long, device=gt_boxes.device
            )
        return Assignment(
            matched,
            box_targets,
            centerness,
            positive,
            valid_gt_mask,
            unrepresentable_indices,
        )
