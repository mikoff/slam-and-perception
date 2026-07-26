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
    fallback_gt_indices: Tensor
    positive_counts_per_level: Tensor


class ATSSAssigner:
    """ATSS with square virtual priors and FCOS-style distance targets."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        prior_sizes: tuple[int, int, int] = (64, 128, 256),
        top_k: int = 9,
        center_radius: float | None = 1.5,
    ) -> None:
        if len(strides) != len(prior_sizes):
            raise ValueError("strides and prior_sizes must have equal length")
        if top_k < 1:
            raise ValueError("top_k must be positive")
        self.strides = strides
        self.prior_sizes = prior_sizes
        self.top_k = top_k
        self.center_radius = center_radius

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
        valid_point_mask: Tensor | None = None,
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
        if valid_point_mask is None:
            valid_point_mask = torch.ones(
                num_points, dtype=torch.bool, device=gt_boxes.device
            )
        else:
            valid_point_mask = valid_point_mask.to(
                device=gt_boxes.device, dtype=torch.bool
            ).flatten()
            if valid_point_mask.shape != (num_points,):
                raise ValueError(
                    f"valid_point_mask must contain {num_points} locations"
                )

        point_strides = torch.empty(
            num_points, dtype=gt_boxes.dtype, device=gt_boxes.device
        )
        for level_slice, stride in zip(level_slices, self.strides, strict=True):
            point_strides[level_slice] = stride

        matched = torch.full(
            (num_points,), -1, dtype=torch.long, device=gt_boxes.device
        )
        box_targets = torch.zeros(
            (num_points, 4), dtype=gt_boxes.dtype, device=gt_boxes.device
        )
        centerness = torch.zeros(
            (num_points,), dtype=gt_boxes.dtype, device=gt_boxes.device
        )
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
            empty = torch.empty((0,), dtype=torch.long, device=gt_boxes.device)
            return Assignment(
                matched,
                box_targets,
                centerness,
                matched >= 0,
                valid_gt_mask,
                empty,
                empty,
                torch.zeros(
                    len(level_slices), dtype=torch.long, device=gt_boxes.device
                ),
            )

        boxes = gt_boxes[valid_indices]
        priors = self._priors(points, level_slices)
        prior_ious = box_iou(priors, boxes)
        candidate_quality = torch.full_like(prior_ious, -1.0)
        candidate_masks = torch.zeros_like(prior_ious, dtype=torch.bool)
        gt_centers = (boxes[:, :2] + boxes[:, 2:]) * 0.5
        squared_distances = (
            (points[:, None, :] - gt_centers[None, :, :]) ** 2
        ).sum(dim=2)

        for gt_index in range(boxes.shape[0]):
            candidate_parts: list[Tensor] = []
            for level_slice in level_slices:
                level_valid = valid_point_mask[level_slice]
                count = min(self.top_k, int(level_valid.sum().item()))
                if count == 0:
                    continue
                level_distances = squared_distances[
                    level_slice, gt_index
                ].masked_fill(~level_valid, torch.inf)
                relative = torch.topk(
                    level_distances, k=count, largest=False
                ).indices
                candidate_parts.append(
                    relative + int(level_slice.start or 0)
                )
            if not candidate_parts:
                continue
            candidates = torch.cat(candidate_parts)
            candidate_masks[candidates, gt_index] = True
            candidate_ious = prior_ious[candidates, gt_index]
            threshold = candidate_ious.mean() + candidate_ious.std(
                unbiased=False
            )
            candidate_points = points[candidates]
            distances = encode_ltrb(
                candidate_points,
                boxes[gt_index].unsqueeze(0).expand(candidates.shape[0], -1),
            )
            inside = distances.min(dim=1).values >= 0
            in_center = torch.ones_like(inside)
            if self.center_radius is not None:
                radius = self.center_radius * point_strides[candidates]
                center_left = torch.maximum(
                    boxes[gt_index, 0].expand_as(radius),
                    gt_centers[gt_index, 0] - radius,
                )
                center_top = torch.maximum(
                    boxes[gt_index, 1].expand_as(radius),
                    gt_centers[gt_index, 1] - radius,
                )
                center_right = torch.minimum(
                    boxes[gt_index, 2].expand_as(radius),
                    gt_centers[gt_index, 0] + radius,
                )
                center_bottom = torch.minimum(
                    boxes[gt_index, 3].expand_as(radius),
                    gt_centers[gt_index, 1] + radius,
                )
                in_center = (
                    (candidate_points[:, 0] >= center_left)
                    & (candidate_points[:, 0] <= center_right)
                    & (candidate_points[:, 1] >= center_top)
                    & (candidate_points[:, 1] <= center_bottom)
                )
            positives = candidates[
                (candidate_ious >= threshold) & inside & in_center
            ]
            candidate_quality[positives, gt_index] = prior_ious[
                positives, gt_index
            ]

        best_quality, best_local_gt = candidate_quality.max(dim=1)
        normally_positive = best_quality >= 0
        matched[normally_positive] = valid_indices[
            best_local_gt[normally_positive]
        ]

        unrepresentable: list[Tensor] = []
        fallbacks: list[Tensor] = []
        for local_gt, original_gt in enumerate(valid_indices):
            if torch.any(matched == original_gt):
                continue
            all_distances = encode_ltrb(
                points, boxes[local_gt].unsqueeze(0).expand(num_points, -1)
            )
            free_inside = (
                (all_distances.min(dim=1).values >= 0)
                & valid_point_mask
                & (matched < 0)
            )
            if not torch.any(free_inside):
                unrepresentable.append(original_gt)
                continue
            preferred = free_inside & candidate_masks[:, local_gt]
            pool = preferred if torch.any(preferred) else free_inside
            quality = prior_ious[:, local_gt].masked_fill(~pool, -1.0)
            best_iou = quality.max()
            tied = pool & torch.isclose(quality, best_iou)
            fallback_point = squared_distances[:, local_gt].masked_fill(
                ~tied, torch.inf
            ).argmin()
            matched[fallback_point] = original_gt
            fallbacks.append(original_gt)

        positive = matched >= 0
        if torch.any(positive):
            targets = encode_ltrb(
                points[positive], gt_boxes[matched[positive]]
            )
            box_targets[positive] = targets
            left, top, right, bottom = targets.unbind(dim=1)
            horizontal = torch.minimum(left, right) / torch.maximum(
                left, right
            ).clamp(min=1e-7)
            vertical = torch.minimum(top, bottom) / torch.maximum(
                top, bottom
            ).clamp(min=1e-7)
            centerness[positive] = torch.sqrt(
                (horizontal * vertical).clamp(min=0)
            )

        unrepresentable_indices = (
            torch.stack(unrepresentable)
            if unrepresentable
            else torch.empty(
                (0,), dtype=torch.long, device=gt_boxes.device
            )
        )
        fallback_indices = (
            torch.stack(fallbacks)
            if fallbacks
            else torch.empty(
                (0,), dtype=torch.long, device=gt_boxes.device
            )
        )
        positive_counts = torch.stack(
            [positive[level_slice].sum() for level_slice in level_slices]
        )
        return Assignment(
            matched,
            box_targets,
            centerness,
            positive,
            valid_gt_mask,
            unrepresentable_indices,
            fallback_indices,
            positive_counts,
        )
