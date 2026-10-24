"""Adaptive Training Sample Selection (ATSS) for one prior per location."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import torch
from torch import Tensor

from .geometry import encode_ltrb, make_grid_points


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
        strides: tuple[int, ...] = (8, 16, 32),
        prior_sizes: tuple[int, ...] = (64, 128, 256),
        top_k: int = 4,
        center_radius: float | None = 1.0,
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
        priors = torch.empty(
            (points.shape[0], 4), device=points.device, dtype=points.dtype
        )
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
        box_wh = (boxes[:, 2:] - boxes[:, :2]).clamp(min=1.0)
        scale_x = (box_wh[:, 0] / box_wh[:, 1]).clamp(min=0.15, max=1.0)
        scale_y = (box_wh[:, 1] / box_wh[:, 0]).clamp(min=0.15, max=1.0)
        base_sides = torch.empty(
            num_points, dtype=gt_boxes.dtype, device=gt_boxes.device
        )
        for level_slice, side in zip(level_slices, self.prior_sizes, strict=True):
            base_sides[level_slice] = side
        half_w = 0.5 * base_sides[:, None] * scale_x[None, :]
        half_h = 0.5 * base_sides[:, None] * scale_y[None, :]
        ix1 = torch.maximum(points[:, 0:1] - half_w, boxes[None, :, 0])
        iy1 = torch.maximum(points[:, 1:2] - half_h, boxes[None, :, 1])
        ix2 = torch.minimum(points[:, 0:1] + half_w, boxes[None, :, 2])
        iy2 = torch.minimum(points[:, 1:2] + half_h, boxes[None, :, 3])
        inter = (ix2 - ix1).clamp(min=0) * (iy2 - iy1).clamp(min=0)
        p_area = (2.0 * half_w) * (2.0 * half_h)
        b_area = box_wh[:, 0] * box_wh[:, 1]
        prior_ious = inter / (p_area + b_area[None, :] - inter).clamp(min=1e-7)

        candidate_quality = torch.full_like(prior_ious, -1.0)
        candidate_masks = torch.zeros_like(prior_ious, dtype=torch.bool)
        gt_centers = (boxes[:, :2] + boxes[:, 2:]) * 0.5
        aspect_scale = (
            torch.sqrt(box_wh.prod(dim=1, keepdim=True)) / box_wh
        ).clamp(min=0.25, max=4.0)
        squared_distances = (
            (
                (points[:, None, :] - gt_centers[None, :, :])
                * aspect_scale[None, :, :]
            )
            ** 2
        ).sum(dim=2)
        inside_all = (
            (points[:, None, 0] >= boxes[None, :, 0])
            & (points[:, None, 0] <= boxes[None, :, 2])
            & (points[:, None, 1] >= boxes[None, :, 1])
            & (points[:, None, 1] <= boxes[None, :, 3])
        )

        # Vectorized candidate selection across all ground-truth boxes simultaneously
        candidate_parts: list[Tensor] = []
        for level_slice in level_slices:
            level_valid = valid_point_mask[level_slice]
            level_inside = inside_all[level_slice] & level_valid[:, None]
            has_inside = level_inside.any(dim=0, keepdim=True)
            eligible_mask = torch.where(
                has_inside, level_inside, level_valid[:, None]
            )
            level_dist = squared_distances[level_slice].masked_fill(
                ~eligible_mask, torch.inf
            )
            count = min(self.top_k, level_slice.stop - level_slice.start)
            if count == 0:
                continue
            relative = torch.topk(level_dist, k=count, dim=0, largest=False).indices
            candidate_parts.append(relative + int(level_slice.start or 0))

        if candidate_parts:
            all_candidates = torch.cat(candidate_parts, dim=0)  # [total_k, num_gt]
            candidate_masks.scatter_(0, all_candidates, True)

            cand_points = points[all_candidates]
            inside = torch.gather(inside_all, 0, all_candidates)
            cand_ious = torch.gather(prior_ious, 0, all_candidates).masked_fill(
                ~inside, 0.0
            )
            threshold = cand_ious.mean(
                dim=0, keepdim=True
            ) + 0.5 * cand_ious.std(dim=0, unbiased=False, keepdim=True)
            if self.center_radius is not None:
                cand_strides = point_strides[all_candidates]
                radius = self.center_radius * cand_strides
                aspect_x = (box_wh[:, 0] / box_wh[:, 1]).clamp(min=1.0, max=4.0)
                aspect_y = (box_wh[:, 1] / box_wh[:, 0]).clamp(min=1.0, max=4.0)
                rx = radius * aspect_x[None, :]
                ry = radius * aspect_y[None, :]
                center_left = torch.maximum(
                    boxes[None, :, 0], gt_centers[None, :, 0] - rx
                )
                center_top = torch.maximum(
                    boxes[None, :, 1], gt_centers[None, :, 1] - ry
                )
                center_right = torch.minimum(
                    boxes[None, :, 2], gt_centers[None, :, 0] + rx
                )
                center_bottom = torch.minimum(
                    boxes[None, :, 3], gt_centers[None, :, 1] + ry
                )
                in_center = (
                    (cand_points[..., 0] >= center_left)
                    & (cand_points[..., 0] <= center_right)
                    & (cand_points[..., 1] >= center_top)
                    & (cand_points[..., 1] <= center_bottom)
                )
            else:
                in_center = torch.ones_like(inside)

            positives = (cand_ious >= threshold) & inside & in_center
            cand_qual_vals = torch.where(positives, cand_ious, -1.0)
            candidate_quality.scatter_(0, all_candidates, cand_qual_vals)

        best_quality, best_local_gt = candidate_quality.max(dim=1)
        normally_positive = best_quality >= 0
        matched[normally_positive] = valid_indices[best_local_gt[normally_positive]]

        unrepresentable: list[Tensor] = []
        fallbacks: list[Tensor] = []
        matched_gts = matched[normally_positive].unique()
        unmatched_local = torch.where(~torch.isin(valid_indices, matched_gts))[0]

        for local_gt in unmatched_local:
            original_gt = valid_indices[local_gt]
            all_distances = encode_ltrb(
                points, boxes[local_gt].unsqueeze(0).expand(num_points, -1)
            )
            free_inside = (
                (all_distances.min(dim=1).values >= 0)
                & valid_point_mask
                & (matched < 0)
            )
            if not free_inside.any():
                unrepresentable.append(original_gt)
                continue
            preferred = free_inside & candidate_masks[:, local_gt]
            pool = preferred if preferred.any() else free_inside
            quality = prior_ious[:, local_gt].masked_fill(~pool, -1.0)
            best_iou = quality.max()
            tied = pool & torch.isclose(quality, best_iou)
            fallback_point = (
                squared_distances[:, local_gt].masked_fill(~tied, torch.inf).argmin()
            )
            matched[fallback_point] = original_gt
            fallbacks.append(original_gt)

        positive = matched >= 0
        if torch.any(positive):
            targets = encode_ltrb(points[positive], gt_boxes[matched[positive]])
            box_targets[positive] = targets
            left, top, right, bottom = targets.unbind(dim=1)
            pos_strides = 0.5 * point_strides[positive]
            horizontal = (torch.minimum(left, right) + pos_strides) / (
                torch.maximum(left, right) + pos_strides
            ).clamp(min=1e-7)
            vertical = (torch.minimum(top, bottom) + pos_strides) / (
                torch.maximum(top, bottom) + pos_strides
            ).clamp(min=1e-7)
            centerness[positive] = torch.sqrt(
                (horizontal * vertical).clamp(min=0, max=1.0)
            )

        unrepresentable_indices = (
            torch.stack(unrepresentable)
            if unrepresentable
            else torch.empty((0,), dtype=torch.long, device=gt_boxes.device)
        )
        fallback_indices = (
            torch.stack(fallbacks)
            if fallbacks
            else torch.empty((0,), dtype=torch.long, device=gt_boxes.device)
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
