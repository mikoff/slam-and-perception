"""Task-Aligned Assigner (TAL) and Consistent Dual 1-to-1 / 1-to-many Assigner."""

from __future__ import annotations

from collections.abc import Sequence

import torch
from torch import Tensor

from .assigner import Assignment
from .geometry import (
    decode_ltrb,
    encode_ltrb,
    make_grid_points,
)
from .losses import aligned_iou
from .quad_assigner import QuadAssignment
from .quad_geometry import (
    aligned_quad_iou,
    canonicalize_quads,
    decode_quad_offsets,
    points_inside_convex_quads,
    quad_validity,
)


def greedy_bipartite_matching(alignment_matrix: Tensor) -> list[tuple[int, int]]:
    """Match candidates to ground truths 1-to-1 maximizing alignment score.

    Args:
        alignment_matrix: Tensor of shape [N_candidates, M_gt] with non-negative scores.

    Returns:
        List of (candidate_idx, gt_idx) tuples.
    """
    num_candidates, num_gt = alignment_matrix.shape
    if num_candidates == 0 or num_gt == 0:
        return []

    pos_idx = torch.nonzero(alignment_matrix > 0)
    if pos_idx.numel() == 0:
        return []

    scores = alignment_matrix[pos_idx[:, 0], pos_idx[:, 1]]
    order = torch.argsort(scores, descending=True)
    c_list = pos_idx[:, 0][order].tolist()
    g_list = pos_idx[:, 1][order].tolist()

    cand_matched: set[int] = set()
    gt_matched: set[int] = set()
    matches: list[tuple[int, int]] = []
    max_matches = min(num_candidates, num_gt)

    for c, g in zip(c_list, g_list, strict=True):
        if c not in cand_matched and g not in gt_matched:
            cand_matched.add(c)
            gt_matched.add(g)
            matches.append((c, g))
            if len(matches) == max_matches:
                break
    return matches


class TaskAlignedAssigner:
    """Task-Aligned Assigner (TAL / TOOD) for horizontal bounding boxes.

    Computes alignment metric t = s^alpha * IoU^beta dynamically between model
    predictions and ground-truth boxes. Supports both 1-to-many auxiliary and
    1-to-1 primary assignment modes.
    """

    def __init__(
        self,
        *,
        strides: tuple[int, ...] = (8, 16, 32),
        top_k: int = 4,
        alpha: float = 1.0,
        beta: float = 6.0,
        center_radius: float = 1.5,
        assignment_mode: str = "one_to_many",
    ) -> None:
        if assignment_mode not in {"one_to_many", "one_to_one"}:
            raise ValueError("assignment_mode must be 'one_to_many' or 'one_to_one'")
        self.strides = strides
        self.top_k = top_k
        self.alpha = alpha
        self.beta = beta
        self.center_radius = center_radius
        self.assignment_mode = assignment_mode

    def assign(
        self,
        gt_boxes: Tensor,
        feature_shapes: Sequence[tuple[int, int]],
        image_size: tuple[int, int],
        valid_point_mask: Tensor | None = None,
        *,
        pred_scores: Tensor | None = None,
        pred_distances: Tensor | None = None,
    ) -> Assignment:
        """Assign ground truths to feature locations using task alignment."""
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
        num_valid_gt = boxes.shape[0]
        gt_centers = (boxes[:, :2] + boxes[:, 2:]) * 0.5

        # Spatial prior: anchor must be inside GT box and within center_radius * stride
        in_gt = (
            (points[:, None, 0] >= boxes[None, :, 0])
            & (points[:, None, 0] <= boxes[None, :, 2])
            & (points[:, None, 1] >= boxes[None, :, 1])
            & (points[:, None, 1] <= boxes[None, :, 3])
        )
        radius = self.center_radius * point_strides[:, None]
        in_center = (
            (points[:, None, 0] >= gt_centers[None, :, 0] - radius)
            & (points[:, None, 0] <= gt_centers[None, :, 0] + radius)
            & (points[:, None, 1] >= gt_centers[None, :, 1] - radius)
            & (points[:, None, 1] <= gt_centers[None, :, 1] + radius)
        )
        candidate_mask = in_gt & in_center & valid_point_mask[:, None]

        # Decode predicted boxes if available; else decode virtual prior boxes
        if pred_distances is not None:
            pred_b = decode_ltrb(points, pred_distances)
        else:
            # Fallback prior: anchor-centered box with side = stride * 4
            half = point_strides[:, None] * 2.0
            pred_b = torch.cat(
                [points - half, points + half], dim=-1
            )

        scores = (
            torch.sigmoid(pred_scores.float())
            if pred_scores is not None
            else torch.ones(num_points, device=gt_boxes.device)
        )

        from .geometry import box_iou

        pair_ious = box_iou(pred_b, boxes).clamp(min=1e-6)
        pair_scores = scores.unsqueeze(1).clamp(min=1e-6)
        raw_t = (pair_scores**self.alpha) * (pair_ious**self.beta)
        alignment_matrix = raw_t * candidate_mask.float()

        # Handle any GT with zero candidates by assigning closest valid anchor
        zero_cands = candidate_mask.sum(dim=0) == 0
        if zero_cands.any():
            zero_gt_indices = torch.where(zero_cands)[0]
            dists = (
                (points[:, None, :] - gt_centers[zero_gt_indices][None, :, :]) ** 2
            ).sum(dim=2)
            dists[~valid_point_mask] = torch.inf
            closest_cands = dists.argmin(dim=0)
            alignment_matrix[closest_cands, zero_gt_indices] = raw_t[
                closest_cands, zero_gt_indices
            ]

        if self.assignment_mode == "one_to_one":
            matches = greedy_bipartite_matching(alignment_matrix)
            for cand_i, gt_j in matches:
                matched[cand_i] = valid_indices[gt_j]
                cand_box = pred_b[cand_i : cand_i + 1]
                target_box = boxes[gt_j : gt_j + 1]
                centerness[cand_i] = aligned_iou(cand_box, target_box).squeeze(0)
        else:
            k = min(self.top_k, num_points)
            topk_vals, topk_indices = torch.topk(alignment_matrix, k, dim=0)
            selected_per_gt = torch.zeros_like(alignment_matrix, dtype=torch.bool)
            keep_topk = topk_vals > 0
            selected_per_gt.scatter_(0, topk_indices, keep_topk)

            best_metric, best_gt = (
                alignment_matrix.masked_fill(~selected_per_gt, -1.0).max(dim=1)
            )
            is_pos = best_metric > 0
            matched[is_pos] = valid_indices[best_gt[is_pos]]

            if is_pos.any():
                pos_pts = torch.where(is_pos)[0]
                pos_gt_local = best_gt[pos_pts]
                matched_gt_boxes = boxes[pos_gt_local]
                pos_ious = aligned_iou(pred_b[pos_pts], matched_gt_boxes)

                max_metric = torch.zeros(num_valid_gt, device=gt_boxes.device)
                max_metric.scatter_reduce_(
                    0,
                    pos_gt_local,
                    alignment_matrix[pos_pts, pos_gt_local],
                    reduce="amax",
                    include_self=False,
                )
                max_iou = torch.zeros(num_valid_gt, device=gt_boxes.device)
                max_iou.scatter_reduce_(
                    0, pos_gt_local, pos_ious, reduce="amax", include_self=False
                )
                norm_t = (
                    alignment_matrix[pos_pts, pos_gt_local]
                    / (max_metric[pos_gt_local] + 1e-6)
                ) * max_iou[pos_gt_local]
                centerness[pos_pts] = norm_t

        pos_mask = matched >= 0
        if pos_mask.any():
            pos_indices = torch.where(pos_mask)[0]
            matched_gts = matched[pos_indices]
            box_targets[pos_indices] = encode_ltrb(
                points[pos_indices], gt_boxes[matched_gts]
            )

        pos_per_level = torch.stack(
            [pos_mask[s].sum() for s in level_slices]
        ).to(dtype=torch.long)

        empty_indices = torch.empty((0,), dtype=torch.long, device=gt_boxes.device)
        return Assignment(
            matched_gt_indices=matched,
            box_targets=box_targets,
            centerness_targets=centerness,
            positive_mask=pos_mask,
            valid_gt_mask=valid_gt_mask,
            unrepresentable_gt_indices=empty_indices,
            fallback_gt_indices=empty_indices,
            positive_counts_per_level=pos_per_level,
        )


class QuadTaskAlignedAssigner:
    """Task-Aligned Assigner for oriented quadrilateral proposals."""

    def __init__(
        self,
        *,
        strides: tuple[int, ...] = (8, 16, 32),
        top_k: int = 4,
        alpha: float = 1.0,
        beta: float = 6.0,
        assignment_mode: str = "one_to_many",
    ) -> None:
        if assignment_mode not in {"one_to_many", "one_to_one"}:
            raise ValueError("assignment_mode must be 'one_to_many' or 'one_to_one'")
        self.strides = strides
        self.top_k = top_k
        self.alpha = alpha
        self.beta = beta
        self.assignment_mode = assignment_mode

    def assign(
        self,
        gt_quads: Tensor,
        feature_shapes: Sequence[tuple[int, int]],
        image_size: tuple[int, int],
        valid_point_mask: Tensor | None = None,
        *,
        pred_quality: Tensor | None = None,
        pred_offsets: Tensor | None = None,
    ) -> QuadAssignment:
        """Assign quad ground truths using task alignment."""
        value = torch.as_tensor(gt_quads)
        if value.ndim != 3 or value.shape[1:] != (4, 2):
            raise ValueError("gt_quads must have shape [num_quads, 4, 2]")
        points, level_slices = make_grid_points(
            feature_shapes,
            self.strides,
            device=value.device,
            dtype=value.dtype,
        )
        num_points = points.shape[0]

        if valid_point_mask is None:
            valid_point_mask = torch.ones(
                num_points, dtype=torch.bool, device=value.device
            )
        else:
            valid_point_mask = valid_point_mask.to(
                device=value.device, dtype=torch.bool
            ).flatten()

        point_strides = torch.empty(
            num_points, dtype=value.dtype, device=value.device
        )
        for level_slice, stride in zip(level_slices, self.strides, strict=True):
            point_strides[level_slice] = stride

        matched = torch.full((num_points,), -1, dtype=torch.long, device=value.device)
        corner_targets = value.new_zeros((num_points, 8))
        quality_targets = value.new_zeros(num_points)
        valid_gt_mask = quad_validity(value) & torch.isfinite(value).all(dim=(1, 2))

        if value.shape[0] == 0:
            empty = torch.empty((0,), dtype=torch.long, device=value.device)
            return QuadAssignment(
                matched,
                corner_targets,
                quality_targets,
                matched >= 0,
                valid_gt_mask,
                empty,
                empty,
                torch.zeros(len(feature_shapes), dtype=torch.long, device=value.device),
            )

        canonical = canonicalize_quads(value)
        num_gt = canonical.shape[0]
        inside = points_inside_convex_quads(points, canonical) & valid_point_mask[:, None]

        if pred_offsets is not None:
            pred_q = decode_quad_offsets(
                points.unsqueeze(0), pred_offsets.unsqueeze(0), point_strides.reshape(1, -1, 1)
            ).squeeze(0)
        else:
            # Fallback: square quad centered at anchor point
            h = point_strides * 2.0
            p0 = points - h[:, None]
            p1 = torch.stack([points[:, 0] + h, points[:, 1] - h], dim=1)
            p2 = points + h[:, None]
            p3 = torch.stack([points[:, 0] - h, points[:, 1] + h], dim=1)
            pred_q = torch.stack([p0, p1, p2, p3], dim=1)

        scores = (
            torch.sigmoid(pred_quality.float())
            if pred_quality is not None
            else torch.ones(num_points, device=value.device)
        )

        alignment_matrix = torch.zeros(
            (num_points, num_gt), dtype=torch.float32, device=value.device
        )
        cand_pts, cand_gts = torch.where(inside)
        if cand_pts.numel() > 0:
            pair_ious = aligned_quad_iou(
                pred_q[cand_pts], canonical[cand_gts]
            ).clamp(min=1e-6)
            pair_scores = scores[cand_pts].clamp(min=1e-6)
            t = (pair_scores**self.alpha) * (pair_ious**self.beta)
            alignment_matrix[cand_pts, cand_gts] = t

        # Handle any GT with zero candidates
        zero_cands = inside.sum(dim=0) == 0
        if zero_cands.any():
            zero_gt_indices = torch.where(zero_cands)[0]
            centers = canonical[zero_gt_indices].mean(dim=1)
            dists = ((points[:, None, :] - centers[None, :, :]) ** 2).sum(dim=2)
            dists[~valid_point_mask] = torch.inf
            closest_cands = dists.argmin(dim=0)
            pair_ious = aligned_quad_iou(
                pred_q[closest_cands], canonical[zero_gt_indices]
            ).clamp(min=1e-6)
            pair_scores = scores[closest_cands].clamp(min=1e-6)
            t = (pair_scores**self.alpha) * (pair_ious**self.beta)
            alignment_matrix[closest_cands, zero_gt_indices] = t

        if self.assignment_mode == "one_to_one":
            matches = greedy_bipartite_matching(alignment_matrix)
            for cand_i, gt_j in matches:
                matched[cand_i] = gt_j
                pair_iou = aligned_quad_iou(
                    pred_q[cand_i : cand_i + 1], canonical[gt_j : gt_j + 1]
                ).squeeze(0)
                quality_targets[cand_i] = pair_iou
        else:
            k = min(self.top_k, num_points)
            topk_vals, topk_indices = torch.topk(alignment_matrix, k, dim=0)
            selected_per_gt = torch.zeros_like(alignment_matrix, dtype=torch.bool)
            keep_topk = topk_vals > 0
            selected_per_gt.scatter_(0, topk_indices, keep_topk)

            best_metric, best_gt = (
                alignment_matrix.masked_fill(~selected_per_gt, -1.0).max(dim=1)
            )
            is_pos = best_metric > 0
            matched[is_pos] = best_gt[is_pos]

            if is_pos.any():
                pos_pts = torch.where(is_pos)[0]
                pos_gt_local = best_gt[pos_pts]
                cand_quads = pred_q[pos_pts]
                target_quads = canonical[pos_gt_local]
                actual_ious = aligned_quad_iou(cand_quads, target_quads)

                max_metric = torch.zeros(num_gt, device=value.device)
                max_metric.scatter_reduce_(
                    0,
                    pos_gt_local,
                    alignment_matrix[pos_pts, pos_gt_local],
                    reduce="amax",
                    include_self=False,
                )
                max_iou = torch.zeros(num_gt, device=value.device)
                max_iou.scatter_reduce_(
                    0, pos_gt_local, actual_ious, reduce="amax", include_self=False
                )
                norm_t = (
                    alignment_matrix[pos_pts, pos_gt_local]
                    / (max_metric[pos_gt_local] + 1e-6)
                ) * max_iou[pos_gt_local]
                quality_targets[pos_pts] = norm_t

        pos_mask = matched >= 0
        if pos_mask.any():
            pos_indices = torch.where(pos_mask)[0]
            gt_indices = matched[pos_indices]
            s = point_strides[pos_indices, None, None]
            corner_targets[pos_indices] = (
                (canonical[gt_indices] - points[pos_indices, None, :]) / s
            ).reshape(-1, 8)

        pos_per_level = torch.stack(
            [pos_mask[s].sum() for s in level_slices]
        ).to(dtype=torch.long)

        empty_indices = torch.empty((0,), dtype=torch.long, device=value.device)
        return QuadAssignment(
            matched_gt_indices=matched,
            corner_targets=corner_targets,
            quality_targets=quality_targets,
            positive_mask=pos_mask,
            valid_gt_mask=valid_gt_mask,
            unrepresentable_gt_indices=empty_indices,
            fallback_gt_indices=empty_indices,
            positive_counts_per_level=pos_per_level,
        )
