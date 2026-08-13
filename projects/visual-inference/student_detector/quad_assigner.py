"""Static anchor-free assignment for class-agnostic quadrilateral targets."""

from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Sequence

import torch
from torch import Tensor

from .geometry import make_grid_points
from .quad_geometry import (
    canonicalize_quads,
    points_inside_convex_quads,
    quad_area,
    quad_centroid,
    quad_validity,
)


@dataclass(frozen=True)
class QuadAssignment:
    """Flattened P3-P5 assignment for one image."""

    matched_gt_indices: Tensor
    corner_targets: Tensor
    quality_targets: Tensor
    positive_mask: Tensor
    valid_gt_mask: Tensor
    unrepresentable_gt_indices: Tensor
    fallback_gt_indices: Tensor
    positive_counts_per_level: Tensor


class QuadAssigner:
    """Assign central in-quad points with aspect-ratio-aware quality."""

    def __init__(
        self,
        *,
        strides: tuple[int, int, int] = (8, 16, 32),
        top_k: int = 9,
        gamma: float = 2.0,
        scale_sigma: float = 0.75,
        eligible_levels: int = 2,
        scale_measure: str = "area",
    ) -> None:
        if len(strides) != 3:
            raise ValueError("the baseline quad assigner expects P3-P5")
        if top_k < 1:
            raise ValueError("top_k must be positive")
        if gamma <= 0 or scale_sigma <= 0:
            raise ValueError("gamma and scale_sigma must be positive")
        if scale_measure not in {"area", "maximum_extent"}:
            raise ValueError("scale_measure must be area or maximum_extent")
        self.strides = strides
        self.top_k = top_k
        self.gamma = gamma
        self.scale_sigma = scale_sigma
        self.eligible_levels = max(1, min(eligible_levels, len(strides)))
        self.scale_measure = scale_measure

    def _scale_scores(self, area_scale: Tensor) -> Tensor:
        references = area_scale.new_tensor(self.strides) * 4.0
        if area_scale.ndim:
            area_scale = area_scale.unsqueeze(-1)
        return torch.exp(
            -(
                torch.log(area_scale.clamp(min=1e-6) / references).abs()
                / self.scale_sigma
            )
        )

    def _object_scales(self, quads: Tensor) -> Tensor:
        if self.scale_measure == "area":
            return torch.sqrt(quad_area(quads).clamp(min=1e-6))
        centers = quad_centroid(quads)
        centered = quads - centers[:, None, :]
        covariance = centered.transpose(1, 2) @ centered / 4.0
        _, axes = torch.linalg.eigh(covariance.to(torch.float32))
        projected = centered @ axes
        return (
            (projected.amax(dim=1) - projected.amin(dim=1)).amax(dim=1).clamp(min=1e-6)
        )

    def _center_quality_many(self, points: Tensor, quads: Tensor) -> Tensor:
        centers = quad_centroid(quads)
        centered = quads - centers[:, None, :]
        covariance = centered.transpose(1, 2) @ centered / 4.0
        eigenvalues, eigenvectors = torch.linalg.eigh(covariance.to(torch.float32))
        major_indices = eigenvalues.argmax(dim=1)
        major = eigenvectors[
            torch.arange(quads.shape[0], device=quads.device), :, major_indices
        ]
        major = major / torch.linalg.vector_norm(major, dim=1, keepdim=True).clamp(
            min=1e-6
        )
        minor = torch.stack((-major[:, 1], major[:, 0]), dim=1)
        axes = torch.stack((major, minor), dim=2)
        projections = centered @ axes
        half_extents = projections.abs().amax(dim=1).clamp(min=1e-4)
        relative = points[:, None, :] - centers[None, :, :]
        local = torch.einsum("pgd,gda->pga", relative, axes)
        normalized_squared = (local / half_extents[None, :, :]).square().sum(dim=2)
        return torch.exp(-self.gamma * normalized_squared)

    def assign(
        self,
        gt_quads: Tensor,
        feature_shapes: Sequence[tuple[int, int]],
        image_size: tuple[int, int],
        valid_point_mask: Tensor | None = None,
    ) -> QuadAssignment:
        """Assign one image's quads to flattened feature locations."""
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
            if valid_point_mask.shape != (num_points,):
                raise ValueError(
                    f"valid_point_mask must contain {num_points} locations"
                )
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

        num_gt = value.shape[0]
        qualities = value.new_full((num_points, num_gt), -1.0)
        center_targets = value.new_zeros((num_points, num_gt))
        valid_indices = torch.where(valid_gt_mask)[0]
        height, width = image_size
        point_level = torch.empty(num_points, dtype=torch.long, device=value.device)
        for level, level_slice in enumerate(level_slices):
            point_level[level_slice] = level

        canonical = canonicalize_quads(value)
        bbox_min = canonical.amin(dim=1)
        bbox_max = canonical.amax(dim=1)
        visible = (
            (bbox_max[:, 0] > 0)
            & (bbox_max[:, 1] > 0)
            & (bbox_min[:, 0] < width)
            & (bbox_min[:, 1] < height)
        )
        assignable_gt = valid_gt_mask & visible
        center_quality = self._center_quality_many(points, canonical)
        inside = points_inside_convex_quads(points, canonical)
        inside &= valid_point_mask[:, None] & assignable_gt[None, :]
        scale_quality = self._scale_scores(self._object_scales(canonical))
        score = center_quality * scale_quality[:, point_level].transpose(0, 1)
        eligible_order = torch.argsort(
            torch.log(scale_quality.clamp(min=1e-7)).abs(), dim=1
        )
        eligible = torch.zeros(
            (num_gt, len(level_slices)), dtype=torch.bool, device=value.device
        )
        eligible.scatter_(1, eligible_order[:, : self.eligible_levels], True)
        selected = torch.zeros_like(inside)
        gt_grid = torch.arange(num_gt, device=value.device)
        for level, level_slice in enumerate(level_slices):
            level_scores = score[level_slice].masked_fill(
                ~inside[level_slice] | ~eligible[:, level][None, :], -1.0
            )
            count = min(self.top_k, level_scores.shape[0])
            top_values, top_indices = torch.topk(
                level_scores, count, dim=0, largest=True
            )
            keep = top_values >= 0
            selected[
                top_indices[keep] + int(level_slice.start or 0),
                gt_grid.expand_as(top_indices)[keep],
            ] = True
        qualities = score.masked_fill(~selected, -1.0)
        center_targets = center_quality.masked_fill(~selected, 0.0)

        best_quality, best_gt_local = qualities.max(dim=1)
        normal = best_quality >= 0
        matched[normal] = best_gt_local[normal]
        fallbacks: list[int] = []
        unrepresentable: list[int] = []
        for original_index in valid_indices.tolist():
            if torch.any(matched == original_index):
                continue
            available = inside[:, original_index] & (matched < 0)
            if not available.any():
                unrepresentable.append(original_index)
                continue
            fallback_scores = score[:, original_index].masked_fill(~available, -1.0)
            fallback_point = fallback_scores.argmax()
            matched[fallback_point] = original_index
            qualities[fallback_point, original_index] = fallback_scores[fallback_point]
            center_targets[fallback_point, original_index] = center_quality[
                fallback_point, original_index
            ]
            fallbacks.append(original_index)

        positive = matched >= 0
        if positive.any():
            point_indices = torch.where(positive)[0]
            gt_indices = matched[point_indices]
            strides = value.new_tensor(self.strides)[point_level[point_indices]]
            corner_targets[point_indices] = (
                (value[gt_indices] - points[point_indices, None, :])
                / strides[:, None, None]
            ).reshape(-1, 8)
            quality_targets[point_indices] = center_targets[
                point_indices, gt_indices
            ].clamp(0, 1)

        positive_counts = torch.stack(
            [positive[level_slice].sum() for level_slice in level_slices]
        ).to(dtype=torch.long)
        return QuadAssignment(
            matched,
            corner_targets,
            quality_targets,
            positive,
            valid_gt_mask,
            torch.tensor(unrepresentable, dtype=torch.long, device=value.device),
            torch.tensor(fallbacks, dtype=torch.long, device=value.device),
            positive_counts,
        )
