"""Static anchor-free assignment for class-agnostic quadrilateral targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import torch
from torch import Tensor

from .geometry import make_grid_points
from .quad_geometry import (
    canonicalize_quad,
    points_inside_convex_quad,
    quad_area,
    quad_centroid,
    quad_offsets,
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

    def _object_scale(self, quad: Tensor) -> Tensor:
        if self.scale_measure == "area":
            return torch.sqrt(quad_area(quad).clamp(min=1e-6))
        center = quad_centroid(quad)
        centered = quad - center
        covariance = centered.transpose(0, 1) @ centered / 4.0
        _, axes = torch.linalg.eigh(covariance.to(torch.float32))
        projected = centered @ axes
        return (projected.amax(dim=0) - projected.amin(dim=0)).amax().clamp(min=1e-6)

    def _scale_scores(self, area_scale: Tensor) -> Tensor:
        references = area_scale.new_tensor(self.strides) * 4.0
        return torch.exp(
            -(
                torch.log(area_scale.clamp(min=1e-6) / references)
                .abs()
                / self.scale_sigma
            )
        )

    def _center_quality(
        self, points: Tensor, quad: Tensor
    ) -> tuple[Tensor, Tensor]:
        center = quad_centroid(quad)
        centered = quad - center
        covariance = centered.transpose(0, 1) @ centered / 4.0
        eigenvalues, eigenvectors = torch.linalg.eigh(covariance.to(torch.float32))
        major = eigenvectors[:, torch.argmax(eigenvalues)]
        major = major / torch.linalg.vector_norm(major).clamp(min=1e-6)
        minor = torch.stack((-major[1], major[0]))
        projections = torch.stack((centered @ major, centered @ minor), dim=1)
        half_extents = projections.abs().amax(dim=0).clamp(min=1e-4)
        relative = points - center
        local = torch.stack((relative @ major, relative @ minor), dim=1)
        normalized_squared = (local / half_extents).square().sum(dim=1)
        return torch.exp(-self.gamma * normalized_squared), normalized_squared

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
            valid_point_mask = torch.ones(num_points, dtype=torch.bool, device=value.device)
        else:
            valid_point_mask = valid_point_mask.to(device=value.device, dtype=torch.bool).flatten()
            if valid_point_mask.shape != (num_points,):
                raise ValueError(f"valid_point_mask must contain {num_points} locations")
        matched = torch.full((num_points,), -1, dtype=torch.long, device=value.device)
        corner_targets = value.new_zeros((num_points, 8))
        quality_targets = value.new_zeros(num_points)
        valid_gt_mask = torch.tensor(
            [bool(quad_validity(item)) and torch.isfinite(item).all() for item in value],
            dtype=torch.bool,
            device=value.device,
        )
        if value.shape[0] == 0:
            empty = torch.empty((0,), dtype=torch.long, device=value.device)
            return QuadAssignment(
                matched, corner_targets, quality_targets, matched >= 0,
                valid_gt_mask, empty, empty,
                torch.zeros(len(feature_shapes), dtype=torch.long, device=value.device),
            )

        qualities = value.new_full((num_points, value.shape[0]), -1.0)
        center_targets = value.new_zeros((num_points, value.shape[0]))
        valid_indices = torch.where(valid_gt_mask)[0]
        height, width = image_size
        point_level = torch.empty(num_points, dtype=torch.long, device=value.device)
        for level, level_slice in enumerate(level_slices):
            point_level[level_slice] = level

        for original_index in valid_indices.tolist():
            quad = canonicalize_quad(value[original_index])
            bbox = torch.stack((quad[:, 0].min(), quad[:, 1].min(), quad[:, 0].max(), quad[:, 1].max()))
            if bbox[2] <= 0 or bbox[3] <= 0 or bbox[0] >= width or bbox[1] >= height:
                continue
            center_quality, _ = self._center_quality(points, quad)
            inside = points_inside_convex_quad(points, quad) & valid_point_mask
            area_scale = self._object_scale(quad)
            scale_quality = self._scale_scores(area_scale)
            level_distance = torch.log(scale_quality.clamp(min=1e-7)).abs()
            eligible_level_order = torch.argsort(level_distance)
            allowed_levels = set(eligible_level_order[: self.eligible_levels].tolist())
            selected = torch.zeros(num_points, dtype=torch.bool, device=value.device)
            for level in allowed_levels:
                level_slice = level_slices[level]
                pool = inside[level_slice]
                if not pool.any():
                    continue
                level_quality = (center_quality[level_slice] * scale_quality[level]).masked_fill(~pool, -1.0)
                count = min(self.top_k, int(pool.sum().item()))
                selected_indices = torch.topk(level_quality, count, largest=True).indices
                selected[level_slice.start + selected_indices] = True  # type: ignore[operator]
            score = center_quality * scale_quality[point_level]
            qualities[:, original_index] = score.masked_fill(~selected, -1.0)
            center_targets[:, original_index] = center_quality.masked_fill(~selected, 0.0)

        best_quality, best_gt_local = qualities.max(dim=1)
        normal = best_quality >= 0
        matched[normal] = best_gt_local[normal]
        fallbacks: list[int] = []
        unrepresentable: list[int] = []
        for original_index in valid_indices.tolist():
            if torch.any(matched == original_index):
                continue
            quad = canonicalize_quad(value[original_index])
            inside = points_inside_convex_quad(points, quad) & valid_point_mask & (matched < 0)
            if not inside.any():
                unrepresentable.append(original_index)
                continue
            center_quality, _ = self._center_quality(points, quad)
            area_scale = self._object_scale(quad)
            scale_quality = self._scale_scores(area_scale)
            score = (center_quality * scale_quality[point_level]).masked_fill(~inside, -1.0)
            fallback_point = score.argmax()
            matched[fallback_point] = original_index
            qualities[fallback_point, original_index] = score[fallback_point]
            center_targets[fallback_point, original_index] = center_quality[fallback_point]
            fallbacks.append(original_index)

        positive = matched >= 0
        if positive.any():
            for level, level_slice in enumerate(level_slices):
                level_positive = positive[level_slice]
                if level_positive.any():
                    point_indices = torch.where(level_positive)[0] + int(level_slice.start or 0)
                    gt_indices = matched[point_indices]
                    corner_targets[point_indices] = torch.stack([
                        quad_offsets(value[gt_index], points[point_index], self.strides[level])
                        for point_index, gt_index in zip(point_indices.tolist(), gt_indices.tolist(), strict=True)
                    ])
                    quality_targets[point_indices] = center_targets[
                        point_indices, gt_indices
                    ].clamp(0, 1)

        positive_counts = torch.stack([
            positive[level_slice].sum() for level_slice in level_slices
        ]).to(dtype=torch.long)
        return QuadAssignment(
            matched, corner_targets, quality_targets, positive, valid_gt_mask,
            torch.tensor(unrepresentable, dtype=torch.long, device=value.device),
            torch.tensor(fallbacks, dtype=torch.long, device=value.device),
            positive_counts,
        )
