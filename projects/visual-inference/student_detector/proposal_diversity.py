"""Fixed-memory spatial and overlap diversity metrics for proposal sets."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import math

import torch
from torch import Tensor

from .quad_geometry import quad_area, quad_centroid


CLUSTER_THRESHOLDS = (0.50, 0.75, 0.90)
CLUSTER_SIZE_LIMITS = (5, 10, 20)
AREA_BINS = (0.0001, 0.0003, 0.001, 0.003, 0.01, 0.03, 0.10, 0.30)


def _distribution(values: list[float]) -> dict[str, float]:
    if not values:
        return {key: 0.0 for key in ("mean", "p50", "p90", "p95", "max")}
    tensor = torch.tensor(values, dtype=torch.float64)
    return {
        "mean": float(tensor.mean()),
        "p50": float(torch.quantile(tensor, 0.50)),
        "p90": float(torch.quantile(tensor, 0.90)),
        "p95": float(torch.quantile(tensor, 0.95)),
        "max": float(tensor.max()),
    }


def _normalized_entropy(indices: Tensor, bins: int) -> float:
    if not indices.numel() or bins <= 1:
        return 0.0
    counts = torch.bincount(indices, minlength=bins).double()
    probabilities = counts[counts > 0] / counts.sum()
    return float(-(probabilities * probabilities.log()).sum() / math.log(bins))


def _cluster_sizes(overlaps: Tensor, threshold: float) -> list[int]:
    count = overlaps.shape[0]
    parents = list(range(count))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parents[right_root] = left_root

    rows, columns = torch.nonzero(
        torch.triu(overlaps >= threshold, diagonal=1), as_tuple=True
    )
    for left, right in zip(rows.tolist(), columns.tolist(), strict=True):
        union(left, right)
    sizes: dict[int, int] = defaultdict(int)
    for index in range(count):
        sizes[find(index)] += 1
    return sorted(sizes.values(), reverse=True)


@dataclass
class ProposalDiversityAccumulator:
    """Aggregate diversity without retaining validation-long proposal tensors."""

    images: int = 0
    proposals: int = 0
    pair_count: int = 0
    pair_iou_sum: float = 0.0
    pair_hits: dict[float, int] = field(default_factory=lambda: defaultdict(int))
    largest_clusters: dict[float, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    cluster_image_hits: dict[tuple[float, int], int] = field(
        default_factory=lambda: defaultdict(int)
    )
    occupied_cells: dict[int, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    spatial_entropy: dict[int, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    log_area_std: list[float] = field(default_factory=list)
    scale_entropy: list[float] = field(default_factory=list)
    log_aspect_std: list[float] = field(default_factory=list)
    unmatched: int = 0
    unmatched_isolated: int = 0
    unmatched_overlap_50: int = 0
    unmatched_overlap_75: int = 0
    unmatched_pair_count: int = 0
    unmatched_pair_iou_sum: float = 0.0
    unmatched_pair_hits: dict[float, int] = field(
        default_factory=lambda: defaultdict(int)
    )
    unmatched_largest_clusters: dict[float, list[float]] = field(
        default_factory=lambda: defaultdict(list)
    )
    unmatched_cluster_image_hits: dict[tuple[float, int], int] = field(
        default_factory=lambda: defaultdict(int)
    )
    unmatched_occupied_cells: list[float] = field(default_factory=list)
    unmatched_spatial_entropy: list[float] = field(default_factory=list)
    unmatched_log_area_std: list[float] = field(default_factory=list)
    unmatched_scale_entropy: list[float] = field(default_factory=list)
    overlap_75_area_ratios: list[float] = field(default_factory=list)
    overlap_75_center_distances: list[float] = field(default_factory=list)

    def update(
        self,
        proposals: Tensor,
        gt_overlaps: Tensor,
        pair_overlaps: Tensor,
        *,
        image_size: int,
    ) -> None:
        """Consume one image's final `[N,4,2]` proposals and overlap matrices."""
        proposals = proposals.detach().cpu().float()
        gt_overlaps = gt_overlaps.detach().cpu().float()
        pair_overlaps = pair_overlaps.detach().cpu().float()
        count = proposals.shape[0]
        if proposals.shape != (count, 4, 2):
            raise ValueError("proposals must have shape [N, 4, 2]")
        if pair_overlaps.shape != (count, count):
            raise ValueError("pair overlaps must have shape [N, N]")
        if gt_overlaps.ndim != 2 or gt_overlaps.shape[1] != count:
            raise ValueError("ground-truth overlaps must have shape [G, N]")
        if image_size < 1:
            raise ValueError("image size must be positive")

        self.images += 1
        self.proposals += count
        upper = torch.triu(torch.ones((count, count), dtype=torch.bool), diagonal=1)
        pair_values = pair_overlaps[upper]
        self.pair_count += pair_values.numel()
        self.pair_iou_sum += float(pair_values.sum())
        for threshold in CLUSTER_THRESHOLDS:
            self.pair_hits[threshold] += int((pair_values >= threshold).sum())
            sizes = _cluster_sizes(pair_overlaps, threshold)
            largest = sizes[0] if sizes else 0
            self.largest_clusters[threshold].append(float(largest))
            for limit in CLUSTER_SIZE_LIMITS:
                self.cluster_image_hits[threshold, limit] += int(largest >= limit)

        if not count:
            for grid in (4, 8):
                self.occupied_cells[grid].append(0.0)
                self.spatial_entropy[grid].append(0.0)
            self.log_area_std.append(0.0)
            self.scale_entropy.append(0.0)
            self.log_aspect_std.append(0.0)
            return

        areas = quad_area(proposals).clamp_min(1e-7)
        normalized_areas = areas / float(image_size * image_size)
        centers = quad_centroid(proposals) / float(image_size)
        centers = centers.clamp(0.0, 1.0 - torch.finfo(torch.float32).eps)
        for grid in (4, 8):
            cells = (centers * grid).long()
            flat_cells = cells[:, 1] * grid + cells[:, 0]
            unique = torch.unique(flat_cells).numel()
            self.occupied_cells[grid].append(float(unique))
            self.spatial_entropy[grid].append(
                _normalized_entropy(flat_cells, grid * grid)
            )

        log_areas = normalized_areas.log2()
        self.log_area_std.append(float(log_areas.std(unbiased=False)))
        area_bins = torch.bucketize(
            normalized_areas,
            torch.tensor(AREA_BINS, dtype=normalized_areas.dtype),
        )
        self.scale_entropy.append(_normalized_entropy(area_bins, len(AREA_BINS) + 1))
        extents = proposals.amax(dim=1) - proposals.amin(dim=1)
        aspects = extents.amax(dim=1) / extents.amin(dim=1).clamp_min(1e-7)
        self.log_aspect_std.append(float(aspects.log2().std(unbiased=False)))

        best_gt = (
            gt_overlaps.amax(dim=0)
            if gt_overlaps.shape[0]
            else torch.zeros(count, dtype=proposals.dtype)
        )
        unmatched = best_gt < 0.10
        other_overlaps = pair_overlaps.clone()
        other_overlaps.fill_diagonal_(-1.0)
        maximum_other = other_overlaps.amax(dim=1) if count > 1 else torch.zeros(1)
        unmatched_maximum = maximum_other[unmatched]
        self.unmatched += unmatched_maximum.numel()
        self.unmatched_isolated += int((unmatched_maximum < 0.10).sum())
        self.unmatched_overlap_50 += int((unmatched_maximum >= 0.50).sum())
        self.unmatched_overlap_75 += int((unmatched_maximum >= 0.75).sum())

        unmatched_overlaps = pair_overlaps[unmatched][:, unmatched]
        unmatched_upper = torch.triu(
            torch.ones_like(unmatched_overlaps, dtype=torch.bool), diagonal=1
        )
        unmatched_pair_values = unmatched_overlaps[unmatched_upper]
        self.unmatched_pair_count += unmatched_pair_values.numel()
        self.unmatched_pair_iou_sum += float(unmatched_pair_values.sum())
        for threshold in CLUSTER_THRESHOLDS:
            self.unmatched_pair_hits[threshold] += int(
                (unmatched_pair_values >= threshold).sum()
            )
            sizes = _cluster_sizes(unmatched_overlaps, threshold)
            largest = sizes[0] if sizes else 0
            self.unmatched_largest_clusters[threshold].append(float(largest))
            for limit in CLUSTER_SIZE_LIMITS:
                self.unmatched_cluster_image_hits[threshold, limit] += int(
                    largest >= limit
                )

        unmatched_centers = centers[unmatched]
        unmatched_areas = normalized_areas[unmatched]
        if unmatched_centers.shape[0]:
            cells = (unmatched_centers * 8).long()
            flat_cells = cells[:, 1] * 8 + cells[:, 0]
            self.unmatched_occupied_cells.append(
                float(torch.unique(flat_cells).numel())
            )
            self.unmatched_spatial_entropy.append(_normalized_entropy(flat_cells, 64))
            unmatched_log_areas = unmatched_areas.log2()
            self.unmatched_log_area_std.append(
                float(unmatched_log_areas.std(unbiased=False))
            )
            unmatched_area_bins = torch.bucketize(
                unmatched_areas,
                torch.tensor(AREA_BINS, dtype=unmatched_areas.dtype),
            )
            self.unmatched_scale_entropy.append(
                _normalized_entropy(unmatched_area_bins, len(AREA_BINS) + 1)
            )
        else:
            self.unmatched_occupied_cells.append(0.0)
            self.unmatched_spatial_entropy.append(0.0)
            self.unmatched_log_area_std.append(0.0)
            self.unmatched_scale_entropy.append(0.0)

        high_rows, high_columns = torch.nonzero(
            torch.triu(pair_overlaps >= 0.75, diagonal=1), as_tuple=True
        )
        if high_rows.numel():
            left_areas = areas[high_rows]
            right_areas = areas[high_columns]
            ratios = torch.minimum(left_areas, right_areas) / torch.maximum(
                left_areas, right_areas
            )
            distances = torch.linalg.vector_norm(
                centers[high_rows] - centers[high_columns], dim=1
            )
            self.overlap_75_area_ratios.extend(ratios.tolist())
            self.overlap_75_center_distances.extend(distances.tolist())

    def compute(self) -> dict[str, object]:
        """Return JSON-ready aggregate metrics."""
        pair_count = max(self.pair_count, 1)
        unmatched = max(self.unmatched, 1)
        unmatched_pair_count = max(self.unmatched_pair_count, 1)
        return {
            "images": self.images,
            "proposals_per_image": self.proposals / max(self.images, 1),
            "pairwise": {
                "iou_mean": self.pair_iou_sum / pair_count,
                **{
                    f"iou_ge_{threshold:.2f}_fraction": self.pair_hits[threshold]
                    / pair_count
                    for threshold in CLUSTER_THRESHOLDS
                },
            },
            "clusters": {
                f"iou_{threshold:.2f}": {
                    "largest_size": _distribution(self.largest_clusters[threshold]),
                    **{
                        f"images_with_size_ge_{limit}_fraction": (
                            self.cluster_image_hits[threshold, limit]
                            / max(self.images, 1)
                        )
                        for limit in CLUSTER_SIZE_LIMITS
                    },
                }
                for threshold in CLUSTER_THRESHOLDS
            },
            "spatial": {
                f"grid_{grid}": {
                    "occupied_cells": _distribution(self.occupied_cells[grid]),
                    "normalized_entropy": _distribution(self.spatial_entropy[grid]),
                }
                for grid in (4, 8)
            },
            "scale": {
                "log2_area_std": _distribution(self.log_area_std),
                "area_bin_entropy": _distribution(self.scale_entropy),
                "log2_aspect_std": _distribution(self.log_aspect_std),
            },
            "unmatched": {
                "count": self.unmatched,
                "fraction_of_proposals": self.unmatched / max(self.proposals, 1),
                "isolated_iou_lt_0.10_fraction": self.unmatched_isolated / unmatched,
                "overlaps_other_iou_ge_0.50_fraction": self.unmatched_overlap_50
                / unmatched,
                "overlaps_other_iou_ge_0.75_fraction": self.unmatched_overlap_75
                / unmatched,
                "within_unmatched": {
                    "pairwise": {
                        "iou_mean": self.unmatched_pair_iou_sum / unmatched_pair_count,
                        **{
                            f"iou_ge_{threshold:.2f}_fraction": (
                                self.unmatched_pair_hits[threshold]
                                / unmatched_pair_count
                            )
                            for threshold in CLUSTER_THRESHOLDS
                        },
                    },
                    "clusters": {
                        f"iou_{threshold:.2f}": {
                            "largest_size": _distribution(
                                self.unmatched_largest_clusters[threshold]
                            ),
                            **{
                                f"images_with_size_ge_{limit}_fraction": (
                                    self.unmatched_cluster_image_hits[threshold, limit]
                                    / max(self.images, 1)
                                )
                                for limit in CLUSTER_SIZE_LIMITS
                            },
                        }
                        for threshold in CLUSTER_THRESHOLDS
                    },
                    "spatial_grid_8": {
                        "occupied_cells": _distribution(self.unmatched_occupied_cells),
                        "normalized_entropy": _distribution(
                            self.unmatched_spatial_entropy
                        ),
                    },
                    "scale": {
                        "log2_area_std": _distribution(self.unmatched_log_area_std),
                        "area_bin_entropy": _distribution(self.unmatched_scale_entropy),
                    },
                },
            },
            "overlap_75_pairs": {
                "count": len(self.overlap_75_area_ratios),
                "area_ratio": _distribution(self.overlap_75_area_ratios),
                "center_distance_image_side": _distribution(
                    self.overlap_75_center_distances
                ),
            },
        }


__all__ = ["ProposalDiversityAccumulator"]
