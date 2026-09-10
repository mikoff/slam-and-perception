"""Fixed-budget utility metrics for class-agnostic proposal sets."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import torch
from torch import Tensor

from .quad_geometry import quad_area
from .suppression import RECALL_BUDGETS, RECALL_THRESHOLDS

UNMATCHED_IOU_THRESHOLD = 0.10
REGION_FRACTION_THRESHOLD = 0.50
RELIABILITY_BIN_EDGES = tuple(index / 10 for index in range(11))


def intersection_from_iou(
    overlaps: Tensor, left_areas: Tensor, right_areas: Tensor
) -> Tensor:
    """Recover pairwise intersection areas from IoU and polygon areas."""
    if overlaps.shape != (left_areas.numel(), right_areas.numel()):
        raise ValueError("overlap shape must match the supplied polygon areas")
    return (
        overlaps
        * (left_areas[:, None] + right_areas[None, :])
        / (1.0 + overlaps).clamp_min(1e-7)
    )


def proposal_region_fraction(
    region_overlaps: Tensor,
    region_quads: Tensor,
    proposal_quads: Tensor,
    *,
    disjoint_regions: bool,
) -> Tensor:
    """Return proposal coverage by a region set from an overlap matrix."""
    if proposal_quads.numel() == 0:
        return proposal_quads.new_zeros((0,))
    if region_quads.numel() == 0:
        return proposal_quads.new_zeros((proposal_quads.shape[0],))
    intersections = intersection_from_iou(
        region_overlaps,
        quad_area(region_quads),
        quad_area(proposal_quads),
    )
    covered = (
        intersections.sum(dim=0) if disjoint_regions else intersections.amax(dim=0)
    )
    return (covered / quad_area(proposal_quads).clamp_min(1e-7)).clamp(0, 1)


@dataclass
class ProposalUtilityAccumulator:
    """Aggregate recall slices, crop coverage, purity, and false-proposal budgets."""

    images: int = 0
    ground_truth: int = 0
    proposals: int = 0
    hits: dict[tuple[int, float], int] = field(default_factory=lambda: defaultdict(int))
    slice_counts: dict[tuple[str, str, int, float], list[int]] = field(
        default_factory=lambda: defaultdict(lambda: [0, 0])
    )
    gt_coverage_sum: dict[int, float] = field(
        default_factory=lambda: defaultdict(float)
    )
    proposal_object_fraction_sum: float = 0.0
    trusted_background_fraction_sum: float = 0.0
    unmatched: int = 0
    ignored_unmatched: int = 0
    trusted_background_false: int = 0
    ego_overlap: int = 0
    ego_images: int = 0
    ego_image_proposals: int = 0

    def update(
        self,
        *,
        gt_overlaps: Tensor,
        gt_quads: Tensor,
        proposal_quads: Tensor,
        trusted_fraction: Tensor,
        ignore_fraction: Tensor,
        ego_fraction: Tensor,
        ego_present: bool,
        slice_labels: dict[str, tuple[str, ...]],
    ) -> None:
        """Consume one score-ranked, post-NMS proposal set."""
        gt_overlaps = gt_overlaps.detach().cpu()
        gt_quads = gt_quads.detach().cpu()
        proposal_quads = proposal_quads.detach().cpu()
        trusted_fraction = trusted_fraction.detach().cpu()
        ignore_fraction = ignore_fraction.detach().cpu()
        ego_fraction = ego_fraction.detach().cpu()
        gt_count, proposal_count = gt_overlaps.shape
        if proposal_count != proposal_quads.shape[0]:
            raise ValueError("proposal overlap columns must match proposal geometry")
        if any(len(labels) != gt_count for labels in slice_labels.values()):
            raise ValueError("slice labels must align with ground truth")
        self.images += 1
        self.ground_truth += gt_count
        final_count = min(100, proposal_count)
        self.proposals += final_count

        gt_areas = quad_area(gt_quads)
        proposal_areas = quad_area(proposal_quads[:final_count])
        intersections = intersection_from_iou(
            gt_overlaps[:, :final_count], gt_areas, proposal_areas
        )
        for budget in RECALL_BUDGETS:
            count = min(budget, final_count)
            selected = gt_overlaps[:, :count]
            best = selected.amax(dim=1) if count else gt_overlaps.new_zeros((gt_count,))
            coverage = (
                intersections[:, :count].amax(dim=1) / gt_areas.clamp_min(1e-7)
                if count
                else gt_areas.new_zeros((gt_count,))
            )
            self.gt_coverage_sum[budget] += float(coverage.sum())
            for threshold in RECALL_THRESHOLDS:
                self.hits[budget, threshold] += int((best >= threshold).sum())
            for slice_name, labels in slice_labels.items():
                for label in set(labels):
                    mask = torch.tensor([item == label for item in labels])
                    for threshold in (0.50, 0.75):
                        counts = self.slice_counts[slice_name, label, budget, threshold]
                        counts[0] += int((best[mask] >= threshold).sum())
                        counts[1] += int(mask.sum())

        if not final_count:
            if ego_present:
                self.ego_images += 1
            return
        object_fraction = (
            intersections.amax(dim=0) / proposal_areas.clamp_min(1e-7)
            if gt_count
            else proposal_areas.new_zeros((final_count,))
        ).clamp(0, 1)
        self.proposal_object_fraction_sum += float(object_fraction.sum())
        self.trusted_background_fraction_sum += float(
            trusted_fraction[:final_count].sum()
        )
        best_iou = (
            gt_overlaps[:, :final_count].amax(dim=0)
            if gt_count
            else proposal_areas.new_zeros((final_count,))
        )
        unmatched = best_iou < UNMATCHED_IOU_THRESHOLD
        ignored = ignore_fraction[:final_count] >= REGION_FRACTION_THRESHOLD
        self.unmatched += int(unmatched.sum())
        self.ignored_unmatched += int((unmatched & ignored).sum())
        self.trusted_background_false += int(
            (
                unmatched
                & ~ignored
                & (trusted_fraction[:final_count] >= REGION_FRACTION_THRESHOLD)
            ).sum()
        )
        if ego_present:
            self.ego_images += 1
            self.ego_image_proposals += final_count
            self.ego_overlap += int(
                (ego_fraction[:final_count] >= REGION_FRACTION_THRESHOLD).sum()
            )

    def compute(self) -> dict[str, float | int]:
        """Finalize utility metrics."""
        result: dict[str, float | int] = {
            "images": self.images,
            "ground_truth": self.ground_truth,
            "proposals": self.proposals,
            "proposal/object_fraction_mean": self.proposal_object_fraction_sum
            / max(self.proposals, 1),
            "proposal/non_object_fraction_mean": 1.0
            - self.proposal_object_fraction_sum / max(self.proposals, 1),
            "proposal/trusted_background_fraction_mean": (
                self.trusted_background_fraction_sum / max(self.proposals, 1)
            ),
            "false/unmatched_fraction": self.unmatched / max(self.proposals, 1),
            "false/unmatched_ignore_exempt_fraction": self.ignored_unmatched
            / max(self.proposals, 1),
            "false/trusted_background_fraction": self.trusted_background_false
            / max(self.proposals, 1),
            "false/ego_body_fraction_on_ego_images": self.ego_overlap
            / max(self.ego_image_proposals, 1),
            "false/ego_body_images": self.ego_images,
        }
        for budget in RECALL_BUDGETS:
            result[f"coverage/object_mean/{budget}"] = self.gt_coverage_sum[
                budget
            ] / max(self.ground_truth, 1)
            for threshold in RECALL_THRESHOLDS:
                result[f"recall/{budget}@{threshold:.2f}"] = self.hits[
                    budget, threshold
                ] / max(self.ground_truth, 1)
            result[f"ar/{budget}"] = sum(
                float(result[f"recall/{budget}@{threshold:.2f}"])
                for threshold in RECALL_THRESHOLDS
            ) / len(RECALL_THRESHOLDS)
        for (name, label, budget, threshold), (hits, total) in sorted(
            self.slice_counts.items()
        ):
            result[f"ground_truth/{name}_{label}"] = total
            result[f"recall/{name}_{label}/{budget}@{threshold:.2f}"] = hits / max(
                total, 1
            )
        return result


@dataclass
class ScoreReliabilityAccumulator:
    """Measure whether proposal scores rank object-like rather than false crops."""

    bin_edges: tuple[float, ...] = RELIABILITY_BIN_EDGES
    counts: list[int] = field(init=False)
    score_sums: list[float] = field(init=False)
    object_hits: list[int] = field(init=False)
    unmatched: list[int] = field(init=False)
    trusted_background_false: list[int] = field(init=False)

    def __post_init__(self) -> None:
        if (
            len(self.bin_edges) < 2
            or self.bin_edges[0] != 0.0
            or self.bin_edges[-1] != 1.0
            or any(
                left >= right
                for left, right in zip(self.bin_edges, self.bin_edges[1:])
            )
        ):
            raise ValueError("reliability bin edges must increase from 0 to 1")
        size = len(self.bin_edges) - 1
        self.counts = [0] * size
        self.score_sums = [0.0] * size
        self.object_hits = [0] * size
        self.unmatched = [0] * size
        self.trusted_background_false = [0] * size

    def update(
        self,
        *,
        scores: Tensor,
        best_object_iou: Tensor,
        trusted_fraction: Tensor,
        ignore_fraction: Tensor,
    ) -> None:
        """Consume post-NMS proposals and their annotation-derived outcomes."""
        values = tuple(
            item.detach().cpu().flatten()
            for item in (scores, best_object_iou, trusted_fraction, ignore_fraction)
        )
        if len({item.numel() for item in values}) != 1:
            raise ValueError("reliability inputs must have equal lengths")
        score_values, object_iou, trusted, ignored = values
        for score, iou, trusted_value, ignore_value in zip(
            score_values.tolist(),
            object_iou.tolist(),
            trusted.tolist(),
            ignored.tolist(),
            strict=True,
        ):
            if not 0.0 <= score <= 1.0:
                raise ValueError("proposal scores must be in [0, 1]")
            index = min(int(score * len(self.counts)), len(self.counts) - 1)
            is_unmatched = iou < UNMATCHED_IOU_THRESHOLD and ignore_value < 0.50
            self.counts[index] += 1
            self.score_sums[index] += score
            self.object_hits[index] += int(iou >= 0.50)
            self.unmatched[index] += int(is_unmatched)
            self.trusted_background_false[index] += int(
                is_unmatched and trusted_value >= REGION_FRACTION_THRESHOLD
            )

    def compute(self) -> list[dict[str, float | int | None]]:
        """Return fixed score bins suitable for aggregate or per-domain curves."""
        result = []
        for index, count in enumerate(self.counts):
            result.append(
                {
                    "score_min": self.bin_edges[index],
                    "score_max": self.bin_edges[index + 1],
                    "count": count,
                    "score_mean": self.score_sums[index] / count if count else None,
                    "object_iou_50_rate": self.object_hits[index] / count if count else None,
                    "unmatched_rate": self.unmatched[index] / count if count else None,
                    "trusted_background_false_rate": (
                        self.trusted_background_false[index] / count if count else None
                    ),
                }
            )
        return result
