"""Deterministic NMS replay from a verified pairwise overlap matrix."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import torch
from torch import Tensor

RECALL_BUDGETS = (10, 50, 100)
RECALL_THRESHOLDS = tuple(0.50 + 0.05 * index for index in range(10))


def greedy_nms_from_overlaps(
    overlaps: Tensor,
    scores: Tensor,
    iou_threshold: float,
    *,
    max_output: int,
) -> Tensor:
    """Return stable greedy-NMS indices without recomputing geometry overlaps."""
    if overlaps.ndim != 2 or overlaps.shape[0] != overlaps.shape[1]:
        raise ValueError("NMS overlaps must be a square matrix")
    if scores.ndim != 1 or scores.shape[0] != overlaps.shape[0]:
        raise ValueError("NMS scores must match the overlap matrix")
    if not 0 <= iou_threshold <= 1:
        raise ValueError("NMS IoU threshold must be in [0, 1]")
    if max_output < 1:
        raise ValueError("max_output must be positive")
    order = torch.argsort(scores.detach().cpu(), descending=True, stable=True)
    overlaps_cpu = overlaps.detach().cpu()
    suppressed = torch.zeros(overlaps.shape[0], dtype=torch.bool)
    kept: list[int] = []
    for index in order.tolist():
        if suppressed[index]:
            continue
        kept.append(index)
        suppressed |= overlaps_cpu[index] > iou_threshold
        if len(kept) >= max_output:
            break
    return torch.tensor(kept, dtype=torch.int64, device=scores.device)


@dataclass
class SuppressionSweepAccumulator:
    """Aggregate recall and redundancy from cached candidate-overlap matrices."""

    images: int = 0
    ground_truth: int = 0
    proposal_counts: dict[int, int] = field(default_factory=lambda: defaultdict(int))
    hits: dict[tuple[int, float], int] = field(
        default_factory=lambda: defaultdict(int)
    )
    matched_proposals: dict[int, int] = field(
        default_factory=lambda: defaultdict(int)
    )
    duplicates: dict[int, int] = field(default_factory=lambda: defaultdict(int))
    pair_counts: dict[int, int] = field(default_factory=lambda: defaultdict(int))
    pair_iou_sums: dict[int, float] = field(
        default_factory=lambda: defaultdict(float)
    )
    pair_hits: dict[tuple[int, float], int] = field(
        default_factory=lambda: defaultdict(int)
    )
    replay_seconds: float = 0.0

    def update(
        self, gt_overlaps: Tensor, proposal_overlaps: Tensor, keep: Tensor
    ) -> None:
        """Consume one image using indices into score-ranked candidates."""
        if gt_overlaps.ndim != 2:
            raise ValueError("ground-truth overlaps must be a matrix")
        candidate_count = proposal_overlaps.shape[0]
        if proposal_overlaps.shape != (candidate_count, candidate_count):
            raise ValueError("proposal overlaps must be square")
        if gt_overlaps.shape[1] != candidate_count:
            raise ValueError("ground-truth and proposal overlap columns must align")
        keep_cpu = keep.detach().cpu()
        gt_cpu = gt_overlaps.detach().cpu()
        proposal_cpu = proposal_overlaps.detach().cpu()
        self.images += 1
        self.ground_truth += gt_cpu.shape[0]
        for budget in RECALL_BUDGETS:
            selected = keep_cpu[:budget]
            count = selected.numel()
            self.proposal_counts[budget] += count
            selected_gt = gt_cpu[:, selected]
            best_gt_overlap = (
                selected_gt.max(dim=1).values
                if selected_gt.shape[1]
                else gt_cpu.new_zeros((gt_cpu.shape[0],))
            )
            for threshold in RECALL_THRESHOLDS:
                self.hits[budget, threshold] += int(
                    (best_gt_overlap >= threshold).sum()
                )
            if selected_gt.shape[0] and count:
                best_proposal_iou, best_gt = selected_gt.max(dim=0)
                matched = best_proposal_iou >= 0.50
                self.matched_proposals[budget] += int(matched.sum())
                covered: set[int] = set()
                for is_match, gt_index in zip(
                    matched.tolist(), best_gt.tolist(), strict=True
                ):
                    if not is_match:
                        continue
                    if gt_index in covered:
                        self.duplicates[budget] += 1
                    else:
                        covered.add(gt_index)
            if count >= 2:
                selected_pairs = proposal_cpu[selected][:, selected]
                upper = torch.triu(
                    torch.ones((count, count), dtype=torch.bool), diagonal=1
                )
                pair_values = selected_pairs[upper]
                self.pair_counts[budget] += pair_values.numel()
                self.pair_iou_sums[budget] += float(pair_values.sum())
                for threshold in (0.50, 0.75):
                    self.pair_hits[budget, threshold] += int(
                        (pair_values >= threshold).sum()
                    )

    def compute(self) -> dict[str, float | int]:
        """Return fixed-budget recall, duplicate, overlap, and volume metrics."""
        result: dict[str, float | int] = {
            "images": self.images,
            "ground_truth": self.ground_truth,
            "replay_nms_ms_per_image": (
                1000 * self.replay_seconds / self.images if self.images else 0.0
            ),
        }
        for budget in RECALL_BUDGETS:
            proposals = self.proposal_counts[budget]
            matched = self.matched_proposals[budget]
            pairs = self.pair_counts[budget]
            result[f"proposals/{budget}_per_image"] = (
                proposals / self.images if self.images else 0.0
            )
            for threshold in RECALL_THRESHOLDS:
                result[f"recall/{budget}@{threshold:.2f}"] = (
                    self.hits[budget, threshold] / self.ground_truth
                    if self.ground_truth
                    else 0.0
                )
            result[f"ar/{budget}"] = sum(
                float(result[f"recall/{budget}@{threshold:.2f}"])
                for threshold in RECALL_THRESHOLDS
            ) / len(RECALL_THRESHOLDS)
            result[f"duplicates/{budget}@0.50_fraction"] = (
                self.duplicates[budget] / proposals if proposals else 0.0
            )
            result[f"duplicates/{budget}@0.50_of_matched"] = (
                self.duplicates[budget] / matched if matched else 0.0
            )
            result[f"pairwise/{budget}_iou_mean"] = (
                self.pair_iou_sums[budget] / pairs if pairs else 0.0
            )
            for threshold in (0.50, 0.75):
                result[f"pairwise/{budget}@{threshold:.2f}_fraction"] = (
                    self.pair_hits[budget, threshold] / pairs if pairs else 0.0
                )
        return result
