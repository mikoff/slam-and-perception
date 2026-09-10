from __future__ import annotations

import pytest
import torch

from student_detector.proposal_utility import (
    ProposalUtilityAccumulator,
    ScoreReliabilityAccumulator,
    intersection_from_iou,
    proposal_region_fraction,
)
from student_detector.quad_geometry import quad_from_bbox


def test_intersection_is_recovered_from_iou() -> None:
    result = intersection_from_iou(
        torch.tensor([[1 / 7]]), torch.tensor([4.0]), torch.tensor([4.0])
    )
    assert result.item() == pytest.approx(1.0)


def test_disjoint_region_fraction_sums_intersections() -> None:
    regions = torch.stack((quad_from_bbox([0, 0, 1, 2]), quad_from_bbox([1, 0, 2, 2])))
    proposal = quad_from_bbox([0, 0, 2, 2]).unsqueeze(0)
    overlaps = torch.tensor([[0.5], [0.5]])
    fraction = proposal_region_fraction(
        overlaps, regions, proposal, disjoint_regions=True
    )
    assert fraction.item() == pytest.approx(1.0)


def test_utility_reports_coverage_purity_and_false_budget() -> None:
    gt = quad_from_bbox([0, 0, 2, 2]).unsqueeze(0)
    proposals = torch.stack(
        (quad_from_bbox([0, 0, 2, 2]), quad_from_bbox([5, 5, 7, 7]))
    )
    accumulator = ProposalUtilityAccumulator()
    accumulator.update(
        gt_overlaps=torch.tensor([[1.0, 0.0]]),
        gt_quads=gt,
        proposal_quads=proposals,
        trusted_fraction=torch.tensor([0.0, 1.0]),
        ignore_fraction=torch.zeros(2),
        ego_fraction=torch.zeros(2),
        ego_present=False,
        slice_labels={"shape": ("regular",)},
    )

    metrics = accumulator.compute()

    assert metrics["ar/10"] == 1.0
    assert metrics["coverage/object_mean/100"] == 1.0
    assert metrics["proposal/object_fraction_mean"] == 0.5
    assert metrics["false/unmatched_fraction"] == 0.5
    assert metrics["false/trusted_background_fraction"] == 0.5
    assert metrics["false/ego_body_images"] == 0


def test_score_reliability_reports_object_and_false_rates_by_bin() -> None:
    accumulator = ScoreReliabilityAccumulator()
    accumulator.update(
        scores=torch.tensor([0.05, 0.15, 0.19, 1.0]),
        best_object_iou=torch.tensor([0.8, 0.0, 0.0, 0.6]),
        trusted_fraction=torch.tensor([0.0, 1.0, 0.0, 0.0]),
        ignore_fraction=torch.tensor([0.0, 0.0, 1.0, 0.0]),
    )

    bins = accumulator.compute()

    assert bins[0]["object_iou_50_rate"] == 1.0
    assert bins[1]["count"] == 2
    assert bins[1]["unmatched_rate"] == 0.5
    assert bins[1]["trusted_background_false_rate"] == 0.5
    assert bins[-1]["object_iou_50_rate"] == 1.0
