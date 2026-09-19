from __future__ import annotations

import pytest
import torch

from student_detector.proposal_diversity import ProposalDiversityAccumulator
from student_detector.quad_geometry import quad_from_bbox


def test_diversity_reports_overlap_clusters_and_unmatched_isolation() -> None:
    proposals = torch.stack(
        (
            quad_from_bbox([0.0, 0.0, 10.0, 10.0]),
            quad_from_bbox([0.5, 0.5, 10.5, 10.5]),
            quad_from_bbox([80.0, 80.0, 90.0, 90.0]),
        )
    )
    pair_overlaps = torch.tensor([[1.0, 0.82, 0.0], [0.82, 1.0, 0.0], [0.0, 0.0, 1.0]])
    gt_overlaps = torch.tensor([[0.9, 0.8, 0.0]])
    accumulator = ProposalDiversityAccumulator()

    accumulator.update(
        proposals,
        gt_overlaps,
        pair_overlaps,
        image_size=100,
    )
    metrics = accumulator.compute()

    assert metrics["proposals_per_image"] == 3.0
    assert metrics["pairwise"]["iou_ge_0.75_fraction"] == pytest.approx(1 / 3)
    assert metrics["clusters"]["iou_0.75"]["largest_size"]["max"] == 2.0
    assert metrics["unmatched"]["count"] == 1
    assert metrics["unmatched"]["isolated_iou_lt_0.10_fraction"] == 1.0
    assert metrics["unmatched"]["within_unmatched"]["pairwise"]["iou_mean"] == 0.0
    assert (
        metrics["unmatched"]["within_unmatched"]["clusters"]["iou_0.75"][
            "largest_size"
        ]["max"]
        == 1.0
    )
    assert (
        metrics["unmatched"]["within_unmatched"]["spatial_grid_8"]["occupied_cells"][
            "mean"
        ]
        == 1.0
    )
    assert metrics["overlap_75_pairs"]["area_ratio"]["mean"] > 0.8


def test_diversity_rejects_inconsistent_overlap_shapes() -> None:
    accumulator = ProposalDiversityAccumulator()
    with pytest.raises(ValueError, match="pair overlaps"):
        accumulator.update(
            torch.empty((1, 4, 2)),
            torch.empty((0, 1)),
            torch.empty((0, 0)),
            image_size=100,
        )
