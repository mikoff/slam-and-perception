from __future__ import annotations

import torch
from torchvision.ops import box_iou

from student_detector.decoder import class_agnostic_nms
from student_detector.quad_geometry import pairwise_quad_iou, polygon_nms, quad_from_bbox
from student_detector.suppression import (
    SuppressionSweepAccumulator,
    greedy_nms_from_overlaps,
)


def test_overlap_replay_matches_optimized_hbb_nms() -> None:
    boxes = torch.tensor(
        [
            [0.0, 0.0, 10.0, 10.0],
            [1.0, 1.0, 11.0, 11.0],
            [20.0, 20.0, 30.0, 30.0],
        ]
    )
    scores = torch.tensor([0.9, 0.8, 0.7])
    expected = class_agnostic_nms(boxes, scores, 0.5)
    replayed = greedy_nms_from_overlaps(
        box_iou(boxes, boxes), scores, 0.5, max_output=100
    )
    torch.testing.assert_close(replayed, expected)


def test_overlap_replay_matches_exact_polygon_nms() -> None:
    quads = torch.stack(
        (
            quad_from_bbox([0.0, 0.0, 10.0, 10.0]),
            quad_from_bbox([1.0, 1.0, 11.0, 11.0]),
            quad_from_bbox([20.0, 20.0, 30.0, 30.0]),
        )
    )
    scores = torch.tensor([0.9, 0.8, 0.7])
    expected = polygon_nms(quads, scores, 0.5)
    replayed = greedy_nms_from_overlaps(
        pairwise_quad_iou(quads, quads), scores, 0.5, max_output=100
    )
    torch.testing.assert_close(replayed, expected)


def test_overlap_replay_has_stable_equal_score_order() -> None:
    overlaps = torch.eye(3)
    keep = greedy_nms_from_overlaps(
        overlaps, torch.ones(3), 0.5, max_output=2
    )
    assert keep.tolist() == [0, 1]


def test_sweep_accumulator_reports_recall_duplicates_and_pairs() -> None:
    gt_overlaps = torch.tensor([[0.9, 0.8, 0.0]])
    proposal_overlaps = torch.tensor(
        [[1.0, 0.7, 0.0], [0.7, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    accumulator = SuppressionSweepAccumulator()
    accumulator.update(gt_overlaps, proposal_overlaps, torch.tensor([0, 1, 2]))

    metrics = accumulator.compute()

    assert metrics["recall/10@0.50"] == 1.0
    assert metrics["ar/10"] == 0.9
    assert metrics["proposals/10_per_image"] == 3.0
    assert metrics["duplicates/10@0.50_fraction"] == 1 / 3
    assert metrics["duplicates/10@0.50_of_matched"] == 1 / 2
    assert metrics["pairwise/10@0.50_fraction"] == 1 / 3
