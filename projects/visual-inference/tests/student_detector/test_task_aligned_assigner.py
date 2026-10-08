"""Unit tests for Task-Aligned Assigner (TAL) and Dual 1-to-1 Assignment."""

from __future__ import annotations

import torch

from student_detector.quad_geometry import quad_from_bbox
from student_detector.task_aligned_assigner import (
    QuadTaskAlignedAssigner,
    TaskAlignedAssigner,
    greedy_bipartite_matching,
)


def test_greedy_bipartite_matching_uniqueness() -> None:
    # 5 candidates, 3 GTs
    scores = torch.tensor(
        [
            [0.9, 0.1, 0.2],
            [0.8, 0.7, 0.1],
            [0.1, 0.95, 0.3],
            [0.2, 0.3, 0.85],
            [0.5, 0.4, 0.6],
        ]
    )
    matches = greedy_bipartite_matching(scores)
    assert len(matches) == 3
    matched_cands = [m[0] for m in matches]
    matched_gts = [m[1] for m in matches]

    # Exactly 1-to-1: no duplicates
    assert len(set(matched_cands)) == 3
    assert len(set(matched_gts)) == 3

    # Best score for GT 1 is candidate 2 (0.95)
    assert (2, 1) in matches


def test_hbb_task_aligned_assigner_one_to_one_vs_one_to_many() -> None:
    gt_boxes = torch.tensor(
        [
            [10.0, 10.0, 50.0, 50.0],
            [100.0, 100.0, 200.0, 200.0],
        ]
    )
    feature_shapes = ((16, 16), (8, 8), (4, 4))
    image_size = (128, 128)

    # 1-to-1 mode: exactly 1 positive anchor per GT
    assigner_1to1 = TaskAlignedAssigner(
        strides=(8, 16, 32),
        assignment_mode="one_to_one",
    )
    assignment_1to1 = assigner_1to1.assign(gt_boxes, feature_shapes, image_size)
    assert assignment_1to1.positive_mask.sum().item() == 2
    assert (assignment_1to1.matched_gt_indices == 0).sum().item() == 1
    assert (assignment_1to1.matched_gt_indices == 1).sum().item() == 1

    # 1-to-many mode: top-k positives per GT
    assigner_many = TaskAlignedAssigner(
        strides=(8, 16, 32),
        top_k=4,
        assignment_mode="one_to_many",
    )
    assignment_many = assigner_many.assign(gt_boxes, feature_shapes, image_size)
    assert assignment_many.positive_mask.sum().item() >= 2
    assert (assignment_many.matched_gt_indices == 0).sum().item() <= 16
    assert (assignment_many.matched_gt_indices == 1).sum().item() <= 16


def test_quad_task_aligned_assigner_one_to_one_vs_one_to_many() -> None:
    q1 = quad_from_bbox([10.0, 10.0, 50.0, 50.0])
    q2 = quad_from_bbox([100.0, 100.0, 200.0, 200.0])
    gt_quads = torch.stack([q1, q2])
    feature_shapes = ((16, 16), (8, 8), (4, 4))
    image_size = (128, 128)

    assigner_1to1 = QuadTaskAlignedAssigner(
        strides=(8, 16, 32),
        assignment_mode="one_to_one",
    )
    assignment_1to1 = assigner_1to1.assign(gt_quads, feature_shapes, image_size)
    assert assignment_1to1.positive_mask.sum().item() == 2
    assert (assignment_1to1.matched_gt_indices == 0).sum().item() == 1
    assert (assignment_1to1.matched_gt_indices == 1).sum().item() == 1

    assigner_many = QuadTaskAlignedAssigner(
        strides=(8, 16, 32),
        top_k=4,
        assignment_mode="one_to_many",
    )
    assignment_many = assigner_many.assign(gt_quads, feature_shapes, image_size)
    assert assignment_many.positive_mask.sum().item() >= 2
    assert (assignment_many.matched_gt_indices == 0).sum().item() <= 16
    assert (assignment_many.matched_gt_indices == 1).sum().item() <= 16


def test_scale_gated_fpn_specialization() -> None:
    # Small box: 20x20 (scale=20) -> should prefer P2 (stride 4) or P3 (stride 8)
    # Huge box: 300x200 (scale=245) -> should prefer P4 (stride 16) or P5 (stride 32)
    gt_boxes = torch.tensor(
        [
            [10.0, 10.0, 30.0, 30.0],
            [10.0, 10.0, 310.0, 210.0],
        ]
    )
    feature_shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    image_size = (384, 384)

    assigner = TaskAlignedAssigner(
        strides=(4, 8, 16, 32),
        assignment_mode="one_to_one",
    )
    assignment = assigner.assign(gt_boxes, feature_shapes, image_size)
    assert assignment.positive_mask.sum().item() == 2

    # Check level distribution: small box on level 0 or 1, huge box on level 2 or 3
    counts = assignment.positive_counts_per_level
    assert counts[0] + counts[1] >= 1  # small box on P2 or P3
    assert counts[2] + counts[3] >= 1  # huge box on P4 or P5

