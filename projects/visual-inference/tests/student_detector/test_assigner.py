from __future__ import annotations

import torch

from student_detector.assigner import ATSSAssigner


FEATURE_SHAPES = ((48, 48), (24, 24), (12, 12))


def test_atss_assigns_every_representable_valid_gt() -> None:
    boxes = torch.tensor(
        [
            [31.0, 41.0, 91.0, 121.0],
            [140.0, 100.0, 300.0, 330.0],
            [370.0, 10.0, 390.0, 30.0],  # valid intersection with the image
            [10.0, 10.0, 10.0, 20.0],  # invalid zero-width box
        ]
    )
    assignment = ATSSAssigner().assign(boxes, FEATURE_SHAPES, (384, 384))

    assert assignment.valid_gt_mask.tolist() == [True, True, True, False]
    assert assignment.unrepresentable_gt_indices.numel() == 0
    for gt_index in (0, 1, 2):
        assert torch.any(assignment.matched_gt_indices == gt_index)
    assert torch.all(assignment.box_targets[assignment.positive_mask] >= 0)
    assert torch.all(assignment.centerness_targets >= 0)
    assert torch.all(assignment.centerness_targets <= 1)


def test_atss_fallback_reports_box_with_no_inside_grid_point() -> None:
    # P3 starts at (4, 4), so this valid 1x1 box contains no model location.
    boxes = torch.tensor([[0.5, 0.5, 1.5, 1.5]])
    assignment = ATSSAssigner().assign(boxes, FEATURE_SHAPES, (384, 384))
    assert assignment.positive_mask.sum() == 0
    assert assignment.unrepresentable_gt_indices.tolist() == [0]


def test_overlap_conflict_assigns_each_point_to_one_gt() -> None:
    boxes = torch.tensor([[40.0, 40.0, 160.0, 160.0], [60.0, 60.0, 180.0, 180.0]])
    assignment = ATSSAssigner().assign(boxes, FEATURE_SHAPES, (384, 384))
    assert set(assignment.matched_gt_indices[assignment.positive_mask].tolist()) == {0, 1}
