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


def test_invalid_locations_are_never_assigned():
    boxes = torch.tensor([[40.0, 40.0, 160.0, 160.0]])
    valid = torch.ones(48 * 48 + 24 * 24 + 12 * 12, dtype=torch.bool)
    valid[: 48 * 48] = False
    assignment = ATSSAssigner().assign(
        boxes, FEATURE_SHAPES, (384, 384), valid
    )
    assert not assignment.positive_mask[: 48 * 48].any()
    assert assignment.positive_mask.any()


def test_atss_assigns_tall_vertical_pillars_and_columns() -> None:
    p2_shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    assigner = ATSSAssigner(
        strides=(4, 8, 16, 32),
        prior_sizes=(32, 64, 128, 256),
        top_k=4,
        center_radius=1.0,
    )
    boxes = torch.tensor(
        [
            [61.0, 40.0, 71.0, 280.0],   # Narrow pillar (10x240)
            [100.0, 30.0, 120.0, 280.0], # Tall column (20x250)
        ]
    )
    assignment = assigner.assign(boxes, p2_shapes, (384, 384))
    assert (assignment.matched_gt_indices == 0).sum().item() >= 4
    assert (assignment.matched_gt_indices == 1).sum().item() >= 6
    # Column (20x250) should route to P4 and P5 (levels 2 and 3)
    assert assignment.positive_counts_per_level[2].item() >= 4
    assert assignment.positive_counts_per_level[3].item() >= 4

