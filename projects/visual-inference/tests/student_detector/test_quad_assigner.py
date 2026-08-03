from __future__ import annotations

import torch

from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_geometry import quad_from_bbox


FEATURE_SHAPES = ((48, 48), (24, 24), (12, 12))


def test_quad_assigner_handles_rotated_and_long_objects() -> None:
    rectangle = quad_from_bbox([40.0, 40.0, 160.0, 160.0])
    long_quad = torch.tensor([[40.0, 180.0], [340.0, 220.0], [338.0, 240.0], [38.0, 200.0]])
    assignment = QuadAssigner().assign(
        torch.stack((rectangle, long_quad)), FEATURE_SHAPES, (384, 384)
    )
    assert assignment.positive_mask.any()
    assert assignment.unrepresentable_gt_indices.numel() == 0
    assert assignment.corner_targets.shape[-1] == 8
    assert torch.isfinite(assignment.corner_targets).all()


def test_quad_assignment_is_invariant_to_winding_and_start() -> None:
    quad = quad_from_bbox([40.0, 40.0, 160.0, 160.0])
    first = QuadAssigner().assign(quad.unsqueeze(0), FEATURE_SHAPES, (384, 384))
    rotated = torch.roll(torch.flip(quad, dims=(0,)), 1, dims=0)
    second = QuadAssigner().assign(rotated.unsqueeze(0), FEATURE_SHAPES, (384, 384))
    assert first.positive_mask.equal(second.positive_mask)
    torch.testing.assert_close(first.quality_targets, second.quality_targets)


def test_quad_assignment_reports_unrepresentable_tiny_quad() -> None:
    quad = quad_from_bbox([0.5, 0.5, 1.5, 1.5])
    assignment = QuadAssigner().assign(quad.unsqueeze(0), FEATURE_SHAPES, (384, 384))
    assert assignment.positive_mask.sum() == 0
    assert assignment.unrepresentable_gt_indices.tolist() == [0]


def test_fallback_positive_keeps_nonzero_quality_target() -> None:
    quad = quad_from_bbox([3.5, 3.5, 20.5, 20.5])
    assignment = QuadAssigner(top_k=1, eligible_levels=1).assign(
        torch.stack((quad, quad)), FEATURE_SHAPES, (384, 384)
    )
    assert assignment.positive_mask.sum() == 2
    assert assignment.fallback_gt_indices.tolist() == [1]
    assert (assignment.quality_targets[assignment.positive_mask] > 0).all()


def test_quality_target_is_centerness_not_scale_compatibility() -> None:
    quad = quad_from_bbox([32.0, 32.0, 352.0, 352.0])
    assignment = QuadAssigner(eligible_levels=1).assign(
        quad.unsqueeze(0), FEATURE_SHAPES, (384, 384)
    )
    # The point nearest the object centroid should retain a near-one quality
    # target even though assignment level selection uses scale compatibility.
    assert assignment.quality_targets.max() > 0.95


def test_maximum_extent_moves_slender_target_to_coarser_levels() -> None:
    quad = torch.tensor([
        [32.0, 100.0], [192.0, 100.0], [192.0, 116.0], [32.0, 116.0]
    ]).unsqueeze(0)
    shapes = ((32, 32), (16, 16), (8, 8))
    area = QuadAssigner(scale_measure="area").assign(quad, shapes, (256, 256))
    extent = QuadAssigner(scale_measure="maximum_extent").assign(
        quad, shapes, (256, 256)
    )
    assert area.positive_counts_per_level[2] == 0
    assert extent.positive_counts_per_level[2] > 0
