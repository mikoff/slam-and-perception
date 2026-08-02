from __future__ import annotations

from types import SimpleNamespace

import torch

from student_detector.head import QuadDetectorOutput
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_geometry import quad_from_bbox
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder


def _sample() -> SimpleNamespace:
    return SimpleNamespace(
        image=torch.zeros(3, 64, 64),
        quads=torch.stack((quad_from_bbox([8.0, 8.0, 40.0, 40.0]),)),
        ignore_quads=torch.empty((0, 4, 2)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        background_supervision=True,
    )


def test_quad_targets_and_loss_are_finite() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    targets = QuadTargetBuilder(QuadAssigner())([_sample()], shapes, device=torch.device("cpu"))
    output = QuadDetectorOutput(
        quality=tuple(torch.zeros(1, 1, h, w, requires_grad=True) for h, w in shapes),
        corner_offsets=tuple(torch.zeros(1, 8, h, w, requires_grad=True) for h, w in shapes),
    )
    result = QuadProposalLoss()(output, targets)
    assert torch.isfinite(result.total)
    result.total.backward()
    assert all(t.grad is not None for t in output.corner_offsets)


def test_corner_loss_is_invariant_to_target_winding() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    sample = _sample()
    reversed_sample = SimpleNamespace(**vars(sample))
    reversed_sample.quads = torch.flip(sample.quads, dims=(1,))
    builder = QuadTargetBuilder(QuadAssigner())
    first = builder([sample], shapes, device=torch.device("cpu"))
    second = builder([reversed_sample], shapes, device=torch.device("cpu"))
    output = QuadDetectorOutput(
        quality=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
        corner_offsets=tuple(torch.randn(1, 8, h, w) for h, w in shapes),
    )
    loss = QuadProposalLoss()
    torch.testing.assert_close(loss(output, first).corner, loss(output, second).corner)


def test_validity_loss_penalizes_collapsed_and_bow_tie_quads() -> None:
    loss = QuadProposalLoss()
    positive = torch.tensor([True])
    valid = torch.tensor([[[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]])
    collapsed = torch.zeros_like(valid)
    bow_tie = torch.tensor([[[-1.0, -1.0], [1.0, 1.0], [1.0, -1.0], [-1.0, 1.0]]])
    assert loss._validity_loss(collapsed, positive) > loss._validity_loss(valid, positive)
    assert loss._validity_loss(bow_tie, positive) > loss._validity_loss(valid, positive)


def test_quality_groups_are_normalized_independently() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    sample = _sample()
    builder = QuadTargetBuilder(QuadAssigner(), weak_negative_weight=0.25)
    targets = builder([sample], shapes, device=torch.device("cpu"))
    assert targets.trusted_background_mask.any()
    assert not targets.weak_background_mask.any()
