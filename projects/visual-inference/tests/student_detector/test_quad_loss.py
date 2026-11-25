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
        trusted_background_quads=torch.stack((quad_from_bbox([0.0, 0.0, 64.0, 64.0]),)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        background_supervision=True,
    )


def test_quad_targets_and_loss_are_finite() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    targets = QuadTargetBuilder(QuadAssigner())(
        [_sample()], shapes, device=torch.device("cpu")
    )
    output = QuadDetectorOutput(
        quality=tuple(torch.zeros(1, 1, h, w, requires_grad=True) for h, w in shapes),
        corner_offsets=tuple(
            torch.zeros(1, 8, h, w, requires_grad=True) for h, w in shapes
        ),
    )
    result = QuadProposalLoss()(output, targets)
    assert torch.isfinite(result.total)
    result.total.backward()
    assert all(t.grad is not None for t in output.corner_offsets)


def test_quad_positive_overrides_coincident_ignore_and_trusted_background() -> None:
    sample = _sample()
    sample.ignore_quads = torch.stack((quad_from_bbox([0.0, 0.0, 64.0, 64.0]),))

    targets = QuadTargetBuilder(QuadAssigner())(
        [sample], ((8, 8), (4, 4), (2, 2)), device=torch.device("cpu")
    )

    assert targets.positive_mask.any()
    assert not targets.trusted_background_mask.any()
    assert not targets.weak_background_mask.any()


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


def test_smaller_corner_beta_preserves_more_pressure_on_small_residuals() -> None:
    predicted = torch.full((1, 8), 0.05)
    target = torch.zeros_like(predicted)
    positive = torch.tensor([True])
    default = QuadProposalLoss(corner_smooth_l1_beta=1.0)
    precise = QuadProposalLoss(corner_smooth_l1_beta=0.1)
    assert precise._corner_loss(predicted, target, positive) > default._corner_loss(
        predicted, target, positive
    )


def test_validity_loss_penalizes_collapsed_and_bow_tie_quads() -> None:
    loss = QuadProposalLoss()
    positive = torch.tensor([True])
    valid = torch.tensor([[[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]])
    collapsed = torch.zeros_like(valid)
    bow_tie = torch.tensor([[[-1.0, -1.0], [1.0, 1.0], [1.0, -1.0], [-1.0, 1.0]]])
    assert loss._validity_loss(collapsed, positive) > loss._validity_loss(
        valid, positive
    )
    assert loss._validity_loss(bow_tie, positive) > loss._validity_loss(valid, positive)


def test_gwd_auxiliary_is_zero_for_identical_quads_and_permutation_invariant() -> None:
    target = torch.tensor([[[-1.0, -0.5], [1.0, -0.5], [1.0, 0.5], [-1.0, 0.5]]])
    positive = torch.tensor([True])
    identical = QuadProposalLoss._gwd_loss(target, target, positive)
    permuted = QuadProposalLoss._gwd_loss(
        torch.roll(target, 2, dims=1), target, positive
    )
    torch.testing.assert_close(
        identical, torch.zeros_like(identical), atol=1e-5, rtol=0
    )
    torch.testing.assert_close(permuted, identical, atol=1e-5, rtol=0)


def test_gwd_auxiliary_has_finite_geometry_gradients() -> None:
    target = torch.tensor([[[-2.0, -0.5], [2.0, -0.5], [2.0, 0.5], [-2.0, 0.5]]])
    predicted = (target + torch.tensor([0.5, 0.25])).requires_grad_()
    positive = torch.tensor([True])
    shifted = QuadProposalLoss._gwd_loss(predicted, target, positive)
    identical = QuadProposalLoss._gwd_loss(target, target, positive)
    assert shifted > identical
    shifted.backward()
    assert predicted.grad is not None
    assert torch.isfinite(predicted.grad).all()


def test_zero_gwd_weight_skips_auxiliary_computation(monkeypatch) -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    targets = QuadTargetBuilder(QuadAssigner())(
        [_sample()], shapes, device=torch.device("cpu")
    )
    output = QuadDetectorOutput(
        quality=tuple(torch.zeros(1, 1, h, w) for h, w in shapes),
        corner_offsets=tuple(torch.zeros(1, 8, h, w) for h, w in shapes),
    )

    def unexpected(*_args, **_kwargs):
        raise AssertionError("disabled GWD branch was evaluated")

    monkeypatch.setattr(QuadProposalLoss, "_gwd_loss", unexpected)
    result = QuadProposalLoss(gwd_weight=0)(output, targets)
    assert result.gwd == 0


def test_quality_groups_are_normalized_independently() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    sample = _sample()
    builder = QuadTargetBuilder(QuadAssigner(), weak_negative_weight=0.25)
    targets = builder([sample], shapes, device=torch.device("cpu"))
    assert targets.trusted_background_mask.any()
    assert not targets.weak_background_mask.any()


def test_corner_quality_proxy_is_scale_invariant_and_detached() -> None:
    target = torch.tensor([[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]])
    predicted = target + 1.0
    first = QuadProposalLoss._corner_quality_proxy(predicted, target)
    second = QuadProposalLoss._corner_quality_proxy(predicted * 2, target * 2)
    torch.testing.assert_close(first, second)
    assert 0 < float(first) < 1


def test_corner_quality_proxy_is_sensitive_to_thin_minor_axis_error() -> None:
    target = torch.tensor([[[0.0, 0.0], [100.0, 0.0], [100.0, 10.0], [0.0, 10.0]]])
    major_error = target + torch.tensor([1.0, 0.0])
    minor_error = target + torch.tensor([0.0, 1.0])
    major_quality = QuadProposalLoss._corner_quality_proxy(major_error, target)
    minor_quality = QuadProposalLoss._corner_quality_proxy(minor_error, target)
    assert minor_quality < major_quality


def test_quad_target_builder_ignores_unmatched_gt_interior() -> None:
    shapes = ((8, 8), (4, 4), (2, 2))
    sample = _sample()
    default_targets = QuadTargetBuilder(
        QuadAssigner(top_k=1, eligible_levels=1),
        ignore_unmatched_gt_interior=False,
    )([sample], shapes, device=torch.device("cpu"))
    guarded_targets = QuadTargetBuilder(
        QuadAssigner(top_k=1, eligible_levels=1),
        ignore_unmatched_gt_interior=True,
    )([sample], shapes, device=torch.device("cpu"))
    assert (
        guarded_targets.trusted_background_mask.sum()
        < default_targets.trusted_background_mask.sum()
    )


def test_corner_loss_preserves_horizontal_gradient_on_tall_vertical_quad() -> None:
    loss = QuadProposalLoss()
    # One square quad (4x4) and one tall slender quad (1x12) in stride units
    target = torch.tensor(
        [
            [-2.0, -2.0, 2.0, -2.0, 2.0, 2.0, -2.0, 2.0],
            [-0.5, -6.0, 0.5, -6.0, 0.5, 6.0, -0.5, 6.0],
        ]
    )
    positive = torch.tensor([True, True])
    x_shift = torch.tensor([0.2, 0.0] * 4)
    y_shift = torch.tensor([0.0, 0.2] * 4)
    loss_x = loss._corner_loss(
        target + torch.stack([torch.zeros(8), x_shift]), target, positive
    )
    loss_y = loss._corner_loss(
        target + torch.stack([torch.zeros(8), y_shift]), target, positive
    )
    assert loss_x > loss_y


def test_quad_target_builder_respects_max_one_to_one_interior_area_frac() -> None:
    from student_detector.task_aligned_assigner import QuadTaskAlignedAssigner

    shapes = ((8, 8), (4, 4), (2, 2))
    # 40x40 quad covers 1600 / 4096 = 39.1% of the 64x64 image.
    sample = SimpleNamespace(
        image=torch.zeros(3, 64, 64),
        quads=torch.stack((quad_from_bbox([12.0, 12.0, 52.0, 52.0]),)),
        ignore_quads=torch.empty((0, 4, 2)),
        trusted_background_quads=torch.empty((0, 4, 2)),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
    )
    assigner = QuadTaskAlignedAssigner(strides=(8, 16, 32), assignment_mode="one_to_one")
    default_targets = QuadTargetBuilder(assigner)(
        [sample], shapes, device=torch.device("cpu")
    )
    capped_targets = QuadTargetBuilder(
        assigner, max_one_to_one_interior_area_frac=0.20
    )([sample], shapes, device=torch.device("cpu"))
    assert default_targets.trusted_background_mask.any()
    assert not capped_targets.trusted_background_mask.any()
