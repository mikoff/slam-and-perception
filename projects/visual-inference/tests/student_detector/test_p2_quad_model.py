"""Unit tests for P2-stride (stride 4) QuadProposalDetector, assigner, and decoder."""

from __future__ import annotations

import torch

from student_detector.backbone import StubP2Backbone
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder


def test_p2_quad_proposal_detector_output_shapes() -> None:
    model = QuadProposalDetector(
        backbone=StubP2Backbone(), strides=(4, 8, 16, 32)
    ).eval()
    with torch.inference_mode():
        output = model(torch.randn(1, 3, 64, 64))

    assert len(output.quality) == 4
    assert len(output.corner_offsets) == 4
    expected_spatial = [(16, 16), (8, 8), (4, 4), (2, 2)]
    for i, shape in enumerate(expected_spatial):
        assert output.quality[i].shape == (1, 1, shape[0], shape[1])
        assert output.corner_offsets[i].shape == (1, 8, shape[0], shape[1])


def test_p2_quad_assigner_allocates_small_objects_to_stride_4() -> None:
    assigner = QuadAssigner(strides=(4, 8, 16, 32))
    # Small object: 12x12 pixels centered at (50, 50)
    small_quad = torch.tensor(
        [[[44.0, 44.0], [56.0, 44.0], [56.0, 56.0], [44.0, 56.0]]]
    )
    feature_shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    assignment = assigner.assign(
        gt_quads=small_quad,
        feature_shapes=feature_shapes,
        image_size=(384, 384),
    )

    assert assignment.valid_gt_mask.item() is True
    assert assignment.positive_mask.any()
    # Level 0 (stride 4) has 96*96 = 9216 points
    stride4_slice = slice(0, 96 * 96)
    assert assignment.positive_mask[stride4_slice].any()
    assert assignment.positive_counts_per_level[0] > 0


def test_p2_quad_decoder_and_loss_forward() -> None:
    model = QuadProposalDetector(backbone=StubP2Backbone(), strides=(4, 8, 16, 32))
    images = torch.randn(2, 3, 64, 64)
    output = model(images)

    decoder = QuadInferenceDecoder(strides=(4, 8, 16, 32))
    detections = decoder(output, image_size=(64, 64))
    assert len(detections) == 2
    assert detections[0].boxes.ndim == 2

    assigner = QuadAssigner(strides=(4, 8, 16, 32))
    builder = QuadTargetBuilder(assigner)

    class DummySample:
        def __init__(self) -> None:
            self.image = torch.zeros(3, 64, 64)
            self.quads = torch.tensor(
                [[[10.0, 10.0], [20.0, 10.0], [20.0, 20.0], [10.0, 20.0]]]
            )
            self.valid_mask = torch.ones(64, 64, dtype=torch.bool)

    samples = [DummySample(), DummySample()]
    feature_shapes = ((16, 16), (8, 8), (4, 4), (2, 2))
    targets = builder(samples, feature_shapes, device=torch.device("cpu"))

    criterion = QuadProposalLoss(strides=(4, 8, 16, 32))
    loss_output = criterion(output, targets)
    assert loss_output.total.ndim == 0
    assert torch.isfinite(loss_output.total)
    loss_output.total.backward()

    # Verify gradients flowed into backbone
    assert model.backbone.c2.weight.grad is not None
