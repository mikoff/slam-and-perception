from __future__ import annotations

import pytest
import torch

from student_detector.backbone import (
    EXPECTED_CHANNELS,
    P2_EXPECTED_CHANNELS,
    MobileNetV4Backbone,
    StubP2Backbone,
    StubBackbone,
    locate_pyramid_levels,
)
from student_detector.model import QuadProposalDetector, StudentDetector


def test_locate_pyramid_levels_uses_reductions_not_fixed_indices() -> None:
    positions, channels = locate_pyramid_levels(
        reductions=(2, 4, 8, 8, 16, 32),
        channels=(16, 24, 48, 80, 160, 256),
    )
    assert positions == (3, 4, 5)
    assert channels == EXPECTED_CHANNELS


def test_locate_pyramid_levels_reports_missing_stride() -> None:
    with pytest.raises(ValueError, match="stride-32"):
        locate_pyramid_levels((2, 4, 8, 16), (16, 24, 80, 160))


@pytest.mark.parametrize(
    ("image_size", "spatial_shapes"),
    [
        (320, ((40, 40), (20, 20), (10, 10))),
        (384, ((48, 48), (24, 24), (12, 12))),
    ],
)
def test_student_detector_fixed_shapes(
    image_size: int, spatial_shapes: tuple[tuple[int, int], ...]
) -> None:
    model = StudentDetector(backbone=StubBackbone()).eval()
    with torch.inference_mode():
        output = model(torch.randn(1, 3, image_size, image_size))

    for tensor, (height, width) in zip(output.objectness, spatial_shapes, strict=True):
        assert tensor.shape == (1, 1, height, width)
    for tensor, (height, width) in zip(
        output.box_distances, spatial_shapes, strict=True
    ):
        assert tensor.shape == (1, 4, height, width)
        assert torch.all(tensor >= 0)
    for tensor, (height, width) in zip(output.centerness, spatial_shapes, strict=True):
        assert tensor.shape == (1, 1, height, width)


def test_p2_student_detector_has_four_outputs() -> None:
    model = StudentDetector(backbone=StubP2Backbone(), strides=(4, 8, 16, 32)).eval()
    with torch.inference_mode():
        output = model(torch.randn(1, 3, 64, 64))

    assert [tensor.shape[-2:] for tensor in output.objectness] == [
        (16, 16),
        (8, 8),
        (4, 4),
        (2, 2),
    ]
    assert len(model.head.scales) == 4


def test_p2_stub_model_exports() -> None:
    model = StudentDetector(backbone=StubP2Backbone(), strides=(4, 8, 16, 32)).eval()
    example = torch.randn(1, 3, 64, 64)
    exported = torch.export.export(model, (example,))
    result = exported.module()(example)
    assert result.objectness[0].shape == (1, 1, 16, 16)


def test_p2_control_shares_identical_p3_p5_and_head_initialization() -> None:
    p3_backbone = StubBackbone()
    p2_backbone = StubP2Backbone()
    torch.manual_seed(123)
    p3 = StudentDetector(backbone=p3_backbone, head_seed=42)
    torch.manual_seed(123)
    p2 = StudentDetector(backbone=p2_backbone, strides=(4, 8, 16, 32), head_seed=42)

    for name in ("lateral3", "lateral4", "lateral5", "output3", "output4", "output5"):
        p3_values = getattr(p3.fpn, name).state_dict()
        p2_values = getattr(p2.fpn, name).state_dict()
        for key in p3_values:
            torch.testing.assert_close(p3_values[key], p2_values[key])
    for key, value in p3.head.state_dict().items():
        if key != "scales":
            torch.testing.assert_close(value, p2.head.state_dict()[key])


def test_prediction_towers_are_shared_across_levels() -> None:
    model = StudentDetector(backbone=StubBackbone())
    assert len(model.head.scales) == 3
    assert len(model.head.regression_tower) == 2
    torch.testing.assert_close(
        model.head.box_regression.bias,
        torch.ones_like(model.head.box_regression.bias),
    )
    assert (
        sum(1 for name, _ in model.head.named_modules() if name == "object_tower") == 1
    )
    assert (
        sum(1 for name, _ in model.head.named_modules() if name == "regression_tower")
        == 1
    )


def test_stub_model_exports() -> None:
    model = StudentDetector(backbone=StubBackbone()).eval()
    example = torch.randn(1, 3, 384, 384)
    exported = torch.export.export(model, (example,))
    result = exported.module()(example)
    assert result.objectness[0].shape == (1, 1, 48, 48)


def test_quad_model_has_one_quality_and_eight_geometry_channels() -> None:
    model = QuadProposalDetector(backbone=StubBackbone()).eval()
    with torch.inference_mode():
        output = model(torch.randn(2, 3, 64, 64))
    assert [tensor.shape[1] for tensor in output.quality] == [1, 1, 1]
    assert [tensor.shape[1] for tensor in output.corner_offsets] == [8, 8, 8]


@pytest.mark.slow
def test_real_mobilenet_feature_shapes() -> None:
    backbone = MobileNetV4Backbone(pretrained=False).eval()
    assert backbone.out_channels == EXPECTED_CHANNELS
    with torch.inference_mode():
        outputs = backbone(torch.randn(1, 3, 384, 384))
    assert [tuple(output.shape) for output in outputs] == [
        (1, 80, 48, 48),
        (1, 160, 24, 24),
        (1, 256, 12, 12),
    ]


@pytest.mark.slow
def test_real_mobilenet_p2_feature_shapes() -> None:
    backbone = MobileNetV4Backbone(
        pretrained=False, desired_reductions=(4, 8, 16, 32)
    ).eval()
    assert backbone.out_channels == P2_EXPECTED_CHANNELS
    with torch.inference_mode():
        outputs = backbone(torch.randn(1, 3, 384, 384))
    assert [tuple(output.shape) for output in outputs] == [
        (1, 48, 96, 96),
        (1, 80, 48, 48),
        (1, 160, 24, 24),
        (1, 256, 12, 12),
    ]


@pytest.mark.slow
def test_real_detector_exports() -> None:
    model = StudentDetector(pretrained_backbone=False).eval()
    example = torch.randn(1, 3, 384, 384)
    exported = torch.export.export(model, (example,))
    result = exported.module()(example)
    assert [tensor.shape[-2:] for tensor in result.objectness] == [
        (48, 48),
        (24, 24),
        (12, 12),
    ]


def test_attn_res_neck_feature_shapes() -> None:
    model = QuadProposalDetector(backbone=StubBackbone(), neck_type="attn_res").eval()
    with torch.inference_mode():
        output = model(torch.randn(2, 3, 64, 64))
    assert [tensor.shape[1] for tensor in output.quality] == [1, 1, 1]
    assert [tensor.shape[1] for tensor in output.corner_offsets] == [8, 8, 8]


@pytest.mark.slow
def test_attn_res_quad_detector_exports() -> None:
    model = QuadProposalDetector(pretrained_backbone=False, neck_type="attn_res").eval()
    example = torch.randn(1, 3, 384, 384)
    exported = torch.export.export(model, (example,))
    result = exported.module()(example)
    assert [tensor.shape[-2:] for tensor in result.quality] == [
        (48, 48),
        (24, 24),
        (12, 12),
    ]
