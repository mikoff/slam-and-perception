from __future__ import annotations

import pytest
import torch

from student_detector.head import QuadDetectorOutput
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_evaluation import (
    QuadEvaluationAccumulator,
    QuadEvaluationImage,
    evaluate_quad_proposals,
)
from student_detector.quad_geometry import polygon_nms, quad_from_bbox


def test_quad_decoder_caps_and_suppresses_candidates() -> None:
    shapes = ((4, 4), (2, 2), (1, 1))
    quality = tuple(torch.full((1, 1, h, w), -10.0) for h, w in shapes)
    offsets = tuple(torch.zeros((1, 8, h, w)) for h, w in shapes)
    quality[0][0, 0, 1, 1] = 10.0
    output = QuadDetectorOutput(quality, offsets)
    detections = QuadInferenceDecoder(pre_nms_top_k=10, max_proposals=3)(
        output, (32, 32)
    )
    assert len(detections) == 1
    assert detections[0].quads.shape[0] <= 3
    assert detections[0].quads.shape[-2:] == (4, 2)
    assert detections[0].pre_nms_quads is not None
    assert detections[0].levels is not None
    assert detections[0].location_indices is not None
    assert detections[0].levels.shape == detections[0].scores.shape
    assert detections[0].location_indices.shape == detections[0].scores.shape


def test_quad_evaluation_reports_fixed_budget_average_recall() -> None:
    quad = quad_from_bbox([4.0, 4.0, 12.0, 12.0])
    detection = type(
        "D", (), {"quads": quad.unsqueeze(0), "scores": torch.tensor([1.0])}
    )()
    image = QuadEvaluationImage(
        image_id=1,
        domain="general",
        camera_type="perspective",
        image_size=(16, 16),
        ground_truth=quad.unsqueeze(0),
        ignore_quads=torch.empty((0, 4, 2)),
        detection=detection,
    )
    metrics = evaluate_quad_proposals([image])
    assert metrics["ar/100"] == 1.0
    assert metrics["recall/100@0.50"] == 1.0


def test_quad_evaluation_accumulates_batches_without_changing_metrics() -> None:
    quad = quad_from_bbox([4.0, 4.0, 12.0, 12.0])
    detection = type(
        "D",
        (),
        {
            "quads": quad.unsqueeze(0),
            "scores": torch.tensor([0.8]),
            "candidate_count": 2,
            "invalid_candidate_count": 0,
        },
    )()
    images = [
        QuadEvaluationImage(
            image_id=image_id,
            domain="general",
            camera_type="perspective",
            image_size=(16, 16),
            ground_truth=quad.unsqueeze(0),
            ignore_quads=torch.empty((0, 4, 2)),
            detection=detection,
            geometry_tiers=("easy",),
            positive_scores=torch.tensor([0.8]),
            trusted_background_scores=torch.tensor([0.1, 0.2]),
        )
        for image_id in (1, 2)
    ]

    expected = evaluate_quad_proposals(images, log_interval=0)
    accumulator = QuadEvaluationAccumulator(log_interval=0)
    accumulator.update(images[:1])
    accumulator.update(images[1:])

    assert accumulator.compute() == expected
    assert expected["score/positive/count"] == 2.0
    assert expected["score/positive/p50"] == pytest.approx(0.8, abs=2e-5)
    assert expected["score/trusted_background/count"] == 4.0
    assert expected["score/trusted_background/p50"] == pytest.approx(0.15, abs=2e-5)


def test_quad_evaluation_reports_duplicate_and_pairwise_overlap_rates() -> None:
    target = quad_from_bbox([0.0, 0.0, 10.0, 10.0])
    distant = quad_from_bbox([20.0, 20.0, 30.0, 30.0])
    image = QuadEvaluationImage(
        image_id=1,
        domain="general",
        camera_type="perspective",
        image_size=(32, 32),
        ground_truth=target.unsqueeze(0),
        ignore_quads=torch.empty((0, 4, 2)),
        detection=type(
            "D",
            (),
            {
                "quads": torch.stack((target, target, distant)),
                "scores": torch.tensor([0.9, 0.8, 0.7]),
            },
        )(),
    )

    metrics = evaluate_quad_proposals([image], log_interval=0)

    assert metrics["proposals/10_per_image"] == 3.0
    assert metrics["duplicates/10@0.50_fraction"] == pytest.approx(1 / 3)
    assert metrics["duplicates/10@0.50_of_matched"] == pytest.approx(1 / 2)
    assert metrics["pairwise/10@0.50_fraction"] == pytest.approx(1 / 3)
    assert metrics["pairwise/10@0.75_fraction"] == pytest.approx(1 / 3)


def test_quad_evaluation_progress_identifies_weight_state(capsys) -> None:
    quad = quad_from_bbox([4.0, 4.0, 12.0, 12.0])
    detection = type(
        "D", (), {"quads": quad.unsqueeze(0), "scores": torch.tensor([0.8])}
    )()
    image = QuadEvaluationImage(
        image_id=1,
        domain="general",
        camera_type="perspective",
        image_size=(16, 16),
        ground_truth=quad.unsqueeze(0),
        ignore_quads=torch.empty((0, 4, 2)),
        detection=detection,
    )

    accumulator = QuadEvaluationAccumulator(log_interval=1, state="raw")
    accumulator.update([image])

    assert "state=raw aggregated=1 images" in capsys.readouterr().out


def test_quad_decoder_centerness_alpha_zero_default() -> None:
    quality = (
        torch.tensor([[[[2.0, -10.0], [-10.0, -10.0]]]]),
        torch.full((1, 1, 1, 1), -10.0),
        torch.full((1, 1, 1, 1), -10.0),
    )
    # Offsets in (B, 8, H, W): x0, y0, x1, y1, x2, y2, x3, y3
    corners = torch.tensor([-0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5, 0.5]).view(
        1, 8, 1, 1
    )
    offsets_level0 = corners.repeat(1, 1, 2, 2)
    offsets = (
        offsets_level0,
        torch.zeros((1, 8, 1, 1)),
        torch.zeros((1, 8, 1, 1)),
    )
    output = QuadDetectorOutput(quality, offsets)
    # Default centerness_alpha should be 0.0 (no post-hoc suppression of quality)
    decoder = QuadInferenceDecoder(pre_nms_top_k=5, max_proposals=2)
    assert decoder.centerness_alpha == 0.0
    detections = decoder(output, (32, 32))
    assert detections[0].scores.numel() > 0
    # Top score should equal sigmoid(2.0) ~ 0.8808, untouched by AABB centerness
    assert detections[0].scores[0].item() == pytest.approx(
        torch.sigmoid(torch.tensor(2.0)).item(), abs=1e-4
    )


def test_polygon_nms_part_aware_preserves_subparts() -> None:
    # Large quad (e.g. building facade)
    large = quad_from_bbox([0.0, 0.0, 100.0, 100.0])
    # Duplicate large quad
    dup = quad_from_bbox([2.0, 2.0, 102.0, 102.0])
    # Sub-part pillar inside
    small = quad_from_bbox([10.0, 20.0, 25.0, 70.0])
    quads = torch.stack([large, dup, small])
    scores = torch.tensor([0.95, 0.85, 0.90])

    keep, kept_scores = polygon_nms(
        quads,
        scores,
        iou_threshold=0.05,
        nms_mode="part_aware",
        part_scale_ratio=0.4,
        return_scores=True,
    )
    assert 0 in keep.tolist()
    assert 1 not in keep.tolist()  # Duplicate suppressed
    assert 2 in keep.tolist()  # Pillar preserved!

