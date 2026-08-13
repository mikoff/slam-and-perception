from __future__ import annotations

import torch

from student_detector.head import QuadDetectorOutput
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_evaluation import (
    QuadEvaluationAccumulator,
    QuadEvaluationImage,
    evaluate_quad_proposals,
)
from student_detector.quad_geometry import quad_from_bbox


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
