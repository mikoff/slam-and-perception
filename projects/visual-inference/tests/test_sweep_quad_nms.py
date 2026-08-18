from __future__ import annotations

import pytest
import torch

from scripts.sweep_quad_nms import (
    _SweepMetrics,
    _geometry_output_fp32,
    _nms_keep_from_overlaps,
    _parse_thresholds,
)
from student_detector.head import QuadDetectorOutput
from student_detector.quad_geometry import (
    pairwise_quad_iou,
    polygon_nms,
    quad_from_bbox,
)


def test_threshold_parser_sorts_and_deduplicates() -> None:
    assert _parse_thresholds("0.9, 0.7,0.9") == (0.7, 0.9)


def test_amp_outputs_are_promoted_before_polygon_geometry() -> None:
    output = QuadDetectorOutput(
        quality=tuple(torch.ones((1, 1, 1, 1), dtype=torch.float16) for _ in range(3)),
        corner_offsets=tuple(
            torch.ones((1, 8, 1, 1), dtype=torch.float16) for _ in range(3)
        ),
    )

    promoted = _geometry_output_fp32(output)

    assert all(tensor.dtype == torch.float32 for tensor in promoted.quality)
    assert all(tensor.dtype == torch.float32 for tensor in promoted.corner_offsets)


@pytest.mark.parametrize("threshold", [0.5, 0.7, 0.9])
def test_precomputed_nms_exactly_replays_production(threshold: float) -> None:
    quads = torch.stack(
        [
            quad_from_bbox([0.0, 0.0, 10.0, 10.0]),
            quad_from_bbox([1.0, 1.0, 11.0, 11.0]),
            quad_from_bbox([20.0, 20.0, 30.0, 30.0]),
            quad_from_bbox([20.5, 20.5, 30.5, 30.5]),
        ]
    )
    scores = torch.tensor([0.8, 0.9, 0.7, 0.6])
    overlaps = pairwise_quad_iou(quads, quads, minimum_iou=0.5)

    actual = _nms_keep_from_overlaps(overlaps, scores, threshold, max_output=3)
    expected = polygon_nms(quads, scores, threshold, max_output=3)

    assert torch.equal(actual, expected.cpu())


def test_sweep_metrics_report_fixed_budget_recall_and_ar() -> None:
    accumulator = _SweepMetrics()
    overlaps = torch.zeros((1, 101))
    overlaps[0, 50] = 0.8

    accumulator.update(overlaps, torch.arange(101))
    metrics = accumulator.compute()

    assert metrics["recall/50@0.50"] == 0.0
    assert metrics["recall/100@0.50"] == 1.0
    assert metrics["recall/100@0.75"] == 1.0
    assert metrics["ar/100"] == pytest.approx(0.7)
