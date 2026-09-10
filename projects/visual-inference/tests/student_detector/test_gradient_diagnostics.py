from __future__ import annotations

import pytest
import torch

from student_detector.gradient_diagnostics import (
    measure_gradient_influence,
    summarize_gradient_measurements,
)


def test_gradient_influence_reports_norms_ratios_and_cosine() -> None:
    fpn = torch.tensor([1.0, 2.0], requires_grad=True)
    backbone = torch.tensor([2.0, 3.0], requires_grad=True)
    quality = fpn.square().sum() + backbone.square().sum()
    localization = 3 * fpn.sum() + 2 * backbone.sum()
    terms = {
        "quality/unweighted": quality,
        "quality/weighted": 2 * quality,
        "localization/unweighted": localization,
        "localization/weighted": 0.5 * localization,
    }

    result = measure_gradient_influence(
        terms,
        {"fpn_shared": (fpn,), "backbone_c5": (backbone,)},
    )

    assert result["gradient/fpn_shared/quality/weighted_norm"] > 0
    assert result["gradient/fpn_shared/localization/weighted_norm"] > 0
    assert 0 < result["gradient/fpn_shared/quality/weighted_to_total_ratio"] < 1
    assert result[
        "gradient/fpn_shared/quality_localization/weighted_cosine"
    ] == pytest.approx(3 / 10**0.5)
    assert fpn.grad is None
    assert backbone.grad is None


def test_gradient_influence_handles_zero_family_gradient() -> None:
    activation = torch.tensor([1.0, -1.0], requires_grad=True)
    zero = activation.sum() * 0
    localization = activation.square().sum()
    result = measure_gradient_influence(
        {
            "quality/unweighted": zero,
            "quality/weighted": zero,
            "localization/unweighted": localization,
            "localization/weighted": localization,
        },
        {"shared": (activation,)},
    )
    assert result["gradient/shared/quality/weighted_norm"] == 0
    assert result["gradient/shared/quality/weighted_to_total_ratio"] == 0
    assert result["gradient/shared/quality_localization/weighted_cosine"] == 0


def test_gradient_measurement_summary_reports_population_variance() -> None:
    summary = summarize_gradient_measurements(
        ({"a": 1.0, "b": 4.0}, {"a": 3.0, "b": 4.0})
    )
    assert summary["batches"] == 2
    metrics = summary["metrics"]
    assert isinstance(metrics, dict)
    assert metrics["a"] == {
        "mean": 2.0,
        "median": 2.0,
        "variance": 1.0,
        "minimum": 1.0,
        "maximum": 3.0,
    }
    assert metrics["b"]["variance"] == 0


def test_gradient_influence_rejects_incomplete_terms() -> None:
    activation = torch.ones(1, requires_grad=True)
    with pytest.raises(ValueError, match="missing"):
        measure_gradient_influence(
            {"quality/weighted": activation.sum()}, {"shared": (activation,)}
        )
