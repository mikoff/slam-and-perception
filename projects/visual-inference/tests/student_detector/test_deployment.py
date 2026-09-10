from __future__ import annotations

import torch
from torch import nn

from student_detector.deployment import count_macs, parity_metrics


def test_count_macs_handles_grouped_convolution() -> None:
    model = nn.Conv2d(4, 6, kernel_size=3, groups=2, bias=False)
    example = torch.zeros(1, 4, 8, 8)
    assert count_macs(model, example) == 1 * 6 * 6 * 6 * 2 * 3 * 3


def test_parity_metrics_reports_exact_and_failed_results() -> None:
    reference = (torch.tensor([1.0, 2.0]),)
    exact = parity_metrics(reference, (reference[0].clone(),))
    failed = parity_metrics(reference, (torch.tensor([1.0, 3.0]),))

    assert exact["shape_match"] is True
    assert exact["allclose_atol_1e-5_rtol_1e-5"] is True
    assert exact["max_absolute_error"] == 0.0
    assert failed["allclose_atol_1e-5_rtol_1e-5"] is False
    assert failed["max_absolute_error"] == 1.0
