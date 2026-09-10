from __future__ import annotations

import torch

from student_detector.provenance import model_state_sha256


def test_model_state_hash_is_value_sensitive_and_repeatable() -> None:
    model = torch.nn.Sequential(
        torch.nn.Linear(2, 3),
        torch.nn.BatchNorm1d(3),
    )
    first = model_state_sha256(model)
    assert model_state_sha256(model) == first

    with torch.no_grad():
        model[0].weight[0, 0] += 1
    assert model_state_sha256(model) != first
