from __future__ import annotations

from pathlib import Path

import pytest
import torch

from scripts.measure_gradient_influence import (
    _checkpoint_arguments,
    _size_filtered_samples,
)
from student_detector.data import ProposalSample


def _sample() -> ProposalSample:
    return ProposalSample(
        image=torch.zeros(3, 64, 64),
        boxes=torch.tensor(
            [
                [0.0, 0.0, 15.0, 10.0],
                [0.0, 0.0, 20.0, 20.0],
                [0.0, 0.0, 40.0, 40.0],
            ]
        ),
        ignore_boxes=torch.tensor([[50.0, 50.0, 60.0, 60.0]]),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        image_id=1,
        source_dataset="coco_2017",
        domain="general",
        camera_type="perspective",
        background_supervision=True,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
        trusted_background_boxes=torch.tensor([[1.0, 1.0, 2.0, 2.0]]),
    )


def test_size_filter_retains_only_band_and_ignores_other_positives() -> None:
    filtered = _size_filtered_samples([_sample()], "small")[0]
    torch.testing.assert_close(
        filtered.boxes, torch.tensor([[0.0, 0.0, 20.0, 20.0]])
    )
    assert filtered.ignore_boxes.shape == (3, 4)
    assert filtered.trusted_background_boxes is not None
    assert filtered.trusted_background_boxes.numel() == 0


def test_checkpoint_arguments_require_unique_labels(tmp_path: Path) -> None:
    result = _checkpoint_arguments([f"warmup={tmp_path / 'warmup.pt'}"])
    assert result == [("warmup", (tmp_path / "warmup.pt").resolve())]
    with pytest.raises(ValueError, match="duplicate"):
        _checkpoint_arguments(["same=a.pt", "same=b.pt"])
    with pytest.raises(ValueError, match="LABEL=PATH"):
        _checkpoint_arguments(["missing-label-separator"])
