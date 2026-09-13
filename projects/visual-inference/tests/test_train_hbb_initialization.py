from __future__ import annotations

from pathlib import Path

import pytest
import torch

from scripts.train_phase3 import _initialize_model
from student_detector.checkpoints import architecture_id
from student_detector.model import StudentDetector


def test_weights_only_ema_checkpoint_strictly_initializes_hbb(tmp_path: Path) -> None:
    source = StudentDetector(pretrained_backbone=False, neck_type="lite")
    expected = {
        name: torch.full_like(value, 0.25) if value.is_floating_point() else value
        for name, value in source.state_dict().items()
    }
    checkpoint = tmp_path / "best_ema.pt"
    torch.save(
        {
            "phase": 3,
            "architecture": architecture_id("hbb", "lite"),
            "selected_state": "ema_model",
            "ema_model": expected,
            "config": {"neck_type": "lite"},
        },
        checkpoint,
    )
    target = StudentDetector(pretrained_backbone=False, neck_type="lite")

    selected = _initialize_model(
        target,
        checkpoint,
        neck_type="lite",
        required_state="ema_model",
    )

    assert selected == "ema_model"
    for name, value in target.state_dict().items():
        assert torch.equal(value, expected[name])


def test_hbb_initialization_rejects_missing_required_state(tmp_path: Path) -> None:
    model = StudentDetector(pretrained_backbone=False, neck_type="lite")
    checkpoint = tmp_path / "raw.pt"
    torch.save(
        {
            "phase": 3,
            "architecture": architecture_id("hbb", "lite"),
            "selected_state": "model",
            "model": model.state_dict(),
            "config": {"neck_type": "lite"},
        },
        checkpoint,
    )

    with pytest.raises(ValueError, match="does not contain state 'ema_model'"):
        _initialize_model(
            model,
            checkpoint,
            neck_type="lite",
            required_state="ema_model",
        )
