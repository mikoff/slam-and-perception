from __future__ import annotations

from pathlib import Path

import torch

from scripts.train_quad_proposals import _initialize_model
from student_detector.checkpoints import architecture_id
from student_detector.model import QuadProposalDetector


def test_weights_only_ema_checkpoint_can_initialize_a_fresh_run(
    tmp_path: Path,
) -> None:
    source = QuadProposalDetector(pretrained_backbone=False, neck_type="lite")
    expected = {
        name: torch.full_like(value, 0.25) if value.is_floating_point() else value
        for name, value in source.state_dict().items()
    }
    checkpoint = tmp_path / "best_ema.pt"
    torch.save(
        {
            "phase": "quad_proposals",
            "architecture": architecture_id("quad", "lite"),
            "selected_state": "ema_model",
            "ema_model": expected,
            "config": {"neck_type": "lite"},
        },
        checkpoint,
    )
    target = QuadProposalDetector(pretrained_backbone=False, neck_type="lite")

    state_key = _initialize_model(target, checkpoint, neck_type="lite")

    assert state_key == "ema_model"
    for name, value in target.state_dict().items():
        assert torch.equal(value, expected[name])
