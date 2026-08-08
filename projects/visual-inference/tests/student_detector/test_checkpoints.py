from __future__ import annotations

import pytest

from student_detector.backbone import StubBackbone
from student_detector.checkpoints import (
    architecture_id,
    checkpoint_neck_type,
    load_model_state_strict,
    selected_checkpoint_state,
)
from student_detector.model import QuadProposalDetector


def _checkpoint(neck_type: str) -> dict:
    model = QuadProposalDetector(backbone=StubBackbone(), neck_type=neck_type)
    return {
        "phase": "quad_proposals",
        "architecture": architecture_id("quad", neck_type),
        "config": {"neck_type": neck_type},
        "model": model.state_dict(),
    }


@pytest.mark.parametrize("neck_type", ["lite", "attn_res"])
def test_checkpoint_neck_round_trip_and_strict_load(neck_type: str) -> None:
    checkpoint = _checkpoint(neck_type)
    model = QuadProposalDetector(backbone=StubBackbone(), neck_type=neck_type)
    assert checkpoint_neck_type(checkpoint) == neck_type
    load_model_state_strict(
        model, checkpoint, kind="quad", neck_type=neck_type
    )


def test_checkpoint_rejects_wrong_neck_before_partial_loading() -> None:
    checkpoint = _checkpoint("lite")
    model = QuadProposalDetector(backbone=StubBackbone(), neck_type="attn_res")
    with pytest.raises(ValueError, match="does not match"):
        load_model_state_strict(
            model, checkpoint, kind="quad", neck_type="attn_res"
        )


def test_legacy_checkpoint_infers_attention_from_state_keys() -> None:
    checkpoint = _checkpoint("attn_res")
    checkpoint["config"].pop("neck_type")
    checkpoint["architecture"] = "mobilenetv4_conv_medium_lite_fpn96_quad_dqco"
    assert checkpoint_neck_type(checkpoint) == "attn_res"


def test_selected_checkpoint_state_is_explicit_and_legacy_safe() -> None:
    checkpoint = _checkpoint("lite")
    assert selected_checkpoint_state(checkpoint) == "model"
    checkpoint["ema_model"] = checkpoint["model"]
    checkpoint["selected_state"] = "ema_model"
    assert selected_checkpoint_state(checkpoint) == "ema_model"
    checkpoint["selected_state"] = "missing"
    with pytest.raises(ValueError, match="unsupported selected_state"):
        selected_checkpoint_state(checkpoint)
