"""Checkpoint architecture contracts for student detector models."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
import pathlib
import sys
import types
from typing import Any, Literal

import torch

DetectorKind = Literal["hbb", "quad"]
NeckType = Literal["lite", "attn_res"]
CheckpointState = Literal["model", "ema_model"]


def load_checkpoint(path: Path) -> dict[str, Any]:
    """Load trusted project checkpoints across the Python 3.12/3.13 pathlib split."""
    if not hasattr(pathlib, "_local"):
        local_module = types.ModuleType("pathlib._local")
        local_module.PosixPath = pathlib.PosixPath
        local_module.WindowsPath = pathlib.WindowsPath
        local_module.Path = pathlib.Path
        sys.modules["pathlib._local"] = local_module
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        raise ValueError("checkpoint root must be a dictionary")
    return checkpoint


def initialize_model_weights(
    model: torch.nn.Module,
    checkpoint_path: Path,
    *,
    kind: DetectorKind,
    neck_type: NeckType,
    required_state: CheckpointState | None = None,
) -> CheckpointState:
    """Strictly warm-start one model state without restoring training state."""
    checkpoint = load_checkpoint(checkpoint_path)
    state_key = required_state or selected_checkpoint_state(checkpoint)
    load_model_state_strict(
        model,
        checkpoint,
        kind=kind,
        neck_type=neck_type,
        state_key=state_key,
    )
    return state_key


def architecture_id(kind: DetectorKind, neck_type: NeckType) -> str:
    """Return the stable architecture identifier stored in new checkpoints."""
    head = "fcos_atss_reg2" if kind == "hbb" else "quad_dqco"
    neck = "lite_fpn96" if neck_type == "lite" else "attn_res_lite_fpn96"
    return f"mobilenetv4_conv_medium_{neck}_{head}"


def checkpoint_neck_type(checkpoint: Mapping[str, Any]) -> NeckType:
    """Read a neck type, with state-key inference for legacy checkpoints."""
    configured = checkpoint.get("config", {}).get("neck_type")
    if configured is not None:
        if configured not in {"lite", "attn_res"}:
            raise ValueError(f"checkpoint has unsupported neck_type: {configured!r}")
        return configured
    state = checkpoint.get("model")
    if not isinstance(state, Mapping):
        raise ValueError("checkpoint does not contain a model state")
    has_attention = any(str(key).startswith("fpn.attn") for key in state)
    return "attn_res" if has_attention else "lite"


def selected_checkpoint_state(checkpoint: Mapping[str, Any]) -> CheckpointState:
    """Return the explicitly selected weights, defaulting legacy files to raw."""
    selected = checkpoint.get("selected_state", "model")
    if selected not in {"model", "ema_model"}:
        raise ValueError(f"checkpoint has unsupported selected_state: {selected!r}")
    if selected not in checkpoint:
        raise ValueError(f"checkpoint does not contain selected state {selected!r}")
    return selected


def validate_checkpoint_contract(
    checkpoint: Mapping[str, Any],
    *,
    kind: DetectorKind,
    neck_type: NeckType,
) -> None:
    """Reject a checkpoint whose phase or architecture does not match a model."""
    phase = checkpoint.get("phase")
    if kind == "quad" and phase != "quad_proposals":
        raise ValueError(f"expected a quad checkpoint, found phase={phase!r}")
    if kind == "hbb" and phase == "quad_proposals":
        raise ValueError("expected an HBB checkpoint, found a quad checkpoint")
    actual_neck = checkpoint_neck_type(checkpoint)
    if actual_neck != neck_type:
        raise ValueError(
            f"checkpoint neck_type={actual_neck!r} does not match "
            f"model neck_type={neck_type!r}"
        )
    recorded = checkpoint.get("architecture")
    expected = architecture_id(kind, neck_type)
    # Legacy IDs did not distinguish attention from LiteFPN. State keys and
    # config above are authoritative for them; new IDs must match exactly.
    if recorded and "attn_res_lite_fpn96" in str(recorded) and recorded != expected:
        raise ValueError(
            f"checkpoint architecture={recorded!r} does not match {expected!r}"
        )


def load_model_state_strict(
    model: torch.nn.Module,
    checkpoint: Mapping[str, Any],
    *,
    kind: DetectorKind,
    neck_type: NeckType,
    state_key: str = "model",
) -> None:
    """Validate the checkpoint contract and load every tensor strictly."""
    validate_checkpoint_contract(checkpoint, kind=kind, neck_type=neck_type)
    if state_key not in checkpoint:
        raise ValueError(f"checkpoint does not contain state {state_key!r}")
    model.load_state_dict(checkpoint[state_key], strict=True)


__all__ = [
    "DetectorKind",
    "CheckpointState",
    "NeckType",
    "architecture_id",
    "checkpoint_neck_type",
    "initialize_model_weights",
    "load_checkpoint",
    "load_model_state_strict",
    "selected_checkpoint_state",
    "validate_checkpoint_contract",
]
