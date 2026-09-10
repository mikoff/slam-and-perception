"""Shared preparation helpers for proposal-utility evaluation scripts."""

from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
import sqlite3
from typing import Any

import torch

from student_detector.checkpoints import checkpoint_neck_type, load_model_state_strict
from student_detector.model import QuadProposalDetector, StudentDetector
from student_detector.proposal_utility import ProposalUtilityAccumulator
from student_detector.quad_geometry import canonicalize_quads, quad_validity
from student_detector.quad_targets import point_validity_from_pixel_mask
from student_detector.suppression import SuppressionSweepAccumulator


def boxes_to_quads(boxes: torch.Tensor) -> torch.Tensor:
    """Convert axis-aligned ``xyxy`` boxes to clockwise four-point quads."""
    if not boxes.numel():
        return boxes.new_empty((0, 4, 2))
    x1, y1, x2, y2 = boxes.unbind(dim=1)
    return torch.stack(
        (
            torch.stack((x1, y1), dim=1),
            torch.stack((x2, y1), dim=1),
            torch.stack((x2, y2), dim=1),
            torch.stack((x1, y2), dim=1),
        ),
        dim=1,
    )


def synchronize_device(device: torch.device) -> None:
    """Synchronize CUDA work while leaving CPU evaluation unchanged."""
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def load_proposal_model(
    kind: str, checkpoint_path: Path, state: str, device: torch.device
) -> torch.nn.Module:
    """Load an HBB or quad proposal checkpoint with strict state validation."""
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    neck_type = checkpoint_neck_type(checkpoint)
    model: torch.nn.Module = (
        StudentDetector(pretrained_backbone=False, neck_type=neck_type)
        if kind == "hbb"
        else QuadProposalDetector(pretrained_backbone=False, neck_type=neck_type)
    )
    load_model_state_strict(
        model, checkpoint, kind=kind, neck_type=neck_type, state_key=state
    )
    return model.to(device).eval()


def valid_output_levels(
    valid_mask: torch.Tensor,
    output_levels: tuple[torch.Tensor, ...],
    strides: tuple[int, ...],
    device: torch.device,
) -> tuple[torch.Tensor, ...]:
    """Project an input validity mask onto detector output levels."""
    shapes = tuple((item.shape[-2], item.shape[-1]) for item in output_levels)
    levels = point_validity_from_pixel_mask(valid_mask.to(device), shapes, strides)[1]
    return tuple(level.unsqueeze(0) for level in levels)


def source_size_bands(quads: torch.Tensor, scale: float) -> tuple[str, ...]:
    """Label quads by their source-space short-side size band."""
    extent = (quads.amax(dim=1) - quads.amin(dim=1)) / scale
    short = extent.amin(dim=1)
    edges = (0, 4, 8, 16, 32, 64, 128, 256, float("inf"))
    return tuple(
        next(
            f"[{lower:g},{upper:g})"
            for lower, upper in zip(edges, edges[1:], strict=True)
            if lower <= value < upper
        )
        for value in short.tolist()
    )


def load_ego_source_quads(
    index_path: Path, image_ids: list[int]
) -> dict[int, torch.Tensor]:
    """Read ego-platform source quads for the selected validation images."""
    result: dict[int, list[Any]] = defaultdict(list)
    placeholders = ",".join("?" for _ in image_ids)
    with sqlite3.connect(
        f"file:{index_path}?mode=ro&immutable=1", uri=True
    ) as connection:
        rows = connection.execute(
            f"""
            SELECT image_id, quad_json FROM annotations
            WHERE category_name='ego_platform_bodywork'
              AND image_id IN ({placeholders})
            """,
            image_ids,
        )
    for image_id, encoded in rows:
        result[int(image_id)].append(json.loads(encoded))
    return {
        image_id: torch.tensor(quads, dtype=torch.float32).reshape(-1, 4, 2)
        for image_id, quads in result.items()
    }


def transform_ego_quads(
    source_quads: torch.Tensor | None,
    transform: tuple[float, float, float],
    input_size: int,
    device: torch.device,
) -> torch.Tensor:
    """Map ego-platform quads into model-input coordinates and filter them."""
    if source_quads is None or not source_quads.numel():
        return torch.empty((0, 4, 2), dtype=torch.float32, device=device)
    scale, offset_x, offset_y = transform
    quads = source_quads.to(device) * scale
    quads[..., 0] += offset_x
    quads[..., 1] += offset_y
    quads.clamp_(0, input_size)
    quads = canonicalize_quads(quads)
    return quads[quad_validity(quads)]


def merge_utility_and_suppression_metrics(
    utility: ProposalUtilityAccumulator, suppression: SuppressionSweepAccumulator
) -> dict[str, float | int]:
    """Merge accumulator outputs while preserving utility metric precedence."""
    result = utility.compute()
    for key, value in suppression.compute().items():
        if key not in result:
            result[key] = value
    return result
