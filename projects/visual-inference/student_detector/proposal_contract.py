"""Common source-coordinate record for decoded HBB and quad proposals."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import torch
from torch import Tensor

from .decoder import Detection
from .quad_decoder import QuadDetection

GeometryType = Literal["hbb", "quad"]
Point = tuple[float, float]
Quad = tuple[Point, Point, Point, Point]
PROPOSAL_CONTRACT_SCHEMA = "visual-inference-proposal.v1"


@dataclass(frozen=True)
class SourceTransform:
    """Aspect-preserving model transform and its source-image bounds."""

    scale: float
    offset_x: float
    offset_y: float
    source_size: tuple[int, int]
    model_size: tuple[int, int]

    def __post_init__(self) -> None:
        values = torch.tensor((self.scale, self.offset_x, self.offset_y))
        if not bool(torch.isfinite(values).all()) or self.scale <= 0:
            raise ValueError("source transform must contain a positive finite scale")
        if min(*self.source_size, *self.model_size) <= 0:
            raise ValueError("source and model dimensions must be positive")

    def model_to_source(self, geometry: Tensor) -> Tensor:
        """Invert letterboxing and clip coordinates to the source image."""
        source_height, source_width = self.source_size
        result = geometry.detach().clone().to(dtype=torch.float32)
        result[..., 0] = ((result[..., 0] - self.offset_x) / self.scale).clamp(
            min=0, max=source_width
        )
        result[..., 1] = ((result[..., 1] - self.offset_y) / self.scale).clamp(
            min=0, max=source_height
        )
        return result


@dataclass(frozen=True)
class ProposalRecord:
    """JSON-safe proposal record shared by HBB and quad decoders."""

    proposal_id: str
    image_id: int
    source_dataset: str
    rank: int
    score: float
    geometry_type: GeometryType
    model_geometry: Quad
    source_geometry: Quad
    pyramid_level: int
    location_index: int
    source_transform: SourceTransform


def _hbb_to_quads(boxes: Tensor) -> Tensor:
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


def _quad_tuple(quad: Tensor) -> Quad:
    values = quad.detach().to(dtype=torch.float32, device="cpu").tolist()
    return tuple((float(x), float(y)) for x, y in values)  # type: ignore[return-value]


def _build_records(
    *,
    geometry_type: GeometryType,
    quads: Tensor,
    scores: Tensor,
    levels: Tensor | None,
    location_indices: Tensor | None,
    image_id: int,
    source_dataset: str,
    source_transform: SourceTransform,
    max_proposals: int,
    score_threshold: float,
) -> tuple[ProposalRecord, ...]:
    if max_proposals < 1:
        raise ValueError("max_proposals must be positive")
    if not 0 <= score_threshold <= 1:
        raise ValueError("score_threshold must be in [0, 1]")
    if levels is None or location_indices is None:
        raise ValueError("decoded proposals must retain level and location identity")
    count = quads.shape[0]
    if scores.shape != (count,) or levels.shape != (count,) or location_indices.shape != (count,):
        raise ValueError("proposal geometry, scores, levels, and locations must align")

    source_quads = source_transform.model_to_source(quads)
    ordering = sorted(
        (
            index
            for index in range(count)
            if float(scores[index].detach().cpu()) >= score_threshold
        ),
        key=lambda index: (
            -float(scores[index].detach().cpu()),
            int(levels[index].detach().cpu()),
            int(location_indices[index].detach().cpu()),
        ),
    )[:max_proposals]
    records = []
    for rank, index in enumerate(ordering):
        level = int(levels[index].detach().cpu())
        location = int(location_indices[index].detach().cpu())
        records.append(
            ProposalRecord(
                proposal_id=(
                    f"{source_dataset}:{image_id}:{geometry_type}:P{level}:{location}"
                ),
                image_id=image_id,
                source_dataset=source_dataset,
                rank=rank,
                score=float(scores[index].detach().cpu()),
                geometry_type=geometry_type,
                model_geometry=_quad_tuple(quads[index]),
                source_geometry=_quad_tuple(source_quads[index]),
                pyramid_level=level,
                location_index=location,
                source_transform=source_transform,
            )
        )
    return tuple(records)


def records_from_hbb(
    detection: Detection,
    *,
    image_id: int,
    source_dataset: str,
    source_transform: SourceTransform,
    max_proposals: int = 100,
    score_threshold: float = 0.0,
) -> tuple[ProposalRecord, ...]:
    """Convert final HBB detections into the common proposal contract."""
    return _build_records(
        geometry_type="hbb",
        quads=_hbb_to_quads(detection.boxes),
        scores=detection.scores,
        levels=detection.levels,
        location_indices=detection.location_indices,
        image_id=image_id,
        source_dataset=source_dataset,
        source_transform=source_transform,
        max_proposals=max_proposals,
        score_threshold=score_threshold,
    )


def records_from_quads(
    detection: QuadDetection,
    *,
    image_id: int,
    source_dataset: str,
    source_transform: SourceTransform,
    max_proposals: int = 100,
    score_threshold: float = 0.0,
) -> tuple[ProposalRecord, ...]:
    """Convert final quad detections into the common proposal contract."""
    return _build_records(
        geometry_type="quad",
        quads=detection.quads,
        scores=detection.scores,
        levels=detection.levels,
        location_indices=detection.location_indices,
        image_id=image_id,
        source_dataset=source_dataset,
        source_transform=source_transform,
        max_proposals=max_proposals,
        score_threshold=score_threshold,
    )
