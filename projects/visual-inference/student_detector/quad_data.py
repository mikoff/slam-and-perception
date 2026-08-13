"""Quad-aware COCO dataset and deterministic geometric transforms."""

from __future__ import annotations

import json
import os
import random
import sqlite3
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import torch
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset

from .config import AugmentationConfig, DataConfig
from .data import (
    IMAGENET_MEAN,
    ImageRecord,
    _mask_to_tensor,
    _pil_to_normalized_tensor,
    build_coco_sqlite_index,
)
from .quad_geometry import (
    canonicalize_quad,
    canonicalize_quads,
    clip_convex_polygon,
    fit_quad_from_points,
    polygon_signed_area,
    quad_validity,
)


@dataclass(frozen=True)
class QuadProposalSample:
    image: Tensor
    quads: Tensor
    ignore_quads: Tensor
    valid_mask: Tensor
    image_id: int
    source_dataset: str
    domain: str
    camera_type: str
    background_supervision: bool
    original_size: tuple[int, int]
    transform: tuple[float, float, float]
    geometry_tiers: tuple[str, ...] = ()
    object_conditions: tuple[str, ...] = ()
    trusted_background_quads: Tensor | None = None
    seen_statuses: tuple[str, ...] = ()
    size_bins: tuple[str, ...] = ()
    aspect_bins: tuple[str, ...] = ()
    radial_bins: tuple[str, ...] = ()


def _quad_slice_labels(quads: Tensor, size: int) -> tuple[tuple[str, ...], ...]:
    if not quads.numel():
        return (), (), ()
    extent = quads.amax(dim=1) - quads.amin(dim=1)
    short, _ = extent.sort(dim=1).values.unbind(dim=1)
    long = extent.amax(dim=1)
    scale = torch.sqrt(extent.prod(dim=1).clamp(min=0))
    centers = quads.mean(dim=1)
    radial = torch.linalg.vector_norm((centers - size / 2) / max(size / 2, 1), dim=1)
    size_bins = tuple(
        "tiny"
        if value < 16
        else "small"
        if value < 32
        else "medium"
        if value < 96
        else "large"
        for value in scale.tolist()
    )
    ratios = long / short.clamp(min=1e-7)
    aspect_bins = tuple(
        "extreme" if value >= 8 else "slender" if value >= 3 else "regular"
        for value in ratios.tolist()
    )
    radial_bins = tuple(
        "center" if value < 0.35 else "mid" if value < 0.7 else "edge"
        for value in radial.tolist()
    )
    return size_bins, aspect_bins, radial_bins


def _quad_tensor(rows: Sequence[Sequence[Sequence[float]]]) -> Tensor:
    return (
        torch.tensor(rows, dtype=torch.float32).reshape(-1, 4, 2)
        if rows
        else torch.empty((0, 4, 2), dtype=torch.float32)
    )


class QuadProposalTransform:
    """Apply one deterministic affine transform to image and all quad states."""

    def __init__(
        self,
        input_size: int,
        augmentation: AugmentationConfig,
        *,
        training: bool,
        regular_min_side: float,
        thin_major_axis_min: float = 8.0,
        thin_aspect_ratio_min: float = 3.0,
        thin_area: float = 16.0,
    ) -> None:
        self.input_size = input_size
        self.augmentation = augmentation
        self.training = training
        self.regular_min_side = regular_min_side
        self.thin_major_axis_min = thin_major_axis_min
        self.thin_aspect_ratio_min = thin_aspect_ratio_min
        self.thin_area = thin_area

    def __call__(
        self,
        image: Image.Image,
        quads: Tensor,
        ignore_quads: Tensor,
        trusted_background_quads: Tensor | None = None,
        *,
        seed: int,
    ) -> tuple[
        Tensor,
        Tensor,
        Tensor,
        Tensor,
        Tensor,
        tuple[float, float, float],
        tuple[int, ...],
    ]:
        generator = random.Random(seed)
        image = image.convert("RGB")
        width, height = image.size
        trusted_background_quads = (
            trusted_background_quads
            if trusted_background_quads is not None
            else quads.new_empty((0, 4, 2))
        )
        all_quads = torch.cat((quads, ignore_quads, trusted_background_quads), dim=0)
        state = torch.cat(
            (
                torch.zeros(quads.shape[0], dtype=torch.long),
                torch.ones(ignore_quads.shape[0], dtype=torch.long),
                torch.full((trusted_background_quads.shape[0],), 2, dtype=torch.long),
            )
        )
        if (
            self.training
            and generator.random() < self.augmentation.horizontal_flip_probability
        ):
            image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            if all_quads.numel():
                all_quads[..., 0] = (width - 1) - all_quads[..., 0]
        base_scale = min(self.input_size / width, self.input_size / height)
        if self.training:
            scale = base_scale * generator.uniform(
                self.augmentation.scale_min, self.augmentation.scale_max
            )
            translation = self.augmentation.translation_fraction * self.input_size
            shift_x = generator.uniform(-translation, translation)
            shift_y = generator.uniform(-translation, translation)
        else:
            scale, shift_x, shift_y = base_scale, 0.0, 0.0
        offset_x = (self.input_size - width * scale) * 0.5 + shift_x
        offset_y = (self.input_size - height * scale) * 0.5 + shift_y
        inverse = (
            1.0 / scale,
            0.0,
            -offset_x / scale,
            0.0,
            1.0 / scale,
            -offset_y / scale,
        )
        image = image.transform(
            (self.input_size, self.input_size),
            Image.Transform.AFFINE,
            inverse,
            resample=Image.Resampling.BILINEAR,
            fillcolor=tuple(round(channel * 255) for channel in IMAGENET_MEAN),
        )
        valid = Image.new("L", (width, height), color=255).transform(
            (self.input_size, self.input_size),
            Image.Transform.AFFINE,
            inverse,
            resample=Image.Resampling.NEAREST,
            fillcolor=0,
        )
        transformed = all_quads * scale
        transformed[..., 0] += offset_x
        transformed[..., 1] += offset_y
        clipped_tensor = torch.empty_like(transformed)
        visibility_tensor = transformed.new_zeros((transformed.shape[0],))
        inside = (
            (transformed[..., 0] >= 0)
            & (transformed[..., 0] <= self.input_size)
            & (transformed[..., 1] >= 0)
            & (transformed[..., 1] <= self.input_size)
        ).all(dim=1)
        if inside.any():
            clipped_tensor[inside] = canonicalize_quads(transformed[inside])
            visibility_tensor[inside] = 1.0
        boundary_indices = (~inside).nonzero().flatten().tolist()
        for index in boundary_indices:
            quad = transformed[index]
            original_area = float(polygon_signed_area(quad).abs())
            clipped = clip_convex_polygon(
                quad,
                quad.new_tensor(
                    [
                        [0.0, 0.0],
                        [self.input_size, 0.0],
                        [self.input_size, self.input_size],
                        [0.0, self.input_size],
                    ]
                ),
            )
            if clipped.shape[0] < 3:
                clipped_tensor[index] = 0
                continue
            if clipped.shape == (4, 2) and bool(quad_validity(clipped)):
                clipped_tensor[index] = canonicalize_quad(clipped)
            else:
                clipped_tensor[index] = fit_quad_from_points(clipped)
            visibility_tensor[index] = float(polygon_signed_area(clipped).abs()) / max(
                original_area, 1e-7
            )
        if not transformed.shape[0]:
            image_tensor = _pil_to_normalized_tensor(image)
            return (
                image_tensor,
                quads.new_empty((0, 4, 2)),
                ignore_quads.new_empty((0, 4, 2)),
                trusted_background_quads.new_empty((0, 4, 2)),
                _mask_to_tensor(valid),
                (scale, offset_x, offset_y),
                (),
            )
        areas = polygon_signed_area(clipped_tensor).abs()
        edge_lengths = torch.linalg.vector_norm(
            torch.roll(clipped_tensor, -1, 1) - clipped_tensor, dim=2
        )
        side = edge_lengths.amin(dim=1)
        major = edge_lengths.amax(dim=1)
        aspect_ratio = major / side.clamp(min=1e-7)
        geometrically_valid = (areas > 1e-4) & (visibility_tensor > 0)
        regular_size = side >= self.regular_min_side
        thin_size = (
            (major >= self.thin_major_axis_min)
            & (aspect_ratio >= self.thin_aspect_ratio_min)
            & (areas >= self.thin_area)
        )
        positive = (
            (state == 0)
            & geometrically_valid
            & (visibility_tensor >= self.augmentation.positive_visible_fraction)
            & (regular_size | thin_size)
        )
        ignore = (
            (state != 2)
            & geometrically_valid
            & (visibility_tensor >= self.augmentation.ignore_visible_fraction)
            & ~positive
        )
        trusted = (
            (state == 2)
            & geometrically_valid
            & (visibility_tensor >= self.augmentation.ignore_visible_fraction)
        )
        image_tensor = _pil_to_normalized_tensor(image)
        return (
            image_tensor,
            clipped_tensor[positive],
            clipped_tensor[ignore],
            clipped_tensor[trusted],
            _mask_to_tensor(valid),
            (scale, offset_x, offset_y),
            tuple(torch.where(positive)[0].tolist()),
        )


class QuadProposalDataset(Dataset[QuadProposalSample]):
    """Random-access dataset backed by the shared streaming COCO index."""

    def __init__(
        self,
        annotations: str | Path,
        image_root: str | Path,
        index_path: str | Path,
        data_config: DataConfig,
        augmentation: AugmentationConfig,
        *,
        training: bool,
        seed: int = 42,
        force_index: bool = False,
    ) -> None:
        self.annotations = Path(annotations).resolve()
        self.image_root = Path(image_root).resolve()
        resolved_index = Path(index_path).resolve()
        require_prebuilt = os.getenv("REQUIRE_PREBUILT_INDEX") == "1"
        if require_prebuilt and not resolved_index.is_file():
            raise FileNotFoundError(
                f"cloud dataset is missing prebuilt index: {resolved_index}"
            )
        self.index_path = build_coco_sqlite_index(
            self.annotations,
            resolved_index,
            force=force_index,
            build_if_missing=not require_prebuilt,
        )
        self.data_config = data_config
        self.training = training
        self.seed = seed
        self._epoch = torch.zeros((), dtype=torch.int64).share_memory_()
        self.transform = QuadProposalTransform(
            data_config.input_size,
            augmentation,
            training=training,
            regular_min_side=data_config.quad_regular_min_side,
            thin_major_axis_min=data_config.thin_major_axis_min,
            thin_aspect_ratio_min=data_config.thin_aspect_ratio_min,
            thin_area=data_config.thin_area,
        )
        with sqlite3.connect(self.index_path) as connection:
            rows = connection.execute(
                """
                SELECT row_index, image_id, file_name, width, height,
                       source_dataset, camera_type, background_supervision,
                       positive_count
                FROM images ORDER BY row_index
                """
            ).fetchall()
        self.records = [ImageRecord(*row) for row in rows]
        self._connection: sqlite3.Connection | None = None
        self._connection_pid: int | None = None

    def _annotation_rows(self, image_id: int) -> list[tuple[str, int, str, str, str]]:
        pid = os.getpid()
        if self._connection is None or self._connection_pid != pid:
            if self._connection is not None:
                self._connection.close()
            uri = f"file:{self.index_path}?mode=ro&immutable=1"
            self._connection = sqlite3.connect(uri, uri=True)
            self._connection_pid = pid
        return self._connection.execute(
            """
            SELECT quad_json, ignore_region, geometry_tier,
                   object_condition, seen_status
            FROM annotations WHERE image_id=?
            """,
            (image_id,),
        ).fetchall()

    def __getstate__(self) -> dict[str, object]:
        state = dict(self.__dict__)
        state["_connection"] = None
        state["_connection_pid"] = None
        return state

    def __len__(self) -> int:
        return len(self.records)

    def set_epoch(self, epoch: int) -> None:
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> QuadProposalSample:
        record = self.records[index]
        rows = self._annotation_rows(record.image_id)
        positive = [json.loads(row[0]) for row in rows if row[1] == 0]
        ignore = [json.loads(row[0]) for row in rows if row[1] == 1]
        trusted = [json.loads(row[0]) for row in rows if row[1] == 2]
        tiers = tuple(str(row[2]) for row in rows if row[1] == 0)
        conditions = tuple(str(row[3]) for row in rows if row[1] == 0)
        statuses = tuple(str(row[4]) for row in rows if row[1] == 0)
        with Image.open(self.image_root / record.file_name) as loaded:
            image = loaded.convert("RGB")
        transformed = self.transform(
            image,
            _quad_tensor(positive),
            _quad_tensor(ignore),
            _quad_tensor(trusted),
            seed=self.seed + int(self._epoch.item()) * max(len(self), 1) + index,
        )
        domain = self.data_config.source_domains.get(record.source_dataset, "unknown")
        retained = transformed[6]
        slice_labels = _quad_slice_labels(transformed[1], self.data_config.input_size)
        return QuadProposalSample(
            transformed[0],
            transformed[1],
            transformed[2],
            transformed[4],
            record.image_id,
            record.source_dataset,
            domain,
            record.camera_type,
            bool(transformed[3].numel()),
            (record.height, record.width),
            transformed[5],
            tuple(tiers[index] for index in retained),
            tuple(conditions[index] for index in retained),
            transformed[3],
            tuple(statuses[index] for index in retained),
            *slice_labels,
        )


def collate_quad_proposal_samples(
    samples: Sequence[QuadProposalSample],
) -> tuple[Tensor, list[QuadProposalSample]]:
    return torch.stack([sample.image for sample in samples]), list(samples)
