"""Quad-aware COCO dataset and deterministic geometric transforms."""

from __future__ import annotations

import json
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
    IMAGENET_STD,
    ImageRecord,
    _mask_to_tensor,
    _pil_to_normalized_tensor,
    build_coco_sqlite_index,
)
from .quad_geometry import (
    canonicalize_quad,
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
        *,
        seed: int,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor, tuple[float, float, float], tuple[int, ...]]:
        generator = random.Random(seed)
        image = image.convert("RGB")
        width, height = image.size
        all_quads = torch.cat((quads, ignore_quads), dim=0)
        is_positive = torch.arange(all_quads.shape[0]) < quads.shape[0]
        if self.training and generator.random() < self.augmentation.horizontal_flip_probability:
            image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            if all_quads.numel():
                all_quads[..., 0] = (width - 1) - all_quads[..., 0]
        base_scale = min(self.input_size / width, self.input_size / height)
        if self.training:
            scale = base_scale * generator.uniform(self.augmentation.scale_min, self.augmentation.scale_max)
            translation = self.augmentation.translation_fraction * self.input_size
            shift_x = generator.uniform(-translation, translation)
            shift_y = generator.uniform(-translation, translation)
        else:
            scale, shift_x, shift_y = base_scale, 0.0, 0.0
        offset_x = (self.input_size - width * scale) * 0.5 + shift_x
        offset_y = (self.input_size - height * scale) * 0.5 + shift_y
        inverse = (1.0 / scale, 0.0, -offset_x / scale, 0.0, 1.0 / scale, -offset_y / scale)
        image = image.transform(
            (self.input_size, self.input_size), Image.Transform.AFFINE, inverse,
            resample=Image.Resampling.BILINEAR, fillcolor=tuple(round(channel * 255) for channel in IMAGENET_MEAN),
        )
        valid = Image.new("L", (width, height), color=255).transform(
            (self.input_size, self.input_size), Image.Transform.AFFINE, inverse,
            resample=Image.Resampling.NEAREST, fillcolor=0,
        )
        transformed = all_quads * scale
        transformed[..., 0] += offset_x
        transformed[..., 1] += offset_y
        clipped_rows: list[Tensor] = []
        visibility: list[float] = []
        for quad in transformed:
            original_area = float(polygon_signed_area(quad).abs())
            clipped = clip_convex_polygon(
                quad,
                quad.new_tensor([[0.0, 0.0], [self.input_size, 0.0], [self.input_size, self.input_size], [0.0, self.input_size]]),
            )
            if clipped.shape[0] < 3:
                clipped_rows.append(quad.new_zeros((4, 2)))
                visibility.append(0.0)
                continue
            if clipped.shape == (4, 2) and bool(quad_validity(clipped)):
                clipped_rows.append(canonicalize_quad(clipped))
            else:
                clipped_rows.append(fit_quad_from_points(clipped))
            visibility.append(float(polygon_signed_area(clipped).abs()) / max(original_area, 1e-7))
        if not clipped_rows:
            image_tensor = _pil_to_normalized_tensor(image)
            return image_tensor, quads.new_empty((0, 4, 2)), ignore_quads.new_empty((0, 4, 2)), _mask_to_tensor(valid), (scale, offset_x, offset_y), ()
        clipped_tensor = torch.stack(clipped_rows)
        visibility_tensor = torch.tensor(visibility)
        areas = polygon_signed_area(clipped_tensor).abs()
        edge_lengths = torch.linalg.vector_norm(torch.roll(clipped_tensor, -1, 1) - clipped_tensor, dim=2)
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
        positive = is_positive & geometrically_valid & (
            visibility_tensor >= self.augmentation.positive_visible_fraction
        ) & (regular_size | thin_size)
        ignore = geometrically_valid & (visibility_tensor >= self.augmentation.ignore_visible_fraction) & ~positive
        image_tensor = _pil_to_normalized_tensor(image)
        return (
            image_tensor,
            clipped_tensor[positive],
            clipped_tensor[ignore],
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
        self.image_root = Path(image_root).resolve()
        self.index_path = build_coco_sqlite_index(annotations, index_path, force=force_index)
        self.data_config = data_config
        self.training = training
        self.seed = seed
        self.epoch = 0
        self.transform = QuadProposalTransform(
            data_config.input_size, augmentation, training=training,
            regular_min_side=data_config.quad_regular_min_side,
            thin_major_axis_min=data_config.thin_major_axis_min,
            thin_aspect_ratio_min=data_config.thin_aspect_ratio_min,
            thin_area=data_config.thin_area,
        )
        with sqlite3.connect(self.index_path) as connection:
            rows = connection.execute(
                "SELECT row_index, image_id, file_name, width, height, source_dataset, camera_type, background_supervision, positive_count FROM images ORDER BY row_index"
            ).fetchall()
        self.records = [ImageRecord(*row) for row in rows]
        self._connection: sqlite3.Connection | None = None

    def _db(self) -> sqlite3.Connection:
        if self._connection is None:
            self._connection = sqlite3.connect(self.index_path)
        return self._connection

    def __len__(self) -> int:
        return len(self.records)

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __getitem__(self, index: int) -> QuadProposalSample:
        record = self.records[index]
        rows = self._db().execute(
            "SELECT quad_json, ignore_region, geometry_tier FROM annotations WHERE image_id=?",
            (record.image_id,),
        ).fetchall()
        positive = [json.loads(row[0]) for row in rows if not row[1]]
        ignore = [json.loads(row[0]) for row in rows if row[1]]
        tiers = tuple(str(row[2]) for row in rows if not row[1])
        with Image.open(self.image_root / record.file_name) as loaded:
            image = loaded.convert("RGB")
        transformed = self.transform(
            image, _quad_tensor(positive), _quad_tensor(ignore),
            seed=self.seed + self.epoch * max(len(self), 1) + index,
        )
        domain = self.data_config.source_domains.get(record.source_dataset, "unknown")
        return QuadProposalSample(
            transformed[0], transformed[1], transformed[2], transformed[3],
            record.image_id, record.source_dataset, domain, record.camera_type,
            record.source_dataset in set(self.data_config.dense_background_sources),
            (record.height, record.width), transformed[4],
            tuple(tiers[index] for index in transformed[5]),
        )


def collate_quad_proposal_samples(samples: Sequence[QuadProposalSample]) -> tuple[Tensor, list[QuadProposalSample]]:
    return torch.stack([sample.image for sample in samples]), list(samples)
