"""Streaming COCO index, Phase-3 transforms, and fixed-domain batch sampling."""

from __future__ import annotations

import io
import json
import math
import os
import random
import sqlite3
import tempfile
from collections import defaultdict
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import ijson
import torch
from PIL import Image, ImageEnhance, ImageFilter
from torch import Tensor
from torch.utils.data import Dataset, Sampler

from .config import AugmentationConfig, DataConfig


IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)
PADDING_RGB = tuple(round(channel * 255) for channel in IMAGENET_MEAN)
INDEX_SCHEMA_VERSION = "2"


@dataclass(frozen=True)
class ImageRecord:
    row_index: int
    image_id: int
    file_name: str
    width: int
    height: int
    source_dataset: str
    camera_type: str
    background_supervision: bool
    positive_count: int


@dataclass(frozen=True)
class ProposalSample:
    image: Tensor
    boxes: Tensor
    ignore_boxes: Tensor
    valid_mask: Tensor
    image_id: int
    source_dataset: str
    domain: str
    camera_type: str
    background_supervision: bool
    original_size: tuple[int, int]
    transform: tuple[float, float, float]


def _source_signature(path: Path) -> str:
    stat = path.stat()
    return f"{path.resolve()}:{stat.st_size}:{stat.st_mtime_ns}"


def build_coco_sqlite_index(
    annotations: str | Path,
    index_path: str | Path,
    *,
    force: bool = False,
) -> Path:
    """Build a compact random-access index without loading COCO JSON into RAM."""
    source = Path(annotations).resolve()
    destination = Path(index_path).resolve()
    signature = _source_signature(source)
    if destination.exists() and not force:
        with sqlite3.connect(destination) as connection:
            metadata = dict(connection.execute(
                "SELECT key, value FROM metadata"
            ).fetchall())
        if (
            metadata.get("source_signature") == signature
            and metadata.get("schema_version") == INDEX_SCHEMA_VERSION
        ):
            return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    os.close(file_descriptor)
    staged = Path(staged_name)
    try:
        with sqlite3.connect(staged) as connection:
            connection.executescript(
                """
                PRAGMA journal_mode=OFF;
                PRAGMA synchronous=OFF;
                CREATE TABLE metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL);
                CREATE TABLE images (
                    row_index INTEGER PRIMARY KEY,
                    image_id INTEGER UNIQUE NOT NULL,
                    file_name TEXT NOT NULL,
                    width INTEGER NOT NULL,
                    height INTEGER NOT NULL,
                    source_dataset TEXT NOT NULL,
                    camera_type TEXT NOT NULL,
                    background_supervision INTEGER NOT NULL,
                    positive_count INTEGER NOT NULL DEFAULT 0
                );
                CREATE TABLE annotations (
                    image_id INTEGER NOT NULL,
                    x1 REAL NOT NULL,
                    y1 REAL NOT NULL,
                    x2 REAL NOT NULL,
                    y2 REAL NOT NULL,
                    ignore_region INTEGER NOT NULL,
                    category_name TEXT NOT NULL
                );
                CREATE INDEX annotations_by_image ON annotations(image_id);
                """
            )
            with source.open("rb") as stream:
                for row_index, image in enumerate(
                    ijson.items(stream, "images.item")
                ):
                    connection.execute(
                        """
                        INSERT INTO images VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0)
                        """,
                        (
                            row_index,
                            int(image["id"]),
                            str(image["file_name"]),
                            int(image["width"]),
                            int(image["height"]),
                            str(image.get("source_dataset", "unknown")),
                            str(image.get("camera_type", "perspective")),
                            int(bool(image.get("background_supervision", False))),
                        ),
                    )
            positive_counts: dict[int, int] = defaultdict(int)
            categories: dict[int, str] = {}
            with source.open("rb") as stream:
                for category in ijson.items(stream, "categories.item"):
                    categories[int(category["id"])] = str(category["name"])
            with source.open("rb") as stream:
                rows = []
                for annotation in ijson.items(stream, "annotations.item"):
                    x, y, width, height = map(float, annotation["bbox"])
                    ignore = bool(
                        annotation.get("ignore_region")
                        or annotation.get("iscrowd")
                    )
                    image_id = int(annotation["image_id"])
                    rows.append((
                        image_id,
                        x,
                        y,
                        x + width,
                        y + height,
                        int(ignore),
                        categories.get(
                            int(annotation.get("category_id", -1)), "unknown"
                        ),
                    ))
                    if not ignore:
                        positive_counts[image_id] += 1
                    if len(rows) >= 10_000:
                        connection.executemany(
                            "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?)",
                            rows,
                        )
                        rows.clear()
                if rows:
                    connection.executemany(
                        "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?)",
                        rows,
                    )
            connection.executemany(
                "UPDATE images SET positive_count=? WHERE image_id=?",
                [(count, image_id) for image_id, count in positive_counts.items()],
            )
            connection.execute(
                "INSERT INTO metadata VALUES ('source_signature', ?)",
                (signature,),
            )
            connection.execute(
                "INSERT INTO metadata VALUES ('schema_version', ?)",
                (INDEX_SCHEMA_VERSION,),
            )
            connection.commit()
        os.replace(staged, destination)
    except Exception:
        staged.unlink(missing_ok=True)
        raise
    return destination


def _pil_to_normalized_tensor(image: Image.Image) -> Tensor:
    width, height = image.size
    storage = torch.frombuffer(
        bytearray(image.tobytes()), dtype=torch.uint8
    ).reshape(height, width, 3)
    tensor = storage.permute(2, 0, 1).to(dtype=torch.float32).div_(255.0)
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    return (tensor - mean) / std


def _mask_to_tensor(mask: Image.Image) -> Tensor:
    width, height = mask.size
    return torch.frombuffer(
        bytearray(mask.tobytes()), dtype=torch.uint8
    ).reshape(height, width).bool()


def _boxes_tensor(rows: Sequence[Sequence[float]]) -> Tensor:
    return (
        torch.tensor(rows, dtype=torch.float32).reshape(-1, 4)
        if rows
        else torch.empty((0, 4), dtype=torch.float32)
    )


class ProposalTransform:
    """Combined aspect resize, affine jitter, visibility policy, and photometrics."""

    def __init__(
        self,
        input_size: int,
        augmentation: AugmentationConfig,
        *,
        training: bool,
        tiny_area: float,
        tiny_min_side: float,
    ) -> None:
        self.input_size = input_size
        self.augmentation = augmentation
        self.training = training
        self.tiny_area = tiny_area
        self.tiny_min_side = tiny_min_side

    def _photometric(self, image: Image.Image, generator: random.Random) -> Image.Image:
        cfg = self.augmentation
        if generator.random() < cfg.color_jitter_probability:
            image = ImageEnhance.Brightness(image).enhance(
                generator.uniform(1 - cfg.brightness, 1 + cfg.brightness)
            )
            image = ImageEnhance.Contrast(image).enhance(
                generator.uniform(1 - cfg.contrast, 1 + cfg.contrast)
            )
            image = ImageEnhance.Color(image).enhance(
                generator.uniform(1 - cfg.saturation, 1 + cfg.saturation)
            )
        if generator.random() < cfg.blur_probability:
            image = image.filter(
                ImageFilter.GaussianBlur(radius=generator.uniform(0.1, 1.0))
            )
        if generator.random() < cfg.jpeg_probability:
            encoded = io.BytesIO()
            image.save(encoded, format="JPEG", quality=generator.randint(45, 95))
            encoded.seek(0)
            with Image.open(encoded) as decoded:
                image = decoded.convert("RGB")
        return image

    def __call__(
        self,
        image: Image.Image,
        boxes: Tensor,
        ignore_boxes: Tensor,
        *,
        seed: int,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor, tuple[float, float, float]]:
        generator = random.Random(seed)
        image = image.convert("RGB")
        width, height = image.size
        all_boxes = torch.cat((boxes, ignore_boxes), dim=0)
        is_positive = torch.arange(all_boxes.shape[0]) < boxes.shape[0]

        if (
            self.training
            and generator.random()
            < self.augmentation.horizontal_flip_probability
        ):
            image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
            if all_boxes.numel():
                old_left = all_boxes[:, 0].clone()
                old_right = all_boxes[:, 2].clone()
                all_boxes[:, 0] = width - old_right
                all_boxes[:, 2] = width - old_left

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
            fillcolor=PADDING_RGB,
        )
        valid = Image.new("L", (width, height), color=255).transform(
            (self.input_size, self.input_size),
            Image.Transform.AFFINE,
            inverse,
            resample=Image.Resampling.NEAREST,
            fillcolor=0,
        )
        if self.training:
            image = self._photometric(image, generator)
        image_tensor = _pil_to_normalized_tensor(image)
        if self.training and generator.random() < self.augmentation.noise_probability:
            noise_generator = torch.Generator().manual_seed(seed)
            image_tensor = image_tensor + torch.randn(
                image_tensor.shape, generator=noise_generator
            ) * 0.02

        if all_boxes.numel() == 0:
            return (
                image_tensor,
                boxes,
                ignore_boxes,
                _mask_to_tensor(valid),
                (scale, offset_x, offset_y),
            )
        transformed = all_boxes * scale
        transformed[:, 0::2] += offset_x
        transformed[:, 1::2] += offset_y
        unclipped_area = (
            (transformed[:, 2] - transformed[:, 0]).clamp(min=0)
            * (transformed[:, 3] - transformed[:, 1]).clamp(min=0)
        )
        clipped = transformed.clone()
        clipped[:, 0::2].clamp_(0, self.input_size)
        clipped[:, 1::2].clamp_(0, self.input_size)
        clipped_width = (clipped[:, 2] - clipped[:, 0]).clamp(min=0)
        clipped_height = (clipped[:, 3] - clipped[:, 1]).clamp(min=0)
        clipped_area = clipped_width * clipped_height
        visibility = clipped_area / unclipped_area.clamp(min=1e-7)
        geometrically_valid = (clipped_width > 0) & (clipped_height > 0)
        positive = (
            is_positive
            & geometrically_valid
            & (visibility >= self.augmentation.positive_visible_fraction)
            & (clipped_area >= self.tiny_area)
            & (torch.minimum(clipped_width, clipped_height) >= self.tiny_min_side)
        )
        ignore = (
            geometrically_valid
            & (visibility >= self.augmentation.ignore_visible_fraction)
            & ~positive
        )
        return (
            image_tensor,
            clipped[positive],
            clipped[ignore],
            _mask_to_tensor(valid),
            (scale, offset_x, offset_y),
        )


class IndexedCocoProposalDataset(Dataset[ProposalSample]):
    """Random-access Phase-3 dataset backed by the streaming SQLite index."""

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
        self.index_path = build_coco_sqlite_index(
            self.annotations, index_path, force=force_index
        )
        self.data_config = data_config
        self.training = training
        self.seed = seed
        self.epoch = 0
        self.transform = ProposalTransform(
            data_config.input_size,
            augmentation,
            training=training,
            tiny_area=data_config.tiny_area,
            tiny_min_side=data_config.tiny_min_side,
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
        self.records = [
            ImageRecord(
                int(row[0]),
                int(row[1]),
                str(row[2]),
                int(row[3]),
                int(row[4]),
                str(row[5]),
                str(row[6]),
                bool(row[7]),
                int(row[8]),
            )
            for row in rows
        ]
        self._connection: sqlite3.Connection | None = None

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return len(self.records)

    def _db(self) -> sqlite3.Connection:
        if self._connection is None:
            self._connection = sqlite3.connect(self.index_path)
        return self._connection

    def __getstate__(self) -> dict[str, Any]:
        state = dict(self.__dict__)
        state["_connection"] = None
        return state

    def __getitem__(self, index: int) -> ProposalSample:
        record = self.records[index]
        rows = self._db().execute(
            """
            SELECT x1, y1, x2, y2, ignore_region, category_name
            FROM annotations WHERE image_id=?
            """,
            (record.image_id,),
        ).fetchall()
        positive_rows = [row for row in rows if not row[4]]
        ignore_rows = [row for row in rows if row[4]]
        component_indices = self._contained_component_indices(positive_rows)
        boxes = _boxes_tensor([
            row[:4]
            for row_index, row in enumerate(positive_rows)
            if row_index not in component_indices
        ])
        ignore_boxes = _boxes_tensor([
            row[:4] for row in ignore_rows
        ] + [
            row[:4]
            for row_index, row in enumerate(positive_rows)
            if row_index in component_indices
        ])
        with Image.open(self.image_root / record.file_name) as loaded:
            image = loaded.convert("RGB")
        image_tensor, boxes, ignore_boxes, valid_mask, transform = self.transform(
            image,
            boxes,
            ignore_boxes,
            seed=self.seed + self.epoch * max(len(self), 1) + index,
        )
        domain = self.data_config.source_domains.get(
            record.source_dataset, "unknown"
        )
        background_supervision = (
            record.source_dataset
            in set(self.data_config.dense_background_sources)
        )
        return ProposalSample(
            image_tensor,
            boxes,
            ignore_boxes,
            valid_mask,
            record.image_id,
            record.source_dataset,
            domain,
            record.camera_type,
            background_supervision,
            (record.height, record.width),
            transform,
        )

    def _contained_component_indices(
        self, rows: Sequence[Sequence[Any]]
    ) -> set[int]:
        components = set(self.data_config.component_categories)
        parents = set(self.data_config.parent_categories)
        parent_boxes = [
            row[:4] for row in rows if str(row[5]) in parents
        ]
        if not parent_boxes:
            return set()
        parent_tensor = _boxes_tensor(parent_boxes)
        ignored: set[int] = set()
        for index, row in enumerate(rows):
            if str(row[5]) not in components:
                continue
            component = _boxes_tensor([row[:4]])
            top_left = torch.maximum(component[:, None, :2], parent_tensor[None, :, :2])
            bottom_right = torch.minimum(
                component[:, None, 2:], parent_tensor[None, :, 2:]
            )
            intersection = (bottom_right - top_left).clamp(min=0).prod(dim=2)
            area = (
                (component[:, 2] - component[:, 0])
                * (component[:, 3] - component[:, 1])
            ).clamp(min=1e-7)
            if (
                intersection.max() / area
                > self.data_config.component_containment_threshold
            ):
                ignored.add(index)
        return ignored


def collate_proposal_samples(
    samples: Sequence[ProposalSample],
) -> tuple[Tensor, list[ProposalSample]]:
    return torch.stack([sample.image for sample in samples]), list(samples)


def _integer_quotas(weights: dict[str, float], batch_size: int) -> dict[str, int]:
    exact = {key: value * batch_size for key, value in weights.items()}
    quotas = {key: math.floor(value) for key, value in exact.items()}
    remainder = batch_size - sum(quotas.values())
    order = sorted(
        weights, key=lambda key: (exact[key] - quotas[key], key), reverse=True
    )
    for key in order[:remainder]:
        quotas[key] += 1
    return quotas


def select_source_mixture_indices(
    records: Sequence[ImageRecord],
    source_weights: dict[str, float],
    count: int,
    *,
    positive_only: bool = False,
) -> list[int]:
    """Select a deterministic finite subset with exact integer source quotas."""
    if count < 1:
        raise ValueError("count must be positive")
    quotas = _integer_quotas(source_weights, count)
    selected: list[int] = []
    observed = {source: 0 for source in quotas}
    for index, record in enumerate(records):
        source = record.source_dataset
        if source not in quotas or observed[source] >= quotas[source]:
            continue
        if positive_only and record.positive_count == 0:
            continue
        selected.append(index)
        observed[source] += 1
        if len(selected) == count:
            break
    missing = {
        source: quotas[source] - observed[source]
        for source in quotas
        if observed[source] < quotas[source]
    }
    if missing:
        raise RuntimeError(
            f"Could not satisfy source-mixture subset quotas: {missing}"
        )
    return selected


class DomainMixtureBatchSampler(Sampler[list[int]]):
    """Fixed domain composition with long-run source and empty-image ratios."""

    def __init__(
        self,
        dataset: IndexedCocoProposalDataset,
        batch_size: int,
        *,
        domain_weights: dict[str, float],
        source_weights: dict[str, float],
        empty_fraction: float,
        seed: int,
        batches_per_epoch: int | None = None,
    ) -> None:
        self.dataset = dataset
        self.batch_size = batch_size
        self.domain_weights = domain_weights
        self.source_weights = source_weights
        self.empty_fraction = empty_fraction
        self.seed = seed
        self.epoch = 0
        self.batches_per_epoch = batches_per_epoch or math.ceil(
            len(dataset) / batch_size
        )
        self.pools: dict[tuple[str, bool], list[int]] = defaultdict(list)
        for index, record in enumerate(dataset.records):
            self.pools[(record.source_dataset, record.positive_count == 0)].append(
                index
            )
        self.domain_sources: dict[str, list[str]] = defaultdict(list)
        for source, domain in dataset.data_config.source_domains.items():
            if any(self.pools[(source, empty)] for empty in (False, True)):
                self.domain_sources[domain].append(source)
        missing = [
            domain
            for domain, weight in domain_weights.items()
            if weight > 0 and not self.domain_sources[domain]
        ]
        if missing:
            raise ValueError(f"No indexed images for domains: {missing}")

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def __len__(self) -> int:
        return self.batches_per_epoch

    def __iter__(self) -> Iterator[list[int]]:
        generator = random.Random(self.seed + self.epoch)
        shuffled: dict[tuple[str, bool], list[int]] = {}
        offsets: dict[tuple[str, bool], int] = defaultdict(int)
        for key, values in self.pools.items():
            shuffled[key] = list(values)
            generator.shuffle(shuffled[key])

        def draw(source: str, want_empty: bool) -> int:
            key = (source, want_empty)
            if not shuffled.get(key):
                key = (source, not want_empty)
            if not shuffled.get(key):
                raise RuntimeError(f"No images available for source {source}")
            if offsets[key] >= len(shuffled[key]):
                generator.shuffle(shuffled[key])
                offsets[key] = 0
            value = shuffled[key][offsets[key]]
            offsets[key] += 1
            return value

        quotas = _integer_quotas(self.domain_weights, self.batch_size)
        for _ in range(self.batches_per_epoch):
            batch: list[int] = []
            for domain, count in quotas.items():
                sources = self.domain_sources[domain]
                weights = [self.source_weights[source] for source in sources]
                for _ in range(count):
                    source = generator.choices(sources, weights=weights, k=1)[0]
                    batch.append(
                        draw(source, generator.random() < self.empty_fraction)
                    )
            generator.shuffle(batch)
            yield batch
