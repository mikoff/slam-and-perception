"""Streaming COCO index, Phase-3 transforms, and fixed-domain batch sampling."""

from __future__ import annotations

import hashlib
import json
import math
import os
import random
import sqlite3
import tempfile
import time
from collections import defaultdict
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
import ijson
import torch
from PIL import Image
from torch import Tensor
from torch.utils.data import Dataset, Sampler

from . import augmentation as _augmentation
from .augmentation import apply_image_augmentation, sample_augmentation_parameters
from .config import AugmentationConfig, DataConfig

IMAGENET_MEAN = _augmentation.IMAGENET_MEAN
IMAGENET_STD = _augmentation.IMAGENET_STD
INDEX_SCHEMA_VERSION = "8"
PROPOSAL_MANIFEST_SCHEMA = "proposal-manifest.v2"


def _json_attribute_default(value: object) -> float:
    if isinstance(value, Decimal):
        return float(value)
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def use_file_system_tensor_sharing(_worker_id: int | None = None) -> None:
    """Avoid file-descriptor exhaustion when DataLoader workers send tensors."""
    torch.multiprocessing.set_sharing_strategy("file_system")


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
    trusted_background_boxes: Tensor | None = None


def _source_signature(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


def build_coco_sqlite_index(
    annotations: str | Path,
    index_path: str | Path,
    *,
    force: bool = False,
    build_if_missing: bool = True,
) -> Path:
    """Build a compact random-access index without loading COCO JSON into RAM."""
    source = Path(annotations).resolve()
    # JSON is emitted with sorted keys, so the manifest's top-level schema
    # field may appear after the first (large) images array.  Inspect only the
    # first streamed image record: manifests use `image_id`, while COCO uses
    # `id`, and this avoids loading the annotation document.
    destination = Path(index_path).resolve()
    if destination.parent.exists():
        cutoff = time.time() - 24 * 60 * 60
        for stale in destination.parent.glob(f".{destination.name}.*.tmp"):
            if stale.is_file() and stale.stat().st_mtime < cutoff:
                stale.unlink()
    signature = _source_signature(source)
    if destination.exists() and not force:
        try:
            with sqlite3.connect(destination) as connection:
                metadata = dict(
                    connection.execute("SELECT key, value FROM metadata").fetchall()
                )
            if metadata.get("source_signature") == signature and metadata.get(
                "schema_version"
            ) in (INDEX_SCHEMA_VERSION, f"{INDEX_SCHEMA_VERSION}:proposal-manifest"):
                return destination
        except Exception:
            pass
    if not build_if_missing:
        raise ValueError(
            f"prebuilt index is missing, corrupt, or does not match annotations: {destination}"
        )

    with source.open("rb") as stream:
        first_image = next(ijson.items(stream, "images.item"), None)

    if isinstance(first_image, dict) and "image_id" in first_image:
        return build_proposal_manifest_sqlite_index(
            source,
            index_path,
            force=force,
            build_if_missing=build_if_missing,
        )

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
                    category_name TEXT NOT NULL,
                    object_condition TEXT NOT NULL,
                    quad_json TEXT NOT NULL,
                    geometry_tier TEXT NOT NULL,
                    fit_coverage REAL NOT NULL,
                    fit_tightness REAL NOT NULL,
                    seen_status TEXT NOT NULL,
                    attributes_json TEXT NOT NULL
                );
                CREATE INDEX annotations_by_image ON annotations(image_id);
                """
            )
            with source.open("rb") as stream:
                for row_index, image in enumerate(ijson.items(stream, "images.item")):
                    image_id = int(image["id"])
                    connection.execute(
                        """
                        INSERT INTO images VALUES (?, ?, ?, ?, ?, ?, ?, ?, 0)
                        """,
                        (
                            row_index,
                            image_id,
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
                    image_id = int(annotation["image_id"])
                    state = str(annotation.get("supervision_state", ""))
                    if state == "trusted_negative":
                        state = "trusted_background"
                    if state:
                        if state not in {
                            "positive",
                            "ignore",
                            "trusted_background",
                        }:
                            raise ValueError(
                                f"unsupported generated supervision state: {state}"
                            )
                        state_code = {
                            "positive": 0,
                            "ignore": 1,
                            "trusted_background": 2,
                        }[state]
                    else:
                        state_code = int(
                            bool(
                                annotation.get("ignore_region")
                                or annotation.get("iscrowd")
                            )
                        )
                    category_name = categories.get(
                        int(annotation.get("category_id", -1)), "unknown"
                    )
                    rows.append(
                        (
                            image_id,
                            x,
                            y,
                            x + width,
                            y + height,
                            state_code,
                            category_name,
                            str(annotation.get("object_condition", "whole_object")),
                            json.dumps(
                                annotation.get("quad")
                                or [
                                    [x, y],
                                    [x + width, y],
                                    [x + width, y + height],
                                    [x, y + height],
                                ],
                                separators=(",", ":"),
                            ),
                            str(annotation.get("geometry_tier", "source_hbb")),
                            float(annotation.get("fit_coverage", 1.0)),
                            float(annotation.get("fit_tightness", 0.0)),
                            str(annotation.get("seen_status", "auxiliary")),
                            json.dumps(
                                annotation.get("attributes", {}),
                                separators=(",", ":"),
                                sort_keys=True,
                                default=_json_attribute_default,
                            ),
                        )
                    )
                    if state_code == 0:
                        positive_counts[image_id] += 1
                    if len(rows) >= 10_000:
                        connection.executemany(
                            "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                            rows,
                        )
                        rows.clear()
                if rows:
                    connection.executemany(
                        "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
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
    except BaseException:
        staged.unlink(missing_ok=True)
        raise
    return destination


def build_proposal_manifest_sqlite_index(
    annotations: str | Path,
    index_path: str | Path,
    *,
    force: bool = False,
    build_if_missing: bool = True,
) -> Path:
    """Index the compact proposal manifest without materializing all images."""
    source = Path(annotations).resolve()
    destination = Path(index_path).resolve()
    signature = _source_signature(source)
    schema_version = f"{INDEX_SCHEMA_VERSION}:proposal-manifest"
    if destination.exists() and not force:
        with sqlite3.connect(destination) as connection:
            metadata = dict(
                connection.execute("SELECT key, value FROM metadata").fetchall()
            )
        if (
            metadata.get("source_signature") == signature
            and metadata.get("schema_version") == schema_version
        ):
            return destination
    if not build_if_missing:
        raise ValueError(
            f"prebuilt index is missing or does not match proposal manifest: {destination}"
        )
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
                    x1 REAL NOT NULL, y1 REAL NOT NULL,
                    x2 REAL NOT NULL, y2 REAL NOT NULL,
                    ignore_region INTEGER NOT NULL,
                    category_name TEXT NOT NULL,
                    object_condition TEXT NOT NULL,
                    quad_json TEXT NOT NULL,
                    geometry_tier TEXT NOT NULL,
                    fit_coverage REAL NOT NULL,
                    fit_tightness REAL NOT NULL,
                    seen_status TEXT NOT NULL,
                    attributes_json TEXT NOT NULL
                );
                CREATE INDEX annotations_by_image ON annotations(image_id);
                """
            )
            with source.open("rb") as stream:
                for row_index, image in enumerate(ijson.items(stream, "images.item")):
                    image_id = int(image["image_id"])
                    connection.execute(
                        "INSERT INTO images VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                        (
                            row_index,
                            image_id,
                            str(image["file_name"]),
                            int(image["width"]),
                            int(image["height"]),
                            str(image.get("source_dataset", "unknown")),
                            str(image.get("camera_type", "perspective")),
                            int(bool(image.get("background_supervision", False))),
                            len(image.get("positive", [])),
                        ),
                    )
                    rows = []
                    for state_code, records in (
                        (0, image.get("positive", [])),
                        (1, image.get("ignore", [])),
                        (2, image.get("trusted_background", [])),
                    ):
                        for record in records:
                            if not record.get("valid", True):
                                continue
                            x, y, width, height = map(float, record["bbox"])
                            quad = [
                                [float(point[0]), float(point[1])]
                                for point in record["quad"]
                            ]
                            rows.append(
                                (
                                    image_id,
                                    x,
                                    y,
                                    x + width,
                                    y + height,
                                    state_code,
                                    str(
                                        record.get(
                                            "canonical_category",
                                            record.get("source_category", "unknown"),
                                        )
                                    ),
                                    str(record.get("object_condition", "whole_object")),
                                    json.dumps(quad, separators=(",", ":")),
                                    str(record.get("geometry_tier", "source_hbb")),
                                    float(record.get("fit_coverage", 1.0)),
                                    float(record.get("fit_tightness", 0.0)),
                                    str(record.get("seen_status", "auxiliary")),
                                    json.dumps(
                                        record.get("attributes", {}),
                                        separators=(",", ":"),
                                        sort_keys=True,
                                        default=_json_attribute_default,
                                    ),
                                )
                            )
                    if rows:
                        connection.executemany(
                            "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                            rows,
                        )
            connection.execute(
                "INSERT INTO metadata VALUES ('source_signature', ?)", (signature,)
            )
            connection.execute(
                "INSERT INTO metadata VALUES ('schema_version', ?)", (schema_version,)
            )
            connection.commit()
        os.replace(staged, destination)
    except BaseException:
        staged.unlink(missing_ok=True)
        raise
    return destination


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

    def __call__(
        self,
        image: Image.Image,
        boxes: Tensor,
        ignore_boxes: Tensor,
        trusted_background_boxes: Tensor | None = None,
        *,
        seed: int,
    ) -> tuple[Tensor, Tensor, Tensor, Tensor, Tensor, tuple[float, float, float]]:
        image = image.convert("RGB")
        width, height = image.size
        parameters = sample_augmentation_parameters(
            self.augmentation,
            self.input_size,
            training=self.training,
            seed=seed,
        )
        trusted_background_boxes = (
            trusted_background_boxes
            if trusted_background_boxes is not None
            else boxes.new_empty((0, 4))
        )
        all_boxes = torch.cat((boxes, ignore_boxes, trusted_background_boxes), dim=0)
        state = torch.cat(
            (
                torch.zeros(boxes.shape[0], dtype=torch.long),
                torch.ones(ignore_boxes.shape[0], dtype=torch.long),
                torch.full((trusted_background_boxes.shape[0],), 2, dtype=torch.long),
            )
        )

        if parameters.horizontal_flip:
            if all_boxes.numel():
                old_left = all_boxes[:, 0].clone()
                old_right = all_boxes[:, 2].clone()
                all_boxes[:, 0] = width - old_right
                all_boxes[:, 2] = width - old_left
        augmented = apply_image_augmentation(image, self.input_size, parameters)
        image_tensor = augmented.tensor
        scale, offset_x, offset_y = augmented.transform

        if all_boxes.numel() == 0:
            return (
                image_tensor,
                boxes,
                ignore_boxes,
                trusted_background_boxes,
                augmented.valid_mask,
                augmented.transform,
            )
        transformed = all_boxes * scale
        transformed[:, 0::2] += offset_x
        transformed[:, 1::2] += offset_y
        unclipped_area = (transformed[:, 2] - transformed[:, 0]).clamp(min=0) * (
            transformed[:, 3] - transformed[:, 1]
        ).clamp(min=0)
        clipped = transformed.clone()
        clipped[:, 0::2].clamp_(0, self.input_size)
        clipped[:, 1::2].clamp_(0, self.input_size)
        clipped_width = (clipped[:, 2] - clipped[:, 0]).clamp(min=0)
        clipped_height = (clipped[:, 3] - clipped[:, 1]).clamp(min=0)
        clipped_area = clipped_width * clipped_height
        visibility = clipped_area / unclipped_area.clamp(min=1e-7)
        geometrically_valid = (clipped_width > 0) & (clipped_height > 0)
        positive = (
            (state == 0)
            & geometrically_valid
            & (visibility >= self.augmentation.positive_visible_fraction)
            & (clipped_area >= self.tiny_area)
            & (torch.minimum(clipped_width, clipped_height) >= self.tiny_min_side)
        )
        ignore = (
            (state != 2)
            & geometrically_valid
            & (visibility >= self.augmentation.ignore_visible_fraction)
            & ~positive
        )
        trusted = (
            (state == 2)
            & geometrically_valid
            & (visibility >= self.augmentation.ignore_visible_fraction)
        )
        return (
            image_tensor,
            clipped[positive],
            clipped[ignore],
            clipped[trusted],
            augmented.valid_mask,
            augmented.transform,
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
            state_rows = connection.execute(
                "SELECT ignore_region, COUNT(*) FROM annotations GROUP BY ignore_region"
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
        state_names = {0: "positive", 1: "ignore", 2: "trusted_background"}
        counts = {int(state): int(count) for state, count in state_rows}
        self.state_counts = {
            name: counts.get(state, 0) for state, name in state_names.items()
        }
        self._connection: sqlite3.Connection | None = None
        self._connection_pid: int | None = None

    def _annotation_rows(
        self, image_id: int
    ) -> list[tuple[float, float, float, float, int]]:
        """Read one image's annotations using a process-local read-only connection."""
        pid = os.getpid()
        if self._connection is None or self._connection_pid != pid:
            if self._connection is not None:
                self._connection.close()
            uri = f"{self.index_path.as_uri()}?mode=ro&immutable=1"
            self._connection = sqlite3.connect(uri, uri=True)
            self._connection_pid = pid
        return self._connection.execute(
            "SELECT x1, y1, x2, y2, ignore_region FROM annotations WHERE image_id=?",
            (image_id,),
        ).fetchall()

    def __getstate__(self) -> dict[str, object]:
        state = dict(self.__dict__)
        state["_connection"] = None
        state["_connection_pid"] = None
        return state

    def set_epoch(self, epoch: int) -> None:
        self._epoch.fill_(epoch)

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int) -> ProposalSample:
        record = self.records[index]
        rows = self._annotation_rows(record.image_id)
        positive_rows = [row for row in rows if row[4] == 0]
        ignore_rows = [row for row in rows if row[4] == 1]
        trusted_rows = [row for row in rows if row[4] == 2]
        boxes = _boxes_tensor([row[:4] for row in positive_rows])
        ignore_boxes = _boxes_tensor([row[:4] for row in ignore_rows])
        trusted_background_boxes = _boxes_tensor([row[:4] for row in trusted_rows])
        with Image.open(self.image_root / record.file_name) as loaded:
            image = loaded.convert("RGB")
        transformed = self.transform(
            image,
            boxes,
            ignore_boxes,
            trusted_background_boxes,
            seed=self.seed + int(self._epoch.item()) * max(len(self), 1) + index,
        )
        domain = self.data_config.source_domains.get(record.source_dataset, "unknown")
        background_supervision = bool(transformed[3].numel())
        return ProposalSample(
            transformed[0],
            transformed[1],
            transformed[2],
            transformed[4],
            record.image_id,
            record.source_dataset,
            domain,
            record.camera_type,
            background_supervision,
            (record.height, record.width),
            transformed[5],
            transformed[3],
        )


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


def mixture_quota_delta(
    weights: dict[str, float], start_sample: int, sample_count: int
) -> dict[str, int]:
    """Return cumulative largest-remainder quotas for a sample interval."""
    if start_sample < 0 or sample_count < 0:
        raise ValueError("sample offsets and counts must be non-negative")
    before = _integer_quotas(weights, start_sample)
    after = _integer_quotas(weights, start_sample + sample_count)
    return {key: after[key] - before[key] for key in weights}


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
        raise RuntimeError(f"Could not satisfy source-mixture subset quotas: {missing}")
    return selected


class DomainMixtureBatchSampler(Sampler[list[int]]):
    """Deterministic residual source quotas with long-run empty-image ratios."""

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
        self.start_batch = 0
        self.batches_per_epoch = batches_per_epoch or math.ceil(
            len(dataset) / batch_size
        )
        self.pools: dict[tuple[str, bool], list[int]] = defaultdict(list)
        for index, record in enumerate(dataset.records):
            self.pools[(record.source_dataset, record.positive_count == 0)].append(
                index
            )
        missing = [
            source
            for source, weight in source_weights.items()
            if weight > 0
            and not any(self.pools[(source, empty)] for empty in (False, True))
        ]
        if missing:
            raise ValueError(f"No indexed images for sources: {missing}")
        source_domain_weights = {domain: 0.0 for domain in domain_weights}
        for source, weight in source_weights.items():
            domain = dataset.data_config.source_domains.get(source)
            if domain not in source_domain_weights:
                raise ValueError(f"Source {source!r} has no configured domain weight")
            source_domain_weights[domain] += weight
        mismatched = {
            domain: (domain_weights[domain], source_domain_weights[domain])
            for domain in domain_weights
            if not math.isclose(
                domain_weights[domain], source_domain_weights[domain], abs_tol=1e-9
            )
        }
        if mismatched:
            raise ValueError(
                "source weights must sum to each configured domain weight: "
                f"{mismatched}"
            )

    def set_epoch(self, epoch: int) -> None:
        self.epoch = epoch

    def set_start_batch(self, start_batch: int) -> None:
        if not 0 <= start_batch <= self.batches_per_epoch:
            raise ValueError(f"start_batch must be in [0, {self.batches_per_epoch}]")
        self.start_batch = start_batch

    def __len__(self) -> int:
        return self.batches_per_epoch

    def batch_source_quotas(self, batch_index: int) -> dict[str, int]:
        """Return the exact source quota scheduled for one epoch batch."""
        if not 0 <= batch_index < self.batches_per_epoch:
            raise ValueError(f"batch_index must be in [0, {self.batches_per_epoch})")
        quotas = mixture_quota_delta(
            self.source_weights,
            batch_index * self.batch_size,
            self.batch_size,
        )
        if sum(quotas.values()) != self.batch_size or any(
            count < 0 for count in quotas.values()
        ):
            raise RuntimeError(
                f"invalid residual quota at batch {batch_index}: {quotas}"
            )
        return quotas

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

        for batch_index in range(self.batches_per_epoch):
            batch: list[int] = []
            for source, count in self.batch_source_quotas(batch_index).items():
                for _ in range(count):
                    batch.append(draw(source, generator.random() < self.empty_fraction))
            generator.shuffle(batch)
            if batch_index >= self.start_batch:
                yield batch
