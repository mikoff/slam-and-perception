"""Authoritative semantic evaluation records preserving object and image identity."""

from __future__ import annotations

import json
import math
import sqlite3
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from .ontology import map_to_canonical


@dataclass(frozen=True)
class SemanticGroundTruthRecord:
    """Deterministic ground-truth object record preserving semantic identity."""

    annotation_id: int
    image_id: int
    source_dataset: str
    source_category: str
    canonical_category: str | None
    quad: tuple[tuple[float, float], ...]
    bbox: tuple[float, float, float, float]
    ignore_region: int
    geometry_tier: str
    size_bin: str
    area: float

    @property
    def is_positive(self) -> bool:
        return self.ignore_region == 0

    @property
    def is_ignore(self) -> bool:
        return self.ignore_region == 1

    @property
    def is_trusted_background(self) -> bool:
        return self.ignore_region == 2


@dataclass(frozen=True)
class SemanticImageRecord:
    """Image-level record binding file path and associated ground truth annotations."""

    image_id: int
    source_dataset: str
    file_name: str
    image_path: Path
    width: int
    height: int
    ground_truths: tuple[SemanticGroundTruthRecord, ...]

    @property
    def positive_ground_truths(self) -> tuple[SemanticGroundTruthRecord, ...]:
        return tuple(gt for gt in self.ground_truths if gt.is_positive)


def _compute_size_bin(w: float, h: float) -> str:
    scale = math.sqrt(max(w * h, 0.0))
    if scale < 16.0:
        return "tiny"
    if scale < 32.0:
        return "small"
    if scale < 96.0:
        return "medium"
    return "large"


def load_semantic_image_records(
    dataset_root: Path,
    image_ids: Sequence[int] | None = None,
    index_path: Path | None = None,
    sanitize_annotations: bool = True,
) -> list[SemanticImageRecord]:
    """Load semantic evaluation records preserving object identities from SQLite.

    /// @brief Reads images and annotations, joining them with canonical ontology mapping.
    /// @param dataset_root Root directory of the visual dataset.
    /// @param image_ids Optional sequence of specific image_ids to filter.
    /// @param index_path Optional explicit path to SQLite index; defaults to dataset_root/indexes/quad_val.sqlite.
    /// @param sanitize_annotations Whether to apply cross-tier deduplication and WoodScape fragment hygiene.
    /// @return Ordered list of SemanticImageRecord instances.
    """
    resolved_root = Path(dataset_root).resolve()
    db_path = index_path.resolve() if index_path is not None else resolved_root / "indexes/quad_val.sqlite"

    with sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True) as connection:
        if image_ids is None:
            image_rows = connection.execute(
                "SELECT image_id, file_name, width, height, source_dataset FROM images ORDER BY row_index"
            ).fetchall()
        else:
            id_list = list(image_ids)
            image_rows = []
            chunk_size = 500
            for i in range(0, len(id_list), chunk_size):
                chunk = id_list[i : i + chunk_size]
                placeholders = ",".join(["?"] * len(chunk))
                image_rows.extend(
                    connection.execute(
                        f"SELECT image_id, file_name, width, height, source_dataset FROM images WHERE image_id IN ({placeholders}) ORDER BY row_index",
                        chunk,
                    ).fetchall()
                )

        if not image_rows:
            return []

        selected_ids = [r[0] for r in image_rows]
        if image_ids is None:
            ann_rows = connection.execute(
                """
                SELECT rowid, image_id, quad_json, x1, y1, x2, y2,
                       ignore_region, category_name, geometry_tier
                FROM annotations
                ORDER BY image_id, rowid
                """
            ).fetchall()
        else:
            ann_rows = []
            chunk_size = 500
            for i in range(0, len(selected_ids), chunk_size):
                chunk = selected_ids[i : i + chunk_size]
                placeholders = ",".join(["?"] * len(chunk))
                ann_rows.extend(
                    connection.execute(
                        f"""
                        SELECT rowid, image_id, quad_json, x1, y1, x2, y2,
                               ignore_region, category_name, geometry_tier
                        FROM annotations
                        WHERE image_id IN ({placeholders})
                        ORDER BY image_id, rowid
                        """,
                        chunk,
                    ).fetchall()
                )

    annotations_by_image: dict[int, list[SemanticGroundTruthRecord]] = defaultdict(list)
    image_source_map = {r[0]: str(r[4]) for r in image_rows}
    grouped_rows: dict[int, list[tuple[Any, ...]]] = defaultdict(list)
    for row in ann_rows:
        grouped_rows[int(row[1])].append(row)

    for img_id, img_rows in grouped_rows.items():
        source_dataset = image_source_map.get(img_id, "unknown")
        if sanitize_annotations:
            hygiene_rows = [
                RawAnnotationRow(
                    x1=float(r[3]),
                    y1=float(r[4]),
                    x2=float(r[5]),
                    y2=float(r[6]),
                    ignore_region=int(r[7]),
                    category_name=str(r[8]),
                    geometry_tier=str(r[9]),
                )
                for r in img_rows
            ]
            decision = sanitize_image_annotations(
                hygiene_rows, source_dataset=source_dataset
            )
            demoted_set = set(decision.demoted_to_ignore_indices)
            dropped_set = set(decision.dropped_indices)
        else:
            demoted_set = set()
            dropped_set = set()

        for row_idx, row in enumerate(img_rows):
            if row_idx in dropped_set:
                continue
            rowid, _img_id, quad_json, x1, y1, x2, y2, ignore, cat_name, tier = row
            eff_ignore = 1 if row_idx in demoted_set else int(ignore)
            pts = json.loads(quad_json)
            quad_tuple = tuple((float(p[0]), float(p[1])) for p in pts)
            w = max(float(x2) - float(x1), 0.0)
            h = max(float(y2) - float(y1), 0.0)
            area = w * h
            size_bin = _compute_size_bin(w, h)
            canonical = map_to_canonical(cat_name)

            rec = SemanticGroundTruthRecord(
                annotation_id=int(rowid),
                image_id=int(img_id),
                source_dataset=source_dataset,
                source_category=str(cat_name),
                canonical_category=canonical,
                quad=quad_tuple,
                bbox=(float(x1), float(y1), float(x2), float(y2)),
                ignore_region=eff_ignore,
                geometry_tier=str(tier),
                size_bin=size_bin,
                area=area,
            )
            annotations_by_image[img_id].append(rec)

    records: list[SemanticImageRecord] = []
    for r in image_rows:
        img_id, file_name, width, height, source_dataset = r
        full_path = resolved_root / str(file_name)
        gt_list = tuple(annotations_by_image.get(img_id, []))
        records.append(
            SemanticImageRecord(
                image_id=int(img_id),
                source_dataset=str(source_dataset),
                file_name=str(file_name),
                image_path=full_path,
                width=int(width),
                height=int(height),
                ground_truths=gt_list,
            )
        )

    return records


def select_stratified_semantic_cohort(
    dataset_root: Path,
    target_categories: Sequence[str],
    quota_per_category: int = 30,
    seed: int = 42,
    index_path: Path | None = None,
) -> list[int]:
    """Select a reproducible subset of image IDs stratified across target categories.

    /// @brief Identifies images containing target (especially rare) categories for balanced evaluation.
    /// @param dataset_root Root directory of dataset.
    /// @param target_categories Categories to stratify over.
    /// @param quota_per_category Desired maximum images per category.
    /// @param seed Deterministic seed for sampling.
    /// @return Ordered list of selected image_ids.
    """
    import random

    resolved_root = Path(dataset_root).resolve()
    db_path = index_path.resolve() if index_path is not None else resolved_root / "indexes/quad_val.sqlite"

    with sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True) as connection:
        ann_rows = connection.execute(
            "SELECT image_id, category_name FROM annotations WHERE ignore_region=0"
        ).fetchall()

    category_images: dict[str, set[int]] = defaultdict(set)
    for img_id, cat_name in ann_rows:
        canonical = map_to_canonical(cat_name)
        if canonical:
            category_images[canonical].add(img_id)

    rng = random.Random(seed)
    selected_set: set[int] = set()

    # Prioritize rarest categories first
    sorted_targets = sorted(
        target_categories,
        key=lambda cat: len(category_images.get(cat, set())),
    )

    for cat in sorted_targets:
        pool = list(category_images.get(cat, set()))
        pool.sort()
        rng.shuffle(pool)
        needed = quota_per_category
        # Count already selected images that contain this category
        already_have = sum(1 for img_id in selected_set if img_id in category_images.get(cat, set()))
        to_add = max(0, needed - already_have)
        for img_id in pool:
            if to_add <= 0:
                break
            if img_id not in selected_set:
                selected_set.add(img_id)
                to_add -= 1

    result = sorted(selected_set)
    return result


__all__ = [
    "SemanticGroundTruthRecord",
    "SemanticImageRecord",
    "load_semantic_image_records",
    "select_stratified_semantic_cohort",
]
