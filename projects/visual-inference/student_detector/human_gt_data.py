"""Dataset loader for high-quality human ground-truth boxes on the 10k cohort.

Supports Stage-2 fine-tuning to tighten bounding box proposals and calibrate objectness.
"""

from __future__ import annotations

from collections.abc import Sequence
import json
import os
from pathlib import Path
import sqlite3
from typing import Any

from PIL import Image
import torch
from torch.utils.data import Dataset

from .annotation_hygiene import RawAnnotationRow, sanitize_image_annotations
from .config import AugmentationConfig
from .data import (
    ProposalSample,
    ProposalTransform,
    _boxes_tensor,
    collate_proposal_samples,
)
from .quad_data import (
    QuadProposalSample,
    QuadProposalTransform,
    _quad_slice_labels,
    _quad_tensor,
    collate_quad_proposal_samples,
)
from .sam_student_data import (
    CAMERA_TYPES,
    DEFAULT_DATA_ROOT,
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_DB,
    SOURCE_DOMAINS,
    split_cohort_indices,
)

DEFAULT_GT_INDEX_PATH = Path(
    os.environ.get(
        "VISUAL_INFERENCE_GT_INDEX",
        str(DEFAULT_DATA_ROOT / "output" / "indexes" / "quad_train.sqlite"),
    )
)
DEFAULT_ENRICHED_GT_INDEX_PATH = Path(
    os.environ.get(
        "VISUAL_INFERENCE_ENRICHED_GT_INDEX",
        str(DEFAULT_DATA_ROOT / "output" / "indexes" / "quad_train_enriched.sqlite"),
    )
)

STAGE2_COHORT_QUOTAS: dict[str, int] = {
    "coco_2017": 5000,
    "bdd100k_images_100k": 2000,
    "nuimages": 2000,
    "woodscape_rgb_fisheye": 1000,
}

STAGE2_ENRICHED_COHORT_QUOTAS: dict[str, int] = {
    "coco_2017": 3500,
    "bdd100k_images_100k": 2500,
    "nuimages": 2000,
    "woodscape_rgb_fisheye": 1500,
    "ade20k": 2500,
    "open_images_subset": 2000,
}

STAGE2_8K_COHORT_QUOTAS: dict[str, int] = {
    "coco_2017": 1600,
    "bdd100k_images_100k": 1400,
    "nuimages": 1200,
    "woodscape_rgb_fisheye": 1000,
    "ade20k": 1400,
    "open_images_subset": 1400,
}


def get_stage2_cohort_ids(
    gt_index_path: Path | str = DEFAULT_GT_INDEX_PATH,
    target_count: int | None = None,
) -> set[int]:
    """Return authoritative cohort image IDs matching the Stage-2 benchmark contract."""
    index_str = str(gt_index_path)
    is_enriched = "enriched" in index_str or (
        target_count is not None and target_count >= 8000
    )
    if is_enriched:
        if target_count == 8000:
            quotas = dict(STAGE2_8K_COHORT_QUOTAS)
        elif target_count is not None and target_count != sum(STAGE2_ENRICHED_COHORT_QUOTAS.values()):
            total_base = sum(STAGE2_ENRICHED_COHORT_QUOTAS.values())
            quotas = {
                k: max(1, int(round(v * target_count / total_base)))
                for k, v in STAGE2_ENRICHED_COHORT_QUOTAS.items()
            }
        else:
            quotas = dict(STAGE2_ENRICHED_COHORT_QUOTAS)
    else:
        quotas = dict(STAGE2_COHORT_QUOTAS)
    limit = target_count or sum(quotas.values())

    with sqlite3.connect(f"file:{gt_index_path}?mode=ro&immutable=1", uri=True) as conn:
        rows = conn.execute(
            "SELECT image_id, source_dataset FROM images ORDER BY row_index"
        ).fetchall()

    observed: dict[str, int] = {k: 0 for k in quotas}
    selected_ids: set[int] = set()
    for r in rows:
        img_id = int(r[0])
        source = str(r[1])
        if source in quotas and observed[source] < quotas[source]:
            selected_ids.add(img_id)
            observed[source] += 1
        if len(selected_ids) >= limit:
            break
    return selected_ids


def get_stage2_10k_cohort_ids(
    gt_index_path: Path | str = DEFAULT_GT_INDEX_PATH,
) -> set[int]:
    """Backwards-compatible alias returning the 10,000 image cohort IDs."""
    return get_stage2_cohort_ids(gt_index_path, target_count=10000)


def load_cohort_image_records(
    cohort_db_path: Path | str,
    gt_index_path: Path | str,
    cohort_size: int | None = 10000,
) -> list[dict[str, Any]]:
    """Load image records matching the requested cohort size and database indexes."""
    cohort_path = Path(cohort_db_path)
    gt_path = Path(gt_index_path)
    is_enriched = "enriched" in str(gt_path) or (
        cohort_size is not None and cohort_size > 10000
    )

    if is_enriched and (
        cohort_path == DEFAULT_PSEUDO_LABELS_DB or cohort_path == gt_path
    ):
        target_ids = get_stage2_cohort_ids(gt_path, target_count=cohort_size)
        with sqlite3.connect(f"file:{gt_path}?mode=ro&immutable=1", uri=True) as conn:
            conn.execute(
                "CREATE TEMP TABLE target_cohort_ids (image_id INTEGER PRIMARY KEY)"
            )
            conn.executemany(
                "INSERT INTO target_cohort_ids VALUES (?)",
                [(i,) for i in target_ids],
            )
            cohort_rows = conn.execute(
                """
                SELECT img.image_id, img.file_name, img.source_dataset, img.width, img.height
                FROM images img
                JOIN target_cohort_ids tgt ON img.image_id = tgt.image_id
                ORDER BY img.image_id
                """
            ).fetchall()
        return [
            {
                "image_id": int(r[0]),
                "file_name": str(r[1]),
                "source_dataset": str(r[2]),
                "width": int(r[3]),
                "height": int(r[4]),
            }
            for r in cohort_rows
        ]

    with sqlite3.connect(
        f"file:{cohort_path}?mode=ro&immutable=1", uri=True
    ) as cohort_conn:
        cohort_rows = cohort_conn.execute(
            "SELECT image_id, file_name, source_dataset, width, height FROM images ORDER BY image_id"
        ).fetchall()

    all_images = [
        {
            "image_id": int(r[0]),
            "file_name": str(r[1]),
            "source_dataset": str(r[2]),
            "width": int(r[3]),
            "height": int(r[4]),
        }
        for r in cohort_rows
    ]

    if cohort_size == 10000:
        target_ids = get_stage2_10k_cohort_ids(gt_path)
        all_images = [img for img in all_images if img["image_id"] in target_ids]
    elif cohort_size is not None:
        target_ids = get_stage2_cohort_ids(gt_path, target_count=cohort_size)
        all_images = [img for img in all_images if img["image_id"] in target_ids]

    return all_images


def _prepare_hygiene_masks(
    rows: Sequence[RawAnnotationRow],
    source_dataset: str,
    sanitize: bool,
) -> tuple[dict[int, int], set[int], set[int], set[int]]:
    """Map raw positive row indices and compute keep/demote/drop row index sets."""
    raw_pos_idx_by_row: dict[int, int] = {}
    raw_pos_counter = 0
    for row_idx, hr in enumerate(rows):
        if hr.ignore_region == 0:
            raw_pos_idx_by_row[row_idx] = raw_pos_counter
            raw_pos_counter += 1

    if sanitize:
        decision = sanitize_image_annotations(rows, source_dataset=source_dataset)
        return (
            raw_pos_idx_by_row,
            set(decision.kept_positive_indices),
            set(decision.demoted_to_ignore_indices),
            set(decision.dropped_indices),
        )
    return raw_pos_idx_by_row, set(raw_pos_idx_by_row.keys()), set(), set()


class HumanGTHBBDataset(Dataset[ProposalSample]):
    """PyTorch Dataset loading authoritative human ground-truth boxes for student fine-tuning.

    /// @brief Loads images from the balanced 10k discovery cohort and binds them
    /// strictly to verified human ground-truth bounding boxes, ignoring pseudo-labels.
    """

    def __init__(
        self,
        cohort_db_path: Path | str = DEFAULT_PSEUDO_LABELS_DB,
        gt_index_path: Path | str = DEFAULT_GT_INDEX_PATH,
        image_root: Path | str = DEFAULT_IMAGE_ROOT,
        split: str = "train",
        val_ratio: float = 0.10,
        split_seed: int = 42,
        input_size: int = 384,
        cohort_size: int | None = 10000,
        augmentation: AugmentationConfig | None = None,
        tiny_area: float = 4.0,
        tiny_min_side: float = 2.0,
        seed: int = 42,
        sanitize_annotations: bool = True,
    ) -> None:
        """Initialize the Human GT HBB dataset.

        @param cohort_db_path Path to the sam_discovery.sqlite database defining the cohort.
        @param gt_index_path Path to quad_train.sqlite index containing human ground truth.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param cohort_size If specified, restrict to the target cohort size.
        @param augmentation Data augmentation configuration (used when split='train').
        @param tiny_area Minimum pixel area in resized input space to retain box.
        @param tiny_min_side Minimum side length in resized input space to retain box.
        @param seed Runtime random seed for epoch-dependent augmentations.
        @param sanitize_annotations Whether to apply cross-tier deduplication and WoodScape fragment hygiene.
        """
        super().__init__()
        self.cohort_db_path = Path(cohort_db_path)
        self.gt_index_path = Path(gt_index_path)
        self.image_root = Path(image_root)
        self.split = split
        self.input_size = input_size
        self.tiny_area = tiny_area
        self.tiny_min_side = tiny_min_side
        self.seed = seed
        self.sanitize_annotations = bool(sanitize_annotations)
        self._epoch = torch.zeros(1, dtype=torch.long)

        self._connection: sqlite3.Connection | None = None

        is_enriched = "enriched" in str(self.gt_index_path) or (
            cohort_size is not None and cohort_size > 10000
        )
        if not is_enriched and not self.cohort_db_path.exists():
            raise FileNotFoundError(f"Cohort database not found: {self.cohort_db_path}")
        if not self.gt_index_path.exists():
            raise FileNotFoundError(
                f"Human GT index database not found: {self.gt_index_path}"
            )

        # 1. Fetch cohort images
        all_images = load_cohort_image_records(
            cohort_db_path=self.cohort_db_path,
            gt_index_path=self.gt_index_path,
            cohort_size=cohort_size,
        )

        train_indices, val_indices = split_cohort_indices(
            all_images, val_ratio=val_ratio, seed=split_seed
        )

        if split == "train":
            indices = train_indices
        elif split == "val":
            indices = val_indices
        elif split == "all":
            indices = list(range(len(all_images)))
        else:
            raise ValueError(
                f"Unknown split: {split}. Must be 'train', 'val', or 'all'"
            )

        self.images = [all_images[idx] for idx in indices]

        if augmentation is None:
            augmentation = AugmentationConfig()
        self.augmentation = augmentation

        self.transform = ProposalTransform(
            input_size=input_size,
            augmentation=augmentation,
            training=(split == "train"),
            tiny_area=tiny_area,
            tiny_min_side=tiny_min_side,
        )

    def _get_connection(self) -> sqlite3.Connection:
        """Lazily initialize SQLite connection per process/worker."""
        if self._connection is None:
            self._connection = sqlite3.connect(
                f"file:{self.gt_index_path}?mode=ro&immutable=1",
                uri=True,
                check_same_thread=False,
            )
        return self._connection

    def _get_category_column(self, conn: sqlite3.Connection) -> str | None:
        if not hasattr(self, "_cat_column_name"):
            cols = [r[1] for r in conn.execute("PRAGMA table_info(annotations)").fetchall()]
            if "category_name" in cols:
                self._cat_column_name: str | None = "category_name"
            elif "category" in cols:
                self._cat_column_name = "category"
            else:
                self._cat_column_name = None
            self._has_geom_tier = "geometry_tier" in cols
        return self._cat_column_name

    def _has_geometry_tier_column(self, conn: sqlite3.Connection) -> bool:
        self._get_category_column(conn)
        return bool(getattr(self, "_has_geom_tier", False))

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> ProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]
        conn = self._get_connection()

        img_id = record["image_id"]
        cat_col = self._get_category_column(conn)
        has_tier = self._has_geometry_tier_column(conn)
        cat_sql = f", {cat_col}" if cat_col else ", 'unknown'"
        tier_sql = ", geometry_tier" if has_tier else ", 'unknown'"
        ann_rows = conn.execute(
            f"""
            SELECT x1, y1, x2, y2, ignore_region{cat_sql}{tier_sql}
            FROM annotations
            WHERE image_id = ?
            ORDER BY rowid
            """,
            (img_id,),
        ).fetchall()

        positive_boxes: list[list[float]] = []
        ignore_boxes: list[list[float]] = []
        trusted_background_boxes: list[list[float]] = []
        positive_categories: list[str] = []
        kept_raw_pos_indices: list[int] = []

        hygiene_rows: list[RawAnnotationRow] = [
            RawAnnotationRow(
                x1=float(r[0]),
                y1=float(r[1]),
                x2=float(r[2]),
                y2=float(r[3]),
                ignore_region=int(r[4]),
                category_name=str(r[5]) if r[5] else "unknown",
                geometry_tier=str(r[6]) if r[6] else "unknown",
            )
            for r in ann_rows
        ]
        raw_pos_idx_by_row, kept_set, demoted_set, dropped_set = _prepare_hygiene_masks(
            hygiene_rows,
            source_dataset=str(record["source_dataset"]),
            sanitize=self.sanitize_annotations,
        )

        for row_idx, hr in enumerate(hygiene_rows):
            bbox = [hr.x1, hr.y1, hr.x2, hr.y2]
            if row_idx in dropped_set:
                continue
            if hr.ignore_region == 0:
                if row_idx in demoted_set:
                    ignore_boxes.append(bbox)
                elif row_idx in kept_set:
                    positive_boxes.append(bbox)
                    positive_categories.append(hr.category_name)
                    kept_raw_pos_indices.append(raw_pos_idx_by_row[row_idx])
            elif hr.ignore_region == 2:
                trusted_background_boxes.append(bbox)
            else:
                ignore_boxes.append(bbox)

        with Image.open(image_path) as loaded:
            image = loaded.convert("RGB")

        seed = self.seed + int(self._epoch.item()) * max(len(self), 1) + index
        transformed = self.transform(
            image,
            _boxes_tensor(positive_boxes),
            _boxes_tensor(ignore_boxes),
            _boxes_tensor(trusted_background_boxes),
            seed=seed,
            return_metadata=True,
        )

        source_dataset = record["source_dataset"]
        domain = SOURCE_DOMAINS.get(source_dataset, "general")
        camera_type = CAMERA_TYPES.get(source_dataset, "pinhole")
        background_supervision = bool(transformed[3].numel())
        retained: tuple[int, ...] = transformed[7]
        horizontal_flip = bool(transformed[8])

        return ProposalSample(
            image=transformed[0],
            boxes=transformed[1],
            ignore_boxes=transformed[2],
            valid_mask=transformed[4],
            image_id=img_id,
            source_dataset=source_dataset,
            domain=domain,
            camera_type=camera_type,
            background_supervision=background_supervision,
            original_size=(int(record["height"]), int(record["width"])),
            transform=transformed[5],
            trusted_background_boxes=transformed[3],
            hard_negative_focus_boxes=transformed[6],
            file_name=str(record["file_name"]),
            category_names=tuple(positive_categories[idx] for idx in retained),
            gt_indices=tuple(kept_raw_pos_indices[idx] for idx in retained),
            horizontal_flip=horizontal_flip,
        )


class HumanGTQuadDataset(Dataset[QuadProposalSample]):
    """PyTorch Dataset loading authoritative human ground-truth quads for student fine-tuning.

    /// @brief Loads images from the balanced 10k discovery cohort and binds them
    /// strictly to verified human ground-truth quadrilaterals, ignoring pseudo-labels.
    """

    def __init__(
        self,
        cohort_db_path: Path | str = DEFAULT_PSEUDO_LABELS_DB,
        gt_index_path: Path | str = DEFAULT_GT_INDEX_PATH,
        image_root: Path | str = DEFAULT_IMAGE_ROOT,
        split: str = "train",
        val_ratio: float = 0.10,
        split_seed: int = 42,
        input_size: int = 384,
        cohort_size: int | None = 10000,
        augmentation: AugmentationConfig | None = None,
        regular_min_side: float = 2.0,
        thin_major_axis_min: float = 4.0,
        thin_aspect_ratio_min: float = 1.5,
        thin_area: float = 4.0,
        seed: int = 42,
        sanitize_annotations: bool = True,
    ) -> None:
        """Initialize the Human GT Quad dataset.

        @param cohort_db_path Path to the sam_discovery.sqlite database defining the cohort.
        @param gt_index_path Path to quad_train.sqlite index containing human ground truth.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param cohort_size If 10000, restrict strictly to the balanced 10k cohort.
        @param augmentation Data augmentation configuration (used when split='train').
        @param regular_min_side Minimum side length in pixels to accept regular quad.
        @param thin_major_axis_min Minimum major axis length in pixels for thin objects.
        @param thin_aspect_ratio_min Minimum aspect ratio for thin objects.
        @param thin_area Minimum pixel area for thin objects.
        @param seed Runtime random seed for epoch-dependent augmentations.
        @param sanitize_annotations Whether to apply cross-tier deduplication and WoodScape fragment hygiene.
        """
        super().__init__()
        self.cohort_db_path = Path(cohort_db_path)
        self.gt_index_path = Path(gt_index_path)
        self.image_root = Path(image_root)
        self.split = split
        self.input_size = input_size
        self.seed = seed
        self.sanitize_annotations = bool(sanitize_annotations)
        self._epoch = torch.zeros(1, dtype=torch.long)

        self._connection: sqlite3.Connection | None = None
        self._connection_pid: int | None = None

        is_enriched = "enriched" in str(self.gt_index_path) or (
            cohort_size is not None and cohort_size > 10000
        )
        if not is_enriched and not self.cohort_db_path.exists():
            raise FileNotFoundError(f"Cohort database not found: {self.cohort_db_path}")
        if not self.gt_index_path.exists():
            raise FileNotFoundError(
                f"Human GT index database not found: {self.gt_index_path}"
            )

        all_images = load_cohort_image_records(
            cohort_db_path=self.cohort_db_path,
            gt_index_path=self.gt_index_path,
            cohort_size=cohort_size,
        )

        train_indices, val_indices = split_cohort_indices(
            all_images, val_ratio=val_ratio, seed=split_seed
        )

        if split == "train":
            indices = train_indices
        elif split == "val":
            indices = val_indices
        elif split == "all":
            indices = list(range(len(all_images)))
        else:
            raise ValueError(
                f"Unknown split: {split}. Must be 'train', 'val', or 'all'"
            )

        self.images = [all_images[idx] for idx in indices]

        if augmentation is None:
            augmentation = AugmentationConfig()
        self.augmentation = augmentation

        self.transform = QuadProposalTransform(
            input_size=input_size,
            augmentation=augmentation,
            training=(split == "train"),
            regular_min_side=regular_min_side,
            thin_major_axis_min=thin_major_axis_min,
            thin_aspect_ratio_min=thin_aspect_ratio_min,
            thin_area=thin_area,
        )

    def _get_connection(self) -> sqlite3.Connection:
        """Lazily initialize SQLite connection per process/worker."""
        pid = os.getpid()
        if self._connection is None or self._connection_pid != pid:
            if self._connection is not None:
                try:
                    self._connection.close()
                except Exception:
                    pass
            self._connection = sqlite3.connect(
                f"file:{self.gt_index_path}?mode=ro&immutable=1",
                uri=True,
                check_same_thread=False,
            )
            self._connection_pid = pid
        return self._connection

    def _get_category_column(self, conn: sqlite3.Connection) -> str | None:
        if not hasattr(self, "_cat_column_name"):
            cols = [r[1] for r in conn.execute("PRAGMA table_info(annotations)").fetchall()]
            if "category_name" in cols:
                self._cat_column_name: str | None = "category_name"
            elif "category" in cols:
                self._cat_column_name = "category"
            else:
                self._cat_column_name = None
        return self._cat_column_name

    def __getstate__(self) -> dict[str, Any]:
        """Drop unpicklable SQLite connection handle during multiprocessing serialization."""
        state = dict(self.__dict__)
        state["_connection"] = None
        state["_connection_pid"] = None
        return state

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> QuadProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]
        conn = self._get_connection()

        img_id = record["image_id"]
        cat_col = self._get_category_column(conn)
        cat_sql = f", {cat_col}" if cat_col else ", 'unknown'"
        ann_rows = conn.execute(
            f"""
            SELECT quad_json, ignore_region, geometry_tier, object_condition, seen_status, x1, y1, x2, y2{cat_sql}
            FROM annotations
            WHERE image_id = ?
            ORDER BY rowid
            """,
            (img_id,),
        ).fetchall()

        positive_quads: list[list[list[float]]] = []
        ignore_quads: list[list[list[float]]] = []
        trusted_background_quads: list[list[list[float]]] = []
        tiers: list[str] = []
        conditions: list[str] = []
        statuses: list[str] = []
        positive_categories: list[str] = []
        kept_raw_pos_indices: list[int] = []

        parsed_rows: list[tuple[list[list[float]], RawAnnotationRow, str, str]] = []
        hygiene_rows: list[RawAnnotationRow] = []
        for r in ann_rows:
            quad = json.loads(str(r[0]))
            cat_val = str(r[9]) if r[9] else "unknown"
            hr = RawAnnotationRow(
                x1=float(r[5]),
                y1=float(r[6]),
                x2=float(r[7]),
                y2=float(r[8]),
                ignore_region=int(r[1]),
                category_name=cat_val,
                geometry_tier=str(r[2]) if r[2] else "unknown",
            )
            hygiene_rows.append(hr)
            parsed_rows.append((quad, hr, str(r[3]), str(r[4])))

        raw_pos_idx_by_row, kept_set, demoted_set, dropped_set = _prepare_hygiene_masks(
            hygiene_rows,
            source_dataset=str(record["source_dataset"]),
            sanitize=self.sanitize_annotations,
        )

        for row_idx, (quad, hr, cond, status) in enumerate(parsed_rows):
            if row_idx in dropped_set:
                continue
            if hr.ignore_region == 0:
                if row_idx in demoted_set:
                    ignore_quads.append(quad)
                elif row_idx in kept_set:
                    positive_quads.append(quad)
                    tiers.append(hr.geometry_tier)
                    conditions.append(cond)
                    statuses.append(status)
                    positive_categories.append(hr.category_name)
                    kept_raw_pos_indices.append(raw_pos_idx_by_row[row_idx])
            elif hr.ignore_region == 1:
                ignore_quads.append(quad)
            elif hr.ignore_region == 2:
                trusted_background_quads.append(quad)

        with Image.open(image_path) as loaded:
            image = loaded.convert("RGB")

        seed = self.seed + int(self._epoch.item()) * max(len(self), 1) + index
        transformed = self.transform(
            image,
            _quad_tensor(positive_quads),
            _quad_tensor(ignore_quads),
            _quad_tensor(trusted_background_quads),
            seed=seed,
            return_metadata=True,
        )

        source_dataset = record["source_dataset"]
        domain = SOURCE_DOMAINS.get(source_dataset, "general")
        camera_type = CAMERA_TYPES.get(source_dataset, "pinhole")
        background_supervision = bool(transformed[3].numel())
        retained = transformed[6]
        horizontal_flip = bool(transformed[7])
        size_bins, aspect_bins, radial_bins = _quad_slice_labels(
            transformed[1], self.input_size
        )

        return QuadProposalSample(
            image=transformed[0],
            quads=transformed[1],
            ignore_quads=transformed[2],
            valid_mask=transformed[4],
            image_id=img_id,
            source_dataset=source_dataset,
            domain=domain,
            camera_type=camera_type,
            background_supervision=background_supervision,
            original_size=(int(record["height"]), int(record["width"])),
            transform=transformed[5],
            geometry_tiers=tuple(tiers[idx] for idx in retained),
            object_conditions=tuple(conditions[idx] for idx in retained),
            trusted_background_quads=transformed[3],
            seen_statuses=tuple(statuses[idx] for idx in retained),
            size_bins=size_bins,
            aspect_bins=aspect_bins,
            radial_bins=radial_bins,
            file_name=str(record["file_name"]),
            category_names=tuple(positive_categories[idx] for idx in retained),
            gt_indices=tuple(kept_raw_pos_indices[idx] for idx in retained),
            horizontal_flip=horizontal_flip,
        )


__all__ = [
    "DEFAULT_GT_INDEX_PATH",
    "DEFAULT_ENRICHED_GT_INDEX_PATH",
    "STAGE2_COHORT_QUOTAS",
    "STAGE2_ENRICHED_COHORT_QUOTAS",
    "STAGE2_8K_COHORT_QUOTAS",
    "HumanGTHBBDataset",
    "HumanGTQuadDataset",
    "collate_proposal_samples",
    "collate_quad_proposal_samples",
    "get_stage2_cohort_ids",
    "get_stage2_10k_cohort_ids",
    "load_cohort_image_records",
]
