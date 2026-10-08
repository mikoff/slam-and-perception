"""Dataset loader consuming SAM pilot discovery pseudo-labels for MobileNetV4 student training."""

from __future__ import annotations

import json
from pathlib import Path
import random
import sqlite3
from typing import Any

from PIL import Image
import torch
from torch.utils.data import Dataset

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

DEFAULT_PSEUDO_LABELS_PATH = (
    Path(__file__).resolve().parent.parent
    / "artifacts"
    / "phase1"
    / "pseudo_labels"
    / "sam_pilot_800.json"
)
DEFAULT_PSEUDO_LABELS_DB = (
    Path(__file__).resolve().parent.parent
    / "artifacts"
    / "pseudo_labels"
    / "sam_discovery.sqlite"
)
DEFAULT_IMAGE_ROOT = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets/output"
)

SOURCE_DOMAINS: dict[str, str] = {
    "coco_2017": "general",
    "nuimages": "automotive",
    "bdd100k_images_100k": "automotive",
    "woodscape_rgb_fisheye": "fisheye",
}

CAMERA_TYPES: dict[str, str] = {
    "coco_2017": "pinhole",
    "nuimages": "pinhole",
    "bdd100k_images_100k": "pinhole",
    "woodscape_rgb_fisheye": "fisheye",
}


def split_cohort_indices(
    images: list[dict[str, Any]],
    val_ratio: float = 0.20,
    seed: int = 42,
) -> tuple[list[int], list[int]]:
    """Deterministically partition images into train and val splits stratified by source_dataset.

    @param images List of image metadata dictionaries from pseudo-labels JSON.
    @param val_ratio Fraction of each dataset cohort reserved for validation.
    @param seed Pseudo-random seed for deterministic partitioning.
    @return Tuple of (train_indices, val_indices).
    """
    by_source: dict[str, list[int]] = {}
    for idx, img in enumerate(images):
        by_source.setdefault(img["source_dataset"], []).append(idx)

    train_indices: list[int] = []
    val_indices: list[int] = []

    rng = random.Random(seed)
    for _source, indices in sorted(by_source.items()):
        shuffled = list(indices)
        rng.shuffle(shuffled)
        n_val = max(1, round(len(shuffled) * val_ratio))
        val_indices.extend(shuffled[:n_val])
        train_indices.extend(shuffled[n_val:])

    train_indices.sort()
    val_indices.sort()
    return train_indices, val_indices


def _load_pseudo_label_records(
    pseudo_labels_path: Path,
    split: str,
    val_ratio: float,
    split_seed: int,
    min_stability: float,
) -> tuple[list[dict[str, Any]], list[int], int, int, int, bool]:
    """Load image records and dataset totals from SQLite or JSON pseudo-labels.

    /// @param pseudo_labels_path Path to SQLite database or JSON file.
    /// @param split One of 'train', 'val', or 'all'.
    /// @param val_ratio Fraction of cohort reserved for validation.
    /// @param split_seed Seed for deterministic stratified cohort splitting.
    /// @param min_stability Minimum SAM mask stability score.
    /// @return Tuple of (images, indices, total_gt, total_novel, total_suppressed, is_sqlite).
    """
    is_sqlite = pseudo_labels_path.suffix in (".sqlite", ".db")
    if is_sqlite:
        with sqlite3.connect(
            f"file:{pseudo_labels_path}?mode=ro&immutable=1", uri=True
        ) as conn:
            cols = [
                c[1] for c in conn.execute("PRAGMA table_info(images)").fetchall()
            ]
            has_explicit_val = False
            if "split" in cols:
                has_explicit_val = bool(
                    conn.execute(
                        "SELECT 1 FROM images WHERE split = 'val' LIMIT 1"
                    ).fetchone()
                )

            if "split" in cols and has_explicit_val and split in ("train", "val"):
                rows = conn.execute(
                    """
                    SELECT image_id, file_name, width, height, source_dataset
                    FROM images
                    WHERE split = ?
                    ORDER BY image_id
                    """,
                    (split,),
                ).fetchall()
                images = [
                    {
                        "image_id": int(r[0]),
                        "file_name": str(r[1]),
                        "width": int(r[2]),
                        "height": int(r[3]),
                        "source_dataset": str(r[4]),
                    }
                    for r in rows
                ]
                indices = list(range(len(images)))
            elif split == "all" or not has_explicit_val:
                rows = conn.execute(
                    """
                    SELECT image_id, file_name, width, height, source_dataset
                    FROM images
                    ORDER BY image_id
                    """
                ).fetchall()
                all_images = [
                    {
                        "image_id": int(r[0]),
                        "file_name": str(r[1]),
                        "width": int(r[2]),
                        "height": int(r[3]),
                        "source_dataset": str(r[4]),
                    }
                    for r in rows
                ]
                if split == "all":
                    images = all_images
                    indices = list(range(len(images)))
                else:
                    train_idx, val_idx = split_cohort_indices(
                        all_images, val_ratio=val_ratio, seed=split_seed
                    )
                    indices = train_idx if split == "train" else val_idx
                    images = [all_images[i] for i in indices]
            else:
                raise ValueError(
                    f"Unknown split: {split}. Must be 'train', 'val', or 'all'"
                )

            total_gt = conn.execute(
                "SELECT COUNT(*) FROM annotations WHERE source = 'human_gt' AND is_positive = 1"
            ).fetchone()[0]
            total_novel = conn.execute(
                """
                SELECT COUNT(*) FROM annotations
                WHERE source = 'sam_discovered' AND is_positive = 1
                  AND (stability_score IS NULL OR stability_score >= ?)
                """,
                (min_stability,),
            ).fetchone()[0]
            total_suppressed = conn.execute(
                "SELECT COUNT(*) FROM annotations WHERE is_suppressed = 1"
            ).fetchone()[0]
    else:
        with open(pseudo_labels_path, encoding="utf-8") as f:
            data = json.load(f)

        all_images = data["images"]
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

        images = [all_images[idx] for idx in indices]
        total_gt = 0
        total_novel = 0
        total_suppressed = 0
        for img in images:
            for ann in img.get("annotations", []):
                if ann.get("is_positive") and not ann.get("is_suppressed"):
                    if ann.get("source") == "human_gt":
                        total_gt += 1
                    elif ann.get("source") == "sam_discovered":
                        if (
                            ann.get("sam_stability_score") or 1.0
                        ) >= min_stability:
                            total_novel += 1
                elif ann.get("is_suppressed"):
                    total_suppressed += 1

    return images, indices, total_gt, total_novel, total_suppressed, is_sqlite


def _fetch_record_annotations(
    record: dict[str, Any],
    is_sqlite: bool,
    sqlite_conn: sqlite3.Connection | None,
    pseudo_labels_path: Path,
) -> tuple[list[dict[str, Any]], sqlite3.Connection | None]:
    """Fetch annotations for an image record from SQLite or in-memory JSON dict.

    /// @param record Image dictionary.
    /// @param is_sqlite Boolean flag indicating whether storage is SQLite.
    /// @param sqlite_conn Existing connection or None.
    /// @param pseudo_labels_path Path to SQLite database.
    /// @return Tuple of (annotations_list, sqlite_conn).
    """
    if not is_sqlite:
        return record.get("annotations", []), sqlite_conn

    if sqlite_conn is None:
        sqlite_conn = sqlite3.connect(
            f"file:{pseudo_labels_path}?mode=ro&immutable=1",
            uri=True,
            check_same_thread=False,
        )

    img_id = record["image_id"]
    rows = sqlite_conn.execute(
        """
        SELECT bbox_x1, bbox_y1, bbox_x2, bbox_y2, quad_json,
               source, is_positive, is_suppressed, stability_score
        FROM annotations
        WHERE image_id = ?
        """,
        (img_id,),
    ).fetchall()

    annotations: list[dict[str, Any]] = []
    for r in rows:
        quad = json.loads(r[4]) if r[4] else None
        annotations.append(
            {
                "bbox": [float(r[0]), float(r[1]), float(r[2]), float(r[3])],
                "quad": quad,
                "source": str(r[5]),
                "is_positive": bool(r[6]),
                "is_suppressed": bool(r[7]),
                "sam_stability_score": float(r[8]) if r[8] is not None else None,
            }
        )
    return annotations, sqlite_conn


def suppress_container_boxes(
    positive_boxes: list[list[float]],
    is_discovered: list[bool],
    img_width: float,
    img_height: float,
    containment_threshold: float = 0.75,
    min_macro_area_ratio: float = 0.20,
) -> tuple[list[list[float]], list[list[float]]]:
    """Downgrade discovered proposals that enclose other objects to ignore_boxes.

    A discovered proposal is topologically a scene container (context/road/sky)
    rather than an atomic object instance if its area is macroscopic (>= min_macro_area_ratio)
    and:
    1. It encloses >= 2 other objects by at least containment_threshold (default 0.75), OR
    2. It encloses 1 other object, while its area is > 6x child area.

    /// @param positive_boxes List of candidate positive bounding boxes [x1, y1, x2, y2].
    /// @param is_discovered Boolean list indicating whether each box is a SAM discovery.
    /// @param img_width Width of the original image.
    /// @param img_height Height of the original image.
    /// @param containment_threshold Minimum fraction of child box area inside candidate.
    /// @param min_macro_area_ratio Minimum image area fraction for a container.
    /// @return Tuple of (filtered_positive_boxes, suppressed_container_boxes).
    """
    if len(positive_boxes) < 2 or not any(is_discovered):
        return positive_boxes, []

    b = torch.tensor(positive_boxes, dtype=torch.float32)
    inter_x1 = torch.maximum(b[:, None, 0], b[None, :, 0])
    inter_y1 = torch.maximum(b[:, None, 1], b[None, :, 1])
    inter_x2 = torch.minimum(b[:, None, 2], b[None, :, 2])
    inter_y2 = torch.minimum(b[:, None, 3], b[None, :, 3])

    inter_w = (inter_x2 - inter_x1).clamp(min=0.0)
    inter_h = (inter_y2 - inter_y1).clamp(min=0.0)
    inter_area = inter_w * inter_h

    areas = (b[:, 2] - b[:, 0]).clamp(min=0.0) * (b[:, 3] - b[:, 1]).clamp(min=0.0)
    img_area = max(float(img_width) * float(img_height), 1.0)

    child_coverage = inter_area / areas[None, :].clamp(min=1e-6)
    child_coverage.fill_diagonal_(0.0)

    contained = child_coverage >= containment_threshold

    filtered_pos: list[list[float]] = []
    containers: list[list[float]] = []

    for i, (box, is_disc) in enumerate(zip(positive_boxes, is_discovered, strict=True)):
        if not is_disc:
            filtered_pos.append(box)
            continue

        contained_children = torch.where(contained[i])[0]
        is_container = False
        area_ratio = float(areas[i].item()) / img_area
        if area_ratio >= min_macro_area_ratio:
            if len(contained_children) >= 2:
                is_container = True
            elif len(contained_children) == 1:
                child_idx = int(contained_children[0].item())
                if areas[i] / max(float(areas[child_idx].item()), 1.0) > 6.0:
                    is_container = True

        if is_container:
            containers.append(box)
        else:
            filtered_pos.append(box)

    return filtered_pos, containers


def suppress_container_quads(
    positive_quads: list[list[list[float]]],
    stabilities: list[float],
    is_discovered: list[bool],
    img_width: float,
    img_height: float,
    containment_threshold: float = 0.75,
    min_macro_area_ratio: float = 0.20,
) -> tuple[list[list[list[float]]], list[float], list[list[list[float]]]]:
    """Downgrade discovered quads that enclose other objects to ignore_quads.

    /// @param positive_quads List of candidate positive quads [4, 2].
    /// @param stabilities Stability scores corresponding to positive quads.
    /// @param is_discovered Boolean list indicating whether each quad is a SAM discovery.
    /// @param img_width Width of the original image.
    /// @param img_height Height of the original image.
    /// @param containment_threshold Minimum fraction of child quad area inside candidate.
    /// @param min_macro_area_ratio Minimum image area fraction for a container.
    /// @return Tuple of (filtered_positive_quads, filtered_stabilities, suppressed_container_quads).
    """
    if len(positive_quads) < 2 or not any(is_discovered):
        return positive_quads, stabilities, []

    boxes = [
        [
            float(min(pt[0] for pt in q)),
            float(min(pt[1] for pt in q)),
            float(max(pt[0] for pt in q)),
            float(max(pt[1] for pt in q)),
        ]
        for q in positive_quads
    ]
    b = torch.tensor(boxes, dtype=torch.float32)
    inter_x1 = torch.maximum(b[:, None, 0], b[None, :, 0])
    inter_y1 = torch.maximum(b[:, None, 1], b[None, :, 1])
    inter_x2 = torch.minimum(b[:, None, 2], b[None, :, 2])
    inter_y2 = torch.minimum(b[:, None, 3], b[None, :, 3])

    inter_w = (inter_x2 - inter_x1).clamp(min=0.0)
    inter_h = (inter_y2 - inter_y1).clamp(min=0.0)
    inter_area = inter_w * inter_h

    areas = (b[:, 2] - b[:, 0]).clamp(min=0.0) * (b[:, 3] - b[:, 1]).clamp(min=0.0)
    img_area = max(float(img_width) * float(img_height), 1.0)

    child_coverage = inter_area / areas[None, :].clamp(min=1e-6)
    child_coverage.fill_diagonal_(0.0)

    contained = child_coverage >= containment_threshold

    filtered_quads: list[list[list[float]]] = []
    filtered_stab: list[float] = []
    container_quads: list[list[list[float]]] = []

    for i, (q, stab, is_disc) in enumerate(
        zip(positive_quads, stabilities, is_discovered, strict=True)
    ):
        if not is_disc:
            filtered_quads.append(q)
            filtered_stab.append(stab)
            continue

        contained_children = torch.where(contained[i])[0]
        is_container = False
        area_ratio = float(areas[i].item()) / img_area
        if area_ratio >= min_macro_area_ratio:
            if len(contained_children) >= 2:
                is_container = True
            elif len(contained_children) == 1:
                child_idx = int(contained_children[0].item())
                if areas[i] / max(float(areas[child_idx].item()), 1.0) > 6.0:
                    is_container = True

        if is_container:
            container_quads.append(q)
        else:
            filtered_quads.append(q)
            filtered_stab.append(stab)

    return filtered_quads, filtered_stab, container_quads


class SAMStudentDataset(Dataset[QuadProposalSample]):
    """PyTorch Dataset loading SAM discovery pseudo-labels and human GT for student training."""

    def __init__(
        self,
        pseudo_labels_path: Path | str = DEFAULT_PSEUDO_LABELS_PATH,
        image_root: Path | str = DEFAULT_IMAGE_ROOT,
        split: str = "train",
        val_ratio: float = 0.20,
        split_seed: int = 42,
        input_size: int = 384,
        min_stability: float = 0.80,
        augmentation: AugmentationConfig | None = None,
        regular_min_side: float = 8.0,
        seed: int = 42,
    ) -> None:
        """Initialize the SAM pseudo-label dataset.

        @param pseudo_labels_path Path to the sam_pilot_800.json pseudo-label file or SQLite db.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param min_stability Minimum SAM mask stability score to accept as positive novel.
        @param augmentation Data augmentation configuration (used when split='train').
        @param regular_min_side Minimum side length in pixels to accept regular quad.
        @param seed Runtime random seed for epoch-dependent augmentations.
        """
        super().__init__()
        self.pseudo_labels_path = Path(pseudo_labels_path)
        self.image_root = Path(image_root)
        self.split = split
        self.input_size = input_size
        self.min_stability = min_stability
        self.seed = seed
        self._epoch = torch.zeros(1, dtype=torch.long)
        self._connection: sqlite3.Connection | None = None

        if not self.pseudo_labels_path.exists():
            raise FileNotFoundError(
                f"Pseudo-labels file not found: {self.pseudo_labels_path}"
            )

        (
            self.images,
            self.indices,
            self.total_gt,
            self.total_novel,
            self.total_suppressed,
            self._is_sqlite,
        ) = _load_pseudo_label_records(
            self.pseudo_labels_path,
            split=split,
            val_ratio=val_ratio,
            split_seed=split_seed,
            min_stability=min_stability,
        )

        if augmentation is None:
            augmentation = AugmentationConfig()
        self.augmentation = augmentation

        self.transform = QuadProposalTransform(
            input_size=input_size,
            augmentation=augmentation,
            training=(split == "train"),
            regular_min_side=regular_min_side,
        )

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> QuadProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]

        positive_quads: list[list[list[float]]] = []
        is_discovered: list[bool] = []
        ignore_quads: list[list[list[float]]] = []
        stabilities: list[float] = []

        anns, self._connection = _fetch_record_annotations(
            record, self._is_sqlite, self._connection, self.pseudo_labels_path
        )

        for ann in anns:
            quad = ann.get("quad")
            if not quad or len(quad) != 4:
                continue

            if ann.get("is_positive") and not ann.get("is_suppressed"):
                if ann.get("source") == "human_gt":
                    positive_quads.append(quad)
                    is_discovered.append(False)
                    stabilities.append(1.0)
                elif ann.get("source") == "sam_discovered":
                    stability = ann.get("sam_stability_score", 1.0)
                    if stability is not None and stability < self.min_stability:
                        ignore_quads.append(quad)
                    else:
                        positive_quads.append(quad)
                        is_discovered.append(True)
                        stabilities.append(
                            float(stability if stability is not None else 1.0)
                        )
            else:
                # Suppressed subparts, duplicate GT, or stuff (road_surface)
                ignore_quads.append(quad)

        # Topological Non-Leaf Suppression (Solution 1):
        orig_w = float(record.get("width", 384))
        orig_h = float(record.get("height", 384))
        positive_quads, stabilities, container_quads = suppress_container_quads(
            positive_quads, stabilities, is_discovered, orig_w, orig_h
        )
        ignore_quads.extend(container_quads)

        with Image.open(image_path) as loaded:
            image = loaded.convert("RGB")

        seed = self.seed + int(self._epoch.item()) * max(len(self), 1) + index
        transformed = self.transform(
            image,
            _quad_tensor(positive_quads),
            _quad_tensor(ignore_quads),
            seed=seed,
        )

        source_dataset = record["source_dataset"]
        domain = SOURCE_DOMAINS.get(source_dataset, "general")
        camera_type = CAMERA_TYPES.get(source_dataset, "pinhole")
        retained = transformed[6]
        slice_labels = _quad_slice_labels(transformed[1], self.input_size)
        retained_stabilities = tuple(stabilities[idx] for idx in retained)

        size_bins, aspect_bins, radial_bins = slice_labels
        return QuadProposalSample(
            image=transformed[0],
            quads=transformed[1],
            ignore_quads=transformed[2],
            valid_mask=transformed[4],
            image_id=int(record.get("image_id", index)),
            source_dataset=source_dataset,
            domain=domain,
            camera_type=camera_type,
            background_supervision=bool(transformed[3].numel()),
            original_size=(int(record["height"]), int(record["width"])),
            transform=transformed[5],
            trusted_background_quads=transformed[3],
            size_bins=size_bins,
            aspect_bins=aspect_bins,
            radial_bins=radial_bins,
            stabilities=retained_stabilities,
        )



class SAMStudentHBBDataset(Dataset[ProposalSample]):
    """PyTorch Dataset loading SAM discovery pseudo-labels and human GT as Horizontal Bounding Boxes (HBB)."""

    def __init__(
        self,
        pseudo_labels_path: Path | str = DEFAULT_PSEUDO_LABELS_PATH,
        image_root: Path | str = DEFAULT_IMAGE_ROOT,
        split: str = "train",
        val_ratio: float = 0.20,
        split_seed: int = 42,
        input_size: int = 384,
        min_stability: float = 0.80,
        augmentation: AugmentationConfig | None = None,
        tiny_area: float = 4.0,
        tiny_min_side: float = 2.0,
        seed: int = 42,
    ) -> None:
        """Initialize the SAM pseudo-label HBB dataset.

        @param pseudo_labels_path Path to the sam_pilot_800.json pseudo-label file.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param min_stability Minimum SAM mask stability score to accept as positive novel.
        @param augmentation Data augmentation configuration (used when split='train').
        @param tiny_area Minimum pixel area in resized input space to retain box.
        @param tiny_min_side Minimum side length in resized input space to retain box.
        @param seed Runtime random seed for epoch-dependent augmentations.
        """
        super().__init__()
        self.pseudo_labels_path = Path(pseudo_labels_path)
        self.image_root = Path(image_root)
        self.split = split
        self.input_size = input_size
        self.min_stability = min_stability
        self.tiny_area = tiny_area
        self.tiny_min_side = tiny_min_side
        self.seed = seed
        self._epoch = torch.zeros(1, dtype=torch.long)

        self._connection: sqlite3.Connection | None = None

        if not self.pseudo_labels_path.exists():
            raise FileNotFoundError(
                f"Pseudo-labels file not found: {self.pseudo_labels_path}"
            )

        (
            self.images,
            self.indices,
            self.total_gt,
            self.total_novel,
            self.total_suppressed,
            self._is_sqlite,
        ) = _load_pseudo_label_records(
            self.pseudo_labels_path,
            split=split,
            val_ratio=val_ratio,
            split_seed=split_seed,
            min_stability=min_stability,
        )

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

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> ProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]

        positive_boxes: list[list[float]] = []
        is_discovered: list[bool] = []
        ignore_boxes: list[list[float]] = []

        anns, self._connection = _fetch_record_annotations(
            record, self._is_sqlite, self._connection, self.pseudo_labels_path
        )

        for ann in anns:
            bbox = ann.get("bbox")
            if not bbox or len(bbox) != 4:
                continue

            if ann.get("is_positive") and not ann.get("is_suppressed"):
                if ann.get("source") == "human_gt":
                    positive_boxes.append(bbox)
                    is_discovered.append(False)
                elif ann.get("source") == "sam_discovered":
                    stability = ann.get("sam_stability_score", 1.0)
                    if stability is not None and stability < self.min_stability:
                        ignore_boxes.append(bbox)
                    else:
                        positive_boxes.append(bbox)
                        is_discovered.append(True)
            else:
                # Suppressed subparts, duplicate GT, or stuff (road_surface)
                ignore_boxes.append(bbox)

        # Topological Non-Leaf Suppression (Solution 1):
        orig_w = float(record.get("width", 384))
        orig_h = float(record.get("height", 384))
        positive_boxes, container_boxes = suppress_container_boxes(
            positive_boxes, is_discovered, orig_w, orig_h
        )
        ignore_boxes.extend(container_boxes)

        with Image.open(image_path) as loaded:
            image = loaded.convert("RGB")

        seed = self.seed + int(self._epoch.item()) * max(len(self), 1) + index
        transformed = self.transform(
            image,
            _boxes_tensor(positive_boxes),
            _boxes_tensor(ignore_boxes),
            seed=seed,
        )

        source_dataset = record["source_dataset"]
        domain = SOURCE_DOMAINS.get(source_dataset, "general")
        camera_type = CAMERA_TYPES.get(source_dataset, "pinhole")
        background_supervision = bool(transformed[3].numel())

        return ProposalSample(
            image=transformed[0],
            boxes=transformed[1],
            ignore_boxes=transformed[2],
            valid_mask=transformed[4],
            image_id=int(record.get("image_id", index)),
            source_dataset=source_dataset,
            domain=domain,
            camera_type=camera_type,
            background_supervision=background_supervision,
            original_size=(int(record["height"]), int(record["width"])),
            transform=transformed[5],
            trusted_background_boxes=transformed[3],
            hard_negative_focus_boxes=transformed[6],
        )


__all__ = [
    "DEFAULT_IMAGE_ROOT",
    "DEFAULT_PSEUDO_LABELS_DB",
    "DEFAULT_PSEUDO_LABELS_PATH",
    "SAMStudentDataset",
    "SAMStudentHBBDataset",
    "collate_proposal_samples",
    "collate_quad_proposal_samples",
    "split_cohort_indices",
]
