"""Dataset loader for high-quality human ground-truth boxes on the 10k cohort.

Supports Stage-2 fine-tuning to tighten bounding box proposals and calibrate objectness.
"""

from __future__ import annotations

from pathlib import Path
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
from .sam_student_data import (
    CAMERA_TYPES,
    DEFAULT_IMAGE_ROOT,
    DEFAULT_PSEUDO_LABELS_DB,
    SOURCE_DOMAINS,
    split_cohort_indices,
)

DEFAULT_GT_INDEX_PATH = Path(
    "/home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train.sqlite"
)


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
        val_ratio: float = 0.20,
        split_seed: int = 42,
        input_size: int = 384,
        augmentation: AugmentationConfig | None = None,
        tiny_area: float = 4.0,
        tiny_min_side: float = 2.0,
        seed: int = 42,
    ) -> None:
        """Initialize the Human GT HBB dataset.

        @param cohort_db_path Path to the 10k sam_discovery.sqlite database defining the cohort.
        @param gt_index_path Path to quad_train.sqlite index containing human ground truth.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param augmentation Data augmentation configuration (used when split='train').
        @param tiny_area Minimum pixel area in resized input space to retain box.
        @param tiny_min_side Minimum side length in resized input space to retain box.
        @param seed Runtime random seed for epoch-dependent augmentations.
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
        self._epoch = torch.zeros(1, dtype=torch.long)

        self._connection: sqlite3.Connection | None = None

        if not self.cohort_db_path.exists():
            raise FileNotFoundError(f"Cohort database not found: {self.cohort_db_path}")
        if not self.gt_index_path.exists():
            raise FileNotFoundError(
                f"Human GT index database not found: {self.gt_index_path}"
            )

        # 1. Fetch cohort images
        with sqlite3.connect(
            f"file:{self.cohort_db_path}?mode=ro&immutable=1", uri=True
        ) as cohort_conn:
            cohort_rows = cohort_conn.execute(
                "SELECT image_id, file_name, source_dataset, width, height FROM images ORDER BY image_id"
            ).fetchall()

        all_images: list[dict[str, Any]] = [
            {
                "image_id": int(r[0]),
                "file_name": str(r[1]),
                "source_dataset": str(r[2]),
                "width": int(r[3]),
                "height": int(r[4]),
            }
            for r in cohort_rows
        ]

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
        ann_rows = conn.execute(
            """
            SELECT x1, y1, x2, y2, ignore_region
            FROM annotations
            WHERE image_id = ?
            """,
            (img_id,),
        ).fetchall()

        positive_boxes: list[list[float]] = []
        ignore_boxes: list[list[float]] = []

        for r in ann_rows:
            bbox = [float(r[0]), float(r[1]), float(r[2]), float(r[3])]
            ign = int(r[4])
            if ign == 0:
                positive_boxes.append(bbox)
            else:
                ignore_boxes.append(bbox)

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
            image_id=img_id,
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
    "DEFAULT_GT_INDEX_PATH",
    "HumanGTHBBDataset",
    "collate_proposal_samples",
]
