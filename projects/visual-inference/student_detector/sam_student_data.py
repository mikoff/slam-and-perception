"""Dataset loader consuming SAM pilot discovery pseudo-labels for MobileNetV4 student training."""

from __future__ import annotations

import json
from pathlib import Path
import random
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
        seed: int = 42,
    ) -> None:
        """Initialize the SAM pseudo-label dataset.

        @param pseudo_labels_path Path to the sam_pilot_800.json pseudo-label file.
        @param image_root Directory containing the output images.
        @param split One of 'train', 'val', or 'all'.
        @param val_ratio Fraction of images allocated to validation split.
        @param split_seed Seed for deterministic stratified cohort splitting.
        @param input_size Input image resolution for the detector (e.g. 384).
        @param min_stability Minimum SAM mask stability score to accept as positive novel.
        @param augmentation Data augmentation configuration (used when split='train').
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

        if not self.pseudo_labels_path.exists():
            raise FileNotFoundError(
                f"Pseudo-labels file not found: {self.pseudo_labels_path}"
            )

        with open(self.pseudo_labels_path, encoding="utf-8") as f:
            data = json.load(f)

        all_images = data["images"]
        train_indices, val_indices = split_cohort_indices(
            all_images, val_ratio=val_ratio, seed=split_seed
        )

        if split == "train":
            selected_indices = train_indices
        elif split == "val":
            selected_indices = val_indices
        elif split == "all":
            selected_indices = list(range(len(all_images)))
        else:
            raise ValueError(
                f"Unknown split: {split}. Must be 'train', 'val', or 'all'"
            )

        self.images: list[dict[str, Any]] = [
            all_images[idx] for idx in selected_indices
        ]
        self.indices = selected_indices

        if augmentation is None:
            augmentation = AugmentationConfig()
        self.augmentation = augmentation

        self.transform = QuadProposalTransform(
            input_size=input_size,
            augmentation=augmentation,
            training=(split == "train"),
            regular_min_side=16.0,
        )

        # Index and aggregate statistics
        self.total_gt = 0
        self.total_novel = 0
        self.total_suppressed = 0
        for img in self.images:
            for ann in img.get("annotations", []):
                if ann.get("is_positive") and not ann.get("is_suppressed"):
                    if ann.get("source") == "human_gt":
                        self.total_gt += 1
                    elif ann.get("source") == "sam_discovered":
                        if (
                            ann.get("sam_stability_score") or 1.0
                        ) >= self.min_stability:
                            self.total_novel += 1
                elif ann.get("is_suppressed"):
                    self.total_suppressed += 1

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> QuadProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]

        positive_quads: list[list[list[float]]] = []
        ignore_quads: list[list[list[float]]] = []
        stabilities: list[float] = []

        for ann in record.get("annotations", []):
            quad = ann.get("quad")
            if not quad or len(quad) != 4:
                continue

            if ann.get("is_positive") and not ann.get("is_suppressed"):
                if ann.get("source") == "human_gt":
                    positive_quads.append(quad)
                    stabilities.append(1.0)
                elif ann.get("source") == "sam_discovered":
                    stability = ann.get("sam_stability_score", 1.0)
                    if stability is not None and stability < self.min_stability:
                        ignore_quads.append(quad)
                    else:
                        positive_quads.append(quad)
                        stabilities.append(
                            float(stability if stability is not None else 1.0)
                        )
            else:
                # Suppressed subparts, duplicate GT, or stuff (road_surface)
                ignore_quads.append(quad)

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
        tiny_area: float = 16.0,
        tiny_min_side: float = 4.0,
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

        if not self.pseudo_labels_path.exists():
            raise FileNotFoundError(
                f"Pseudo-labels file not found: {self.pseudo_labels_path}"
            )

        with open(self.pseudo_labels_path, encoding="utf-8") as f:
            data = json.load(f)

        all_images = data["images"]
        train_indices, val_indices = split_cohort_indices(
            all_images, val_ratio=val_ratio, seed=split_seed
        )

        if split == "train":
            selected_indices = train_indices
        elif split == "val":
            selected_indices = val_indices
        elif split == "all":
            selected_indices = list(range(len(all_images)))
        else:
            raise ValueError(
                f"Unknown split: {split}. Must be 'train', 'val', or 'all'"
            )

        self.images: list[dict[str, Any]] = [
            all_images[idx] for idx in selected_indices
        ]
        self.indices = selected_indices

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

        # Index and aggregate statistics
        self.total_gt = 0
        self.total_novel = 0
        self.total_suppressed = 0
        for img in self.images:
            for ann in img.get("annotations", []):
                if ann.get("is_positive") and not ann.get("is_suppressed"):
                    if ann.get("source") == "human_gt":
                        self.total_gt += 1
                    elif ann.get("source") == "sam_discovered":
                        if (
                            ann.get("sam_stability_score") or 1.0
                        ) >= self.min_stability:
                            self.total_novel += 1
                elif ann.get("is_suppressed"):
                    self.total_suppressed += 1

    def __len__(self) -> int:
        return len(self.images)

    def set_epoch(self, epoch: int) -> None:
        """Update epoch counter for deterministic per-epoch augmentation perturbations."""
        self._epoch.fill_(epoch)

    def __getitem__(self, index: int) -> ProposalSample:
        record = self.images[index]
        image_path = self.image_root / record["file_name"]

        positive_boxes: list[list[float]] = []
        ignore_boxes: list[list[float]] = []

        for ann in record.get("annotations", []):
            bbox = ann.get("bbox")
            if not bbox or len(bbox) != 4:
                continue

            if ann.get("is_positive") and not ann.get("is_suppressed"):
                if ann.get("source") == "human_gt":
                    positive_boxes.append(bbox)
                elif ann.get("source") == "sam_discovered":
                    stability = ann.get("sam_stability_score", 1.0)
                    if stability is not None and stability < self.min_stability:
                        ignore_boxes.append(bbox)
                    else:
                        positive_boxes.append(bbox)
            else:
                # Suppressed subparts, duplicate GT, or stuff (road_surface)
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
    "DEFAULT_PSEUDO_LABELS_PATH",
    "SAMStudentDataset",
    "SAMStudentHBBDataset",
    "collate_proposal_samples",
    "collate_quad_proposal_samples",
    "split_cohort_indices",
]
