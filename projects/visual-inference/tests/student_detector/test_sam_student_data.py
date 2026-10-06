"""Unit tests for SAMStudentDataset pseudo-label loader and stratified splitting."""

from __future__ import annotations

import torch
from torch.utils.data import DataLoader

from student_detector.assigner import ATSSAssigner
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.sam_student_data import (
    SAMStudentDataset,
    SAMStudentHBBDataset,
    collate_proposal_samples,
    collate_quad_proposal_samples,
    split_cohort_indices,
)
from student_detector.targets import TargetBuilder


def test_split_cohort_indices_stratification() -> None:
    # Synthetic cohort with 4 datasets
    images = (
        [{"source_dataset": "coco_2017", "id": i} for i in range(100)]
        + [{"source_dataset": "nuimages", "id": 100 + i} for i in range(50)]
        + [
            {"source_dataset": "woodscape_rgb_fisheye", "id": 150 + i}
            for i in range(50)
        ]
    )
    train_idx, val_idx = split_cohort_indices(images, val_ratio=0.20, seed=42)
    assert len(train_idx) == 160
    assert len(val_idx) == 40
    assert set(train_idx).isdisjoint(set(val_idx))


def test_sam_student_dataset_initialization_and_shapes() -> None:
    train_ds = SAMStudentDataset(split="train")
    val_ds = SAMStudentDataset(split="val")

    assert len(train_ds) == 640
    assert len(val_ds) == 160
    assert train_ds.total_novel > 0
    assert train_ds.total_gt > 0

    sample = train_ds[0]
    assert sample.image.shape == (3, 384, 384)
    assert sample.quads.ndim == 3
    assert sample.quads.shape[1:] == (4, 2)
    assert sample.valid_mask.shape == (384, 384)
    assert len(sample.stabilities) == sample.quads.shape[0]


def test_sam_student_dataset_collate_and_target_builder() -> None:
    val_ds = SAMStudentDataset(split="val")
    loader = DataLoader(
        val_ds,
        batch_size=2,
        shuffle=False,
        collate_fn=collate_quad_proposal_samples,
    )
    images, samples = next(iter(loader))
    assert images.shape == (2, 3, 384, 384)
    assert len(samples) == 2

    assigner = QuadAssigner(strides=(4, 8, 16, 32))
    builder = QuadTargetBuilder(assigner)
    feature_shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    targets = builder(samples, feature_shapes, device=torch.device("cpu"))

    assert targets.quality.ndim == 2
    assert targets.corner_offsets.ndim == 3
    assert targets.corner_offsets.shape[-1] == 8
    assert len(targets.valid_point_masks) == 4


def test_sam_student_hbb_dataset_initialization_and_shapes() -> None:
    train_ds = SAMStudentHBBDataset(split="train")
    val_ds = SAMStudentHBBDataset(split="val")

    assert len(train_ds) == 640
    assert len(val_ds) == 160
    assert train_ds.total_novel > 0
    assert train_ds.total_gt > 0

    sample = train_ds[0]
    assert sample.image.shape == (3, 384, 384)
    assert sample.boxes.ndim == 2
    assert sample.boxes.shape[-1] == 4
    assert sample.valid_mask.shape == (384, 384)


def test_sam_student_hbb_dataset_collate_and_target_builder() -> None:
    val_ds = SAMStudentHBBDataset(split="val")
    loader = DataLoader(
        val_ds,
        batch_size=2,
        shuffle=False,
        collate_fn=collate_proposal_samples,
    )
    images, samples = next(iter(loader))
    assert images.shape == (2, 3, 384, 384)
    assert len(samples) == 2

    assigner = ATSSAssigner(
        strides=(4, 8, 16, 32),
        prior_sizes=(32, 64, 128, 256),
    )
    builder = TargetBuilder(assigner)
    feature_shapes = ((96, 96), (48, 48), (24, 24), (12, 12))
    targets = builder(samples, feature_shapes, device=torch.device("cpu"))

    assert targets.objectness.ndim == 2
    assert targets.box_distances.ndim == 3
    assert targets.box_distances.shape[-1] == 4
    assert len(targets.valid_point_masks) == 4

