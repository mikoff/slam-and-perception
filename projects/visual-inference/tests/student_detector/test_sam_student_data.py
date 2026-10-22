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

    assert len(train_ds) == 720
    assert len(val_ds) == 80
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

    assert len(train_ds) == 720
    assert len(val_ds) == 80
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


def test_topological_container_suppression() -> None:
    from student_detector.sam_student_data import (
        suppress_container_boxes,
    )

    # 2 small cars inside 1 giant road box
    # Car 1: [50, 200, 100, 250]
    # Car 2: [150, 200, 200, 250]
    # Giant Road: [0, 150, 384, 300] (encloses Car 1 and Car 2)
    # Background Sky: [0, 0, 384, 100] (doesn't enclose cars)
    boxes = [
        [50.0, 200.0, 100.0, 250.0],
        [150.0, 200.0, 200.0, 250.0],
        [0.0, 150.0, 384.0, 300.0],
        [0.0, 0.0, 384.0, 100.0],
    ]
    # Cars are human GT (False), Road & Sky are discovered (True)
    is_disc = [False, False, True, True]

    filtered_boxes, containers = suppress_container_boxes(
        boxes, is_disc, img_width=384.0, img_height=384.0
    )

    # The giant road should be identified as a container and suppressed
    assert len(containers) == 1
    assert containers[0] == [0.0, 150.0, 384.0, 300.0]

    # The 2 cars and the non-enclosing sky remain
    assert len(filtered_boxes) == 3
    assert [50.0, 200.0, 100.0, 250.0] in filtered_boxes
    assert [150.0, 200.0, 200.0, 250.0] in filtered_boxes
    assert [0.0, 0.0, 384.0, 100.0] in filtered_boxes


def test_car_with_subparts_not_suppressed_as_container() -> None:
    from student_detector.sam_student_data import suppress_container_boxes

    # A discovered car containing 2 discovered wheels (area fraction 3.4% of image)
    # Car: [50, 50, 150, 100] (area 5000)
    # Wheel 1: [60, 80, 80, 100] (area 400)
    # Wheel 2: [120, 80, 140, 100] (area 400)
    boxes = [
        [50.0, 50.0, 150.0, 100.0],
        [60.0, 80.0, 80.0, 100.0],
        [120.0, 80.0, 140.0, 100.0],
    ]
    is_disc = [True, True, True]

    filtered_boxes, containers = suppress_container_boxes(
        boxes, is_disc, img_width=384.0, img_height=384.0
    )

    # Car must NOT be suppressed as a container because it is an atomic object (< 20% image area)
    assert len(containers) == 0
    assert len(filtered_boxes) == 3
    assert [50.0, 50.0, 150.0, 100.0] in filtered_boxes


