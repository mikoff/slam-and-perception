from __future__ import annotations

import pytest
from torch.utils.data import DataLoader

from student_detector.data import collate_proposal_samples
from student_detector.human_gt_data import (
    DEFAULT_GT_INDEX_PATH,
    HumanGTHBBDataset,
    HumanGTQuadDataset,
    collate_quad_proposal_samples,
)
from student_detector.sam_student_data import DEFAULT_PSEUDO_LABELS_DB


def test_human_gt_hbb_dataset_loads_samples() -> None:
    if not DEFAULT_PSEUDO_LABELS_DB.exists() or not DEFAULT_GT_INDEX_PATH.exists():
        pytest.skip("Required SQLite databases not available in test environment.")

    dataset = HumanGTHBBDataset(
        cohort_db_path=DEFAULT_PSEUDO_LABELS_DB,
        gt_index_path=DEFAULT_GT_INDEX_PATH,
        split="val",
        val_ratio=0.01,
        input_size=128,
    )
    assert len(dataset) > 0

    sample = dataset[0]
    assert sample.image.shape == (3, 128, 128)
    assert sample.boxes.ndim == 2
    assert sample.boxes.shape[-1] == 4
    assert sample.ignore_boxes.ndim == 2
    assert sample.ignore_boxes.shape[-1] == 4

    loader = DataLoader(
        dataset,
        batch_size=2,
        collate_fn=collate_proposal_samples,
        num_workers=0,
    )
    images, batch = next(iter(loader))
    assert images.shape == (2, 3, 128, 128)
    assert len(batch) == 2


def test_human_gt_quad_dataset_loads_samples() -> None:
    if not DEFAULT_PSEUDO_LABELS_DB.exists() or not DEFAULT_GT_INDEX_PATH.exists():
        pytest.skip("Required SQLite databases not available in test environment.")

    dataset = HumanGTQuadDataset(
        cohort_db_path=DEFAULT_PSEUDO_LABELS_DB,
        gt_index_path=DEFAULT_GT_INDEX_PATH,
        split="val",
        val_ratio=0.01,
        input_size=128,
    )
    assert len(dataset) > 0

    sample = dataset[0]
    assert sample.image.shape == (3, 128, 128)
    assert sample.quads.ndim == 3
    assert sample.quads.shape[-2:] == (4, 2)
    assert sample.ignore_quads.ndim == 3
    assert sample.ignore_quads.shape[-2:] == (4, 2)

    loader = DataLoader(
        dataset,
        batch_size=2,
        collate_fn=collate_quad_proposal_samples,
        num_workers=0,
    )
    images, batch = next(iter(loader))
    assert images.shape == (2, 3, 128, 128)
    assert len(batch) == 2


def test_get_stage2_cohort_ids_enriched() -> None:
    from student_detector.human_gt_data import (
        DEFAULT_ENRICHED_GT_INDEX_PATH,
        STAGE2_ENRICHED_COHORT_QUOTAS,
        get_stage2_cohort_ids,
    )

    if not DEFAULT_ENRICHED_GT_INDEX_PATH.exists():
        pytest.skip("Enriched GT index not available.")

    cohort_ids = get_stage2_cohort_ids(
        DEFAULT_ENRICHED_GT_INDEX_PATH, target_count=14000
    )
    expected_sum = sum(STAGE2_ENRICHED_COHORT_QUOTAS.values())
    assert len(cohort_ids) == expected_sum or len(cohort_ids) > 13900


def test_human_gt_enriched_dataset_loads_samples() -> None:
    from student_detector.human_gt_data import (
        DEFAULT_ENRICHED_GT_INDEX_PATH,
        HumanGTHBBDataset,
        HumanGTQuadDataset,
    )

    if not DEFAULT_ENRICHED_GT_INDEX_PATH.exists():
        pytest.skip("Enriched GT index not available.")

    hbb_ds = HumanGTHBBDataset(
        gt_index_path=DEFAULT_ENRICHED_GT_INDEX_PATH,
        cohort_size=14000,
        split="val",
        val_ratio=0.01,
        input_size=128,
    )
    assert len(hbb_ds) > 0
    hbb_sample = hbb_ds[0]
    assert hbb_sample.image.shape == (3, 128, 128)
    assert hbb_sample.boxes.shape[-1] == 4

    quad_ds = HumanGTQuadDataset(
        gt_index_path=DEFAULT_ENRICHED_GT_INDEX_PATH,
        cohort_size=14000,
        split="val",
        val_ratio=0.01,
        input_size=128,
    )
    assert len(quad_ds) > 0
    quad_sample = quad_ds[0]
    assert quad_sample.image.shape == (3, 128, 128)
    assert quad_sample.quads.ndim == 3
