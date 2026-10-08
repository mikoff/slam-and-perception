from __future__ import annotations

import pytest
from torch.utils.data import DataLoader

from student_detector.data import collate_proposal_samples
from student_detector.human_gt_data import (
    DEFAULT_GT_INDEX_PATH,
    HumanGTHBBDataset,
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
