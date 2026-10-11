"""Unit tests for SQLite transactional discovery storage and PyTorch DataLoader integration."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from torch.utils.data import DataLoader

from student_detector.discovery_storage import (
    get_dataset_summary,
    get_processed_image_ids,
    init_discovery_db,
    migrate_json_to_sqlite,
    save_image_batch,
    save_image_record,
)
from student_detector.sam_student_data import (
    SAMStudentDataset,
    SAMStudentHBBDataset,
    collate_proposal_samples,
    collate_quad_proposal_samples,
)


@pytest.fixture
def temp_db(tmp_path: Path) -> Path:
    """Fixture providing an initialized SQLite database in temporary path."""
    db_path = tmp_path / "test_discovery.sqlite"
    conn = init_discovery_db(db_path)
    conn.close()
    return db_path


def test_init_discovery_db(temp_db: Path) -> None:
    assert temp_db.exists()
    assert get_processed_image_ids(temp_db) == set()


def test_save_and_retrieve_image_record(temp_db: Path) -> None:
    conn = init_discovery_db(temp_db)
    record = {
        "image_id": 101,
        "file_name": "images/test1.jpg",
        "source_dataset": "coco_2017",
        "width": 640,
        "height": 480,
        "split": "train",
        "annotations": [
            {
                "id": 1,
                "source": "human_gt",
                "category": "car",
                "source_category": "car",
                "bbox": [10.0, 20.0, 100.0, 120.0],
                "quad": [[10.0, 20.0], [100.0, 20.0], [100.0, 120.0], [10.0, 120.0]],
                "polygon": [[10.0, 20.0], [100.0, 20.0], [100.0, 120.0], [10.0, 120.0]],
                "area": 9000.0,
                "is_positive": True,
                "is_discovered": False,
                "is_suppressed": False,
                "suppression_reason": None,
                "sam_stability_score": None,
                "sam_predicted_iou": None,
                "matched_gt_id": None,
                "matched_gt_category": None,
                "matched_gt_iou": None,
            },
            {
                "id": 2,
                "source": "sam_discovered",
                "category": "unlabeled_object",
                "source_category": "sam_amg",
                "bbox": [150.0, 160.0, 250.0, 300.0],
                "quad": [[150.0, 160.0], [250.0, 160.0], [250.0, 300.0], [150.0, 300.0]],
                "polygon": [],
                "area": 14000.0,
                "is_positive": True,
                "is_discovered": True,
                "is_suppressed": False,
                "suppression_reason": None,
                "sam_stability_score": 0.92,
                "sam_predicted_iou": 0.88,
                "matched_gt_id": None,
                "matched_gt_category": None,
                "matched_gt_iou": 0.0,
            },
        ],
    }

    save_image_record(conn, record)
    conn.close()

    processed = get_processed_image_ids(temp_db)
    assert processed == {101}

    summary = get_dataset_summary(temp_db)
    assert summary["total_images"] == 1
    assert summary["total_annotations"] == 2
    assert summary["positive_annotations"] == 2
    assert summary["novel_annotations"] == 1
    assert summary["human_gt_annotations"] == 1


def test_save_image_batch_and_summary(temp_db: Path) -> None:
    conn = init_discovery_db(temp_db)
    batch = [
        {
            "image_id": i,
            "file_name": f"images/test_{i}.jpg",
            "source_dataset": "coco_2017" if i % 2 == 0 else "bdd100k_images_100k",
            "width": 640,
            "height": 480,
            "split": "val" if i <= 3 else "train",
            "annotations": [
                {
                    "source": "human_gt",
                    "category": "person",
                    "bbox": [10.0, 10.0, 50.0, 50.0],
                    "quad": [[10.0, 10.0], [50.0, 10.0], [50.0, 50.0], [10.0, 50.0]],
                    "is_positive": True,
                    "is_discovered": False,
                    "is_suppressed": False,
                }
            ],
        }
        for i in range(1, 11)
    ]

    save_image_batch(conn, batch)
    conn.close()

    assert len(get_processed_image_ids(temp_db)) == 10
    summary = get_dataset_summary(temp_db)
    assert summary["total_images"] == 10
    assert summary["split_counts"]["val"] == 3
    assert summary["split_counts"]["train"] == 7
    assert summary["source_counts"]["coco_2017"] == 5
    assert summary["source_counts"]["bdd100k_images_100k"] == 5


def test_migrate_json_to_sqlite(tmp_path: Path) -> None:
    # Create small synthetic JSON
    json_path = tmp_path / "mock_pilot.json"
    db_path = tmp_path / "mock_discovery.sqlite"

    mock_data = {
        "cohort_sha256": "mock_sha",
        "schema_version": "v1",
        "points_per_side": 24,
        "images": [
            {
                "image_id": i,
                "file_name": f"images/mock_{i}.jpg",
                "source_dataset": "coco_2017",
                "width": 384,
                "height": 384,
                "annotations": [
                    {
                        "source": "human_gt",
                        "category": "car",
                        "bbox": [10.0, 10.0, 50.0, 50.0],
                        "quad": [[10.0, 10.0], [50.0, 10.0], [50.0, 50.0], [10.0, 50.0]],
                        "is_positive": True,
                        "is_discovered": False,
                        "is_suppressed": False,
                    }
                ],
            }
            for i in range(20)
        ],
    }
    json_path.write_text(json.dumps(mock_data), encoding="utf-8")

    n_train, n_val = migrate_json_to_sqlite(json_path, db_path, val_ratio=0.20, split_seed=42)
    assert n_train == 16
    assert n_val == 4

    summary = get_dataset_summary(db_path)
    assert summary["total_images"] == 20
    assert summary["split_counts"]["train"] == 16
    assert summary["split_counts"]["val"] == 4
    assert summary["positive_annotations"] == 20


def test_pytorch_dataset_with_sqlite_multiworker(tmp_path: Path) -> None:
    db_path = tmp_path / "mock_loader.sqlite"
    conn = init_discovery_db(db_path)

    # Create dummy images on disk so Image.open succeeds
    from PIL import Image
    dummy_img = Image.new("RGB", (100, 100), color=(128, 128, 128))
    img_dir = tmp_path / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    batch = []
    for i in range(12):
        img_name = f"images/img_{i}.jpg"
        dummy_img.save(tmp_path / img_name)
        batch.append(
            {
                "image_id": i + 1,
                "file_name": img_name,
                "source_dataset": "coco_2017",
                "width": 100,
                "height": 100,
                "split": "val" if i < 4 else "train",
                "annotations": [
                    {
                        "source": "human_gt",
                        "category": "car",
                        "bbox": [10.0, 10.0, 50.0, 50.0],
                        "quad": [[10.0, 10.0], [50.0, 10.0], [50.0, 50.0], [10.0, 50.0]],
                        "is_positive": True,
                        "is_discovered": False,
                        "is_suppressed": False,
                    }
                ],
            }
        )

    save_image_batch(conn, batch)
    conn.close()

    # Test SAMStudentHBBDataset loading from SQLite
    hbb_train = SAMStudentHBBDataset(
        pseudo_labels_path=db_path,
        image_root=tmp_path,
        split="train",
        input_size=128,
    )
    hbb_val = SAMStudentHBBDataset(
        pseudo_labels_path=db_path,
        image_root=tmp_path,
        split="val",
        input_size=128,
    )

    assert len(hbb_train) == 8
    assert len(hbb_val) == 4

    # Multi-worker DataLoader test (validates thread/process safety of SQLite connections)
    train_loader = DataLoader(
        hbb_train,
        batch_size=4,
        shuffle=True,
        num_workers=2,
        collate_fn=collate_proposal_samples,
    )

    batches = list(train_loader)
    assert len(batches) == 2
    images_tensor, samples = batches[0]
    assert images_tensor.shape == (4, 3, 128, 128)
    assert len(samples) == 4

    # Test SAMStudentDataset (Quads)
    quad_train = SAMStudentDataset(
        pseudo_labels_path=db_path,
        image_root=tmp_path,
        split="train",
        input_size=128,
    )
    assert len(quad_train) == 8
    quad_loader = DataLoader(
        quad_train,
        batch_size=4,
        shuffle=False,
        num_workers=2,
        collate_fn=collate_quad_proposal_samples,
    )
    quad_batches = list(quad_loader)
    assert len(quad_batches) == 2
    q_images, q_samples = quad_batches[0]
    assert q_images.shape == (4, 3, 128, 128)
    assert len(q_samples) == 4
