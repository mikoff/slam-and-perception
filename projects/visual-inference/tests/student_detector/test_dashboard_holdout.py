"""Training leakage checks for evaluation-only dashboard manifests."""

from pathlib import Path
import sqlite3

import pytest

from student_detector.dashboard_holdout import (
    audit_records,
    file_sha256,
    training_inventory,
    validation_inventory,
)


def test_training_inventory_excludes_the_whole_candidate_cohort(
    tmp_path: Path,
) -> None:
    """Dashboard images must be outside both pipeline train and validation splits."""
    db = tmp_path / "quad_train.sqlite"
    with sqlite3.connect(db) as conn:
        conn.execute(
            "CREATE TABLE images (row_index INTEGER, image_id INTEGER, file_name TEXT, source_dataset TEXT)"
        )
        for i in range(3):
            (tmp_path / f"{i}.jpg").write_bytes(f"image-{i}".encode())
            conn.execute(
                "INSERT INTO images VALUES (?, ?, ?, 'coco_2017')", (i, i, f"{i}.jpg")
            )
    inventory = training_inventory(db, tmp_path, 2)
    assert {row["image_id"] for row in inventory} == {0, 1}
    audit_records([{"image_id": 2, "file_name": "2.jpg"}], inventory, tmp_path)
    with pytest.raises(ValueError, match="Training overlap"):
        audit_records([{"image_id": 1, "file_name": "1.jpg"}], inventory, tmp_path)


@pytest.mark.parametrize("alias", ["symlink", "copy", "reused_id"])
def test_relabeling_training_images_cannot_bypass_audit(
    tmp_path: Path, alias: str
) -> None:
    training = tmp_path / "train.jpg"
    training.write_bytes(b"training-image")
    evaluation = tmp_path / "eval.jpg"
    if alias == "symlink":
        evaluation.symlink_to(training)
    elif alias == "copy":
        evaluation.write_bytes(training.read_bytes())
    else:
        evaluation.write_bytes(b"different-image")
    inventory = [
        {"image_id": 1, "file_name": "train.jpg", "sha256": file_sha256(training)}
    ]
    with pytest.raises(ValueError, match="Training overlap"):
        audit_records(
            [{"image_id": 1 if alias == "reused_id" else 2, "file_name": "eval.jpg"}],
            inventory,
            tmp_path,
        )


def test_manifest_detects_changed_images_and_duplicate_cards(tmp_path: Path) -> None:
    path = tmp_path / "eval.jpg"
    path.write_bytes(b"held-out-image")
    row = {"image_id": 2, "file_name": "eval.jpg", "sha256": file_sha256(path)}
    with pytest.raises(ValueError, match="Duplicate evaluation image"):
        audit_records([row, {**row, "image_id": 3}], [], tmp_path)
    path.write_bytes(b"replacement-image")
    with pytest.raises(ValueError, match="Evaluation image changed"):
        audit_records([row], [], tmp_path)


def test_validation_inventory_hashes_external_split(tmp_path: Path) -> None:
    image = tmp_path / "validation.jpg"
    image.write_bytes(b"validation-image")
    labels = tmp_path / "validation.json"
    labels.write_text(
        '{"images":[{"image_id":1,"file_name":"validation.jpg",'
        '"source_dataset":"coco_2017"}]}'
    )
    inventory = validation_inventory(labels, tmp_path)
    assert inventory == [
        {
            "image_id": None,
            "file_name": "validation.jpg",
            "source_dataset": "coco_2017",
            "sha256": file_sha256(image),
        }
    ]
    with pytest.raises(ValueError, match="Training overlap"):
        audit_records(
            [{"image_id": 999, "file_name": "validation.jpg"}], inventory, tmp_path
        )
