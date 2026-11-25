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


def test_tracked_holdout_210_ids_lockfile_is_consistent_and_leak_free() -> None:
    import json

    proj_root = Path(__file__).resolve().parents[2]
    lock_path = proj_root / "configs" / "holdout_210_ids.json"
    assert lock_path.exists(), f"Missing tracked holdout lockfile: {lock_path}"
    data = json.loads(lock_path.read_text(encoding="utf-8"))

    images = data["images"]
    assert len(images) == 210
    holdout_ids = [int(r["image_id"]) for r in images]
    holdout_shas = [str(r["sha256"]) for r in images]
    assert len(set(holdout_ids)) == 210
    assert len(set(holdout_shas)) == 210
    assert all(not str(r["file_name"]).startswith("/home/") for r in images)

    excl_train_ids = data["excluded_training_image_ids"]
    excl_val_shas = data["excluded_validation_sha256s"]
    cohort_val_ids = data["cohort_val_800_image_ids"]
    assert len(excl_train_ids) == 7200
    assert len(excl_val_shas) == 800
    assert len(cohort_val_ids) == 800
    assert set(holdout_ids).isdisjoint({int(x) for x in excl_train_ids})
    assert set(holdout_shas).isdisjoint(set(excl_val_shas))
    assert set(holdout_ids).issubset({int(x) for x in cohort_val_ids})

    full_manifest = proj_root / "artifacts" / "dashboard_holdout_8k_8k_full.json"
    if full_manifest.exists():
        full = json.loads(full_manifest.read_text(encoding="utf-8"))
        full_imgs = full["images"]
        assert [int(r["image_id"]) for r in full_imgs] == holdout_ids
        assert [str(r["sha256"]) for r in full_imgs] == holdout_shas

    cache_db = proj_root / "artifacts" / "cache" / "owlv2_teacher_30k_patched.sqlite"
    if cache_db.exists():
        with sqlite3.connect(f"file:{cache_db}?mode=ro&immutable=1", uri=True) as conn:
            cache_ids = {
                int(r[0]) for r in conn.execute("SELECT image_id FROM processed_images")
            }
        assert set(holdout_ids).isdisjoint(cache_ids)
