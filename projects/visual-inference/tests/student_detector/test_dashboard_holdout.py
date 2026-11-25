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


def test_default_paths_resolve_to_repo_root_and_env_overrides(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    import importlib

    import student_detector.human_gt_data as hgt
    import student_detector.sam_student_data as ssd

    repo_root = Path(__file__).resolve().parents[4]
    monkeypatch.delenv("VISUAL_INFERENCE_DATA_ROOT", raising=False)
    monkeypatch.delenv("VISUAL_INFERENCE_IMAGE_ROOT", raising=False)
    monkeypatch.delenv("VISUAL_INFERENCE_GT_INDEX", raising=False)
    monkeypatch.delenv("VISUAL_INFERENCE_ENRICHED_GT_INDEX", raising=False)
    importlib.reload(ssd)
    importlib.reload(hgt)

    expected_data = repo_root / "data" / "visual-inference-datasets"
    assert ssd.DEFAULT_DATA_ROOT == expected_data
    assert ssd.DEFAULT_IMAGE_ROOT == expected_data / "output"
    assert hgt.DEFAULT_GT_INDEX_PATH == expected_data / "output" / "indexes" / "quad_train.sqlite"
    assert (
        hgt.DEFAULT_ENRICHED_GT_INDEX_PATH
        == expected_data / "output" / "indexes" / "quad_train_enriched.sqlite"
    )

    monkeypatch.setenv("VISUAL_INFERENCE_DATA_ROOT", str(tmp_path / "custom_data"))
    importlib.reload(ssd)
    importlib.reload(hgt)
    assert ssd.DEFAULT_IMAGE_ROOT == tmp_path / "custom_data" / "output"
    assert (
        hgt.DEFAULT_ENRICHED_GT_INDEX_PATH
        == tmp_path / "custom_data" / "output" / "indexes" / "quad_train_enriched.sqlite"
    )

    monkeypatch.delenv("VISUAL_INFERENCE_DATA_ROOT", raising=False)
    importlib.reload(ssd)
    importlib.reload(hgt)


def test_release_bundle_integrity() -> None:
    import json

    proj_root = Path(__file__).resolve().parents[2]
    rel = proj_root / "release"
    assert (rel / "checkpoints" / "owlv2_30k_hbb_v3_best.pt").is_file()
    assert (rel / "checkpoints" / "owlv2_30k_quad_best.pt").is_file()

    for sub in ("vocabs/hbb_v3", "vocabs/quad"):
        for name in (
            "owlv2_adapted_vocab_core27.pt",
            "owlv2_adapted_vocab_31classes_core27_zeroshot.pt",
            "owlv2_adapted_vocab_37classes.pt",
            "owlv2_adapted_vocab_41classes_zeroshot.pt",
        ):
            assert (rel / sub / name).is_file()

    for name in (
        "owlv2_vocabulary_bank_core27.pt",
        "owlv2_vocabulary_bank_31classes_core27_zeroshot.pt",
        "owlv2_vocabulary_bank_37classes.pt",
        "owlv2_vocabulary_bank_41classes_zeroshot.pt",
        "owlv2_vocabulary_bank_distractors.pt",
    ):
        assert (rel / "vocabs" / "frozen" / name).is_file()

    holdout_db = rel / "benchmarks" / "holdout_210.sqlite"
    assert holdout_db.is_file()
    with sqlite3.connect(f"file:{holdout_db}?mode=ro&immutable=1", uri=True) as conn:
        assert conn.execute("PRAGMA quick_check").fetchone()[0] == "ok"
        assert conn.execute("SELECT COUNT(*) FROM images").fetchone()[0] == 210
        assert conn.execute("SELECT COUNT(*) FROM annotations").fetchone()[0] == 24434

    lock = json.loads((proj_root / "configs" / "holdout_210_ids.json").read_text())
    manifest = json.loads((rel / "benchmarks" / "holdout_210_manifest.json").read_text())
    assert len(manifest["images"]) == 210
    assert [int(r["image_id"]) for r in manifest["images"]] == [
        int(r["image_id"]) for r in lock["images"]
    ]
    assert [str(r["sha256"]) for r in manifest["images"]] == [
        str(r["sha256"]) for r in lock["images"]
    ]
    assert all(not str(r["file_name"]).startswith("/") for r in manifest["images"])


def test_benchmark_build_split_datasets_only_holdout(tmp_path: Path) -> None:
    import json

    from scripts.benchmark_open_vocab_options import build_split_datasets

    mini_db = tmp_path / "mini_gt.sqlite"
    with sqlite3.connect(mini_db) as conn:
        conn.execute(
            "CREATE TABLE images (row_index INTEGER, image_id INTEGER PRIMARY KEY, file_name TEXT, source_dataset TEXT, width INTEGER, height INTEGER)"
        )
        conn.execute(
            "CREATE TABLE annotations (image_id INTEGER, x1 REAL, y1 REAL, x2 REAL, y2 REAL, ignore_region INTEGER, category_name TEXT, geometry_tier TEXT)"
        )
        conn.execute(
            "INSERT INTO images VALUES (0, 101, 'images/sample.jpg', 'coco_2017', 64, 64)"
        )
        conn.execute(
            "INSERT INTO annotations VALUES (101, 4.0, 4.0, 32.0, 32.0, 0, 'car', 'source_hbb')"
        )
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "images": [
                    {
                        "image_id": 101,
                        "file_name": "images/sample.jpg",
                        "source_dataset": "coco_2017",
                        "width": 64,
                        "height": 64,
                    }
                ]
            }
        )
    )
    splits = build_split_datasets(
        "hbb",
        mini_db,
        tmp_path,
        manifest,
        only_holdout=True,
    )
    assert list(splits.keys()) == ["holdout_210"]
    assert len(splits["holdout_210"]) == 1
    assert splits["holdout_210"].images[0]["image_id"] == 101
