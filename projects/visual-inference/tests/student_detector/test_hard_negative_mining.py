from __future__ import annotations

from pathlib import Path
import sqlite3

from student_detector.data import ImageRecord, build_coco_sqlite_index
from student_detector.hard_negative_mining import (
    deduplicate_and_cap,
    hamming_distance,
    review_classification,
    select_hashed_pool_indices,
)


def _record(row: int, source: str) -> ImageRecord:
    return ImageRecord(row, row + 10, f"{row}.jpg", 20, 10, source, "camera", True, 1)


def test_hashed_pool_is_order_independent_and_stratified() -> None:
    records = [_record(index, "a" if index < 6 else "b") for index in range(10)]
    selected = select_hashed_pool_indices(records, {"a": 0.6, "b": 0.4}, 5, seed=7)
    identities = {
        (records[index].source_dataset, records[index].image_id) for index in selected
    }
    reversed_records = list(reversed(records))
    reversed_selected = select_hashed_pool_indices(
        reversed_records, {"a": 0.6, "b": 0.4}, 5, seed=7
    )
    reversed_identities = {
        (reversed_records[index].source_dataset, reversed_records[index].image_id)
        for index in reversed_selected
    }

    assert identities == reversed_identities
    assert sum(source == "a" for source, _ in identities) == 3
    assert sum(source == "b" for source, _ in identities) == 2


def test_review_classification_preserves_uncertain_background() -> None:
    common = {
        "best_object_iou": 0.05,
        "ignore_fraction": 0.0,
        "trusted_background_fraction": 0.0,
        "semantic_state": None,
        "semantic_fraction": 0.0,
        "unmatched_iou": 0.1,
        "region_fraction": 0.5,
    }
    assert review_classification(**common) == "unverified_background"
    assert review_classification(**(common | {"best_object_iou": 0.2})) is None
    assert (
        review_classification(**(common | {"ignore_fraction": 0.6})) == "ignore_region"
    )
    assert (
        review_classification(**(common | {"trusted_background_fraction": 0.6}))
        == "certified_trusted_background"
    )
    assert (
        review_classification(
            **(common | {"semantic_state": "positive", "semantic_fraction": 0.8})
        )
        == "possible_labeled_object"
    )


def test_deduplication_keeps_highest_score_before_caps() -> None:
    def candidate(
        identity: str, score: float, crop_hash: str, image: int
    ) -> dict[str, object]:
        return {
            "candidate_id": identity,
            "score": score,
            "crop_dhash": crop_hash,
            "domain": "general",
            "source_dataset": "source",
            "image_id": image,
        }

    candidates = [
        candidate("high", 0.9, "0000000000000000", 1),
        candidate("duplicate", 0.8, "0000000000000001", 2),
        candidate("same-image", 0.7, "ffffffffffffffff", 1),
    ]
    kept, rejected = deduplicate_and_cap(
        candidates,
        per_image=1,
        per_domain={"general": 2},
        perceptual_hamming=1,
    )

    assert [item["candidate_id"] for item in kept] == ["high"]
    assert rejected == {"near_duplicate_crop": 1, "per_image_cap": 1}
    assert hamming_distance("0f", "0e") == 1


def test_verified_prebuilt_index_can_skip_large_source_rescan(tmp_path: Path) -> None:
    index = tmp_path / "quad_train.sqlite"
    with sqlite3.connect(index) as connection:
        connection.execute("CREATE TABLE metadata (key TEXT, value TEXT)")
        connection.executemany(
            "INSERT INTO metadata VALUES (?, ?)",
            [
                ("schema_version", "8:proposal-manifest"),
                ("source_signature", "sha256:frozen"),
            ],
        )

    result = build_coco_sqlite_index(
        tmp_path / "large-manifest-is-not-read.json",
        index,
        build_if_missing=False,
        verify_source_signature=False,
    )

    assert result == index.resolve()
