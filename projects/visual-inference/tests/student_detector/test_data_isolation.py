"""Tests for detector training and validation isolation checks."""

from pathlib import Path

import pytest

from student_detector.data_isolation import (
    assert_same_image_records,
    audit_training_isolation,
)


def _record(file_name: str, image_id: int) -> dict[str, object]:
    return {
        "file_name": file_name,
        "image_id": image_id,
        "source_dataset": "dataset",
    }


def test_audit_rejects_shared_identity_and_path(tmp_path: Path) -> None:
    (tmp_path / "train.jpg").write_bytes(b"train")
    training = [_record("train.jpg", 1)]

    with pytest.raises(ValueError, match="identities=1"):
        audit_training_isolation(
            training,
            {"reserved": [_record("different.jpg", 1)]},
            image_root=tmp_path,
            identity_validation_sets={"reserved"},
        )
    with pytest.raises(ValueError, match="paths=1"):
        audit_training_isolation(
            training,
            {"external": [_record("train.jpg", 1)]},
            image_root=tmp_path,
            identity_validation_sets=set(),
        )


def test_audit_hashes_external_images_with_local_ids(tmp_path: Path) -> None:
    (tmp_path / "train.jpg").write_bytes(b"same-image")
    (tmp_path / "copy.jpg").write_bytes(b"same-image")
    training = [_record("train.jpg", 1)]

    with pytest.raises(ValueError, match="sha256=1"):
        audit_training_isolation(
            training,
            {"external": [_record("copy.jpg", 1)]},
            image_root=tmp_path,
            verify_hashes=True,
            identity_validation_sets=set(),
        )


def test_independent_external_id_namespace_does_not_false_positive(
    tmp_path: Path,
) -> None:
    (tmp_path / "train.jpg").write_bytes(b"train")
    (tmp_path / "external.jpg").write_bytes(b"external")

    report = audit_training_isolation(
        [_record("train.jpg", 1)],
        {"external": [_record("external.jpg", 1)]},
        image_root=tmp_path,
        verify_hashes=True,
        identity_validation_sets=set(),
    )

    assert report["validation"]["external"] == {
        "images": 1,
        "identity_check": False,
        "identity_overlap": 0,
        "path_overlap": 0,
        "sha256_overlap": 0,
    }


def test_same_record_assertion_compares_resolved_paths(tmp_path: Path) -> None:
    expected = [_record("first.jpg", 1)]
    actual = [_record("second.jpg", 1)]

    with pytest.raises(ValueError, match="Split mismatch"):
        assert_same_image_records(
            expected,
            actual,
            image_root=tmp_path,
            expected_name="expected",
            actual_name="actual",
        )
