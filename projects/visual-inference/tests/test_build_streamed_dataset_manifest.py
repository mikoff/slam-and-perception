from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.cloud.build_streamed_dataset_manifest import build_manifest
from scripts.cloud.dataset_staging import validate_manifest


def _dataset(tmp_path: Path) -> Path:
    root = tmp_path / "dataset"
    target = tmp_path / "raw.jpg"
    target.write_bytes(b"pixels")
    (root / "images").mkdir(parents=True)
    (root / "indexes").mkdir()
    (root / "images/example.jpg").symlink_to(target)
    (root / "indexes/quad_train.sqlite").write_bytes(b"train")
    (root / "indexes/quad_val.sqlite").write_bytes(b"val")
    return root


def test_manifest_records_archive_level_contract(tmp_path: Path) -> None:
    root = _dataset(tmp_path)
    output = tmp_path / "dataset-manifest.json"
    build_manifest(
        root=root,
        dataset_id="release-1",
        archive_key="datasets/release-1/dataset.tar.gz",
        archive_size=123,
        archive_sha256="a" * 64,
        extracted_size=456,
        dataset_contract_sha256="b" * 64,
        output=output,
    )

    manifest = json.loads(output.read_text(encoding="utf-8"))
    validate_manifest(manifest, "release-1")
    assert manifest["archive"] == {
        "key": "datasets/release-1/dataset.tar.gz",
        "sha256": "a" * 64,
        "size": 123,
    }
    assert manifest["extracted_size"] == 456
    assert manifest["dataset_contract_sha256"] == "b" * 64
    assert set(manifest["required_files"]) == {
        "indexes/quad_train.sqlite",
        "indexes/quad_val.sqlite",
    }


def test_manifest_rejects_a_mismatched_archive_key(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="archive key must be"):
        build_manifest(
            root=_dataset(tmp_path),
            dataset_id="release-1",
            archive_key="datasets/other/dataset.tar.gz",
            archive_size=123,
            archive_sha256="a" * 64,
            extracted_size=456,
            dataset_contract_sha256="b" * 64,
            output=tmp_path / "dataset-manifest.json",
        )
