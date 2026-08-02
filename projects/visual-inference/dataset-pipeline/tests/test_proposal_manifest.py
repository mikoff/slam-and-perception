from __future__ import annotations

from pathlib import Path

from dataset_pipeline.proposal_manifest import SCHEMA_VERSION, build_manifest, validate_manifest
from dataset_pipeline.reports import write_json
import pytest


def test_build_and_validate_compact_proposal_manifest(tmp_path: Path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    (image_root / "one.jpg").write_bytes(b"placeholder")
    manifest = build_manifest({
        "images": [{
            "id": 1,
            "file_name": "one.jpg",
            "width": 20,
            "height": 10,
            "source_dataset": "coco_2017",
            "source_split": "train",
            "source_image_id": "one",
        }],
        "annotations": [{
            "image_id": 1,
            "bbox": [1.0, 2.0, 8.0, 6.0],
            "quad": [[1.0, 2.0], [9.0, 2.0], [9.0, 8.0], [1.0, 8.0]],
            "geometry_tier": "source_quad",
            "fit_coverage": 1.0,
            "source_annotation_id": "a1",
            "source_category": "car",
        }],
    }, "train")
    assert manifest["schema_version"] == SCHEMA_VERSION
    path = tmp_path / "proposals_train.json"
    write_json(path, manifest, compact=True)
    report = validate_manifest(path, image_root)
    assert report["accepted_geometry"] == 1
    assert report["fit_coverage_failures"] == 0


def test_manifest_validation_rejects_counter_clockwise_quad(tmp_path: Path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    (image_root / "one.jpg").write_bytes(b"placeholder")
    manifest = build_manifest({
        "images": [{"id": 1, "file_name": "one.jpg", "width": 20, "height": 10}],
        "annotations": [{
            "image_id": 1,
            "bbox": [1.0, 2.0, 8.0, 6.0],
            "quad": [[1.0, 2.0], [1.0, 8.0], [9.0, 8.0], [9.0, 2.0]],
            "fit_coverage": 1.0,
            "fit_tightness": 1.0,
        }],
    }, "train")
    path = tmp_path / "proposals_train.json"
    write_json(path, manifest, compact=True)
    with pytest.raises(ValueError, match="manifest validation failed"):
        validate_manifest(path, image_root)
