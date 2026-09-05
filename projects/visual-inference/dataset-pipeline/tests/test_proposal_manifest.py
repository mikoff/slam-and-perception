from __future__ import annotations

from pathlib import Path

from dataset_pipeline.proposal_manifest import (
    SCHEMA_VERSION,
    build_manifest,
    validate_manifest,
)
from dataset_pipeline.reports import write_json
import pytest


def test_build_and_validate_compact_proposal_manifest(tmp_path: Path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    (image_root / "one.jpg").write_bytes(b"placeholder")
    manifest = build_manifest(
        {
            "images": [
                {
                    "id": 1,
                    "file_name": "one.jpg",
                    "width": 20,
                    "height": 10,
                    "source_dataset": "coco_2017",
                    "source_split": "train",
                    "source_image_id": "one",
                }
            ],
            "annotations": [
                {
                    "image_id": 1,
                    "bbox": [1.0, 2.0, 8.0, 6.0],
                    "quad": [[1.0, 2.0], [9.0, 2.0], [9.0, 8.0], [1.0, 8.0]],
                    "geometry_tier": "source_quad",
                    "fit_coverage": 1.0,
                    "source_annotation_id": "a1",
                    "source_annotation_identity_kind": "official_coco_annotation_id",
                    "source_dataset": "coco_2017",
                    "source_split": "train2017",
                    "source_image_id": "42",
                    "source_category": "car",
                    "canonical_category": "car",
                    "original_iscrowd": True,
                    "original_group": True,
                    "geometry_conversion_method": "official_bbox_fallback",
                    "attributes": {"distance": 15.0, "visibility": "partial"},
                    "supervision_state": "ignore",
                    "exclusion_reason": "",
                    "ignore_region": True,
                }
            ],
        },
        "train",
    )
    assert manifest["schema_version"] == SCHEMA_VERSION
    record = manifest["images"][0]["ignore"][0]
    assert record["source_annotation_id"] == "a1"
    assert record["source_annotation_identity_kind"] == "official_coco_annotation_id"
    assert record["original_category"] == "car"
    assert record["canonical_category"] == "car"
    assert record["original_iscrowd"] is True
    assert record["original_group"] is True
    assert record["geometry_conversion_method"] == "official_bbox_fallback"
    assert record["attributes"] == {"distance": 15.0, "visibility": "partial"}
    assert record["supervision_state"] == "ignore"
    assert record["exclusion_reason"] == ""
    path = tmp_path / "proposals_train.json"
    write_json(path, manifest, compact=True)
    report = validate_manifest(path, image_root)
    assert report["accepted_geometry"] == 1
    assert report["fit_coverage_failures"] == 0


def test_manifest_validation_rejects_counter_clockwise_quad(tmp_path: Path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    (image_root / "one.jpg").write_bytes(b"placeholder")
    manifest = build_manifest(
        {
            "images": [{"id": 1, "file_name": "one.jpg", "width": 20, "height": 10}],
            "annotations": [
                {
                    "image_id": 1,
                    "bbox": [1.0, 2.0, 8.0, 6.0],
                    "quad": [[1.0, 2.0], [1.0, 8.0], [9.0, 8.0], [9.0, 2.0]],
                    "fit_coverage": 1.0,
                    "fit_tightness": 1.0,
                }
            ],
        },
        "train",
    )
    path = tmp_path / "proposals_train.json"
    write_json(path, manifest, compact=True)
    with pytest.raises(ValueError, match="manifest validation failed"):
        validate_manifest(path, image_root)


def test_manifest_is_the_single_authority_for_all_three_states(taxonomy) -> None:
    annotations = []
    for index, state in enumerate(
        ("positive", "ignore", "trusted_background"), start=1
    ):
        annotations.append(
            {
                "image_id": 1,
                "bbox": [index * 2, 1, 1, 1],
                "quad": [
                    [index * 2, 1],
                    [index * 2 + 1, 1],
                    [index * 2 + 1, 2],
                    [index * 2, 2],
                ],
                "source_category": "car",
                "canonical_category": "car",
                "supervision_state": state,
            }
        )
    manifest = build_manifest(
        {
            "images": [{"id": 1, "file_name": "one.jpg", "width": 20, "height": 10}],
            "annotations": annotations,
        },
        "train",
        taxonomy,
    )
    image = manifest["images"][0]
    assert len(image["positive"]) == 1
    assert len(image["ignore"]) == 1
    assert len(image["trusted_background"]) == 1


def test_contained_component_policy_is_applied_during_manifest_generation(
    taxonomy,
) -> None:
    annotations = [
        {
            "image_id": 1,
            "bbox": [1, 1, 16, 8],
            "source_category": "car",
            "canonical_category": "car",
            "supervision_state": "positive",
        },
        {
            "image_id": 1,
            "bbox": [5, 5, 3, 2],
            "source_category": "license_plate",
            "canonical_category": "license_plate",
            "supervision_state": "positive",
        },
    ]
    manifest = build_manifest(
        {
            "images": [{"id": 1, "file_name": "one.jpg", "width": 20, "height": 10}],
            "annotations": annotations,
        },
        "train",
        taxonomy,
    )
    image = manifest["images"][0]
    assert [row["canonical_category"] for row in image["positive"]] == ["car"]
    assert image["ignore"][0]["canonical_category"] == "license_plate"
    assert image["ignore"][0]["exclusion_reason"] == "contained_component_of_parent"


def test_manifest_rejects_positive_with_ignore_flags() -> None:
    with pytest.raises(ValueError, match="contradicts"):
        build_manifest(
            {
                "images": [
                    {"id": 1, "file_name": "one.jpg", "width": 20, "height": 10}
                ],
                "annotations": [
                    {
                        "image_id": 1,
                        "bbox": [1, 1, 3, 3],
                        "supervision_state": "positive",
                        "ignore_region": True,
                    }
                ],
            },
            "train",
        )
