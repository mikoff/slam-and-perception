from __future__ import annotations

import hashlib
import json
from pathlib import Path

from PIL import Image

from dataset_pipeline.coco_official import filter_official_coco
from dataset_pipeline.config import DatasetConfig, OfficialAnnotationConfig
from dataset_pipeline.config import ensure_workspace
from dataset_pipeline.detection_conversion import quad_object
from dataset_pipeline.identity_audit import (
    generate_identity_audit_bundle,
    recover_identity_review_text,
)
from dataset_pipeline.reports import write_json


def _official_fixture(root: Path) -> tuple[DatasetConfig, Path]:
    root.mkdir(parents=True)
    (root / "meta.json").write_text(
        json.dumps({"classes": [{"title": "car"}, {"title": "person"}]}),
        encoding="utf-8",
    )
    image_dir = root / "train2017/img"
    annotation_dir = root / "train2017/ann"
    image_dir.mkdir(parents=True)
    annotation_dir.mkdir()
    Image.new("RGB", (40, 30), "white").save(image_dir / "one.jpg")
    (annotation_dir / "one.jpg.json").write_text(
        json.dumps({"size": {"width": 40, "height": 30}, "objects": []}),
        encoding="utf-8",
    )
    official = root / "official.json"
    annotations = [
        {
            "id": 10,
            "image_id": 1,
            "category_id": 1,
            "bbox": [1, 1, 12, 10],
            "area": 40,
            "iscrowd": 0,
            "segmentation": [
                [1, 1, 5, 1, 5, 5, 1, 5],
                [9, 7, 13, 7, 13, 11, 9, 11],
            ],
        },
        {
            "id": 11,
            "image_id": 1,
            "category_id": 2,
            "bbox": [15, 2, 8, 12],
            "area": 80,
            "iscrowd": 1,
            "segmentation": {"counts": "encoded", "size": [30, 40]},
        },
        {
            "id": 12,
            "image_id": 1,
            "category_id": 1,
            "bbox": [25, 2, 5, 5],
            "area": 25,
            "iscrowd": 0,
            "segmentation": [],
        },
        {
            "id": 13,
            "image_id": 1,
            "category_id": 1,
            "bbox": [25, 2, 5, 5],
            "area": 25,
            "iscrowd": 0,
            "segmentation": [],
        },
    ]
    official.write_text(
        json.dumps(
            {
                "images": [
                    {"id": 1, "file_name": "one.jpg", "width": 40, "height": 30}
                ],
                "annotations": annotations,
                "categories": [
                    {"id": 1, "name": "car"},
                    {"id": 2, "name": "person"},
                ],
            }
        ),
        encoding="utf-8",
    )
    digest = hashlib.sha256(official.read_bytes()).hexdigest()
    dataset = DatasetConfig(
        "coco_2017",
        root / "unused.tar",
        root,
        official_annotations={"train2017": OfficialAnnotationConfig(official, digest)},
    )
    return dataset, official


def test_official_coco_preserves_identity_multipart_and_crowd(
    tmp_path: Path, taxonomy
) -> None:
    dataset, _ = _official_fixture(tmp_path / "raw/coco_2017")
    destination = tmp_path / "filtered/coco_2017"

    plan, _, provenance = filter_official_coco(
        dataset, destination, taxonomy, "symlink", True, None, False, False
    )

    data = json.loads(
        (destination / "train2017/ann/one.jpg.json").read_text(encoding="utf-8")
    )
    by_id = {obj["sourceAnnotationId"]: obj for obj in data["objects"]}
    assert set(by_id) == {"10", "11", "12", "13"}
    assert by_id["10"]["geometryType"] == "multipolygon"
    assert by_id["10"]["sourceSegmentCount"] == 2
    assert by_id["11"]["originalIsCrowd"] is True
    assert by_id["11"]["ignoreRegion"] is True
    assert by_id["11"]["supervisionState"] == "ignore"
    assert plan["source_annotation_count"] == 4
    assert provenance["multi_part_instances_merged"] == 1
    assert provenance["crowd_regions_mapped_to_ignore"] == 1
    group = provenance["high_multiplicity_exact_geometry_groups"][0]
    assert group["annotation_ids"] == [12, 13]
    assert provenance["exact_geometric_duplicates_removed"] == 0


def test_official_coco_geometry_converts_to_one_target_per_identity(
    tmp_path: Path, taxonomy
) -> None:
    dataset, _ = _official_fixture(tmp_path / "raw/coco_2017")
    destination = tmp_path / "filtered/coco_2017"
    filter_official_coco(
        dataset, destination, taxonomy, "symlink", True, None, False, False
    )
    data = json.loads(
        (destination / "train2017/ann/one.jpg.json").read_text(encoding="utf-8")
    )

    converted = [quad_object(obj, 40, 30, True)[0] for obj in data["objects"]]

    assert len(converted) == 4
    assert len({obj["sourceAnnotationId"] for obj in converted}) == 4
    assert converted[0]["geometryTier"] == "hbb_fallback"
    assert converted[0]["geometryConversionMethod"] == "fallback_hbb"
    assert converted[1]["supervisionState"] == "ignore"


def test_identity_audit_includes_random_sample_and_every_coincident_group(
    tmp_path: Path, taxonomy, config_factory
) -> None:
    config = config_factory({"coco_2017": {"archive": tmp_path / "unused-coco.tar"}})
    ensure_workspace(config)
    dataset, _ = _official_fixture(tmp_path / "raw/coco_2017")
    destination = config.workspace_root / "intermediate/filtered/coco_2017"
    _, _, provenance = filter_official_coco(
        dataset, destination, taxonomy, "symlink", True, None, False, False
    )
    write_json(config.reports / "source_provenance.json", [provenance])

    report = generate_identity_audit_bundle(config, 1)

    assert report["random_sample_rendered"] == 1
    assert report["high_multiplicity_groups_rendered"] == 1
    assert report["total_cards"] == 2
    bundle = config.reports / "coco_identity_audit_bundle"
    assert (bundle / "index.html").is_file()
    assert len(list((bundle / "assets").glob("*.jpg"))) == 2

    review_text = tmp_path / "identity-review.txt"
    review_text.write_text(
        "Reviewer\nmikoff\n"
        "group-00000\nDecision\nPASS\nIssue\nNotes\n"
        "sample-00000\nDecision\nFAIL\nIssue\nidentity\nNotes\n",
        encoding="utf-8",
    )
    recovery = recover_identity_review_text(bundle, review_text, tmp_path / "recovered")
    assert recovery["reviewer"] == "mikoff"
    assert recovery["decision_counts"] == {"FAIL": 1, "PASS": 1}
    assert recovery["failures"][0]["issue"] == "identity"
    assert recovery["review_complete"] is True


def test_official_coco_records_dataset_ninja_appended_image_extension(
    tmp_path: Path, taxonomy
) -> None:
    root = tmp_path / "raw/coco_2017"
    dataset, _ = _official_fixture(root)
    (root / "train2017/img/one.jpg").rename(root / "train2017/img/one.jpg.png")

    _, _, provenance = filter_official_coco(
        dataset,
        tmp_path / "filtered/coco_2017",
        taxonomy,
        "symlink",
        True,
        None,
        False,
        False,
    )

    assert provenance["image_filename_fallbacks"] == [
        {
            "split": "train2017",
            "image_id": 1,
            "official_file_name": "one.jpg",
            "local_file_name": "one.jpg.png",
            "reason": "dataset_ninja_appended_image_format_extension",
        }
    ]
