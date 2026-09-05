import math

import pytest

from dataset_pipeline.config import ensure_workspace
from dataset_pipeline.detection_conversion import (
    GeometryError,
    _provenance_report,
    clip_bbox,
    convert_all,
    deduplicate_geometry_representations,
    object_bbox,
    quad_object,
    rectangle_object,
)
from conftest import make_project


@pytest.mark.parametrize("geometry", ["rectangle", "polygon", "polyline"])
def test_geometry_enclosing_rectangle(geometry):
    obj = {
        "geometryType": geometry,
        "points": {"exterior": [[5, 6], [1, 2], [8, 4]], "interior": []},
    }
    assert object_bbox(obj) == (1.0, 2.0, 8.0, 6.0)
    converted, _ = rectangle_object(obj, 10, 10, True)
    assert converted["geometryType"] == "rectangle"


def test_quad_conversion_preserves_source_geometry_and_adds_metrics():
    obj = {
        "classTitle": "car",
        "geometryType": "polygon",
        "points": {"exterior": [[2, 2], [8, 1], [9, 7], [1, 8]], "interior": []},
    }
    converted, changed = quad_object(obj, 20, 10, True)
    assert not changed
    assert converted["geometryType"] == "polygon"
    assert converted["geometryTier"] == "source_quad"
    assert len(converted["quad"]) == 4
    assert converted["fitCoverage"] >= 0.98
    assert 0 <= converted["fitTightness"] <= 1


def test_quad_conversion_canonicalizes_counter_clockwise_source() -> None:
    obj = {
        "geometryType": "polygon",
        "points": {"exterior": [[2, 2], [2, 8], [8, 8], [8, 2]], "interior": []},
    }
    converted, _ = quad_object(obj, 20, 20, True)
    quad = converted["quad"]
    area = 0.5 * sum(
        quad[index][0] * quad[(index + 1) % 4][1]
        - quad[index][1] * quad[(index + 1) % 4][0]
        for index in range(4)
    )
    assert area > 0


def test_invalid_four_point_traversal_is_fitted_not_relabelled_as_source_quad() -> None:
    obj = {
        "geometryType": "polygon",
        "points": {
            "exterior": [[2, 2], [8, 8], [2, 8], [8, 2]],
            "interior": [],
        },
    }

    converted, _ = quad_object(obj, 20, 20, True)

    assert converted["geometryTier"] == "rotated_rect"
    assert converted["fitCoverage"] >= 0.98
    assert "self_intersection" in converted["geometryRepairReasons"]
    assert "invalid_source_quad_fitted" in converted["geometryRepairReasons"]


def test_out_of_frame_vertices_are_clipped_and_logged() -> None:
    obj = {
        "geometryType": "polygon",
        "points": {
            "exterior": [[-2, 2], [8, 2], [8, 8], [-2, 8]],
            "interior": [],
        },
    }

    converted, changed = quad_object(obj, 20, 20, True)

    assert changed
    assert "out_of_frame_coordinates" in converted["geometryRepairReasons"]


def test_degenerate_geometry_becomes_ignore_in_conversion(tmp_path, config_factory):
    config = config_factory(
        {"bdd100k_images_100k": {"archive": tmp_path / "unused.tar"}}
    )
    ensure_workspace(config)
    source = config.workspace_root / "intermediate/filtered/bdd100k_images_100k"
    make_project(source, "car", "polygon")
    annotation_path = next((source / "train/ann").iterdir())
    import json

    annotation = json.loads(annotation_path.read_text())
    annotation["objects"][0]["points"]["exterior"] = [[5, 5], [5, 5], [5, 8], [5, 8]]
    annotation["objects"][0]["supervisionState"] = "trusted_background"
    annotation["objects"][0]["trustedBackgroundRegion"] = True
    annotation_path.write_text(json.dumps(annotation))
    convert_all(config)
    converted_path = (
        config.workspace_root
        / "intermediate/detection/bdd100k_images_100k/train/ann"
        / annotation_path.name
    )
    converted = json.loads(converted_path.read_text())["objects"][0]
    assert converted["ignoreRegion"] is True
    assert converted["geometryTier"] == "hbb_fallback"
    assert converted["supervisionState"] == "ignore"
    assert "trustedBackgroundRegion" not in converted


def test_clips_partial_and_rejects_external():
    assert clip_bbox((-2, 1, 5, 12), 10, 10, True) == ((0.0, 1, 5, 9.0), True)
    with pytest.raises(GeometryError, match="entirely"):
        clip_bbox((11, 1, 12, 2), 10, 10)
    with pytest.raises(GeometryError, match="non-finite"):
        clip_bbox((1, 1, math.inf, 2), 10, 10)


def test_rejects_empty_polygon():
    with pytest.raises(GeometryError, match="no points"):
        object_bbox({"geometryType": "polygon", "points": {"exterior": []}})


def test_bitmap_bbox_uses_nonzero_pixels():
    import numpy as np
    import supervisely as sly

    mask = np.zeros((6, 7), dtype=bool)
    mask[1:3, 2:5] = True
    assert object_bbox(sly.Bitmap(mask).to_json()) == (2.0, 1.0, 4.0, 2.0)


def test_parallel_detection_conversion(tmp_path, config_factory):
    config = config_factory(
        {"bdd100k_images_100k": {"archive": tmp_path / "unused.tar"}}
    )
    ensure_workspace(config)
    source = config.workspace_root / "intermediate/filtered/bdd100k_images_100k"
    make_project(source, "car", "polygon")
    reports = convert_all(config, workers=2)
    output = config.workspace_root / "intermediate/detection/bdd100k_images_100k"
    assert reports[0]["converted_annotations"] == 1
    assert (output / ".detection_complete").exists()


def test_deduplicates_only_when_source_identity_agrees():
    rectangle = {
        "id": 2,
        "sourceAnnotationId": "official-10",
        "classTitle": "car",
        "geometryType": "rectangle",
        "points": {"exterior": [[1, 2], [8, 7]]},
    }
    polygon = {
        "id": 1,
        "sourceAnnotationId": "official-10",
        "classTitle": "car",
        "geometryType": "polygon",
        "points": {"exterior": [[1, 2], [8, 2], [8, 7], [1, 7]]},
    }
    other_rectangle = {
        **rectangle,
        "id": 3,
        "sourceAnnotationId": "official-11",
    }
    retained, removals = deduplicate_geometry_representations(
        [polygon, rectangle, other_rectangle]
    )
    assert len(removals) == 1
    assert removals[0]["source_annotation_id"] == "official-10"
    assert removals[0]["reason"] == "superseded_representation_same_source_identity"
    assert retained == [polygon, other_rectangle]


def test_coincident_geometry_with_distinct_source_identities_is_retained():
    objects = [
        {
            "id": index,
            "sourceAnnotationId": f"official-{index}",
            "classTitle": "car",
            "geometryType": "rectangle",
            "points": {"exterior": [[1, 2], [8, 7]]},
        }
        for index in (1, 2)
    ]

    retained, removals = deduplicate_geometry_representations(objects)

    assert retained == objects
    assert removals == []


def test_provenance_report_reconciles_category_counts_and_removal_reasons():
    report = _provenance_report(
        [
            {
                "dataset": "coco_2017",
                "identity_authority": "official_coco_annotation_id",
                "source_annotations_read": 3,
                "unique_source_annotation_identities_retained": 2,
                "multi_part_instances_merged": 1,
                "crowd_regions_mapped_to_ignore": 1,
                "per_category": {
                    "car": {
                        "source_annotations_read": 3,
                        "unique_source_annotation_identities_retained": 2,
                        "multi_part_instances_merged": 1,
                        "crowd_regions_mapped_to_ignore": 1,
                        "dropped_reason:approved_drop": 1,
                    },
                    "person": {
                        "source_annotations_read": 0,
                        "unique_source_annotation_identities_retained": 0,
                    },
                },
            }
        ],
        [
            {
                "dataset": "coco_2017",
                "removals": 1,
                "geometry_tiers_by_category": {"car": {"source_quad": 1}},
            }
        ],
        [
            {
                "dataset": "coco_2017",
                "source_category": "car",
                "reason": "exact_duplicate_same_source_identity",
            }
        ],
    )

    dataset = report["datasets"][0]
    category = dataset["per_category"][0]
    assert len(dataset["per_category"]) == 2
    assert category["annotations_dropped"] == 2
    assert category["drop_reasons"] == {
        "approved_drop": 1,
        "exact_duplicate_same_source_identity": 1,
    }
    assert category["geometry_conversion_tiers"] == {"source_quad": 1}
