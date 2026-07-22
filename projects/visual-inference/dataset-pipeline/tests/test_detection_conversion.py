import math

import pytest

from dataset_pipeline.config import ensure_workspace
from dataset_pipeline.detection_conversion import GeometryError, clip_bbox, convert_all, object_bbox, rectangle_object
from conftest import make_project


@pytest.mark.parametrize("geometry", ["rectangle", "polygon", "polyline"])
def test_geometry_enclosing_rectangle(geometry):
    obj = {"geometryType": geometry, "points": {"exterior": [[5, 6], [1, 2], [8, 4]], "interior": []}}
    assert object_bbox(obj) == (1.0, 2.0, 8.0, 6.0)
    converted, _ = rectangle_object(obj, 10, 10, True)
    assert converted["geometryType"] == "rectangle"


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
    config = config_factory({"bdd100k_images_100k": {"archive": tmp_path / "unused.tar"}})
    ensure_workspace(config)
    source = config.workspace_root / "intermediate/filtered/bdd100k_images_100k"
    make_project(source, "car", "polygon")
    reports = convert_all(config, workers=2)
    output = config.workspace_root / "intermediate/detection/bdd100k_images_100k"
    assert reports[0]["converted_annotations"] == 1
    assert (output / ".detection_complete").exists()
