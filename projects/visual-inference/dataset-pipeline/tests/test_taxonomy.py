import pytest

from dataset_pipeline.taxonomy import UnmappedCategoryError


def test_normalization_and_ids(taxonomy):
    assert taxonomy.normalize(" Traffic Light ") == "traffic_light"
    result = taxonomy.map("bdd100k_images_100k", "Traffic Sign")
    assert result.canonical == "traffic_sign"
    assert result.category_id == taxonomy.data["canonical_id_order"].index("traffic_sign") + 1


def test_ignore_and_unmapped(taxonomy):
    assert taxonomy.map("bdd100k_images_100k", "drivable area").ignored
    ignore_region = taxonomy.map("woodscape_rgb_fisheye", "grouped vehicles")
    assert not ignore_region.ignored
    assert ignore_region.ignore_region
    assert ignore_region.category_id == taxonomy.ignore_region_category_id
    with pytest.raises(UnmappedCategoryError):
        taxonomy.map("bdd100k_images_100k", "hovercraft")


def test_woodscape_construction_is_an_ignored_static_region(taxonomy):
    result = taxonomy.map("woodscape_rgb_fisheye", "construction")
    assert result.ignore_region
    assert result.canonical == taxonomy.ignore_region_token


def test_nuimages_other_guard(taxonomy):
    with pytest.raises(ValueError, match="guard"):
        taxonomy.map("nuimages", "other", "other vehicle")
    assert taxonomy.map("nuimages", "other").canonical == "pedestrian_other"
    assert taxonomy.map("nuimages", "other", "human pedestrian other").canonical == "pedestrian_other"


def test_coco_identity(taxonomy):
    assert taxonomy.map("coco_2017", "sports ball").canonical == "sports_ball"
