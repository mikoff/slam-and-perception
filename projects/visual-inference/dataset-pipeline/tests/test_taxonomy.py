import pytest

from dataset_pipeline.taxonomy import UnmappedCategoryError


def test_normalization_and_ids(taxonomy):
    assert taxonomy.normalize(" Traffic Light ") == "traffic_light"
    result = taxonomy.map("bdd100k_images_100k", "Traffic Sign")
    assert result.canonical == "traffic_sign"
    assert result.category_id == taxonomy.data["canonical_id_order"].index("traffic_sign") + 1


def test_ignore_and_unmapped(taxonomy):
    assert taxonomy.map("bdd100k_images_100k", "drivable area").ignored
    with pytest.raises(UnmappedCategoryError):
        taxonomy.map("bdd100k_images_100k", "hovercraft")


def test_nuimages_other_guard(taxonomy):
    with pytest.raises(ValueError, match="guard"):
        taxonomy.map("nuimages", "other", "other vehicle")
    assert taxonomy.map("nuimages", "other").canonical == "pedestrian_other"
    assert taxonomy.map("nuimages", "other", "human pedestrian other").canonical == "pedestrian_other"


def test_coco_identity(taxonomy):
    assert taxonomy.map("coco_2017", "sports ball").canonical == "sports_ball"
