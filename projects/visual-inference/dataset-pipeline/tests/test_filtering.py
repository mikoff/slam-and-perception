import json

import pytest

import dataset_pipeline.supervisely_filter as filtering_module
from dataset_pipeline.config import DatasetConfig
from dataset_pipeline.supervisely_filter import filter_project
from conftest import make_project


def test_filter_renames_and_links(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "car")
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"
    plan, actions = filter_project(dataset, source, destination, taxonomy)
    assert plan["retained_annotation_count"] == 1
    assert (destination / "train/img/one.jpg").is_symlink()
    annotation = json.loads((destination / "train/ann/one.jpg.json").read_text())
    assert annotation["objects"][0]["sourceCategory"] == "car"
    assert actions["new_symlink"] == 1


def test_approved_scene_region_is_retained_as_trusted_background(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "drivable area", "polygon")
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"
    plan, _ = filter_project(dataset, source, destination, taxonomy)
    annotation = json.loads((destination / "train/ann/one.jpg.json").read_text())
    assert plan["removed_annotation_count"] == 0
    assert plan["has_trusted_background"]
    obj = annotation["objects"][0]
    assert obj["classTitle"] == "drivable_surface"
    assert obj["supervisionState"] == "trusted_background"
    assert obj["trustedBackgroundRegion"] is True


def test_ignore_region_and_tag_attributes_are_preserved(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "grouped vehicles", "polygon")
    annotation_path = source / "train/ann/one.jpg.json"
    source_annotation = json.loads(annotation_path.read_text())
    source_annotation["objects"][0]["tags"] = [
        {"name": "occlusion", "value": "50-74%"},
        {"name": "depiction"},
    ]
    annotation_path.write_text(json.dumps(source_annotation))
    dataset = DatasetConfig(
        "woodscape_rgb_fisheye", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"
    plan, _ = filter_project(dataset, source, destination, taxonomy)
    annotation = json.loads((destination / "train/ann/one.jpg.json").read_text())
    obj = annotation["objects"][0]
    assert plan["has_ignore_regions"]
    assert obj["classTitle"] == "grouped_vehicles"
    assert obj["supervisionState"] == "ignore"
    assert obj["ignoreRegion"] is True
    assert obj["attributes"] == {
        "occlusion": "50-74%",
        "depiction": True,
    }


def test_woodscape_ego_requires_plausible_boundary_geometry(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "ego vehicle", "polygon")
    annotation_path = source / "train/ann/one.jpg.json"
    annotation = json.loads(annotation_path.read_text())
    annotation["objects"][0]["points"]["exterior"] = [
        [0, 6],
        [19, 6],
        [19, 9],
        [0, 9],
    ]
    annotation_path.write_text(json.dumps(annotation))
    dataset = DatasetConfig(
        "woodscape_rgb_fisheye", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"

    filter_project(dataset, source, destination, taxonomy)

    output = json.loads((destination / "train/ann/one.jpg.json").read_text())
    assert output["objects"][0]["supervisionState"] == "trusted_background"


def test_detached_woodscape_ego_is_quarantined_as_ignore(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "ego vehicle", "polygon")
    annotation_path = source / "train/ann/one.jpg.json"
    annotation = json.loads(annotation_path.read_text())
    annotation["objects"][0]["points"]["exterior"] = [
        [6, 4],
        [13, 4],
        [13, 5],
        [6, 5],
    ]
    annotation_path.write_text(json.dumps(annotation))
    dataset = DatasetConfig(
        "woodscape_rgb_fisheye", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"

    filter_project(dataset, source, destination, taxonomy)

    output = json.loads((destination / "train/ann/one.jpg.json").read_text())
    obj = output["objects"][0]
    assert obj["supervisionState"] == "ignore"
    assert obj["exclusionReason"] == "ego_integrity:detached_from_image_boundary"


def test_parallel_filter_matches_sequential_output(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "car")
    image = (source / "train/img/one.jpg").read_bytes()
    annotation = (source / "train/ann/one.jpg.json").read_bytes()
    for index in range(100):
        name = f"copy-{index:03d}.jpg"
        (source / "train/img" / name).write_bytes(image)
        (source / "train/ann" / f"{name}.json").write_bytes(annotation)
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    sequential = tmp_path / "sequential"
    parallel = tmp_path / "parallel"
    _, sequential_actions = filter_project(dataset, source, sequential, taxonomy)
    _, parallel_actions = filter_project(dataset, source, parallel, taxonomy, workers=2)
    assert sequential_actions == parallel_actions == {"new_symlink": 101}
    for path in (sequential / "train/ann").iterdir():
        assert path.read_bytes() == (parallel / "train/ann" / path.name).read_bytes()


def test_filter_marker_rejects_a_different_image_limit(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "car")
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"
    filter_project(dataset, source, destination, taxonomy, limit_images=1)
    with pytest.raises(RuntimeError, match="incompatible"):
        filter_project(dataset, source, destination, taxonomy)


def test_bounded_filter_prefers_retained_images(tmp_path, taxonomy):
    source = make_project(tmp_path / "source", "car")
    image = (source / "train/img/one.jpg").read_bytes()
    (source / "train/img/aaa.jpg").write_bytes(image)
    (source / "train/ann/aaa.jpg.json").write_text(
        json.dumps({"size": {"width": 32, "height": 24}, "objects": []})
    )
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"
    plan, _ = filter_project(dataset, source, destination, taxonomy, limit_images=1)
    assert plan["selected_positive_image_count"] == 1
    assert (destination / "train/ann/one.jpg.json").exists()
    assert not (destination / "train/ann/aaa.jpg.json").exists()


def test_filter_failure_removes_partial_output(tmp_path, taxonomy, monkeypatch):
    source = make_project(tmp_path / "source", "car")
    dataset = DatasetConfig(
        "bdd100k_images_100k", tmp_path / "unused.tar", tmp_path / "raw"
    )
    destination = tmp_path / "filtered"

    def fail(_):
        raise RuntimeError("simulated worker failure")

    monkeypatch.setattr(filtering_module, "_filter_batch", fail)
    with pytest.raises(RuntimeError, match="simulated"):
        filter_project(dataset, source, destination, taxonomy)
    assert not destination.exists()
