from __future__ import annotations

import json

import torch
from PIL import Image

from student_detector.config import AugmentationConfig, DataConfig
from student_detector.data import (
    DomainMixtureBatchSampler,
    ImageRecord,
    IndexedCocoProposalDataset,
    select_source_mixture_indices,
)


def _dataset(tmp_path):
    image_root = tmp_path / "data"
    image_root.mkdir()
    images = []
    annotations = []
    sources = (
        "coco_2017",
        "nuimages",
        "bdd100k_images_100k",
        "woodscape_rgb_fisheye",
    )
    for index, source in enumerate(sources, 1):
        filename = f"{index}.jpg"
        Image.new("RGB", (80, 40), "white").save(image_root / filename)
        images.append({
            "id": index,
            "file_name": filename,
            "width": 80,
            "height": 40,
            "source_dataset": source,
            "camera_type": (
                "fisheye" if source == "woodscape_rgb_fisheye" else "perspective"
            ),
        })
        annotations.append({
            "id": index,
            "image_id": index,
            "category_id": 1,
            "bbox": [20, 10, 30, 20],
            "area": 600,
            "iscrowd": 0,
        })
    annotations.append({
        "id": 99,
        "image_id": 1,
        "category_id": 999,
        "bbox": [0, 0, 5, 5],
        "area": 25,
        "iscrowd": 1,
        "ignore_region": True,
    })
    annotation_path = tmp_path / "instances.json"
    annotation_path.write_text(json.dumps({
        "images": images,
        "annotations": annotations,
        "categories": [{"id": 1, "name": "car"}],
    }))
    config = DataConfig(
        annotation_path,
        annotation_path,
        image_root,
        tmp_path / "index",
        input_size=64,
        batch_size=10,
        tiny_area=10,
        tiny_min_side=2,
    )
    dataset = IndexedCocoProposalDataset(
        annotation_path,
        image_root,
        tmp_path / "index.sqlite",
        config,
        AugmentationConfig(
            horizontal_flip_probability=0,
            color_jitter_probability=0,
            blur_probability=0,
            noise_probability=0,
            jpeg_probability=0,
        ),
        training=False,
    )
    return dataset


def test_streaming_index_and_letterbox_preserve_supervision(tmp_path):
    dataset = _dataset(tmp_path)
    sample = dataset[0]
    assert sample.image.shape == (3, 64, 64)
    assert sample.boxes.shape == (1, 4)
    assert sample.ignore_boxes.shape == (1, 4)
    assert sample.valid_mask.sum() == 64 * 32
    # Image-level COCO completeness is not spatial trusted-background proof.
    assert not sample.background_supervision
    assert not dataset[1].background_supervision


def test_fixed_domain_batch_composition(tmp_path):
    dataset = _dataset(tmp_path)
    sampler = DomainMixtureBatchSampler(
        dataset,
        10,
        domain_weights=dataset.data_config.domain_weights,
        source_weights=dataset.data_config.source_weights,
        empty_fraction=0,
        seed=4,
        batches_per_epoch=1,
    )
    batch = next(iter(sampler))
    domains = [
        dataset.data_config.source_domains[dataset.records[index].source_dataset]
        for index in batch
    ]
    assert domains.count("general") == 5
    assert domains.count("automotive") == 3
    assert domains.count("fisheye") == 2


def test_only_components_contained_in_parent_become_ignore(tmp_path):
    dataset = _dataset(tmp_path)
    rows = [
        (10.0, 10.0, 50.0, 60.0, 0, "person"),
        (20.0, 40.0, 30.0, 50.0, 0, "shoe"),
        (60.0, 40.0, 70.0, 50.0, 0, "shoe"),
    ]
    assert dataset._contained_component_indices(rows) == {1}


def test_finite_subset_enforces_source_mixture() -> None:
    sources = ("coco", "nuimages", "bdd", "woodscape")
    records = [
        ImageRecord(
            index, index, f"{index}.jpg", 10, 10, source,
            "perspective", False, 1,
        )
        for source in sources
        for index in range(100 * sources.index(source), 100 * (
            sources.index(source) + 1
        ))
    ]
    weights = {
        "coco": 0.50,
        "nuimages": 0.18,
        "bdd": 0.12,
        "woodscape": 0.20,
    }
    indices = select_source_mixture_indices(
        records, weights, 50, positive_only=True
    )
    selected_sources = [records[index].source_dataset for index in indices]
    assert selected_sources.count("coco") == 25
    assert selected_sources.count("nuimages") == 9
    assert selected_sources.count("bdd") == 6
    assert selected_sources.count("woodscape") == 10
