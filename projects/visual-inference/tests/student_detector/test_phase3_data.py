from __future__ import annotations

import json
import pickle
import sqlite3
from collections import Counter
from pathlib import Path

from PIL import Image
import pytest
import torch
from torch.utils.data import DataLoader

from student_detector.config import AugmentationConfig, DataConfig
from student_detector.data import (
    DomainMixtureBatchSampler,
    ImageRecord,
    IndexedCocoProposalDataset,
    collate_proposal_samples,
    use_file_system_tensor_sharing,
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
        images.append(
            {
                "id": index,
                "file_name": filename,
                "width": 80,
                "height": 40,
                "source_dataset": source,
                "camera_type": (
                    "fisheye" if source == "woodscape_rgb_fisheye" else "perspective"
                ),
            }
        )
        annotations.append(
            {
                "id": index,
                "image_id": index,
                "category_id": 1,
                "bbox": [20, 10, 30, 20],
                "area": 600,
                "iscrowd": 0,
            }
        )
    annotations.append(
        {
            "id": 99,
            "image_id": 1,
            "category_id": 999,
            "bbox": [0, 0, 5, 5],
            "area": 25,
            "iscrowd": 1,
            "ignore_region": True,
        }
    )
    annotation_path = tmp_path / "instances.json"
    annotation_path.write_text(
        json.dumps(
            {
                "images": images,
                "annotations": annotations,
                "categories": [{"id": 1, "name": "car"}],
            }
        )
    )
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


def test_hbb_annotations_are_lazy_readonly_and_pickle_safe(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path)
    assert dataset._connection is None
    assert not hasattr(dataset, "_annotations_by_image")
    assert dataset.state_counts == {"positive": 4, "ignore": 1, "trusted_background": 0}
    sample = dataset[0]
    assert dataset._connection is not None
    with pytest.raises(sqlite3.OperationalError, match="readonly"):
        dataset._connection.execute("DELETE FROM annotations")
    restored = pickle.loads(pickle.dumps(dataset))
    assert restored._connection is None
    assert torch.equal(restored[0].boxes, sample.boxes)
    assert torch.equal(restored[0].ignore_boxes, sample.ignore_boxes)


def test_hbb_persistent_spawn_worker_tracks_epoch(tmp_path: Path) -> None:
    dataset = _dataset(tmp_path)
    dataset.transform.training = True
    # Open in the parent first: the live connection must not reach the worker.
    dataset[0]
    loader = DataLoader(
        dataset,
        batch_size=4,
        num_workers=1,
        persistent_workers=True,
        multiprocessing_context="spawn",
        collate_fn=collate_proposal_samples,
        worker_init_fn=use_file_system_tensor_sharing,
        timeout=30,
    )
    images0, _ = next(iter(loader))
    dataset.set_epoch(1)
    images1, samples1 = next(iter(loader))
    assert not torch.equal(images0, images1)
    for index, sample in enumerate(samples1):
        expected = dataset[index]
        assert torch.equal(sample.image, expected.image)
        assert torch.equal(sample.boxes, expected.boxes)
    del loader


def test_hbb_requires_prebuilt_index_in_cloud(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("REQUIRE_PREBUILT_INDEX", "1")
    with pytest.raises(FileNotFoundError, match="missing prebuilt index"):
        _dataset(tmp_path)


def test_hbb_accepts_schema6_proposal_index_without_rebuild(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original = _dataset(tmp_path)
    with sqlite3.connect(original.index_path) as connection:
        connection.execute(
            "UPDATE metadata SET value='6:proposal-manifest' WHERE key='schema_version'"
        )
    monkeypatch.setenv("REQUIRE_PREBUILT_INDEX", "1")

    restored = IndexedCocoProposalDataset(
        original.annotations,
        original.image_root,
        original.index_path,
        original.data_config,
        AugmentationConfig(
            horizontal_flip_probability=0,
            color_jitter_probability=0,
            blur_probability=0,
            noise_probability=0,
            jpeg_probability=0,
        ),
        training=False,
    )

    assert len(restored) == len(original)
    assert restored[0].boxes.shape == (1, 4)


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


@pytest.mark.parametrize(
    ("batch_size", "batches_per_epoch", "accumulation_steps"),
    [(8, 25, 8), (128, 25, 1)],
)
def test_residual_source_quotas_match_local_and_cloud_mixtures(
    tmp_path, batch_size, batches_per_epoch, accumulation_steps
):
    dataset = _dataset(tmp_path)
    sampler = DomainMixtureBatchSampler(
        dataset,
        batch_size,
        domain_weights=dataset.data_config.domain_weights,
        source_weights=dataset.data_config.source_weights,
        empty_fraction=0,
        seed=4,
        batches_per_epoch=batches_per_epoch,
    )

    batches = list(sampler)
    assert batches == list(sampler)
    observed = Counter(
        dataset.records[index].source_dataset for batch in batches for index in batch
    )
    total = batch_size * batches_per_epoch
    assert observed == Counter(
        {
            source: round(weight * total)
            for source, weight in dataset.data_config.source_weights.items()
        }
    )

    for start in range(0, batches_per_epoch, accumulation_steps):
        window = batches[start : start + accumulation_steps]
        window_observed = Counter(
            dataset.records[index].source_dataset for batch in window for index in batch
        )
        window_quotas = Counter()
        for batch_index in range(start, start + len(window)):
            window_quotas.update(sampler.batch_source_quotas(batch_index))
        assert window_observed == window_quotas


def test_finite_subset_enforces_source_mixture() -> None:
    sources = ("coco", "nuimages", "bdd", "woodscape")
    records = [
        ImageRecord(
            index,
            index,
            f"{index}.jpg",
            10,
            10,
            source,
            "perspective",
            False,
            1,
        )
        for source in sources
        for index in range(
            100 * sources.index(source), 100 * (sources.index(source) + 1)
        )
    ]
    weights = {
        "coco": 0.50,
        "nuimages": 0.18,
        "bdd": 0.12,
        "woodscape": 0.20,
    }
    indices = select_source_mixture_indices(records, weights, 50, positive_only=True)
    selected_sources = [records[index].source_dataset for index in indices]
    assert selected_sources.count("coco") == 25
    assert selected_sources.count("nuimages") == 9
    assert selected_sources.count("bdd") == 6
    assert selected_sources.count("woodscape") == 10
