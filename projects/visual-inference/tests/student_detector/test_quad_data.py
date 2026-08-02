from __future__ import annotations

import json

import torch
from PIL import Image

from student_detector.config import AugmentationConfig, DataConfig
from student_detector.quad_data import QuadProposalDataset, QuadProposalTransform


def test_quad_dataset_reads_quad_manifest(tmp_path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    Image.new("RGB", (32, 24), "white").save(image_root / "one.jpg")
    annotations = tmp_path / "instances.json"
    annotations.write_text(json.dumps({
        "images": [{
            "id": 1, "file_name": "one.jpg", "width": 32, "height": 24,
            "source_dataset": "coco_2017", "camera_type": "perspective",
        }],
        "annotations": [{
            "id": 1, "image_id": 1, "category_id": 1,
            "bbox": [4, 3, 16, 12], "quad": [[4, 3], [20, 3], [19, 15], [5, 15]],
            "geometry_tier": "source_quad", "fit_coverage": 1.0,
            "iscrowd": 0,
        }],
        "categories": [{"id": 1, "name": "car"}],
    }))
    config = DataConfig(
        annotations, annotations, image_root, tmp_path / "index",
        input_size=32, quad_regular_min_side=8,
    )
    dataset = QuadProposalDataset(
        annotations, image_root, tmp_path / "index.sqlite", config,
        AugmentationConfig(horizontal_flip_probability=0, color_jitter_probability=0),
        training=False,
    )
    sample = dataset[0]
    assert sample.image.shape == (3, 32, 32)
    assert sample.quads.shape == (1, 4, 2)
    assert sample.geometry_tiers == ("source_quad",)
    assert torch.isfinite(sample.quads).all()


def test_quad_dataset_reads_compact_proposal_manifest(tmp_path) -> None:
    image_root = tmp_path / "images"
    image_root.mkdir()
    Image.new("RGB", (32, 24), "white").save(image_root / "one.jpg")
    annotations = tmp_path / "proposals.json"
    annotations.write_text(json.dumps({
        "schema_version": "quad-proposal-manifest.v1",
        "split": "train",
        "object_contract": {"positive": "visually distinct object"},
        "images": [{
            "image_id": 1, "file_name": "one.jpg", "width": 32, "height": 24,
            "source_dataset": "coco_2017", "camera_type": "perspective",
            "positive": [{
                "bbox": [4, 3, 16, 12],
                "quad": [[4, 3], [20, 3], [19, 15], [5, 15]],
                "geometry_tier": "source_quad", "fit_coverage": 1.0,
                "fit_tightness": 0.25, "valid": True,
            }],
            "ignore": [],
            "trusted_background": [{
                "bbox": [0, 16, 32, 8],
                "quad": [[0, 16], [32, 16], [32, 24], [0, 24]],
                "geometry_tier": "trusted_stuff_tile", "fit_coverage": 1.0,
                "fit_tightness": 1.0, "state": "trusted_background", "valid": True,
            }],
        }],
    }))
    config = DataConfig(
        annotations, annotations, image_root, tmp_path / "index",
        input_size=32, quad_regular_min_side=8,
    )
    dataset = QuadProposalDataset(
        annotations, image_root, tmp_path / "manifest.sqlite", config,
        AugmentationConfig(horizontal_flip_probability=0, color_jitter_probability=0),
        training=False,
    )
    sample = dataset[0]
    assert sample.quads.shape == (1, 4, 2)
    assert sample.geometry_tiers == ("source_quad",)
    assert sample.trusted_background_quads is not None
    assert sample.trusted_background_quads.shape == (1, 4, 2)


def test_thin_long_quad_uses_major_axis_size_tier() -> None:
    transform = QuadProposalTransform(
        input_size=384,
        augmentation=AugmentationConfig(horizontal_flip_probability=0, color_jitter_probability=0),
        training=False,
        regular_min_side=16,
        thin_major_axis_min=8,
        thin_aspect_ratio_min=3,
        thin_area=16,
    )
    image = Image.new("RGB", (1280, 966), "white")
    # After the 0.3 resize this is approximately 15 x 1.5 px: too thin for
    # the regular gate, but large enough for the explicit thin-object tier.
    quad = torch.tensor([[[100.0, 100.0], [150.0, 100.0], [150.0, 105.0], [100.0, 105.0]]])
    _, positives, ignores, _, _, _, _ = transform(
        image, quad, torch.empty((0, 4, 2)), seed=0
    )
    assert positives.shape == (1, 4, 2)
    assert ignores.shape == (0, 4, 2)


def test_transform_preserves_unclipped_trapezoid() -> None:
    transform = QuadProposalTransform(
        input_size=64,
        augmentation=AugmentationConfig(horizontal_flip_probability=0, color_jitter_probability=0),
        training=False,
        regular_min_side=16,
    )
    image = Image.new("RGB", (64, 64), "white")
    trapezoid = torch.tensor([[[8.0, 8.0], [52.0, 12.0], [45.0, 48.0], [14.0, 44.0]]])
    _, positives, _, _, _, _, retained = transform(
        image, trapezoid, torch.empty((0, 4, 2)), seed=0
    )
    assert retained == (0,)
    torch.testing.assert_close(positives[0], trapezoid[0])
