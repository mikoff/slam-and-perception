from __future__ import annotations

import copy
import json
from pathlib import Path

import torch
from PIL import Image

from student_detector.augmentation import (
    effective_augmentation_policy,
    sample_augmentation_parameters,
)
from student_detector.config import AugmentationConfig, DataConfig
from student_detector.data import ProposalTransform
from student_detector.quad_data import QuadProposalDataset, QuadProposalTransform
from student_detector.quad_geometry import canonicalize_quads


def _image() -> Image.Image:
    values = torch.arange(48 * 32 * 3, dtype=torch.int64).remainder(256).to(torch.uint8)
    return Image.fromarray(values.reshape(32, 48, 3).numpy(), mode="RGB")


def _corners(boxes: torch.Tensor) -> torch.Tensor:
    return canonicalize_quads(
        torch.stack(
            (
                boxes[:, [0, 1]],
                boxes[:, [2, 1]],
                boxes[:, [2, 3]],
                boxes[:, [0, 3]],
            ),
            dim=1,
        )
    )


def _full_augmentation() -> AugmentationConfig:
    return AugmentationConfig(
        horizontal_flip_probability=1,
        color_jitter_probability=1,
        blur_probability=1,
        noise_probability=1,
        jpeg_probability=1,
        scale_min=0.9,
        scale_max=1.1,
        translation_fraction=0.05,
        positive_visible_fraction=0.5,
        ignore_visible_fraction=0.2,
    )


def test_hbb_and_quad_share_pixels_affine_geometry_and_states() -> None:
    augmentation = _full_augmentation()
    boxes = torch.tensor([[8.0, 6.0, 30.0, 24.0]])
    ignores = torch.tensor([[1.0, 2.0, 7.0, 12.0]])
    trusted = torch.tensor([[31.0, 3.0, 46.0, 18.0]])
    hbb = ProposalTransform(
        64, augmentation, training=True, tiny_area=1, tiny_min_side=1
    )(_image(), boxes, ignores, trusted, seed=1234)
    quad = QuadProposalTransform(
        64,
        augmentation,
        training=True,
        regular_min_side=1,
        thin_major_axis_min=1,
        thin_aspect_ratio_min=100,
        thin_area=1,
    )(_image(), _corners(boxes), _corners(ignores), _corners(trusted), seed=1234)

    torch.testing.assert_close(hbb[0], quad[0], rtol=0, atol=0)
    assert torch.equal(hbb[4], quad[4])
    assert hbb[5] == quad[5]
    torch.testing.assert_close(_corners(hbb[1]), quad[1], rtol=0, atol=1e-5)
    torch.testing.assert_close(_corners(hbb[2]), quad[2], rtol=0, atol=1e-5)
    torch.testing.assert_close(_corners(hbb[3]), quad[3], rtol=0, atol=1e-5)


def test_validation_is_seed_independent_letterbox_only() -> None:
    augmentation = _full_augmentation()
    boxes = torch.tensor([[8.0, 6.0, 30.0, 24.0]])
    hbb = ProposalTransform(
        64, augmentation, training=False, tiny_area=1, tiny_min_side=1
    )
    first = hbb(_image(), boxes, boxes.new_empty((0, 4)), seed=1)
    second = hbb(_image(), boxes, boxes.new_empty((0, 4)), seed=999)

    torch.testing.assert_close(first[0], second[0], rtol=0, atol=0)
    assert torch.equal(first[4], second[4])
    assert first[5] == second[5]
    parameters = sample_augmentation_parameters(
        augmentation, 64, training=False, seed=999
    )
    assert parameters.selected_operations == ("letterbox",)


def _manifest_fixture(tmp_path: Path) -> tuple[Path, Path, DataConfig]:
    image_root = tmp_path / "images"
    image_root.mkdir()
    _image().save(image_root / "one.png")
    manifest = tmp_path / "proposals.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": "proposal-manifest.v2",
                "images": [
                    {
                        "image_id": 1,
                        "file_name": "one.png",
                        "width": 48,
                        "height": 32,
                        "source_dataset": "coco_2017",
                        "positive": [
                            {
                                "bbox": [8, 6, 22, 18],
                                "quad": [[8, 6], [30, 6], [30, 24], [8, 24]],
                                "valid": True,
                            }
                        ],
                        "ignore": [],
                        "trusted_background": [],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    config = DataConfig(
        manifest,
        manifest,
        image_root,
        tmp_path,
        input_size=64,
        batch_size=1,
        workers=1,
        quad_regular_min_side=1,
    )
    return image_root, manifest, config


def test_worker_copy_observes_shared_epoch_and_exact_replay(tmp_path: Path) -> None:
    image_root, manifest, config = _manifest_fixture(tmp_path)
    augmentation = _full_augmentation()
    dataset = QuadProposalDataset(
        manifest,
        image_root,
        tmp_path / "worker.sqlite",
        config,
        augmentation,
        training=True,
        seed=77,
    )
    worker_copy = copy.copy(dataset)

    def read_epoch(epoch: int) -> torch.Tensor:
        dataset.set_epoch(epoch)
        assert int(worker_copy._epoch.item()) == epoch
        return worker_copy[0].image

    epoch_zero = read_epoch(0)
    epoch_one = read_epoch(1)
    replay = read_epoch(0)

    assert not torch.equal(epoch_zero, epoch_one)
    torch.testing.assert_close(epoch_zero, replay, rtol=0, atol=0)


def test_effective_policy_declares_bounds_and_exclusions() -> None:
    policy = effective_augmentation_policy(_full_augmentation())
    assert policy["schema_version"] == "shared-augmentation.v1"
    assert policy["effective_bounds"]["jpeg_quality"] == (45, 95)
    assert "perspective" in policy["excluded_operations"]
