from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import torch
from PIL import Image

from student_detector.assigner import ATSSAssigner
from student_detector.config import AugmentationConfig
from student_detector.data import (
    HardNegativeFocusBatchSampler,
    ImageRecord,
    ProposalSample,
    ProposalTransform,
)
from student_detector.targets import TargetBuilder


def _sample() -> ProposalSample:
    return ProposalSample(
        image=torch.zeros((3, 64, 64)),
        boxes=torch.tensor([[24.0, 24.0, 40.0, 40.0]]),
        ignore_boxes=torch.empty((0, 4)),
        valid_mask=torch.ones((64, 64), dtype=torch.bool),
        image_id=1,
        source_dataset="nuimages",
        domain="automotive",
        camera_type="perspective",
        background_supervision=True,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
        trusted_background_boxes=torch.tensor([[0.0, 0.0, 64.0, 64.0]]),
        hard_negative_focus_boxes=torch.tensor([[0.0, 0.0, 24.0, 64.0]]),
    )


def test_focus_transform_tracks_the_same_affine_without_changing_states() -> None:
    transform = ProposalTransform(
        64,
        AugmentationConfig(
            horizontal_flip_probability=1.0,
            scale_min=1.0,
            scale_max=1.0,
            translation_fraction=0.0,
        ),
        training=True,
        tiny_area=1,
        tiny_min_side=1,
    )
    result = transform(
        Image.new("RGB", (64, 64)),
        torch.tensor([[24.0, 24.0, 40.0, 40.0]]),
        torch.empty((0, 4)),
        torch.tensor([[0.0, 0.0, 64.0, 64.0]]),
        torch.tensor([[0.0, 0.0, 24.0, 64.0]]),
        seed=4,
    )

    torch.testing.assert_close(result[6], torch.tensor([[40.0, 0.0, 64.0, 64.0]]))
    assert result[1].shape == (1, 4)
    assert result[2].numel() == 0
    assert result[3].shape == (1, 4)


def test_focus_weight_only_strengthens_existing_trusted_background() -> None:
    sample = _sample()
    builder = TargetBuilder(ATSSAssigner(), hard_negative_focus_weight=2.0)
    targets = builder([sample], ((8, 8), (4, 4), (2, 2)), device=torch.device("cpu"))
    focus = targets.hard_negative_focus_mask
    trusted_nonfocus = targets.trusted_background_mask & ~focus

    assert focus.any()
    assert trusted_nonfocus.any()
    assert torch.all(focus <= targets.trusted_background_mask)
    torch.testing.assert_close(
        targets.objectness_weights[focus],
        torch.full_like(targets.objectness_weights[focus], 2.0),
    )
    torch.testing.assert_close(
        targets.objectness_weights[trusted_nonfocus],
        torch.ones_like(targets.objectness_weights[trusted_nonfocus]),
    )
    torch.testing.assert_close(
        targets.objectness_weights[targets.positive_mask],
        torch.ones_like(targets.objectness_weights[targets.positive_mask]),
    )


def test_focus_weight_one_is_numerically_identical_to_baseline() -> None:
    sample = _sample()
    shapes = ((8, 8), (4, 4), (2, 2))
    baseline = TargetBuilder(ATSSAssigner())(
        [replace(sample, hard_negative_focus_boxes=None)],
        shapes,
        device=torch.device("cpu"),
    )
    focused = TargetBuilder(ATSSAssigner(), hard_negative_focus_weight=1.0)(
        [sample], shapes, device=torch.device("cpu")
    )

    torch.testing.assert_close(
        focused.objectness_weights, baseline.objectness_weights, rtol=0, atol=0
    )
    assert focused.hard_negative_focus_mask.any()


def test_focus_sampler_preserves_source_and_nonempty_quotas_and_resume() -> None:
    records = []
    for source, count in (("bdd", 13), ("wood", 25)):
        for _ in range(count):
            index = len(records)
            records.append(
                ImageRecord(
                    index,
                    index + 100,
                    f"{index}.jpg",
                    64,
                    64,
                    source,
                    "camera",
                    True,
                    2,
                )
            )
    focus_indices = {12, 37}
    dataset = SimpleNamespace(
        records=records,
        data_config=SimpleNamespace(
            source_domains={"bdd": "automotive", "wood": "fisheye"}
        ),
        hard_negative_focus={
            (records[index].source_dataset, records[index].image_id): object()
            for index in focus_indices
        },
    )
    sampler = HardNegativeFocusBatchSampler(
        dataset,
        4,
        domain_weights={"automotive": 0.25, "fisheye": 0.75},
        source_weights={"bdd": 0.25, "wood": 0.75},
        empty_fraction=0.0,
        seed=42,
        accumulation_steps=2,
        focus_source_weights={"bdd": 0.25, "wood": 0.75},
        batches_per_epoch=8,
    )
    batches = list(sampler)

    assert all(
        {
            source: sum(records[index].source_dataset == source for index in batch)
            for source in ("bdd", "wood")
        }
        == sampler.batch_source_quotas(batch_index)
        for batch_index, batch in enumerate(batches)
    )
    window_sources = []
    for start in range(0, len(batches), 2):
        focused = [
            index
            for batch in batches[start : start + 2]
            for index in batch
            if index in focus_indices
        ]
        assert len(focused) == 1
        window_sources.append(records[focused[0]].source_dataset)
    assert window_sources.count("bdd") == 1
    assert window_sources.count("wood") == 3

    sampler.set_start_batch(3)
    assert list(sampler) == batches[3:]
