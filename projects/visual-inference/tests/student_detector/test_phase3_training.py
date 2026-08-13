from __future__ import annotations

from dataclasses import replace

import torch
from torch.utils.data import DataLoader, Dataset

from student_detector.assigner import ATSSAssigner
from student_detector.backbone import StubBackbone
from student_detector.config import DataConfig, Phase3Config, ScheduleConfig
from student_detector.data import ProposalSample, collate_proposal_samples
from student_detector.losses import (
    ProposalLoss,
    aligned_ciou_loss,
    aligned_giou_loss,
    quality_focal_loss,
)
from student_detector.model import StudentDetector
from student_detector.targets import TargetBuilder
from student_detector.training import train_phase3
from student_detector.training_optimization import ExponentialMovingAverage


def _sample(*, dense: bool = True, ignore: bool = False) -> ProposalSample:
    return ProposalSample(
        image=torch.randn(3, 64, 64),
        boxes=torch.tensor([[12.0, 12.0, 48.0, 52.0]]),
        ignore_boxes=(
            torch.tensor([[0.0, 0.0, 10.0, 10.0]]) if ignore else torch.empty((0, 4))
        ),
        valid_mask=torch.ones(64, 64, dtype=torch.bool),
        image_id=1,
        source_dataset="coco_2017" if dense else "nuimages",
        domain="general" if dense else "automotive",
        camera_type="perspective",
        background_supervision=dense,
        original_size=(64, 64),
        transform=(1.0, 0.0, 0.0),
    )


def test_ciou_is_zero_for_identical_boxes():
    boxes = torch.tensor([[1.0, 2.0, 9.0, 12.0]])
    torch.testing.assert_close(aligned_ciou_loss(boxes, boxes), torch.zeros(1))


def test_ema_ramp_tracks_early_update_then_approaches_decay_cap() -> None:
    model = torch.nn.Linear(1, 1, bias=False)
    model.weight.data.fill_(0.0)
    ema = ExponentialMovingAverage(model, decay=0.99, ramp_steps=10)
    model.weight.data.fill_(1.0)
    ema.update(model)
    assert ema.current_decay() < 0.1
    assert float(ema.module.weight) > 0.9
    for _ in range(100):
        ema.update(model)
    assert 0.98 < ema.current_decay() < 0.99
    assert ema.initialization_weight < 1e-6


def test_giou_is_zero_for_identical_boxes():
    boxes = torch.tensor([[1.0, 2.0, 9.0, 12.0]])
    torch.testing.assert_close(aligned_giou_loss(boxes, boxes), torch.zeros(1))


def test_quality_focal_loss_accepts_continuous_iou_target():
    target = torch.tensor([0.75])
    matching_logit = torch.logit(target)
    torch.testing.assert_close(
        quality_focal_loss(matching_logit, target),
        torch.zeros(1),
        atol=1e-7,
        rtol=0,
    )


def test_sparse_dataset_masks_unannotated_background():
    assigner = ATSSAssigner()
    targets = TargetBuilder(assigner)(
        [_sample(dense=False)],
        ((8, 8), (4, 4), (2, 2)),
        device=torch.device("cpu"),
    )
    assert torch.equal(targets.objectness_mask, targets.positive_mask)
    assert targets.positive_mask.any()


def test_weak_background_supervision_preserves_positive_and_ignore_weights():
    sample = _sample(dense=False, ignore=True)
    sample = ProposalSample(
        sample.image,
        sample.boxes,
        sample.ignore_boxes,
        sample.valid_mask,
        sample.image_id,
        "woodscape_rgb_fisheye",
        "fisheye",
        "fisheye",
        False,
        sample.original_size,
        sample.transform,
    )
    targets = TargetBuilder(
        ATSSAssigner(),
        background_loss_weights={"woodscape_rgb_fisheye": 0.05},
    )(
        [sample],
        ((8, 8), (4, 4), (2, 2)),
        device=torch.device("cpu"),
    )
    positive = targets.positive_mask
    torch.testing.assert_close(
        targets.objectness_weights[positive],
        torch.ones_like(targets.objectness_weights[positive]),
    )
    weak_negative = (
        targets.objectness_mask & ~positive & (targets.objectness_weights > 0)
    )
    assert weak_negative.any()
    torch.testing.assert_close(
        targets.objectness_weights[weak_negative],
        torch.full_like(targets.objectness_weights[weak_negative], 0.05),
    )
    assert (targets.objectness_weights == 0).any()


def test_padding_and_ignore_are_masked_but_positive_overrides():
    sample = _sample(ignore=True)
    valid_mask = sample.valid_mask.clone()
    valid_mask[:, 56:] = False
    sample = ProposalSample(
        sample.image,
        sample.boxes,
        sample.ignore_boxes,
        valid_mask,
        sample.image_id,
        sample.source_dataset,
        sample.domain,
        sample.camera_type,
        sample.background_supervision,
        sample.original_size,
        sample.transform,
    )
    targets = TargetBuilder(ATSSAssigner())(
        [sample],
        ((8, 8), (4, 4), (2, 2)),
        device=torch.device("cpu"),
    )
    assert targets.positive_mask.any()
    assert torch.all(targets.objectness_mask[targets.positive_mask])
    assert not targets.valid_point_masks[0][0, :, 7].any()


def test_synthetic_forward_loss_backward_has_finite_gradients():
    model = StudentDetector(backbone=StubBackbone()).train()
    samples = [_sample(), _sample(dense=False)]
    images = torch.stack([sample.image for sample in samples])
    output = model(images)
    shapes = tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness)
    targets = TargetBuilder(ATSSAssigner())(samples, shapes, device=torch.device("cpu"))
    losses = ProposalLoss()(output, targets)
    losses.total.backward()
    assert torch.isfinite(losses.total)
    for module in (model.backbone, model.fpn, model.head):
        gradients = [
            parameter.grad
            for parameter in module.parameters()
            if parameter.requires_grad and parameter.grad is not None
        ]
        assert gradients
        assert all(torch.isfinite(gradient).all() for gradient in gradients)


def test_qfl_mode_has_finite_gradients_and_dormant_centerness():
    model = StudentDetector(backbone=StubBackbone()).train()
    samples = [_sample(), _sample(dense=False)]
    images = torch.stack([sample.image for sample in samples])
    output = model(images)
    shapes = tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness)
    targets = TargetBuilder(ATSSAssigner())(samples, shapes, device=torch.device("cpu"))
    losses = ProposalLoss(
        objectness_loss="quality_focal",
        ltrb_weight=0.5,
        centerness_weight=0.0,
        box_weighting="uniform",
    )(output, targets)
    losses.total.backward()
    assert torch.isfinite(losses.total)
    assert losses.box_ltrb.item() >= 0
    assert losses.centerness.item() == 0
    assert model.head.objectness.weight.grad is not None
    assert model.head.box_regression.weight.grad is not None
    assert model.head.centerness.weight.grad is None


def test_empty_positive_batch_losses_are_finite():
    model = StudentDetector(backbone=StubBackbone()).eval()
    sample = _sample()
    sample = ProposalSample(
        sample.image,
        torch.empty((0, 4)),
        torch.empty((0, 4)),
        sample.valid_mask,
        sample.image_id,
        sample.source_dataset,
        sample.domain,
        sample.camera_type,
        sample.background_supervision,
        sample.original_size,
        sample.transform,
    )
    output = model(sample.image.unsqueeze(0))
    shapes = tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness)
    targets = TargetBuilder(ATSSAssigner())(
        [sample], shapes, device=torch.device("cpu")
    )
    losses = ProposalLoss(ltrb_weight=0.5)(output, targets)
    assert torch.isfinite(losses.total)
    assert losses.box_ciou.item() == 0
    assert losses.box_ltrb.item() == 0
    assert losses.centerness.item() == 0


def test_disabled_dense_branch_is_finite_in_float16():
    model = StudentDetector(backbone=StubBackbone()).eval()
    sample = _sample()
    output = model(sample.image.unsqueeze(0))
    output = type(output)(
        objectness=tuple(
            torch.full_like(tensor, -20.0).half() for tensor in output.objectness
        ),
        box_distances=tuple(tensor.half() for tensor in output.box_distances),
        centerness=tuple(tensor.half() for tensor in output.centerness),
    )
    shapes = tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness)
    targets = TargetBuilder(ATSSAssigner())(
        [sample], shapes, device=torch.device("cpu")
    )
    losses = ProposalLoss(
        objectness_loss="quality_focal",
        ltrb_weight=0.5,
        centerness_weight=0.0,
    )(output, targets)
    assert torch.isfinite(losses.total)
    assert losses.centerness.item() == 0


def test_ltrb_auxiliary_loss_is_stride_normalized_and_component_averaged():
    model = StudentDetector(backbone=StubBackbone()).eval()
    sample = _sample()
    output = model(sample.image.unsqueeze(0))
    shapes = tuple((tensor.shape[-2], tensor.shape[-1]) for tensor in output.objectness)
    targets = TargetBuilder(ATSSAssigner())(
        [sample], shapes, device=torch.device("cpu")
    )
    positive = targets.positive_mask
    assert positive.any()

    # A one-stride error on every LTRB coordinate has normalized error 1.
    # SmoothL1(1) is 0.5, and averaging over coordinates and positives must
    # therefore also produce exactly 0.5.
    level_strides = torch.cat(
        [
            torch.full((height * width,), float(stride))
            for (height, width), stride in zip(shapes, (8, 16, 32), strict=True)
        ]
    )
    predicted = torch.cat(
        [
            tensor.permute(0, 2, 3, 1).reshape(1, -1, 4)
            for tensor in output.box_distances
        ],
        dim=1,
    )
    stride_offsets = level_strides.unsqueeze(0).expand_as(positive)
    predicted[positive] = targets.box_distances[positive] + stride_offsets[
        positive
    ].unsqueeze(1)
    offset = 0
    distance_levels = []
    for height, width in shapes:
        count = height * width
        distance_levels.append(
            predicted[:, offset : offset + count]
            .reshape(1, height, width, 4)
            .permute(0, 3, 1, 2)
        )
        offset += count
    modified_output = type(output)(
        objectness=output.objectness,
        box_distances=tuple(distance_levels),
        centerness=output.centerness,
    )
    losses = ProposalLoss(
        objectness_weight=0,
        box_weight=0,
        ltrb_weight=1,
        centerness_weight=0,
    )(modified_output, targets)
    torch.testing.assert_close(losses.box_ltrb, torch.tensor(0.5), atol=1e-6, rtol=0)
    torch.testing.assert_close(losses.total, losses.box_ltrb)


def test_one_step_trainer_writes_resumable_checkpoints(tmp_path):
    class Samples(Dataset):
        def __init__(self):
            self.values = [_sample(), _sample(dense=False)]
            self.epoch = 0

        def __len__(self):
            return len(self.values)

        def __getitem__(self, index):
            return self.values[index]

        def set_epoch(self, epoch):
            self.epoch = epoch

    dataset = Samples()
    loader = DataLoader(
        dataset,
        batch_size=2,
        collate_fn=collate_proposal_samples,
    )
    placeholder = tmp_path / "unused.json"
    config = Phase3Config(
        data=DataConfig(
            placeholder,
            placeholder,
            tmp_path,
            tmp_path,
            input_size=64,
            batch_size=2,
            workers=0,
        ),
        schedule=ScheduleConfig(
            epochs=1,
            freeze_backbone_epochs=1,
            warmup_steps=1,
            amp=False,
        ),
        output_dir=tmp_path / "run",
        pretrained_backbone=False,
    )
    result = train_phase3(
        StudentDetector(backbone=StubBackbone()),
        loader,
        loader,
        TargetBuilder(ATSSAssigner()),
        ProposalLoss(),
        config,
        torch.device("cpu"),
        max_steps=1,
        max_val_batches=1,
    )
    assert result["global_step"] == 1
    for name in ("last.pt", "best.pt"):
        checkpoint = torch.load(
            config.output_dir / name, map_location="cpu", weights_only=False
        )
        assert checkpoint["phase"] == 3
        assert checkpoint["selected_state"] == (
            "ema_model" if name == "best.pt" else "model"
        )
        if name == "last.pt":
            assert "model" in checkpoint
            assert "ema_model" in checkpoint
            assert "optimizer" in checkpoint
            assert "rng_state" in checkpoint
        else:
            assert checkpoint["checkpoint_kind"] == "weights_only"
            assert "ema_model" in checkpoint
            assert "optimizer" not in checkpoint
    resumed_config = replace(config, schedule=replace(config.schedule, epochs=2))
    resumed = train_phase3(
        StudentDetector(backbone=StubBackbone()),
        loader,
        loader,
        TargetBuilder(ATSSAssigner()),
        ProposalLoss(),
        resumed_config,
        torch.device("cpu"),
        max_steps=2,
        max_val_batches=1,
        resume=config.output_dir / "last.pt",
    )
    assert resumed["global_step"] == 2


def test_mid_epoch_checkpoint_resumes_the_same_epoch(tmp_path):
    class Samples(Dataset):
        def __init__(self):
            self.values = [_sample() for _ in range(4)]
            self.epoch = 0

        def __len__(self):
            return len(self.values)

        def __getitem__(self, index):
            return self.values[index]

        def set_epoch(self, epoch):
            self.epoch = epoch

    loader = DataLoader(
        Samples(),
        batch_size=1,
        collate_fn=collate_proposal_samples,
    )
    placeholder = tmp_path / "unused.json"
    config = Phase3Config(
        data=DataConfig(
            placeholder,
            placeholder,
            tmp_path,
            tmp_path,
            input_size=64,
            batch_size=1,
            workers=0,
        ),
        schedule=ScheduleConfig(
            epochs=1,
            freeze_backbone_epochs=1,
            warmup_steps=1,
            amp=False,
            checkpoint_every_steps=1,
        ),
        output_dir=tmp_path / "run",
        pretrained_backbone=False,
    )
    common = (
        loader,
        loader,
        TargetBuilder(ATSSAssigner()),
        ProposalLoss(),
        config,
        torch.device("cpu"),
    )
    first = train_phase3(
        StudentDetector(backbone=StubBackbone()),
        *common,
        max_steps=1,
        max_val_batches=1,
    )
    checkpoint = torch.load(
        config.output_dir / "last.pt",
        map_location="cpu",
        weights_only=False,
    )
    assert first["global_step"] == 1
    assert checkpoint["epoch"] == 0
    assert checkpoint["batch_in_epoch"] == 1
    assert checkpoint["epoch_complete"] is False

    resumed = train_phase3(
        StudentDetector(backbone=StubBackbone()),
        *common,
        max_steps=2,
        max_val_batches=1,
        resume=config.output_dir / "last.pt",
    )
    checkpoint = torch.load(
        config.output_dir / "last.pt",
        map_location="cpu",
        weights_only=False,
    )
    assert resumed["global_step"] == 2
    assert checkpoint["epoch"] == 0
    assert checkpoint["batch_in_epoch"] == 2
    assert checkpoint["epoch_complete"] is False
