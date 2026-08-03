from __future__ import annotations

from dataclasses import replace

import torch
from torch.utils.data import DataLoader, Dataset

from student_detector.backbone import StubBackbone
from student_detector.config import DataConfig, Phase3Config, ScheduleConfig
from student_detector.model import QuadProposalDetector
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_data import QuadProposalSample, collate_quad_proposal_samples
from student_detector.quad_geometry import quad_from_bbox
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import train_quad_proposals


class _Samples(Dataset[QuadProposalSample]):
    def __init__(self) -> None:
        self.sample = QuadProposalSample(
            image=torch.zeros(3, 64, 64),
            quads=quad_from_bbox([8.0, 8.0, 48.0, 48.0]).unsqueeze(0),
            ignore_quads=torch.empty((0, 4, 2)),
            valid_mask=torch.ones(64, 64, dtype=torch.bool),
            image_id=1,
            source_dataset="test",
            domain="general",
            camera_type="perspective",
            background_supervision=True,
            original_size=(64, 64),
            transform=(1.0, 0.0, 0.0),
            geometry_tiers=("source_hbb",),
            object_conditions=("whole_object",),
        )

    def __len__(self) -> int:
        return 1

    def __getitem__(self, index: int) -> QuadProposalSample:
        return self.sample


def test_quad_checkpoint_resumes_and_round_trips_config(tmp_path) -> None:
    loader = DataLoader(
        _Samples(), batch_size=1, collate_fn=collate_quad_proposal_samples
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
            quad_regular_min_side=8,
        ),
        schedule=ScheduleConfig(
            epochs=1,
            freeze_backbone_epochs=0,
            warmup_steps=1,
            amp=False,
        ),
        output_dir=tmp_path / "run",
        pretrained_backbone=False,
    )
    builder = QuadTargetBuilder(QuadAssigner())
    first = train_quad_proposals(
        QuadProposalDetector(backbone=StubBackbone()),
        loader,
        loader,
        builder,
        QuadProposalLoss(),
        config,
        torch.device("cpu"),
        max_steps=1,
        max_val_batches=1,
    )
    assert first["global_step"] == 1
    checkpoint_path = config.output_dir / "last.pt"
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    assert checkpoint["phase"] == "quad_proposals"
    assert checkpoint["format_version"] == 2
    assert checkpoint["config"]["data"]["input_size"] == 64
    assert "scaler" in checkpoint
    assert checkpoint["ema_updates"] == 1
    assert checkpoint["ema_initialization_weight"] < 1.0

    resumed_config = replace(
        config, schedule=replace(config.schedule, epochs=2)
    )
    resumed = train_quad_proposals(
        QuadProposalDetector(backbone=StubBackbone()),
        loader,
        loader,
        builder,
        QuadProposalLoss(),
        resumed_config,
        torch.device("cpu"),
        max_steps=2,
        max_val_batches=1,
        resume=checkpoint_path,
    )
    assert resumed["global_step"] == 2
