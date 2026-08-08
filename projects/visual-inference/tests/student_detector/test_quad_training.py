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
    def __init__(self, count: int = 1) -> None:
        self.count = count
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
        return self.count

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
    assert checkpoint["selected_state"] == "model"
    assert "rng_state" in checkpoint
    best_ema = torch.load(
        config.output_dir / "best_ema.pt", map_location="cpu", weights_only=False
    )
    assert best_ema["selected_state"] == "ema_model"

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


def test_accelerate_accumulation_counts_optimizer_updates_once(tmp_path) -> None:
    loader = DataLoader(
        _Samples(count=2), batch_size=1, collate_fn=collate_quad_proposal_samples
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
            accumulation_steps=2,
            amp=False,
        ),
        output_dir=tmp_path / "accumulated",
        pretrained_backbone=False,
    )
    result = train_quad_proposals(
        QuadProposalDetector(backbone=StubBackbone()),
        loader,
        loader,
        QuadTargetBuilder(QuadAssigner()),
        QuadProposalLoss(),
        config,
        torch.device("cpu"),
        max_steps=1,
        max_val_batches=1,
    )
    checkpoint = torch.load(
        config.output_dir / "last.pt", map_location="cpu", weights_only=False
    )
    assert result["global_step"] == 1
    assert checkpoint["global_step"] == 1
    assert checkpoint["scheduler"]["step_number"] == 1
    assert checkpoint["ema_updates"] == 1


def test_resumed_quad_run_matches_uninterrupted_weights(tmp_path) -> None:
    loader = DataLoader(
        _Samples(), batch_size=1, collate_fn=collate_quad_proposal_samples
    )
    placeholder = tmp_path / "unused.json"
    base_config = Phase3Config(
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
            epochs=2,
            freeze_backbone_epochs=0,
            warmup_steps=1,
            amp=False,
        ),
        output_dir=tmp_path / "unused",
        pretrained_backbone=False,
    )
    initial_model = QuadProposalDetector(backbone=StubBackbone())
    initial_state = {
        key: value.clone() for key, value in initial_model.state_dict().items()
    }

    def run(output_dir, max_steps, resume=None):
        model = QuadProposalDetector(backbone=StubBackbone())
        model.load_state_dict(initial_state)
        config = replace(base_config, output_dir=output_dir)
        train_quad_proposals(
            model,
            loader,
            loader,
            QuadTargetBuilder(QuadAssigner()),
            QuadProposalLoss(),
            config,
            torch.device("cpu"),
            max_steps=max_steps,
            max_val_batches=1,
            resume=resume,
        )
        return torch.load(
            output_dir / "last.pt", map_location="cpu", weights_only=False
        )

    uninterrupted = run(tmp_path / "uninterrupted", 2)
    first = run(tmp_path / "resumed", 1)
    resumed = run(tmp_path / "resumed", 2, tmp_path / "resumed" / "last.pt")
    assert first["global_step"] == 1
    assert resumed["global_step"] == uninterrupted["global_step"] == 2
    for state_key in ("model", "ema_model"):
        for key, expected in uninterrupted[state_key].items():
            torch.testing.assert_close(resumed[state_key][key], expected)
