from __future__ import annotations

from pathlib import Path

import pytest

from student_detector.config import load_phase3_config
from student_detector.quad_assigner import QuadAssigner
from student_detector.quad_losses import QuadProposalLoss
from student_detector.quad_targets import QuadTargetBuilder
from student_detector.quad_training import _QuadTask
from student_detector.training_optimization import build_detector_optimizer


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "configs/phase3_rtx3060_bs8_min8_v1.yaml"


def test_min8_recipe_has_an_isolated_reproducible_contract() -> None:
    config = load_phase3_config(CONFIG)

    assert config.data.quad_regular_min_side == 8
    assert config.data.batch_size == 8
    assert config.schedule.accumulation_steps == 8
    assert config.data.batch_size * config.schedule.accumulation_steps == 64
    assert config.schedule.epochs == 20
    assert config.schedule.warmup_steps == 500
    assert config.schedule.validation_interval == 5
    assert (
        config.data.index_dir
        == (
            PROJECT_ROOT.parents[1] / "data/visual-inference-datasets/output/indexes"
        ).resolve()
    )
    assert (
        config.output_dir
        == (PROJECT_ROOT / "artifacts/phase3/runs/rtx3060_bs8_min8_v1").resolve()
    )


def test_min8_recipe_uses_reference_learning_rate() -> None:
    config = load_phase3_config(CONFIG)

    class Model:
        pass

    # The optimizer only requires the three parameter-owning submodules.
    import torch

    model = Model()
    model.backbone = torch.nn.Linear(1, 1)
    model.fpn = torch.nn.Linear(1, 1)
    model.head = torch.nn.Linear(1, 1)
    optimizer = build_detector_optimizer(model, config)  # type: ignore[arg-type]

    assert optimizer.param_groups[0]["lr"] == pytest.approx(1e-5)
    assert optimizer.param_groups[1]["lr"] == pytest.approx(1e-4)


def test_min8_warm_start_keeps_geometry_quality_from_first_epoch() -> None:
    config = load_phase3_config(CONFIG)
    criterion = QuadProposalLoss(
        quality_target_mode=config.quad.quality_target_mode,
        quality_blend=config.quad.quality_blend,
        geometry_quality_target=config.quad.geometry_quality_target,
    )
    task = _QuadTask(
        QuadTargetBuilder(QuadAssigner()),
        criterion,
        config,
    )

    task.on_epoch_complete(epoch=0, total_epochs=config.schedule.epochs)

    assert criterion.quality_target_mode == "iou"
    assert criterion.quality_blend == 1.0
    assert criterion.geometry_quality_target == "corner_proxy"
