from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from student_detector.config import AugmentationConfig
from student_detector.training_reporting import StandardReporter


@dataclass
class _Data:
    batch_size: int = 128


@dataclass
class _Schedule:
    epochs: int = 40


@dataclass
class _Config:
    data: _Data
    schedule: _Schedule
    augmentation: AugmentationConfig = AugmentationConfig()


class _Accelerator:
    def __init__(self) -> None:
        self.initialized = False
        self.logged_artifacts: list[object] = []

    def init_trackers(self, *_args: object, **_kwargs: object) -> None:
        self.initialized = True

    def get_tracker(self, *_args: object, **_kwargs: object) -> _Accelerator:
        return self

    def log_artifact(self, artifact: object, **_kwargs: object) -> None:
        self.logged_artifacts.append(artifact)


def test_dataset_artifact_failure_does_not_abort_training_start(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    class Artifact:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def add_reference(self, _uri: str, *, checksum: bool) -> None:
            assert checksum is False
            raise RuntimeError("reference unavailable")

    accelerator = _Accelerator()
    reporter = StandardReporter(
        tmp_path,
        wandb_project="project",
        run_id="run-1",
        accelerator=accelerator,
    )
    monkeypatch.setitem(sys.modules, "wandb", SimpleNamespace(Artifact=Artifact))
    monkeypatch.setenv("S3_BUCKET", "bucket")
    monkeypatch.setenv("DATASET_ID", "release-1")
    monkeypatch.setenv("S3_ENDPOINT_URL", "https://objects.example.test")
    monkeypatch.delenv("AWS_S3_ENDPOINT_URL", raising=False)

    reporter.on_start(
        config=_Config(data=_Data(), schedule=_Schedule()),
        train_loader=SimpleNamespace(dataset=range(10)),
        val_loader=SimpleNamespace(dataset=range(2)),
        world_size=1,
    )

    assert accelerator.initialized
    assert reporter.tracking_started
    assert not accelerator.logged_artifacts
    assert "dataset artifact reference failed" in capsys.readouterr().err
    events = [
        json.loads(line)
        for line in (tmp_path / "events.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert events[-1]["operation"] == "dataset_artifact"
    assert events[-1]["error"] == "reference unavailable"
    assert events[-1]["timestamp"].endswith("Z")
    assert os.environ["AWS_S3_ENDPOINT_URL"] == "https://objects.example.test"


def test_console_and_jsonl_records_have_utc_timestamps(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    reporter = StandardReporter(tmp_path, batch_log="metrics.jsonl")
    optimizer = SimpleNamespace(param_groups=[{"lr": 1e-5}, {"lr": 1e-4}])

    reporter.on_batch(
        metrics={"loss": 0.25},
        epoch=0,
        batch=10,
        batches_per_epoch=100,
        global_step=1,
        optimizer=optimizer,
    )

    output = capsys.readouterr().out
    assert output.startswith("20")
    assert "Z Training progress: epoch=1 batch=10/100 step=1 loss=0.250000" in output
    record = json.loads(
        (tmp_path / "metrics.jsonl").read_text(encoding="utf-8").strip()
    )
    assert record["timestamp"].endswith("Z")


def test_mixture_counts_are_persisted_for_optimizer_windows(tmp_path: Path) -> None:
    reporter = StandardReporter(tmp_path)
    reporter.on_mixture(
        scope="optimizer_window",
        epoch=0,
        batch=8,
        global_step=1,
        intended_sources={"coco": 32, "woodscape": 13},
        observed_sources={"coco": 32, "woodscape": 13},
        intended_domains={"general": 32, "fisheye": 13},
        observed_domains={"general": 32, "fisheye": 13},
    )

    event = json.loads((tmp_path / "events.jsonl").read_text(encoding="utf-8").strip())
    assert event["kind"] == "training_mixture"
    assert event["scope"] == "optimizer_window"
    assert event["intended_sources"] == event["observed_sources"]
    assert event["intended_domains"] == event["observed_domains"]
