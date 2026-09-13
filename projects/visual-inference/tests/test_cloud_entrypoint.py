from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.cloud.run_training import (
    build_training_command,
    build_workload_command,
    stage_initialization,
    upload_batch_preflight_report,
    verify_checkpoint_io,
    verify_environment,
    verify_remote_dataset,
    verify_resume_source,
)


def _environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("RUN_ID", "vi-123-1")
    monkeypatch.setenv("DATASET_ID", "phase3-release-1")
    monkeypatch.setenv("CONFIG_PATH", "configs/phase3.yaml")
    monkeypatch.setenv("RUN_MODE", "production")


def _phase4_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("RUN_ID", "vi-phase4-1")
    monkeypatch.setenv("DATASET_ID", "phase3-production-bg-policy-v2-2026-08-29")
    monkeypatch.setenv("CONFIG_PATH", "configs/correction_phase4_hbb_p3_v1.yaml")
    monkeypatch.setenv("RUN_MODE", "production")
    monkeypatch.setenv("CLOUD_PROVIDER", "packet")
    monkeypatch.setenv("DSTACK_GPU", "RTX4090")
    monkeypatch.setenv("PACKET_HOURLY_RATE_USD", "0.75")


def test_cloud_environment_rejects_unversioned_recipe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("CONFIG_PATH", "configs/debug.yaml")
    with pytest.raises(ValueError, match="CONFIG_PATH"):
        verify_environment()


def test_cloud_environment_accepts_rtx4090_production_recipe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("CONFIG_PATH", "configs/phase3_rtx4090_bs128_v1.yaml")
    assert verify_environment()["config"] == "configs/phase3_rtx4090_bs128_v1.yaml"


def test_phase4_environment_is_packet_rtx4090_only(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    values = verify_environment()
    assert values["geometry"] == "hbb"

    monkeypatch.setenv("CLOUD_PROVIDER", "runpod")
    with pytest.raises(ValueError, match="only for Packet"):
        verify_environment()


def test_phase4_environment_rejects_cost_above_ceiling(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    monkeypatch.setenv("PACKET_HOURLY_RATE_USD", "1.01")

    with pytest.raises(ValueError, match="projected 12-hour cost exceeds"):
        verify_environment()


def test_production_command_can_resume_from_an_immutable_parent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("RESUME_FROM_RUN_ID", "vi-parent-1")
    values = verify_environment()

    command = build_training_command(values, Path("/runs/vi-123-1"))

    assert command[command.index("--resume-from-run-id") + 1] == "vi-parent-1"


def test_smoke_rejects_cross_run_resume(monkeypatch: pytest.MonkeyPatch) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "smoke")
    monkeypatch.setenv("RESUME_FROM_RUN_ID", "vi-parent-1")
    with pytest.raises(ValueError, match="only in production or pilot"):
        verify_environment()


def test_resume_parent_manifest_is_verified_before_submission(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("SOURCE_COMMIT", "child-commit")
    monkeypatch.setenv("RESUME_FROM_RUN_ID", "vi-parent-1")
    values = verify_environment()
    manifest = {
        "schema_version": "visual-inference-checkpoints.v1",
        "run_id": "vi-parent-1",
        "contract": {
            "source_commit": "parent-commit",
            "dataset_id": values["dataset_id"],
            "config_path": values["config"],
        },
        "latest": {
            "global_step": 100,
            "key": "runs/vi-parent-1/checkpoints/last/last-step-100.pt",
        },
    }

    class Aws:
        def download(self, _uri: str, destination: Path) -> None:
            destination.write_text(json.dumps(manifest), encoding="utf-8")

    monkeypatch.setattr(
        "scripts.cloud.run_training.subprocess.run",
        lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stderr=""),
    )

    verify_resume_source(values, bucket="bucket", aws=Aws())


def test_smoke_command_is_bounded_and_resume_enabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    values = verify_environment()
    values["mode"] = "smoke"
    command = build_training_command(values, Path("/runs/vi-123-1"))
    assert command[0:3] == [command[0], "-m", "accelerate.commands.launch"]
    assert command[command.index("--max-steps") + 1] == "5"
    assert command[command.index("--max-val-batches") + 1] == "2"
    assert command[command.index("--log-interval") + 1] == "1"
    assert command[command.index("--resume-mode") + 1] == "auto"
    assert command[command.index("--validation-interval") + 1] == "1"


def test_phase4_command_selects_hbb_and_strict_ema_initialization(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    values = verify_environment()
    values["initialization_path"] = "/runs/vi-phase4-1/preflight/initialization.pt"

    command = build_training_command(values, Path("/runs/vi-phase4-1"))

    assert "scripts/train_phase3.py" in command
    assert command[command.index("--initialize-state") + 1] == "ema_model"
    assert (
        command[command.index("--initialize-from") + 1] == values["initialization_path"]
    )
    assert "--validation-interval" not in command


def test_phase4_pilot_is_exactly_bounded_and_checkpointed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "pilot")
    values = verify_environment()
    values["initialization_path"] = "/runs/vi-phase4-1/preflight/initialization.pt"

    command = build_training_command(values, Path("/runs/vi-phase4-1"))

    assert command[command.index("--max-steps") + 1] == "2000"
    assert command[command.index("--checkpoint-every-steps") + 1] == "500"
    assert command[command.index("--log-interval") + 1] == "50"
    assert "--max-val-batches" not in command
    assert "--validation-interval" not in command


def test_phase4_pilot_accepts_verified_parent_resume(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "pilot")
    monkeypatch.setenv("RESUME_FROM_RUN_ID", "vi-phase4-parent")

    values = verify_environment()
    command = build_training_command(values, Path("/runs/vi-phase4-1"))

    assert command[command.index("--resume-from-run-id") + 1] == "vi-phase4-parent"
    assert "--initialize-from" not in command


def test_phase4_pilot_rejects_parent_from_another_mode(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _phase4_environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "pilot")
    monkeypatch.setenv("RESUME_FROM_RUN_ID", "vi-phase4-parent")
    monkeypatch.setenv("SOURCE_COMMIT", "same-commit")
    values = verify_environment()
    manifest = {
        "schema_version": "visual-inference-checkpoints.v1",
        "run_id": "vi-phase4-parent",
        "contract": {
            "source_commit": "same-commit",
            "dataset_id": values["dataset_id"],
            "config_path": values["config"],
            "run_mode": "production",
        },
        "latest": {
            "global_step": 500,
            "key": "runs/vi-phase4-parent/checkpoints/last/last-step-500.pt",
        },
    }

    class Store:
        def download(self, _uri: str, destination: Path) -> None:
            destination.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(ValueError, match="contract mismatch for run_mode"):
        verify_resume_source(values, bucket="bucket", aws=Store())  # type: ignore[arg-type]


def test_pilot_rejects_non_phase4_recipe(monkeypatch: pytest.MonkeyPatch) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "pilot")

    with pytest.raises(ValueError, match="reserved for the approved Phase 4"):
        verify_environment()


def test_phase4_initialization_is_downloaded_and_hash_checked(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _phase4_environment(monkeypatch)
    values = verify_environment()
    payload = b"approved checkpoint"
    monkeypatch.setattr(
        "scripts.cloud.run_training.PHASE4_INITIALIZATION_SHA256",
        hashlib.sha256(payload).hexdigest(),
    )

    class Store:
        def download(self, _uri: str, destination: Path) -> None:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(payload)

    staged = stage_initialization(
        values,
        output_dir=tmp_path,
        bucket="bucket",
        aws=Store(),  # type: ignore[arg-type]
    )

    assert staged is not None
    assert staged.read_bytes() == payload
    assert values["initialization_path"] == str(staged)


def test_batch_preflight_uses_isolated_benchmark_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("RUN_MODE", "batch_preflight")
    monkeypatch.setenv("BATCH_CANDIDATES", "32,64,96")
    values = verify_environment()

    command = build_workload_command(values, Path("/runs/vi-123-1"))

    assert command[1] == "scripts/benchmark_batch_size.py"
    assert command[command.index("--candidates") + 1] == "32,64,96"
    assert command[command.index("--output") + 1].endswith("batch-preflight.json")
    assert "accelerate.commands.launch" not in command


def test_batch_preflight_rejects_invalid_candidates(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    monkeypatch.setenv("BATCH_CANDIDATES", "32,nope")
    with pytest.raises(ValueError, match="BATCH_CANDIDATES"):
        verify_environment()


def test_failed_batch_preflight_report_is_still_uploaded(tmp_path: Path) -> None:
    output_dir = tmp_path / "run"
    output_dir.mkdir()
    report = output_dir / "batch-preflight.json"
    report.write_text('{"recommended_batch_size": null}\n', encoding="utf-8")
    uploads: list[tuple[Path, str]] = []

    class Store:
        def upload(self, source: Path, destination: str) -> None:
            uploads.append((source, destination))

    assert upload_batch_preflight_report(
        {"run_id": "vi-123-1"},
        output_dir,
        "bucket",
        Store(),  # type: ignore[arg-type]
    )
    assert uploads == [(report, "s3://bucket/runs/vi-123-1/batch-preflight.json")]


def test_remote_dataset_check_downloads_only_manifest(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    values = verify_environment()
    downloads: list[str] = []

    class _ManifestStore:
        def download(self, uri: str, destination: Path) -> None:
            downloads.append(uri)
            destination.write_text(
                json.dumps(
                    {
                        "schema_version": "visual-inference-dataset.v1",
                        "dataset_id": values["dataset_id"],
                        "archive": {
                            "key": (f"datasets/{values['dataset_id']}/dataset.tar.gz"),
                            "size": 123,
                            "sha256": "a" * 64,
                        },
                        "files": [{"path": "image.jpg", "size": 1, "sha256": "b" * 64}],
                    }
                ),
                encoding="utf-8",
            )

    verify_remote_dataset(values, bucket="bucket", aws=_ManifestStore())  # type: ignore[arg-type]

    assert downloads == ["s3://bucket/datasets/phase3-release-1/dataset-manifest.json"]


def test_checkpoint_io_probe_uploads_and_reads_back_run_scoped_object(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _environment(monkeypatch)
    values = verify_environment()
    objects: dict[str, bytes] = {}

    class Store:
        def upload(self, source: Path, uri: str) -> None:
            objects[uri] = source.read_bytes()

        def download(self, uri: str, destination: Path) -> None:
            destination.write_bytes(objects[uri])

    verify_checkpoint_io(values, bucket="bucket", aws=Store())  # type: ignore[arg-type]

    uri = "s3://bucket/runs/vi-123-1/preflight/checkpoint-io.json"
    assert json.loads(objects[uri]) == {
        "run_id": "vi-123-1",
        "schema_version": "visual-inference-checkpoint-io.v1",
    }
