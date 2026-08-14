from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.cloud.run_training import (
    build_training_command,
    build_workload_command,
    upload_batch_preflight_report,
    verify_checkpoint_io,
    verify_environment,
    verify_remote_dataset,
)


def _environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("RUN_ID", "vi-123-1")
    monkeypatch.setenv("DATASET_ID", "phase3-release-1")
    monkeypatch.setenv("CONFIG_PATH", "configs/phase3.yaml")
    monkeypatch.setenv("RUN_MODE", "production")


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
