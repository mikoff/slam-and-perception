from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from student_detector.checkpoint_transport import (
    AwsCheckpointStore,
    CheckpointUploader,
    resolve_auto_resume,
    resolve_resume_checkpoint,
)


class _Store:
    def __init__(self) -> None:
        self.values: dict[str, bytes] = {}
        self.active_uploads = 0
        self.max_active_uploads = 0

    def upload(self, source: Path, key: str) -> None:
        self.active_uploads += 1
        self.max_active_uploads = max(self.max_active_uploads, self.active_uploads)
        self.values[key] = source.read_bytes()
        self.active_uploads -= 1

    def download(self, key: str, destination: Path) -> bool:
        if key not in self.values:
            return False
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(self.values[key])
        return True


def test_uploader_serializes_and_resume_falls_back(tmp_path: Path) -> None:
    store = _Store()
    uploader = CheckpointUploader(run_id="run-1", output_dir=tmp_path, store=store)
    checkpoint = tmp_path / "last.pt"
    checkpoint.write_bytes(b"first")
    uploader.enqueue(checkpoint, 10)
    checkpoint.write_bytes(b"second")
    uploader.enqueue(checkpoint, 20)
    uploader.close()

    manifest_key = "runs/run-1/checkpoints/latest.json"
    manifest = json.loads(store.values[manifest_key])
    assert manifest["latest"]["global_step"] == 20
    assert manifest["previous"][0]["global_step"] == 10
    assert store.max_active_uploads == 1

    store.values[manifest["latest"]["key"]] = b"corrupt"
    restored = resolve_auto_resume(run_id="run-1", output_dir=tmp_path, store=store)
    assert restored is not None
    assert restored.read_bytes() == b"first"


def test_resolved_checkpoint_preserves_parent_contract(tmp_path: Path) -> None:
    store = _Store()
    contract = {
        "source_commit": "parent-commit",
        "dataset_id": "dataset-1",
        "config_path": "configs/phase3.yaml",
    }
    uploader = CheckpointUploader(
        run_id="parent-run",
        output_dir=tmp_path,
        store=store,
        contract=contract,
    )
    checkpoint = tmp_path / "last.pt"
    checkpoint.write_bytes(b"checkpoint")
    uploader.enqueue(checkpoint, 100)
    uploader.close()

    resolved = resolve_resume_checkpoint(
        run_id="parent-run",
        output_dir=tmp_path,
        store=store,
        expected_contract={"dataset_id": "dataset-1"},
    )

    assert resolved is not None
    assert resolved.run_id == "parent-run"
    assert resolved.contract == contract
    assert resolved.path.read_bytes() == b"checkpoint"


def test_best_checkpoint_does_not_replace_resume_pointer(tmp_path: Path) -> None:
    store = _Store()
    uploader = CheckpointUploader(run_id="run-2", output_dir=tmp_path, store=store)
    checkpoint = tmp_path / "last.pt"
    checkpoint.write_bytes(b"full")
    uploader.enqueue(checkpoint, 4)
    best = tmp_path / "best_ema.pt"
    best.write_bytes(b"weights")
    uploader.enqueue(best, 4, logical_name=best.name)
    uploader.close()
    manifest = json.loads(store.values["runs/run-2/checkpoints/latest.json"])
    assert manifest["latest"]["global_step"] == 4
    assert "runs/run-2/checkpoints/aliases/best_ema.pt.json" in store.values


def test_auto_resume_propagates_remote_failures(tmp_path: Path) -> None:
    class FailingStore:
        def download(self, _key: str, _destination: Path) -> bool:
            raise RuntimeError("S3 is unavailable")

    with pytest.raises(RuntimeError, match="S3 is unavailable"):
        resolve_auto_resume(run_id="run-3", output_dir=tmp_path, store=FailingStore())


def test_uploader_rejects_existing_manifest_from_another_contract(
    tmp_path: Path,
) -> None:
    store = _Store()
    first = CheckpointUploader(
        run_id="run-4",
        output_dir=tmp_path,
        store=store,
        contract={"source_commit": "first"},
    )
    checkpoint = tmp_path / "last.pt"
    checkpoint.write_bytes(b"first")
    first.enqueue(checkpoint, 1)
    first.close()

    second = CheckpointUploader(
        run_id="run-4",
        output_dir=tmp_path,
        store=store,
        contract={"source_commit": "second"},
    )
    checkpoint.write_bytes(b"second")
    second.enqueue(checkpoint, 2)
    with pytest.raises(RuntimeError, match="checkpoint upload failed"):
        second.close()


@pytest.mark.parametrize(
    ("stderr", "expected"),
    [
        ("fatal error: An error occurred (404) when calling HeadObject", False),
        ("fatal error: An error occurred (AccessDenied)", RuntimeError),
    ],
)
def test_aws_download_distinguishes_missing_from_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    stderr: str,
    expected: bool | type[RuntimeError],
) -> None:
    def run(*_args: object, **_kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess([], 1, stderr=stderr)

    monkeypatch.setattr(subprocess, "run", run)
    store = AwsCheckpointStore("bucket")
    destination = tmp_path / "checkpoint.pt"
    if expected is RuntimeError:
        with pytest.raises(RuntimeError, match="checkpoint download failed"):
            store.download("runs/run/checkpoint.pt", destination)
    else:
        assert store.download("runs/run/checkpoint.pt", destination) is expected


def test_aws_download_removes_partial_file_after_timeout(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    destination = tmp_path / "checkpoint.pt"

    def run(*_args: object, **_kwargs: object) -> subprocess.CompletedProcess[str]:
        destination.write_bytes(b"partial")
        raise subprocess.TimeoutExpired("aws", 300)

    monkeypatch.setattr(subprocess, "run", run)
    with pytest.raises(RuntimeError, match="checkpoint download failed"):
        AwsCheckpointStore("bucket").download("runs/run/checkpoint.pt", destination)
    assert not destination.exists()
