from __future__ import annotations

import json
from pathlib import Path

from student_detector.checkpoint_transport import (
    CheckpointUploader,
    resolve_auto_resume,
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
