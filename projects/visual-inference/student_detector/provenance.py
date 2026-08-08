"""Reproducibility contracts for bounded detector experiments."""

from __future__ import annotations

import hashlib
import json
import subprocess
from dataclasses import asdict
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .config import Phase3Config

if TYPE_CHECKING:
    import torch


def sha256_file(path: Path) -> str:
    """Hash a file without loading it into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _git_value(repository: Path, *arguments: str) -> str | None:
    result = subprocess.run(
        ["git", *arguments],
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def model_signature(model: torch.nn.Module) -> str:
    """Hash parameter and buffer names, shapes, and dtypes—not their values."""
    description = [
        (name, tuple(value.shape), str(value.dtype))
        for name, value in model.state_dict().items()
    ]
    payload = json.dumps(description, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def write_run_contract(
    output_dir: Path,
    *,
    config: Phase3Config,
    model: torch.nn.Module,
    repository: Path,
    train_images: int,
    validation_images: int,
    batches_per_epoch: int,
    optimizer_steps: int,
    world_size: int,
) -> Path:
    """Write the resolved, content-addressed training contract once per run."""
    import torch

    train_manifest = Path(
        config.data.quad_train_annotations or config.data.train_annotations
    )
    validation_manifest = Path(
        config.data.quad_val_annotations or config.data.val_annotations
    )
    lockfile = repository / "projects" / "visual-inference" / "uv.lock"
    status = _git_value(repository, "status", "--short")
    contract = {
        "schema_version": "phase3-run-contract.v1",
        "config": _jsonable(asdict(config)),
        "data": {
            "train_manifest": str(train_manifest),
            "train_manifest_sha256": (
                sha256_file(train_manifest) if train_manifest.is_file() else None
            ),
            "validation_manifest": str(validation_manifest),
            "validation_manifest_sha256": (
                sha256_file(validation_manifest)
                if validation_manifest.is_file()
                else None
            ),
            "train_images": train_images,
            "validation_images": validation_images,
        },
        "optimization": {
            "batches_per_epoch_per_process": batches_per_epoch,
            "optimizer_steps": optimizer_steps,
            "world_size": world_size,
            "global_effective_batch": (
                config.data.batch_size
                * config.schedule.accumulation_steps
            ),
        },
        "model_signature_sha256": model_signature(model),
        "environment": {
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "uv_lock_sha256": sha256_file(lockfile) if lockfile.exists() else None,
        },
        "git": {
            "commit": _git_value(repository, "rev-parse", "HEAD"),
            "dirty": bool(status),
            "status": status,
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    target = output_dir / "run_contract.json"
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    temporary.replace(target)
    return target


__all__ = ["model_signature", "sha256_file", "write_run_contract"]
