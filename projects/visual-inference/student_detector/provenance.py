"""Reproducibility contracts for bounded detector experiments."""

from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import subprocess
from dataclasses import asdict
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from .augmentation import effective_augmentation_policy
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


def model_state_sha256(model: torch.nn.Module) -> str:
    """Hash every initial parameter and buffer value in stable name order."""
    import torch

    digest = hashlib.sha256()
    for name, value in sorted(model.state_dict().items()):
        tensor = value.detach().cpu().contiguous()
        digest.update(name.encode())
        digest.update(b"\0")
        digest.update(str(tensor.dtype).encode())
        digest.update(b"\0")
        digest.update(json.dumps(tuple(tensor.shape)).encode())
        digest.update(b"\0")
        digest.update(tensor.reshape(-1).view(torch.uint8).numpy().tobytes())
    return digest.hexdigest()


def _manifest_sha256(manifest: Path, index: Path) -> str | None:
    """Use the prebuilt index's verified source signature when available."""
    if index.is_file():
        try:
            with sqlite3.connect(
                f"file:{index}?mode=ro&immutable=1", uri=True
            ) as connection:
                row = connection.execute(
                    "SELECT value FROM metadata WHERE key='source_signature'"
                ).fetchone()
            if row and str(row[0]).startswith("sha256:"):
                return str(row[0]).removeprefix("sha256:")
        except sqlite3.Error:
            pass
    return sha256_file(manifest) if manifest.is_file() else None


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
    geometry: Literal["hbb", "quad"],
) -> Path:
    """Write the resolved, content-addressed training contract once per run."""
    import torch

    if geometry == "quad":
        train_manifest = Path(
            config.data.quad_train_annotations or config.data.train_annotations
        )
        validation_manifest = Path(
            config.data.quad_val_annotations or config.data.val_annotations
        )
    else:
        train_manifest = config.data.train_annotations
        validation_manifest = config.data.val_annotations
    train_index = config.data.index_dir / "quad_train.sqlite"
    validation_index = config.data.index_dir / "quad_val.sqlite"
    lockfile = repository / "projects" / "visual-inference" / "uv.lock"
    status = _git_value(repository, "status", "--short")
    contract = {
        "schema_version": "phase3-run-contract.v1",
        "geometry": geometry,
        "config": _jsonable(asdict(config)),
        "data": {
            "train_manifest": str(train_manifest),
            "train_manifest_sha256": _manifest_sha256(train_manifest, train_index),
            "train_manifest_hash_source": str(train_index),
            "validation_manifest": str(validation_manifest),
            "validation_manifest_sha256": _manifest_sha256(
                validation_manifest, validation_index
            ),
            "validation_manifest_hash_source": str(validation_index),
            "train_images": train_images,
            "validation_images": validation_images,
        },
        "optimization": {
            "batches_per_epoch_per_process": batches_per_epoch,
            "optimizer_steps": optimizer_steps,
            "world_size": world_size,
            "global_effective_batch": (
                config.data.batch_size * config.schedule.accumulation_steps
            ),
        },
        "augmentation_effective": effective_augmentation_policy(config.augmentation),
        "model_signature_sha256": model_signature(model),
        "initial_model_state_sha256": model_state_sha256(model),
        "environment": {
            "torch": torch.__version__,
            "cuda": torch.version.cuda,
            "uv_lock_sha256": sha256_file(lockfile) if lockfile.exists() else None,
            "resume_from_run_id": os.getenv("RESUME_FROM_RUN_ID") or None,
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


__all__ = [
    "model_signature",
    "model_state_sha256",
    "sha256_file",
    "write_run_contract",
]
