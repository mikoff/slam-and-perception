"""Fail-closed local identity checks for Phase 4 training inputs."""

from __future__ import annotations

import copy
import json
import sqlite3
from dataclasses import asdict
from pathlib import Path
from typing import Any

import yaml

from .checkpoints import load_checkpoint, validate_checkpoint_contract
from .config import Phase3Config
from .provenance import sha256_file


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _json_object(path: Path, name: str) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Phase 4 {name} is missing: {path}")
    return _mapping(json.loads(path.read_text(encoding="utf-8")), name)


def _index_metadata(path: Path) -> tuple[dict[str, str], int]:
    if not path.is_file():
        raise FileNotFoundError(f"Phase 4 prebuilt index is missing: {path}")
    with sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True) as connection:
        metadata = dict(connection.execute("SELECT key, value FROM metadata"))
        images = int(connection.execute("SELECT COUNT(*) FROM images").fetchone()[0])
    return metadata, images


def _require_equal(name: str, actual: object, expected: object) -> None:
    if actual != expected:
        raise ValueError(
            f"Phase 4 {name} mismatch: expected {expected!r}, found {actual!r}"
        )


def verify_phase4_launch_inputs(
    config: Phase3Config,
    *,
    training_contract_path: Path,
    initialization_path: Path | None,
    initialization_state: str | None,
    initialization_required: bool,
) -> dict[str, str]:
    """Verify the immutable dataset and fresh-start checkpoint before GPU setup."""
    training_contract = _mapping(
        yaml.safe_load(training_contract_path.read_text(encoding="utf-8")),
        "training contract",
    )
    expected_dataset = _mapping(training_contract.get("dataset"), "dataset contract")
    expected_initialization = _mapping(
        training_contract.get("initialization"), "initialization contract"
    )

    output = config.data.image_root.resolve()
    workspace = output.parent
    _require_equal("dataset output directory", output, workspace / "output")
    _require_equal("dataset workspace name", workspace.name, expected_dataset["id"])
    expected_paths = {
        "training manifest path": output / "annotations/proposals_train.json",
        "validation manifest path": output / "annotations/proposals_val.json",
        "training index directory": output / "indexes",
    }
    actual_paths = {
        "training manifest path": config.data.train_annotations.resolve(),
        "validation manifest path": config.data.val_annotations.resolve(),
        "training index directory": config.data.index_dir.resolve(),
    }
    for name, expected in expected_paths.items():
        _require_equal(name, actual_paths[name], expected)

    dataset_contract_path = workspace / "dataset_contract.json"
    dataset_contract = _json_object(dataset_contract_path, "dataset contract")
    dataset_contract_sha256 = sha256_file(dataset_contract_path)
    _require_equal(
        "dataset contract SHA-256",
        dataset_contract_sha256,
        expected_dataset["contract_sha256"],
    )
    _require_equal(
        "dataset contract schema",
        dataset_contract.get("schema_version"),
        "phase3-dataset-contract.v1",
    )
    _require_equal(
        "dataset identifier", dataset_contract.get("dataset_id"), expected_dataset["id"]
    )

    cloud_manifest_path = workspace / "reports/cloud_bundle/dataset-manifest.json"
    cloud_manifest = _json_object(cloud_manifest_path, "cloud dataset manifest")
    _require_equal(
        "cloud dataset manifest SHA-256",
        sha256_file(cloud_manifest_path),
        expected_dataset["manifest_sha256"],
    )
    _require_equal(
        "cloud manifest dataset identifier",
        cloud_manifest.get("dataset_id"),
        expected_dataset["id"],
    )
    _require_equal(
        "cloud manifest dataset contract SHA-256",
        cloud_manifest.get("dataset_contract_sha256"),
        expected_dataset["contract_sha256"],
    )

    artifacts = _mapping(dataset_contract.get("artifacts"), "dataset artifacts")
    signatures: dict[str, str] = {}
    for split in ("train", "val"):
        manifest_path = output / f"annotations/proposals_{split}.json"
        index_path = output / f"indexes/quad_{split}.sqlite"
        manifest_contract = _mapping(
            artifacts.get(f"proposals_{split}.json"), f"{split} manifest contract"
        )
        index_contract = _mapping(
            artifacts.get(f"quad_{split}.sqlite"), f"{split} index contract"
        )
        if not manifest_path.is_file():
            raise FileNotFoundError(
                f"Phase 4 {split} manifest is missing: {manifest_path}"
            )
        _require_equal(
            f"{split} manifest size",
            manifest_path.stat().st_size,
            manifest_contract["bytes"],
        )
        metadata, images = _index_metadata(index_path)
        _require_equal(
            f"{split} index size", index_path.stat().st_size, index_contract["bytes"]
        )
        _require_equal(
            f"{split} index schema",
            metadata.get("schema_version"),
            "8:proposal-manifest",
        )
        _require_equal(
            f"{split} index source signature",
            metadata.get("source_signature"),
            index_contract["source_signature"],
        )
        _require_equal(f"{split} index image count", images, index_contract["images"])
        _require_equal(
            f"{split} manifest/index binding",
            metadata.get("source_signature"),
            f"sha256:{manifest_contract['sha256']}",
        )
        signatures[split] = str(manifest_contract["sha256"])

    if initialization_required and initialization_path is None:
        raise ValueError("Phase 4 fresh start requires --initialize-from")
    if initialization_path is not None:
        resolved_initialization = initialization_path.resolve()
        if not resolved_initialization.is_file():
            raise FileNotFoundError(
                f"Phase 4 initialization is missing: {resolved_initialization}"
            )
        _require_equal(
            "initialization SHA-256",
            sha256_file(resolved_initialization),
            expected_initialization["artifact_sha256"],
        )
        _require_equal(
            "initialization state",
            initialization_state,
            expected_initialization["checkpoint_state"],
        )

    return {
        "dataset_id": str(expected_dataset["id"]),
        "dataset_contract_sha256": dataset_contract_sha256,
        "train_manifest_sha256": signatures["train"],
        "validation_manifest_sha256": signatures["val"],
        "index_schema": "8:proposal-manifest",
    }


def verify_phase4_quad_continuation_inputs(
    config: Phase3Config,
    *,
    config_path: Path,
    training_contract_path: Path,
    resume_path: Path | None,
    target_global_step: int | None,
) -> dict[str, str]:
    """Verify an exact staged resume and its isolated output boundary."""
    identity = verify_phase4_launch_inputs(
        config,
        training_contract_path=training_contract_path,
        initialization_path=None,
        initialization_state=None,
        initialization_required=False,
    )
    contract = _mapping(
        yaml.safe_load(training_contract_path.read_text(encoding="utf-8")),
        "quad continuation contract",
    )
    configuration = _mapping(contract.get("configuration"), "configuration")
    resume = _mapping(contract.get("resume"), "resume")
    optimization = _mapping(contract.get("optimization"), "optimization")
    project_root = config_path.resolve().parent.parent

    source_run_contract = resume.get("source_run_contract")
    if source_run_contract is not None:
        source_run_contract_path = Path(str(source_run_contract))
        if not source_run_contract_path.is_absolute():
            source_run_contract_path = project_root / source_run_contract_path
        if not source_run_contract_path.is_file():
            raise FileNotFoundError(
                f"Phase 4 source run contract is missing: {source_run_contract_path}"
            )
        _require_equal(
            "source run contract SHA-256",
            sha256_file(source_run_contract_path),
            resume["source_run_contract_sha256"],
        )

    entry_evidence = contract.get("entry_evidence")
    if entry_evidence is not None:
        evidence = _mapping(entry_evidence, "entry evidence")
        for report_name in (
            "proposal_utility",
            "suppression_sweep",
            "diversity_audit",
        ):
            if report_name not in evidence:
                continue
            report = _mapping(evidence.get(report_name), report_name)
            report_path = Path(str(report["path"]))
            if not report_path.is_absolute():
                report_path = project_root / report_path
            if not report_path.is_file():
                raise FileNotFoundError(
                    f"Phase 4 {report_name} is missing: {report_path}"
                )
            _require_equal(
                f"{report_name} SHA-256",
                sha256_file(report_path),
                report["sha256"],
            )

    _require_equal(
        "continuation config SHA-256",
        sha256_file(config_path),
        configuration["sha256"],
    )
    if resume_path is None:
        raise ValueError("Phase 4 quad continuation requires --resume")
    source_resume = Path(str(resume["artifact"]))
    if not source_resume.is_absolute():
        source_resume = project_root / source_resume
    interrupted_resume = Path(str(resume["interrupted_stage_artifact"]))
    if not interrupted_resume.is_absolute():
        interrupted_resume = project_root / interrupted_resume
    actual_resume = resume_path.resolve()
    if not actual_resume.is_file():
        raise FileNotFoundError(
            f"Phase 4 continuation checkpoint is missing: {actual_resume}"
        )
    _require_equal(
        "continuation target global step",
        target_global_step,
        optimization["target_global_step"],
    )
    expected_output = Path(str(optimization["output_dir"]))
    if not expected_output.is_absolute():
        expected_output = project_root / expected_output
    _require_equal("continuation output directory", config.output_dir, expected_output)
    if actual_resume == source_resume:
        resume_sha256 = sha256_file(source_resume)
        _require_equal(
            "continuation resume SHA-256",
            resume_sha256,
            resume["artifact_sha256"],
        )
        if (
            optimization.get("initial_output_must_be_absent")
            and expected_output.exists()
        ):
            raise FileExistsError(
                f"Phase 4 continuation output already exists: {expected_output}"
            )
        resume_mode = "initial"
    elif actual_resume == interrupted_resume:
        checkpoint = load_checkpoint(interrupted_resume)
        validate_checkpoint_contract(
            checkpoint, kind="quad", neck_type=config.neck_type
        )
        recorded_config = copy.deepcopy(checkpoint.get("config"))
        expected_config = asdict(config)
        if not isinstance(recorded_config, dict):
            raise ValueError("interrupted checkpoint config must be a mapping")
        for dotted_path in resume.get("allowed_config_overrides", []):
            recorded_cursor = recorded_config
            expected_cursor = expected_config
            parts = str(dotted_path).split(".")
            for part in parts[:-1]:
                recorded_cursor = _mapping(
                    recorded_cursor.get(part), f"recorded config {dotted_path}"
                )
                expected_cursor = _mapping(
                    expected_cursor.get(part), f"expected config {dotted_path}"
                )
            recorded_cursor[parts[-1]] = expected_cursor[parts[-1]]
        _require_equal(
            "interrupted continuation config",
            recorded_config,
            expected_config,
        )
        global_step = int(checkpoint.get("global_step", -1))
        minimum_step, maximum_step = resume["interrupted_stage_step_range"]
        if not int(minimum_step) <= global_step <= int(maximum_step):
            raise ValueError(
                "interrupted continuation global step is outside "
                f"[{minimum_step}, {maximum_step}]: {global_step}"
            )
        resume_sha256 = sha256_file(interrupted_resume)
        recovery_entry = resume.get("recovery_entry")
        if recovery_entry is not None:
            entry = _mapping(recovery_entry, "recovery entry")
            entry_step = int(entry["global_step"])
            if global_step < entry_step:
                raise ValueError(
                    "interrupted continuation precedes recovery entry: "
                    f"expected at least {entry_step}, found {global_step}"
                )
            if global_step == entry_step:
                _require_equal(
                    "recovery entry SHA-256",
                    resume_sha256,
                    entry["artifact_sha256"],
                )
                for field in (
                    "epoch",
                    "batch_in_epoch",
                    "epoch_complete",
                    "optimizer_step_skips",
                ):
                    _require_equal(
                        f"recovery entry {field}", checkpoint.get(field), entry[field]
                    )
                scaler = _mapping(checkpoint.get("scaler"), "checkpoint scaler")
                _require_equal(
                    "recovery entry scaler scale",
                    float(scaler["scale"]),
                    float(entry["scaler_scale"]),
                )
        resume_mode = "interrupted_stage"
    else:
        raise ValueError(
            "Phase 4 continuation resume path must be the immutable source "
            f"artifact or the stage recovery checkpoint, found {actual_resume}"
        )
    return {
        **identity,
        "resume_mode": resume_mode,
        "resume_sha256": resume_sha256,
    }


__all__ = [
    "verify_phase4_launch_inputs",
    "verify_phase4_quad_continuation_inputs",
]
