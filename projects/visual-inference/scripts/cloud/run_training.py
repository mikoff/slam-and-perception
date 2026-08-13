"""Validated dstack task entrypoint for one visual-inference training run."""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
from pathlib import Path

if __package__:
    from .dataset_staging import (
        AwsCli,
        download_manifest,
        sha256_file,
        stage_dataset,
    )
else:
    from dataset_staging import (
        AwsCli,
        download_manifest,
        sha256_file,
        stage_dataset,
    )

PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPOSITORY_ROOT = PROJECT_ROOT.parents[1]
ALLOWED_CONFIGS = {"configs/phase3.yaml", "configs/phase3_attnres.yaml", "configs/phase3_rtx4090_bs128_v1.yaml"}
SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


def _required(name: str) -> str:
    value = os.getenv(name, "").strip()
    if not value:
        raise ValueError(f"{name} is required")
    return value


def verify_environment() -> dict[str, str]:
    """Reject unsafe or recipe-changing dispatch values before cloud spend."""
    run_id = _required("RUN_ID")
    dataset_id = _required("DATASET_ID")
    config = _required("CONFIG_PATH")
    mode = _required("RUN_MODE")
    if SAFE_ID.fullmatch(run_id) is None:
        raise ValueError("RUN_ID contains unsupported characters")
    if SAFE_ID.fullmatch(dataset_id) is None:
        raise ValueError("DATASET_ID contains unsupported characters")
    if config not in ALLOWED_CONFIGS:
        raise ValueError(f"CONFIG_PATH must be one of {sorted(ALLOWED_CONFIGS)}")
    if mode not in {"production", "smoke", "batch_preflight"}:
        raise ValueError("RUN_MODE must be production, smoke, or batch_preflight")
    batch_candidates = os.getenv("BATCH_CANDIDATES", "16,32,64,96,128").strip()
    try:
        parsed_candidates = [
            int(value.strip()) for value in batch_candidates.split(",")
        ]
    except ValueError as error:
        raise ValueError("BATCH_CANDIDATES must contain integers") from error
    if not parsed_candidates or any(value < 1 for value in parsed_candidates):
        raise ValueError("BATCH_CANDIDATES must contain positive integers")
    return {
        "run_id": run_id,
        "dataset_id": dataset_id,
        "config": config,
        "mode": mode,
        "batch_candidates": ",".join(str(value) for value in parsed_candidates),
    }


def _replace_with_symlink(link: Path, target: Path) -> None:
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink() and link.resolve() == target.resolve():
        return
    if link.exists() or link.is_symlink():
        if link.is_dir() and not any(link.iterdir()):
            link.rmdir()
        else:
            raise FileExistsError(f"refusing to replace non-empty path: {link}")
    link.symlink_to(target, target_is_directory=True)


def verify_remote_dataset(values: dict[str, str], *, bucket: str, aws: AwsCli) -> None:
    """Validate the small S3 manifest without downloading the dataset archive."""
    download_manifest(bucket=bucket, dataset_id=values["dataset_id"], aws=aws)


def build_training_command(values: dict[str, str], output_dir: Path) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "accelerate.commands.launch",
        "--num_processes",
        "1",
        "--num_machines",
        "1",
        "--dynamo_backend",
        "no",
        "--mixed_precision",
        "fp16",
        "scripts/train_quad_proposals.py",
        "--config",
        values["config"],
        "--output-dir",
        str(output_dir),
        "--run-id",
        values["run_id"],
        "--resume-mode",
        "auto",
        "--validation-interval",
        "5" if values["mode"] == "production" else "1",
    ]
    if values["mode"] == "smoke":
        command.extend(["--max-steps", "5", "--max-val-batches", "2"])
    for option, variable in (
        ("--wandb-project", "WANDB_PROJECT"),
        ("--wandb-entity", "WANDB_ENTITY"),
        ("--wandb-run-name", "RUN_DISPLAY_NAME"),
    ):
        if value := os.getenv(variable, "").strip():
            command.extend([option, value])
    return command


def build_workload_command(values: dict[str, str], output_dir: Path) -> list[str]:
    if values["mode"] != "batch_preflight":
        return build_training_command(values, output_dir)
    return [
        sys.executable,
        "scripts/benchmark_batch_size.py",
        "--config",
        values["config"],
        "--candidates",
        values["batch_candidates"],
        "--output",
        str(output_dir / "batch-preflight.json"),
    ]


def upload_batch_preflight_report(
    values: dict[str, str], output_dir: Path, bucket: str, aws: AwsCli
) -> bool:
    """Persist a completed preflight report even when it contains failures."""
    report = output_dir / "batch-preflight.json"
    if not report.is_file():
        print("Batch preflight exited without a report to upload", flush=True)
        return False
    destination = f"s3://{bucket}/runs/{values['run_id']}/batch-preflight.json"
    aws.upload(report, destination)
    print(f"Uploaded batch preflight report to {destination}", flush=True)
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--verify-env", action="store_true")
    parser.add_argument("--verify-dataset", action="store_true")
    args = parser.parse_args()
    values = verify_environment()
    if args.verify_dataset:
        verify_remote_dataset(
            values,
            bucket=_required("S3_BUCKET"),
            aws=AwsCli(os.getenv("S3_ENDPOINT_URL") or None),
        )
        print("Remote immutable dataset manifest verified")
    if args.verify_env:
        print("Cloud training environment verified")
        return

    bucket = _required("S3_BUCKET")
    dataset_root = Path(os.getenv("DATASET_ROOT", "/workspace/datasets"))
    aws = AwsCli(os.getenv("S3_ENDPOINT_URL") or None)
    dataset = stage_dataset(
        bucket=bucket,
        dataset_id=values["dataset_id"],
        destination_root=dataset_root,
        aws=aws,
    )
    os.environ["DATASET_MANIFEST_SHA256"] = sha256_file(
        dataset / ".dataset-manifest.json"
    )
    _replace_with_symlink(
        REPOSITORY_ROOT / "data/visual-inference-datasets/output", dataset
    )
    index_source = dataset / "indexes"
    if not index_source.is_dir():
        raise FileNotFoundError("immutable dataset is missing prebuilt indexes/")
    _replace_with_symlink(PROJECT_ROOT / "artifacts/phase3/index", index_source)

    output_root = Path(os.getenv("RUN_ROOT", "/workspace/runs"))
    output_dir = output_root / values["run_id"]
    output_dir.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        build_workload_command(values, output_dir),
        cwd=PROJECT_ROOT,
        start_new_session=True,
    )

    def forward(signum: int, _frame: object) -> None:
        if process.poll() is None:
            os.killpg(process.pid, signum)

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    return_code = process.wait()
    if values["mode"] == "batch_preflight":
        upload_batch_preflight_report(values, output_dir, bucket, aws)
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
