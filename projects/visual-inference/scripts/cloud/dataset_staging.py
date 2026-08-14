"""Fail-closed staging for immutable visual-inference dataset archives."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path, PurePosixPath
from typing import Any

MANIFEST_SCHEMA = "visual-inference-dataset.v1"


def sha256_file(path: Path, *, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_relative_path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if not value or path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe dataset path: {value!r}")
    return path


def validate_manifest(manifest: dict[str, Any], dataset_id: str) -> None:
    """Validate the immutable manifest before downloading a large archive."""
    if manifest.get("schema_version") != MANIFEST_SCHEMA:
        raise ValueError("unsupported dataset manifest schema")
    if manifest.get("dataset_id") != dataset_id:
        raise ValueError("dataset manifest ID does not match requested DATASET_ID")
    archive = manifest.get("archive")
    if not isinstance(archive, dict):
        raise ValueError("dataset manifest is missing archive metadata")
    key = str(archive.get("key", ""))
    if not key.startswith(f"datasets/{dataset_id}/"):
        raise ValueError("dataset archive must be inside its immutable dataset prefix")
    if len(str(archive.get("sha256", ""))) != 64:
        raise ValueError("dataset archive SHA-256 is required")
    if int(archive.get("size", 0)) <= 0:
        raise ValueError("dataset archive size must be positive")
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("dataset manifest must hash every extracted file")
    for entry in files:
        if not isinstance(entry, dict):
            raise ValueError("invalid dataset file entry")
        _validate_relative_path(str(entry.get("path", "")))
        if len(str(entry.get("sha256", ""))) != 64:
            raise ValueError("every dataset file requires SHA-256")
        if int(entry.get("size", -1)) < 0:
            raise ValueError("dataset file size cannot be negative")


def required_staging_bytes(manifest: dict[str, Any]) -> int:
    """Return peak bytes needed while archive and extracted tree coexist."""
    archive_size = int(manifest["archive"]["size"])
    extracted_size = sum(int(entry["size"]) for entry in manifest["files"])
    return archive_size + extracted_size


def download_manifest(*, bucket: str, dataset_id: str, aws: AwsCli) -> dict[str, Any]:
    """Download and validate an immutable dataset manifest."""
    uri = f"s3://{bucket}/datasets/{dataset_id}/dataset-manifest.json"
    with tempfile.TemporaryDirectory(prefix="dataset-manifest-") as temporary:
        path = Path(temporary) / "dataset-manifest.json"
        aws.download(uri, path)
        manifest = json.loads(path.read_text(encoding="utf-8"))
    validate_manifest(manifest, dataset_id)
    return manifest


class AwsCli:
    """Small checked AWS CLI boundary shared by staging tests and cloud runs."""

    def __init__(self, endpoint_url: str | None = None) -> None:
        self.endpoint_url = endpoint_url

    def _command(self, *arguments: str) -> list[str]:
        command = ["aws", *arguments]
        if self.endpoint_url:
            command.extend(["--endpoint-url", self.endpoint_url])
        return command

    def download(self, uri: str, destination: Path) -> None:
        destination.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            self._command("s3", "cp", uri, str(destination), "--no-progress"),
            check=True,
        )

    def upload(self, source: Path, uri: str) -> None:
        subprocess.run(
            self._command("s3", "cp", str(source), uri, "--no-progress"),
            check=True,
        )


def _safe_extract(archive: Path, destination: Path) -> None:
    with tarfile.open(archive, mode="r:*") as bundle:
        members = bundle.getmembers()
        for member in members:
            _validate_relative_path(member.name)
            if member.issym() or member.islnk() or member.isdev():
                raise ValueError(
                    f"dataset archive contains unsafe member: {member.name}"
                )
        # Every member was validated above. The filter argument is unavailable
        # on the oldest supported Python (3.11).
        if sys.version_info >= (3, 12):
            bundle.extractall(destination, members=members, filter="data")
        else:  # pragma: no cover - exercised by the Python 3.11 CI lane
            bundle.extractall(destination, members=members)


def _verify_tree(root: Path, manifest: dict[str, Any]) -> None:
    expected = {str(entry["path"]): entry for entry in manifest["files"]}
    actual_files = {
        path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()
    }
    if actual_files != set(expected):
        missing = sorted(set(expected) - actual_files)[:10]
        unexpected = sorted(actual_files - set(expected))[:10]
        raise ValueError(
            f"dataset file set mismatch; missing={missing}, unexpected={unexpected}"
        )
    for relative, entry in expected.items():
        path = root / relative
        if path.stat().st_size != int(entry["size"]):
            raise ValueError(f"dataset size mismatch: {relative}")
        if sha256_file(path) != entry["sha256"]:
            raise ValueError(f"dataset checksum mismatch: {relative}")


def stage_dataset(
    *,
    bucket: str,
    dataset_id: str,
    destination_root: Path,
    aws: AwsCli,
) -> Path:
    """Download, verify, safely extract, and atomically publish one dataset."""
    destination = destination_root / dataset_id
    published_manifest = destination / ".dataset-manifest.json"
    manifest_uri = f"s3://{bucket}/datasets/{dataset_id}/dataset-manifest.json"
    print(f"Dataset staging: downloading manifest for {dataset_id}", flush=True)
    destination_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{dataset_id}.", dir=destination_root
    ) as temporary_name:
        temporary = Path(temporary_name)
        manifest_path = temporary / "dataset-manifest.json"
        aws.download(manifest_uri, manifest_path)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        validate_manifest(manifest, dataset_id)
        manifest_digest = sha256_file(manifest_path)
        if destination.is_dir() and published_manifest.is_file():
            if sha256_file(published_manifest) == manifest_digest:
                print(f"Dataset staging: reusing verified {destination}", flush=True)
                return destination
            raise ValueError("existing dataset directory has a different manifest")

        archive_size = int(manifest["archive"]["size"])
        required = required_staging_bytes(manifest)
        free = shutil.disk_usage(destination_root).free
        if free < required:
            raise OSError(
                f"insufficient staging disk: need {required} bytes, have {free}"
            )
        archive = temporary / "dataset.tar"
        print(
            "Dataset staging: downloading "
            f"{archive_size / 1024**3:.1f} GiB archive for {dataset_id}",
            flush=True,
        )
        aws.download(f"s3://{bucket}/{manifest['archive']['key']}", archive)
        if archive.stat().st_size != archive_size:
            raise ValueError("dataset archive size mismatch")
        if sha256_file(archive) != manifest["archive"]["sha256"]:
            raise ValueError("dataset archive checksum mismatch")
        extracted = temporary / "extracted"
        extracted.mkdir()
        print(
            f"Dataset staging: extracting {len(manifest['files'])} files",
            flush=True,
        )
        _safe_extract(archive, extracted)
        print("Dataset staging: verifying extracted files", flush=True)
        _verify_tree(extracted, manifest)
        shutil.copy2(manifest_path, extracted / ".dataset-manifest.json")
        os.replace(extracted, destination)
        print(f"Dataset staging: published verified {destination}", flush=True)
    return destination


__all__ = [
    "AwsCli",
    "MANIFEST_SCHEMA",
    "download_manifest",
    "required_staging_bytes",
    "sha256_file",
    "stage_dataset",
    "validate_manifest",
]
