"""Stream an immutable dataset bundle to S3 without local archive space."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path
from typing import BinaryIO

if __package__:
    from .dataset_staging import MANIFEST_SCHEMA, sha256_file
else:
    from dataset_staging import MANIFEST_SCHEMA, sha256_file

SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


class _DigestWriter:
    def __init__(self, destination: BinaryIO) -> None:
        self.destination = destination
        self.digest = hashlib.sha256()
        self.size = 0

    def write(self, data: bytes) -> int:
        written = self.destination.write(data)
        if written is None:
            written = len(data)
        self.digest.update(data[:written])
        self.size += written
        return written

    def flush(self) -> None:
        self.destination.flush()


def dataset_files(root: Path) -> list[Path]:
    """Return regular files, allowing file symlinks that will be dereferenced."""
    files = sorted(path for path in root.rglob("*") if path.is_file())
    if not files:
        raise ValueError("dataset root contains no files")
    for path in files:
        resolved = path.resolve(strict=True)
        if not resolved.is_file():
            raise ValueError(f"dataset entry is not a regular file: {path}")
    for required in ("indexes/quad_train.sqlite", "indexes/quad_val.sqlite"):
        if not (root / required).is_file():
            raise FileNotFoundError(f"prebuilt cloud index is missing: {required}")
    return files


def file_records(root: Path, files: list[Path]) -> list[dict[str, str | int]]:
    records: list[dict[str, str | int]] = []
    for index, path in enumerate(files, start=1):
        records.append(
            {
                "path": path.relative_to(root).as_posix(),
                "size": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
        if index % 10_000 == 0:
            print(f"Hashed {index}/{len(files)} files", flush=True)
    return records


def write_archive(
    root: Path, files: list[Path], destination: BinaryIO
) -> tuple[int, str]:
    """Write a safe tar.gz stream, replacing symlinks with regular file bytes."""
    writer = _DigestWriter(destination)
    with tarfile.open(fileobj=writer, mode="w|gz") as bundle:
        bundle.dereference = True
        for index, path in enumerate(files, start=1):
            relative = path.relative_to(root).as_posix()
            info = bundle.gettarinfo(str(path), arcname=relative)
            if not info.isreg():
                raise ValueError(f"dataset archive entry is not regular: {relative}")
            with path.open("rb") as stream:
                bundle.addfile(info, stream)
            if index % 10_000 == 0:
                print(f"Uploaded archive input {index}/{len(files)} files", flush=True)
    writer.flush()
    return writer.size, writer.digest.hexdigest()


def _aws_command(endpoint: str | None, *arguments: str) -> list[str]:
    command = ["aws", *arguments]
    if endpoint:
        command.extend(["--endpoint-url", endpoint])
    return command


def _object_exists(bucket: str, key: str, endpoint: str | None) -> bool:
    result = subprocess.run(
        _aws_command(
            endpoint, "s3api", "head-object", "--bucket", bucket, "--key", key
        ),
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    if result.returncode == 0:
        return True
    details = result.stderr.lower()
    if any(marker in details for marker in ("404", "not found", "nosuchkey")):
        return False
    raise RuntimeError(f"could not check s3://{bucket}/{key}: {result.stderr.strip()}")


def upload_bundle(
    *,
    root: Path,
    dataset_id: str,
    bucket: str,
    endpoint: str | None,
    manifest_output: Path,
) -> Path:
    """Hash files, stream the archive, then publish its verified manifest."""
    if SAFE_ID.fullmatch(dataset_id) is None:
        raise ValueError("dataset ID contains unsupported characters")
    if shutil.which("aws") is None:
        raise FileNotFoundError("aws CLI is required")
    root = root.resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    archive_key = f"datasets/{dataset_id}/dataset.tar.gz"
    manifest_key = f"datasets/{dataset_id}/dataset-manifest.json"
    for key in (archive_key, manifest_key):
        if _object_exists(bucket, key, endpoint):
            raise FileExistsError(f"immutable S3 object already exists: {key}")

    files = dataset_files(root)
    records = file_records(root, files)
    expected_size = sum(int(record["size"]) for record in records)
    command = _aws_command(
        endpoint,
        "s3",
        "cp",
        "-",
        f"s3://{bucket}/{archive_key}",
        "--no-progress",
        "--expected-size",
        str(expected_size),
    )
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    if process.stdin is None:
        raise RuntimeError("AWS upload process has no stdin")
    try:
        archive_size, archive_sha256 = write_archive(root, files, process.stdin)
        process.stdin.close()
        return_code = process.wait()
    except BaseException:
        process.kill()
        process.wait()
        raise
    if return_code != 0:
        raise subprocess.CalledProcessError(return_code, command)

    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "dataset_id": dataset_id,
        "archive": {
            "key": archive_key,
            "size": archive_size,
            "sha256": archive_sha256,
        },
        "files": records,
    }
    manifest_output = manifest_output.resolve()
    manifest_output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", suffix=".json", dir=manifest_output.parent, encoding="utf-8", delete=False
    ) as stream:
        staged = Path(stream.name)
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    try:
        os.replace(staged, manifest_output)
    finally:
        staged.unlink(missing_ok=True)
    subprocess.run(
        _aws_command(
            endpoint,
            "s3",
            "cp",
            str(manifest_output),
            f"s3://{bucket}/{manifest_key}",
            "--no-progress",
        ),
        check=True,
    )
    return manifest_output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--bucket", required=True)
    parser.add_argument("--endpoint")
    parser.add_argument("--manifest-output", type=Path, required=True)
    args = parser.parse_args()
    manifest = upload_bundle(
        root=args.root,
        dataset_id=args.dataset_id,
        bucket=args.bucket,
        endpoint=args.endpoint,
        manifest_output=args.manifest_output,
    )
    print(f"Uploaded immutable dataset {args.dataset_id}")
    print(f"Saved local manifest {manifest}")


if __name__ == "__main__":
    main()
