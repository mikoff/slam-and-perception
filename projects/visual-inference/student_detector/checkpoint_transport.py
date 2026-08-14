"""Serialized, checksum-verified S3 transport for training checkpoints."""

from __future__ import annotations

import hashlib
import json
import os
import queue
import shutil
import subprocess
import tempfile
import threading
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol

MANIFEST_SCHEMA = "visual-inference-checkpoints.v1"
CHECKPOINT_TRANSFER_TIMEOUT_SECONDS = 300
_NOT_FOUND_MARKERS = ("404", "not found", "nosuchkey", "no such key")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class CheckpointObject:
    key: str
    size: int
    sha256: str
    global_step: int


@dataclass(frozen=True)
class ResolvedCheckpoint:
    """A verified checkpoint plus the immutable contract of its source run."""

    path: Path
    run_id: str
    contract: dict[str, str]


class ObjectStore(Protocol):
    def upload(self, source: Path, key: str) -> None: ...
    def download(self, key: str, destination: Path) -> bool: ...


class AwsCheckpointStore:
    def __init__(self, bucket: str, endpoint: str | None = None) -> None:
        self.bucket = bucket
        self.endpoint = endpoint

    def _command(self, *arguments: str) -> list[str]:
        command = ["aws", *arguments]
        if self.endpoint:
            command.extend(["--endpoint-url", self.endpoint])
        return command

    def upload(self, source: Path, key: str) -> None:
        subprocess.run(
            self._command(
                "s3", "cp", str(source), f"s3://{self.bucket}/{key}", "--no-progress"
            ),
            check=True,
            timeout=CHECKPOINT_TRANSFER_TIMEOUT_SECONDS,
        )

    def download(self, key: str, destination: Path) -> bool:
        """Download one object, returning false only for an authoritative miss."""
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            result = subprocess.run(
                self._command(
                    "s3",
                    "cp",
                    f"s3://{self.bucket}/{key}",
                    str(destination),
                    "--no-progress",
                ),
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=CHECKPOINT_TRANSFER_TIMEOUT_SECONDS,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            destination.unlink(missing_ok=True)
            raise RuntimeError(
                f"checkpoint download failed for s3://{self.bucket}/{key}"
            ) from error
        if result.returncode == 0:
            return True
        destination.unlink(missing_ok=True)
        details = result.stderr.strip()
        if any(marker in details.lower() for marker in _NOT_FOUND_MARKERS):
            return False
        raise RuntimeError(
            f"checkpoint download failed for s3://{self.bucket}/{key}: "
            f"{details or f'AWS CLI exited {result.returncode}'}"
        )


def _manifest_key(run_id: str) -> str:
    return f"runs/{run_id}/checkpoints/latest.json"


def resolve_resume_checkpoint(
    *,
    run_id: str,
    output_dir: Path,
    store: ObjectStore,
    expected_contract: dict[str, str] | None = None,
) -> ResolvedCheckpoint | None:
    """Download the newest valid checkpoint, falling back through history."""
    with tempfile.TemporaryDirectory(prefix="checkpoint-resume-") as temporary_name:
        temporary = Path(temporary_name)
        manifest_path = temporary / "latest.json"
        if not store.download(_manifest_key(run_id), manifest_path):
            return None
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("schema_version") != MANIFEST_SCHEMA:
            raise ValueError("unsupported remote checkpoint manifest")
        if manifest.get("run_id") != run_id:
            raise ValueError("remote checkpoint manifest run ID mismatch")
        for key, expected in (expected_contract or {}).items():
            actual = manifest.get("contract", {}).get(key)
            if expected and actual != expected:
                raise ValueError(
                    f"remote checkpoint contract mismatch for {key}: "
                    f"expected {expected!r}, found {actual!r}"
                )
        candidates = [manifest.get("latest"), *manifest.get("previous", [])]
        for value in candidates:
            if not isinstance(value, dict):
                continue
            candidate = CheckpointObject(**value)
            staged = temporary / f"{candidate.global_step}.pt"
            if not store.download(candidate.key, staged):
                continue
            if (
                staged.stat().st_size != candidate.size
                or _sha256(staged) != candidate.sha256
            ):
                continue
            destination = output_dir / "resume.pt"
            destination.parent.mkdir(parents=True, exist_ok=True)
            os.replace(staged, destination)
            contract = manifest.get("contract", {})
            if not isinstance(contract, dict) or not all(
                isinstance(key, str) and isinstance(value, str)
                for key, value in contract.items()
            ):
                raise ValueError("remote checkpoint manifest contract is invalid")
            return ResolvedCheckpoint(destination, run_id, dict(contract))
    raise RuntimeError("remote checkpoint manifest contains no valid checkpoint")


def resolve_auto_resume(
    *,
    run_id: str,
    output_dir: Path,
    store: ObjectStore,
    expected_contract: dict[str, str] | None = None,
) -> Path | None:
    """Compatibility wrapper returning only the verified checkpoint path."""
    resolved = resolve_resume_checkpoint(
        run_id=run_id,
        output_dir=output_dir,
        store=store,
        expected_contract=expected_contract,
    )
    return resolved.path if resolved is not None else None


@dataclass(frozen=True)
class _Upload:
    snapshot: Path
    global_step: int
    logical_name: str


class CheckpointUploader:
    """One bounded background writer; latest is published only after data."""

    def __init__(
        self,
        *,
        run_id: str,
        output_dir: Path,
        store: ObjectStore,
        contract: dict[str, str] | None = None,
    ) -> None:
        self.run_id = run_id
        self.output_dir = output_dir
        self.store = store
        self.contract = contract or {}
        self.staging = output_dir / ".checkpoint_uploads"
        self.staging.mkdir(parents=True, exist_ok=True)
        self._queue: queue.Queue[_Upload | None] = queue.Queue(maxsize=16)
        self._failure: BaseException | None = None
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def enqueue(
        self, source: Path, global_step: int, *, logical_name: str = "last.pt"
    ) -> None:
        if self._failure is not None:
            raise RuntimeError("checkpoint upload worker failed") from self._failure
        snapshot = self.staging / (
            f"{Path(logical_name).stem}-step-{global_step}-{os.urandom(4).hex()}.pt"
        )
        shutil.copy2(source, snapshot)
        upload = _Upload(snapshot, global_step, logical_name)
        try:
            self._queue.put_nowait(upload)
        except queue.Full:
            obsolete = self._queue.get_nowait()
            if obsolete is not None:
                obsolete.snapshot.unlink(missing_ok=True)
                self._queue.task_done()
            self._queue.put_nowait(upload)

    @property
    def pending(self) -> int:
        return self._queue.qsize()

    def _read_manifest(self) -> dict[str, Any]:
        with tempfile.TemporaryDirectory(prefix="checkpoint-manifest-") as temporary:
            target = Path(temporary) / "latest.json"
            if not self.store.download(_manifest_key(self.run_id), target):
                return {
                    "schema_version": MANIFEST_SCHEMA,
                    "run_id": self.run_id,
                    "contract": self.contract,
                    "previous": [],
                }
            value = json.loads(target.read_text(encoding="utf-8"))
            if value.get("schema_version") != MANIFEST_SCHEMA:
                raise ValueError("existing checkpoint manifest has incompatible schema")
            if value.get("run_id") != self.run_id:
                raise ValueError("existing checkpoint manifest run ID mismatch")
            if value.get("contract", {}) != self.contract:
                raise ValueError("existing checkpoint manifest contract mismatch")
            return value

    def _upload(self, item: _Upload) -> None:
        digest = _sha256(item.snapshot)
        kind = "last" if item.logical_name == "last.pt" else "best"
        record = CheckpointObject(
            key=(
                f"runs/{self.run_id}/checkpoints/{kind}/"
                f"{Path(item.logical_name).stem}-step-{item.global_step}-{digest[:16]}.pt"
            ),
            size=item.snapshot.stat().st_size,
            sha256=digest,
            global_step=item.global_step,
        )
        self.store.upload(item.snapshot, record.key)
        if kind == "best":
            alias_key = (
                f"runs/{self.run_id}/checkpoints/aliases/{item.logical_name}.json"
            )
            with tempfile.NamedTemporaryFile(
                "w", suffix=".json", encoding="utf-8"
            ) as stream:
                json.dump(asdict(record), stream, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                self.store.upload(Path(stream.name), alias_key)
            return
        manifest = self._read_manifest()
        history = (
            [manifest["latest"]] if isinstance(manifest.get("latest"), dict) else []
        )
        history.extend(manifest.get("previous", []))
        deduplicated = []
        for previous in history:
            if previous.get("sha256") != record.sha256:
                deduplicated.append(previous)
        manifest.update(
            {
                "schema_version": MANIFEST_SCHEMA,
                "run_id": self.run_id,
                "contract": self.contract,
                "updated_at": datetime.now(UTC).isoformat(),
                "latest": asdict(record),
                "previous": deduplicated[:2],
            }
        )
        with tempfile.NamedTemporaryFile(
            "w", suffix=".json", encoding="utf-8"
        ) as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            self.store.upload(Path(stream.name), _manifest_key(self.run_id))

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                self._upload(item)
            except BaseException as error:
                self._failure = error
            finally:
                if item is not None:
                    item.snapshot.unlink(missing_ok=True)
                self._queue.task_done()

    def close(self) -> None:
        self._queue.put(None)
        self._queue.join()
        self._thread.join()
        if self._failure is not None:
            raise RuntimeError("checkpoint upload failed") from self._failure


def uploader_from_environment(
    output_dir: Path, run_id: str | None
) -> CheckpointUploader | None:
    bucket = os.getenv("S3_BUCKET")
    if not bucket or not run_id:
        return None
    contract = {
        "source_commit": os.getenv("SOURCE_COMMIT", ""),
        "dataset_id": os.getenv("DATASET_ID", ""),
        "dataset_manifest_sha256": os.getenv("DATASET_MANIFEST_SHA256", ""),
        "config_path": os.getenv("CONFIG_PATH", ""),
    }
    if parent_run_id := os.getenv("RESUME_FROM_RUN_ID", ""):
        contract["resume_from_run_id"] = parent_run_id
    return CheckpointUploader(
        run_id=run_id,
        output_dir=output_dir,
        store=AwsCheckpointStore(bucket, os.getenv("S3_ENDPOINT_URL") or None),
        contract=contract,
    )


__all__ = [
    "AwsCheckpointStore",
    "CheckpointObject",
    "CheckpointUploader",
    "MANIFEST_SCHEMA",
    "ResolvedCheckpoint",
    "resolve_auto_resume",
    "resolve_resume_checkpoint",
    "uploader_from_environment",
]
