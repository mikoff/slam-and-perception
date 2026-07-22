from __future__ import annotations

import hashlib
import logging
import os
import shutil
import tarfile
import tempfile
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO

from .config import Config, DatasetConfig
from .progress import Progress
from .reports import read_json, write_json


class UnsafeArchiveError(RuntimeError):
    pass


def sha256(path: Path, label: str | None = None) -> str:
    digest = hashlib.sha256()
    progress = Progress(label or f"Hashing {path.name}", "MiB")
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
            progress.add(len(block) // (1024 * 1024))
    progress.finish()
    return digest.hexdigest()


def archive_format(path: Path) -> str:
    lower = path.name.lower()
    if lower.endswith(".tar.zst"):
        return "tar.zst"
    if lower.endswith((".tar.gz", ".tgz")):
        return "tar.gz"
    if lower.endswith(".tar"):
        return "tar"
    return "unsupported"


def _unsafe_reason(member: tarfile.TarInfo) -> str | None:
    pure = PurePosixPath(member.name)
    if pure.is_absolute() or member.name.startswith(("/", "\\")):
        return "absolute_path"
    if ".." in pure.parts:
        return "path_traversal"
    if member.isdev() or member.isfifo():
        return "device_or_fifo"
    if member.issym() or member.islnk():
        target = PurePosixPath(member.linkname)
        parent = pure.parent
        combined = parent / target
        depth = 0
        if target.is_absolute():
            return "absolute_link"
        for part in combined.parts:
            depth += -1 if part == ".." else (0 if part in ("", ".") else 1)
            if depth < 0:
                return "escaping_link"
    return None


def _open_tar(path: Path) -> tuple[tarfile.TarFile, str, BinaryIO | None]:
    if archive_format(path) != "tar.zst":
        return tarfile.open(path, mode="r:*"), "python-tarfile", None
    try:
        import zstandard
    except ImportError as exc:
        raise RuntimeError("zstandard is required for .tar.zst archives") from exc
    raw = path.open("rb")
    reader = zstandard.ZstdDecompressor().stream_reader(raw)
    return tarfile.open(fileobj=reader, mode="r|"), "python-tarfile+zstandard", raw


def inspect_archive(dataset: DatasetConfig) -> dict[str, Any]:
    path = dataset.archive
    logging.info("Inspecting archive %s", dataset.name)
    info = path.stat() if path.exists() else None
    report: dict[str, Any] = {
        "dataset": dataset.name, "archive": str(path), "exists": path.exists(),
        "readable": os.access(path, os.R_OK), "format": archive_format(path),
        "archive_mtime_ns": info.st_mtime_ns if info else None,
    }
    if not path.is_file() or report["format"] == "unsupported":
        report["error"] = "archive missing or unsupported"
        return report
    report.update({"compressed_size": path.stat().st_size, "sha256": sha256(path)})
    suspicious: list[dict[str, str]] = []
    member_count = 0
    contains_meta = False
    top_levels: set[str] = set()
    estimated = 0
    try:
        archive, backend, raw = _open_tar(path)
        progress = Progress(f"Inspecting {dataset.name}", "members")
        try:
            for member in archive:
                member_count += 1
                progress.add()
                contains_meta = contains_meta or PurePosixPath(member.name).name == "meta.json"
                top_levels.add(PurePosixPath(member.name).parts[0] if PurePosixPath(member.name).parts else "")
                estimated += max(0, member.size)
                reason = _unsafe_reason(member)
                if reason:
                    suspicious.append({"path": member.name, "reason": reason})
        finally:
            archive.close()
            if raw:
                raw.close()
        progress.finish()
        report.update({
            "backend": backend, "member_count": member_count,
            "top_level_directories": sorted(top_levels),
            "contains_meta_json": contains_meta,
            "estimated_extracted_size": estimated, "suspicious_entries": suspicious,
        })
    except (tarfile.TarError, OSError, RuntimeError) as exc:
        report["error"] = str(exc)
    return report


def _cached_inspection(dataset: DatasetConfig, cached: dict[str, Any]) -> bool:
    try:
        info = dataset.archive.stat()
        return (
            cached.get("archive") == str(dataset.archive)
            and cached.get("compressed_size") == info.st_size
            and cached.get("archive_mtime_ns") == info.st_mtime_ns
            and not cached.get("error")
        )
    except OSError:
        return False


def inspect_archives(
    config: Config, dataset_name: str | None = None, reuse_cached: bool = False,
) -> list[dict[str, Any]]:
    cached_by_name: dict[str, dict[str, Any]] = {}
    report_path = config.reports / "archive_inventory.json"
    if reuse_cached and report_path.exists():
        try:
            cached_by_name = {item["dataset"]: item for item in read_json(report_path)}
        except (OSError, ValueError, KeyError, TypeError):
            pass
    result = []
    for item in config.selected(dataset_name):
        cached = cached_by_name.get(item.name, {})
        if cached and _cached_inspection(item, cached):
            logging.info("Reusing archive inspection for %s", item.name)
            result.append(cached)
        else:
            result.append(inspect_archive(item))
    write_json(config.reports / "archive_inventory.json", result)
    return result


def extract_one(
    dataset: DatasetConfig, force: bool = False, inspection: dict[str, Any] | None = None,
    verify: bool = False,
) -> dict[str, Any]:
    if verify and inspection is None:
        inspection = inspect_archive(dataset)
    if inspection and (inspection.get("error") or inspection.get("suspicious_entries")):
        raise UnsafeArchiveError(f"Archive failed inspection: {inspection}")

    destination = dataset.extracted_dir
    manifest_path = destination / ".dataset_pipeline_manifest.json"
    complete = destination / ".extraction_complete"
    if destination.exists() and manifest_path.is_file() and complete.is_file():
        current = read_json(manifest_path)
        info = dataset.archive.stat()
        identity_unchanged = (
            current.get("canonical_archive_path") == str(dataset.archive.resolve())
            and current.get("archive_size") == info.st_size
            and current.get("archive_mtime_ns") == info.st_mtime_ns
        )
        inspected_digest = inspection.get("sha256") if inspection else None
        digest_matches = not verify or current.get("archive_sha256") == (inspected_digest or sha256(dataset.archive))
        unchanged = identity_unchanged and digest_matches
        if unchanged:
            return {**current, "status": "reused"}
        if not force:
            raise RuntimeError(f"Existing extraction fingerprint changed: {destination}; use --force-extract")
    elif destination.exists() and not force:
        raise RuntimeError(f"Existing extraction is incomplete: {destination}; use --force-extract")
    inspection = inspection or inspect_archive(dataset)
    if inspection.get("error") or inspection.get("suspicious_entries"):
        raise UnsafeArchiveError(f"Archive failed inspection: {inspection}")
    archive_stat = dataset.archive.stat()
    wanted = {
        "canonical_archive_path": str(dataset.archive.resolve()),
        "archive_sha256": inspection["sha256"],
        "archive_size": archive_stat.st_size,
        "archive_mtime_ns": archive_stat.st_mtime_ns,
    }
    temp_parent = destination.parent
    temp_parent.mkdir(parents=True, exist_ok=True)
    temp = Path(tempfile.mkdtemp(prefix=f".{destination.name}.extract-", dir=temp_parent))
    try:
        logging.info("Extracting archive %s", dataset.name)
        archive, backend, raw = _open_tar(dataset.archive)
        progress = Progress(f"Extracting {dataset.name}", "members")
        try:
            if hasattr(tarfile, "data_filter"):
                def members():
                    for member in archive:
                        progress.add()
                        yield member

                archive.extractall(temp, members=members(), filter="data")
            else:
                members = list(archive)
                for member in members:
                    progress.add()
                    reason = _unsafe_reason(member)
                    if reason:
                        raise UnsafeArchiveError(f"{member.name}: {reason}")
                archive.extractall(temp, members=members)
        finally:
            archive.close()
            if raw:
                raw.close()
        progress.finish()
        image_count = sum(1 for p in temp.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"} and p.is_file())
        annotation_count = sum(1 for p in temp.rglob("*.json") if p.parent.name == "ann")
        source_classes: list[str] = []
        for meta in temp.rglob("meta.json"):
            try:
                data = read_json(meta)
                source_classes.extend(item["title"] for item in data.get("classes", []))
            except (OSError, ValueError, KeyError):
                pass
        manifest = {
            **wanted, "extraction_timestamp": datetime.now(timezone.utc).isoformat(),
            "extraction_backend": backend, "discovered_project_root": None,
            "image_count": image_count, "annotation_count": annotation_count,
            "source_class_list": sorted(set(source_classes)),
        }
        write_json(temp / ".dataset_pipeline_manifest.json", manifest)
        (temp / ".extraction_complete").write_text("complete\n", encoding="utf-8")
        if destination.exists():
            shutil.rmtree(destination)
        temp.rename(destination)
        return {**manifest, "status": "extracted"}
    except Exception:
        shutil.rmtree(temp, ignore_errors=True)
        raise


def extract_all(
    config: Config, dataset_name: str | None = None, force: bool = False,
    inspections: dict[str, dict[str, Any]] | None = None, verify: bool = False,
) -> list[dict[str, Any]]:
    inspections = inspections or {}
    return [extract_one(item, force, inspections.get(item.name), verify) for item in config.selected(dataset_name)]
