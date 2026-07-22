from __future__ import annotations

import csv
import json
import os
import platform
import sys
from importlib import metadata
from pathlib import Path
from typing import Any, Iterable


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any, *, compact: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(
                value, stream,
                indent=None if compact else 2,
                separators=(",", ":") if compact else None,
                sort_keys=True,
                ensure_ascii=False,
            )
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def runtime_info(path: Path | None = None) -> dict[str, Any]:
    def version(package: str) -> str:
        try:
            return metadata.version(package)
        except metadata.PackageNotFoundError:
            return "not-installed"
    result = {
        "python": sys.version,
        "supervisely": version("supervisely"),
        "pycocotools": version("pycocotools"),
        "operating_system": platform.platform(),
    }
    if path is not None:
        try:
            result["filesystem_device"] = os.stat(path if path.exists() else path.parent).st_dev
        except OSError:
            result["filesystem_device"] = "unknown"
    return result
