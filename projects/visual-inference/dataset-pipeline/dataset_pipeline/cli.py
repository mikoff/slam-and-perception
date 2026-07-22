from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path
from typing import Any, Callable

from .archives import extract_all, inspect_archives
from .cleanup import cleanup, disk_usage
from .coco_export import export_all
from .coco_merge import merge_exports
from .config import ensure_workspace, load_config
from .detection_conversion import convert_all
from .discovery import discover_all, inspect_projects
from .preview import generate_previews
from .reports import runtime_info, write_json
from .supervisely_filter import filter_all
from .taxonomy import Taxonomy
from .validation import validate_all, verify_final_links


PIPELINE_COMMANDS = (
    "inspect-archives", "extract", "discover", "filter",
    "convert-detection", "export-coco", "merge", "validate", "preview",
)
COMMANDS = PIPELINE_COMMANDS + ("inspect-projects", "verify-links", "disk-usage", "cleanup", "all")


def _positive_int(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Storage-efficient Supervisely automotive dataset conversion")
    parser.add_argument("command", choices=COMMANDS)
    parser.add_argument("--config", type=Path, default=Path("configs/datasets.yaml"))
    parser.add_argument("--dataset")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--limit-images", type=_positive_int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--force-extract", action="store_true")
    parser.add_argument("--verify-archives", action="store_true",
                        help="ignore cached archive inspections and recalculate checksums")
    parser.add_argument("--workers", type=_positive_int, default=2,
                        help="worker processes for annotation transforms (default: 2)")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    return parser


def _run(command: str, args: argparse.Namespace) -> Any:
    config = load_config(args.config)
    ensure_workspace(config)
    taxonomy = Taxonomy.load(config.taxonomy_path)
    write_json(config.reports / "runtime.json", runtime_info(config.workspace_root))
    archive_inspections: dict[str, dict[str, Any]] = {}

    def inspect() -> list[dict[str, Any]]:
        reports = inspect_archives(
            config, args.dataset,
            reuse_cached=command == "all" and not args.verify_archives,
        )
        archive_inspections.update({item["dataset"]: item for item in reports})
        return reports

    operations: dict[str, Callable[[], Any]] = {
        "inspect-archives": inspect,
        "extract": lambda: extract_all(
            config, args.dataset, args.force_extract, archive_inspections, args.verify_archives,
        ),
        "discover": lambda: discover_all(config, args.dataset),
        "inspect-projects": lambda: inspect_projects(config, taxonomy, args.dataset),
        "filter": lambda: filter_all(
            config, taxonomy, args.dataset, args.limit_images, args.force, args.dry_run, args.workers,
        ),
        "convert-detection": lambda: convert_all(config, args.dataset, args.force, args.workers),
        "export-coco": lambda: export_all(config, taxonomy, args.dataset),
        "merge": lambda: merge_exports(config, taxonomy, args.force),
        "validate": lambda: validate_all(config, taxonomy),
        "preview": lambda: generate_previews(config),
        "verify-links": lambda: verify_final_links(config),
        "disk-usage": lambda: disk_usage(config),
        "cleanup": lambda: cleanup(config, args.dry_run),
    }

    def run_stage(name: str) -> Any:
        logging.info("Running %s", name)
        started = time.monotonic()
        result = operations[name]()
        logging.info("Finished %s in %.1fs", name, time.monotonic() - started)
        return result

    if command != "all":
        return run_stage(command)
    results = {}
    logging.info("Using %d annotation workers", args.workers)
    for name in PIPELINE_COMMANDS:
        results[name] = run_stage(name)
    if config.storage["delete_intermediate_projects_after_export"]:
        results["cleanup"] = run_stage("cleanup")
    return results


def main(argv: list[str] | None = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.dry_run and args.command not in {"filter", "cleanup"}:
        parser.error("--dry-run is supported only by filter and cleanup")
    logging.basicConfig(level=args.log_level, format="%(levelname)s %(message)s")
    result = _run(args.command, args)
    if result is not None:
        logging.info("Completed %s", args.command)
