#!/usr/bin/env python3
"""Hash-bind and exhaustively audit a generated proposal dataset.

The cheap binding gate runs before the expensive image/region scan. A stale or
unbound candidate is reported, never promoted by an audit of the wrong bytes.
"""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
import sqlite3
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import ijson
from PIL import Image, ImageDraw
from shapely import Polygon, union_all


EXPECTED_SCHEMA = "proposal-manifest.v2"
STATES = {0: "positive", 1: "ignore", 2: "trusted_background"}
SIZE_EDGES = (0, 4, 8, 16, 32, 64, 128, 256, math.inf)
TIGHTNESS_EDGES = (0, 0.5, 0.8, 0.95, 0.98, 1.000001)


def sha256_file(path: Path) -> str:
    """Return a streaming SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def manifest_scalar(path: Path, name: str) -> Any:
    """Read a sorted top-level trailer scalar without scanning the images array."""
    trailer_bytes = min(path.stat().st_size, 64 * 1024)
    with path.open("rb") as stream:
        stream.seek(-trailer_bytes, 2)
        trailer = stream.read().decode("utf-8")
    token = f"{json.dumps(name)}:"
    position = trailer.rfind(token)
    if position >= 0:
        value, _ = json.JSONDecoder().raw_decode(
            trailer[position + len(token) :].lstrip()
        )
        if not isinstance(value, (dict, list)):
            return value
    # Retain a safe fallback for externally produced, non-sorted manifests.
    with path.open("rb") as stream:
        for prefix, event, value in ijson.parse(stream):
            if prefix == name and event in {"string", "number", "boolean", "null"}:
                return value
    return None


def _bin(value: float, edges: tuple[float, ...]) -> str:
    for lower, upper in zip(edges, edges[1:]):
        if lower <= value < upper:
            return f"[{lower:g},{upper:g})"
    return "invalid"


def _index_metadata(path: Path) -> dict[str, str]:
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
        return dict(connection.execute("SELECT key, value FROM metadata"))


def _tree_hash(paths: Iterable[Path], root: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    entries = []
    for path in sorted(set(path.resolve() for path in paths)):
        file_hash = sha256_file(path)
        try:
            label = path.relative_to(root.resolve()).as_posix()
        except ValueError:
            label = str(path)
        digest.update(label.encode())
        digest.update(b"\0")
        digest.update(file_hash.encode())
        entries.append(
            {"path": label, "sha256": file_hash, "bytes": path.stat().st_size}
        )
    return {"sha256": digest.hexdigest(), "files": entries}


def binding_preflight(project: Path, workspace: Path) -> dict[str, Any]:
    """Bind manifests, indexes, policy, split decision, and cached raw versions."""
    annotations = workspace / "output/annotations"
    indexes = workspace / "output/indexes"
    manifests = {
        split: annotations / f"proposals_{split}.json" for split in ("train", "val")
    }
    index_paths = {
        split: indexes / f"quad_{split}.sqlite" for split in ("train", "val")
    }
    failures: list[str] = []
    contract_path = workspace / "dataset_contract.json"
    contract = json.loads(contract_path.read_text())
    if contract.get("dataset_id") != workspace.name:
        failures.append("dataset contract identifier does not match workspace name")
    if contract.get("schema_version") != "phase3-dataset-contract.v1":
        failures.append("dataset contract schema is not phase3-dataset-contract.v1")
    manifest_info: dict[str, Any] = {}
    for split, path in manifests.items():
        schema = manifest_scalar(path, "schema_version")
        declared_split = manifest_scalar(path, "split")
        digest = sha256_file(path)
        metadata = _index_metadata(index_paths[split])
        source_signature = metadata.get("source_signature", "")
        expected_signature = f"sha256:{digest}"
        # Historical indexes used a stat signature, so equality is deliberately strict.
        manifest_info[split] = {
            "path": str(path.resolve()),
            "bytes": path.stat().st_size,
            "sha256": digest,
            "schema_version": schema,
            "declared_split": declared_split,
            "index_path": str(index_paths[split].resolve()),
            "index_sha256": sha256_file(index_paths[split]),
            "index_source_signature": source_signature,
            "cryptographically_bound_index": source_signature == expected_signature,
        }
        if schema != EXPECTED_SCHEMA:
            failures.append(
                f"{split} manifest schema is {schema!r}, expected {EXPECTED_SCHEMA!r}"
            )
        if declared_split != split:
            failures.append(f"{split} manifest declares split {declared_split!r}")
        if source_signature != expected_signature:
            failures.append(f"{split} index is not bound to the manifest SHA-256")
        expected_manifest = contract.get("artifacts", {}).get(
            f"proposals_{split}.json", {}
        )
        expected_index = contract.get("artifacts", {}).get(f"quad_{split}.sqlite", {})
        if expected_manifest.get("sha256") != digest:
            failures.append(f"{split} manifest does not match the dataset contract")
        if expected_index.get("sha256") != manifest_info[split]["index_sha256"]:
            failures.append(f"{split} index does not match the dataset contract")

    split_path = workspace / "reports/woodscape_sequence_split.json"
    if not split_path.is_file():
        split_path = (
            workspace
            / "reports/woodscape_supervision_audit_phase_1_4/woodscape_sequence_split.json"
        )
    split_decision = json.loads(split_path.read_text())
    current_split = json.loads((workspace / "reports/split_report.json").read_text())
    soiling_train_images = sum(
        1
        for path in (workspace / "raw/woodscape_rgb_fisheye/train/img").glob(
            "soiling_*"
        )
        if path.is_file()
    )
    expected_woodscape = {
        "train": int(split_decision["train_rgb_images"]) + soiling_train_images,
        "val": int(split_decision["validation_rgb_images"]),
        "excluded_test": int(split_decision["qualitative_test_images"]),
    }
    if current_split.get("woodscape_rgb_fisheye") != expected_woodscape:
        failures.append(
            "generated WoodScape split does not match the approved RGB, soiling, and qualitative-test policy"
        )

    inventory_path = workspace / "reports/archive_inventory.json"
    inventory = json.loads(inventory_path.read_text())
    raw_sources = []
    for item in inventory:
        archive = Path(item["archive"])
        stat_matches = (
            archive.exists()
            and archive.stat().st_size == item["compressed_size"]
            and archive.stat().st_mtime_ns == item["archive_mtime_ns"]
        )
        raw_sources.append(
            {
                "dataset": item["dataset"],
                "path": str(archive),
                "sha256": item["sha256"],
                "bytes": item["compressed_size"],
                "mtime_ns": item["archive_mtime_ns"],
                "cached_identity_stat_matches": stat_matches,
            }
        )
        if not stat_matches:
            reason = "is unavailable" if not archive.exists() else "stat changed"
            failures.append(
                f"raw archive {reason} for {item['dataset']}; exact rehash required"
            )

    candidate_config = project / "dataset-pipeline/configs" / f"{workspace.name}.yaml"
    policy_files = [
        project / "automotive_taxonomy_mapping.json",
        project / "dataset-pipeline/configs/proposal_object_contract_review.csv",
        candidate_config,
        split_path,
    ] + list((project / "dataset-pipeline/dataset_pipeline").glob("*.py"))
    contract_policy = contract.get("policy", {})
    for name, path in (
        ("taxonomy", project / "automotive_taxonomy_mapping.json"),
        (
            "review_decisions",
            project / "dataset-pipeline/configs/proposal_object_contract_review.csv",
        ),
        ("pipeline_config", candidate_config),
    ):
        if contract_policy.get(name, {}).get("sha256") != sha256_file(path):
            failures.append(f"{name} does not match the dataset contract")
    if contract.get("splits", {}).get("woodscape_sequence_split", {}).get(
        "sha256"
    ) != sha256_file(split_path):
        failures.append("approved WoodScape split does not match the dataset contract")
    return {
        "schema_version": "production-audit-binding.v1",
        "expected_manifest_schema": EXPECTED_SCHEMA,
        "dataset_contract": {
            "path": str(contract_path.resolve()),
            "sha256": sha256_file(contract_path),
            "dataset_id": contract.get("dataset_id"),
            "publication_status": contract.get("publication_status"),
        },
        "manifests": manifest_info,
        "conversion_policy": _tree_hash(policy_files, project),
        "approved_split": {
            "path": str(split_path.resolve()),
            "sha256": sha256_file(split_path),
            "decision": split_decision,
            "soiling_train_images": soiling_train_images,
            "expected_generated_counts": expected_woodscape,
        },
        "generated_split_report": {
            "path": str((workspace / "reports/split_report.json").resolve()),
            "sha256": sha256_file(workspace / "reports/split_report.json"),
            "counts": current_split,
        },
        "raw_source_inventory": {
            "path": str(inventory_path.resolve()),
            "sha256": sha256_file(inventory_path),
            "archives": raw_sources,
            "verification": "cached SHA-256 plus exact size/mtime binding",
        },
        "dropped_region_provenance": {
            "path": str((workspace / "reports/annotation_provenance.json").resolve()),
            "sha256": sha256_file(workspace / "reports/annotation_provenance.json"),
            "datasets": json.loads(
                (workspace / "reports/annotation_provenance.json").read_text()
            ).get("datasets", []),
        },
        "hard_failures": failures,
        "pass": not failures,
    }


def _scan_index(path: Path, input_size: int = 384) -> dict[str, Any]:
    """Scan every indexed generated region using fixed-memory aggregates."""
    counts: Counter[str] = Counter()
    categories: Counter[str] = Counter()
    geometry: Counter[str] = Counter()
    tightness: Counter[str] = Counter()
    source_size: Counter[str] = Counter()
    model_size: Counter[str] = Counter()
    area_by_state: Counter[str] = Counter()
    no_p2_assignment = 0
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as connection:
        image_counts = dict(
            connection.execute(
                "SELECT source_dataset, COUNT(*) FROM images GROUP BY source_dataset"
            )
        )
        image_pixels = connection.execute(
            "SELECT COALESCE(SUM(width * height), 0) FROM images"
        ).fetchone()[0]
        query = """
            SELECT i.source_dataset, i.width, i.height, a.ignore_region,
                   a.category_name, a.x1, a.y1, a.x2, a.y2,
                   a.geometry_tier, a.fit_tightness
            FROM annotations a JOIN images i ON i.image_id=a.image_id
            ORDER BY a.image_id
        """
        for (
            dataset,
            width,
            height,
            code,
            category,
            x1,
            y1,
            x2,
            y2,
            tier,
            fit,
        ) in connection.execute(query):
            state = STATES.get(int(code), f"invalid:{code}")
            counts[state] += 1
            categories[f"{dataset}/{state}/{category}"] += 1
            geometry[f"{state}/{tier}"] += 1
            tightness[f"{state}/{_bin(float(fit), TIGHTNESS_EDGES)}"] += 1
            box_w, box_h = max(0.0, x2 - x1), max(0.0, y2 - y1)
            short = min(box_w, box_h)
            scale = min(input_size / width, input_size / height)
            model_short = short * scale
            source_size[f"{state}/{_bin(short, SIZE_EDGES)}"] += 1
            model_size[f"{state}/{_bin(model_short, SIZE_EDGES)}"] += 1
            area_by_state[state] += box_w * box_h
            if state == "positive" and model_short < 4:
                no_p2_assignment += 1
    weak_area = max(0.0, float(image_pixels) - sum(area_by_state.values()))
    return {
        "images_by_source": dict(sorted(image_counts.items())),
        "image_pixels": image_pixels,
        "regions_by_state": dict(sorted(counts.items())),
        "category_source_state": dict(sorted(categories.items())),
        "geometry_tier": dict(sorted(geometry.items())),
        "conversion_tightness": dict(sorted(tightness.items())),
        "source_short_side_px": dict(sorted(source_size.items())),
        "model_short_side_px_at_384": dict(sorted(model_size.items())),
        "bbox_area_by_state_px2": dict(sorted(area_by_state.items())),
        "weak_uncovered_bbox_approximation_px2": weak_area,
        "no_valid_selected_p2_min4_assignment_upper_bound": no_p2_assignment,
        "coverage_note": "BBox sums are diagnostic only; exact polygon-union coverage and overlap require a v2 candidate and are intentionally not certified from this stale v1 index.",
    }


def _polygon(record: dict[str, Any]) -> Polygon:
    points = [[float(value) for value in point] for point in record["quad"]]
    return Polygon(points)


def _scan_manifest(
    path: Path, image_root: Path, split: str, assets: Path
) -> dict[str, Any]:
    """Audit every v2 image and polygon while retaining only bounded examples."""
    counts: Counter[str] = Counter()
    area: Counter[str] = Counter()
    overlaps: Counter[str] = Counter()
    datasets: Counter[str] = Counter()
    unreadable: list[str] = []
    invalid: list[str] = []
    effective_conflicts: list[str] = []
    identities: set[tuple[str, str]] = set()
    sequences: set[tuple[str, str]] = set()
    identity_collisions: list[tuple[str, str]] = []
    content_paths: dict[str, str] = {}
    duplicates: list[dict[str, str]] = []
    samples: dict[tuple[str, str, str], dict[str, Any]] = {}
    split_digest = hashlib.sha256()
    total_pixels = 0.0
    scanned_images = 0
    started = time.monotonic()
    with path.open("rb") as stream:
        for image in ijson.items(stream, "images.item", use_float=True):
            scanned_images += 1
            dataset = str(image.get("source_dataset", "unknown"))
            source_id = str(image.get("source_image_id", ""))
            sequence_id = str(image.get("source_sequence_id", ""))
            identity = (dataset, source_id)
            if identity in identities:
                identity_collisions.append(identity)
            identities.add(identity)
            if sequence_id:
                sequences.add((dataset, sequence_id))
            datasets[dataset] += 1
            split_digest.update(
                f"{split}\0{dataset}\0{source_id}\0{sequence_id}\n".encode()
            )
            width, height = int(image["width"]), int(image["height"])
            total_pixels += width * height
            image_path = image_root / str(image["file_name"])
            try:
                digest = sha256_file(image_path)
                with Image.open(image_path) as opened:
                    opened.verify()
                previous = content_paths.setdefault(digest, str(image_path))
                if previous != str(image_path):
                    duplicates.append(
                        {"sha256": digest, "first": previous, "second": str(image_path)}
                    )
            except Exception as error:
                unreadable.append(f"{image_path}: {error}")
            polygons: dict[str, list[Polygon]] = defaultdict(list)
            annotation_states: dict[tuple[str, str], str] = {}
            for state in ("positive", "ignore", "trusted_background"):
                for record in image.get(state, []):
                    counts[state] += 1
                    annotation_id = str(record.get("source_annotation_id", ""))
                    identity_kind = str(
                        record.get("source_annotation_identity_kind", "")
                    )
                    declared_state = str(
                        record.get("supervision_state") or record.get("state") or ""
                    )
                    if declared_state == "trusted_negative":
                        declared_state = "trusted_background"
                    label = f"{split}:{dataset}:{source_id}:{state}:{annotation_id}"
                    if declared_state and declared_state != state:
                        effective_conflicts.append(
                            f"{label}: declares state {declared_state!r}"
                        )
                    if annotation_id:
                        annotation_key = (identity_kind, annotation_id)
                        previous_state = annotation_states.setdefault(
                            annotation_key, state
                        )
                        if previous_state != state:
                            effective_conflicts.append(
                                f"{label}: source annotation also appears as "
                                f"{previous_state}"
                            )
                    try:
                        polygon = _polygon(record)
                        if not polygon.is_valid or polygon.area <= 0:
                            invalid.append(
                                f"{split}:{dataset}:{source_id}:{state}:{record.get('source_annotation_id', '')}"
                            )
                            continue
                        polygons[state].append(polygon)
                        category = str(
                            record.get("canonical_category")
                            or record.get("source_category", "unknown")
                        )
                        samples.setdefault(
                            (dataset, state, category),
                            {
                                "path": str(image_path),
                                "quad": list(polygon.exterior.coords)[:-1],
                                "dataset": dataset,
                                "state": state,
                                "category": category,
                            },
                        )
                    except Exception:
                        invalid.append(
                            f"{split}:{dataset}:{source_id}:{state}:{record.get('source_annotation_id', '')}"
                        )
            unions = {
                state: union_all(items) for state, items in polygons.items() if items
            }
            occupied = union_all(list(unions.values())) if unions else None
            for state, geometry in unions.items():
                area[state] += geometry.area
            area["weak"] += max(
                0.0, width * height - (occupied.area if occupied else 0.0)
            )
            for left, right in (
                ("positive", "ignore"),
                ("positive", "trusted_background"),
                ("ignore", "trusted_background"),
            ):
                if left in unions and right in unions:
                    overlap = unions[left].intersection(unions[right]).area
                    if overlap > 1e-7:
                        overlaps[f"{left}/{right}_images"] += 1
                        overlaps[f"{left}/{right}_px2"] += overlap
            if scanned_images % 1_000 == 0:
                elapsed = time.monotonic() - started
                print(
                    f"AUDIT {split}: {scanned_images:,} images in {elapsed:.1f}s "
                    f"({scanned_images / max(elapsed, 1e-9):.1f} images/s)",
                    flush=True,
                )
    assets.mkdir(parents=True, exist_ok=True)
    for key, sample in samples.items():
        try:
            with Image.open(sample["path"]).convert("RGB") as source:
                source.thumbnail((800, 800))
                sx = source.width / Image.open(sample["path"]).width
                sy = source.height / Image.open(sample["path"]).height
                draw = ImageDraw.Draw(source)
                draw.polygon(
                    [(x * sx, y * sy) for x, y in sample["quad"]],
                    outline="red",
                    width=3,
                )
                name = hashlib.sha256("\0".join(key).encode()).hexdigest()[:16] + ".jpg"
                source.save(assets / name, quality=90)
                sample["asset"] = f"assets/{name}"
        except Exception:
            pass
    return {
        "images": sum(datasets.values()),
        "images_by_source": dict(sorted(datasets.items())),
        "unreadable_images": unreadable,
        "invalid_polygons": invalid,
        "regions_by_state": dict(sorted(counts.items())),
        "area_by_state_px2": dict(sorted(area.items())),
        "area_fraction_by_state": {
            key: value / total_pixels for key, value in sorted(area.items())
        },
        "raw_state_intersections": dict(sorted(overlaps.items())),
        "effective_supervision_conflicts": effective_conflicts,
        "source_identities": identities,
        "source_sequences": sequences,
        "source_identity_collisions": identity_collisions,
        "content_hashes": set(content_paths),
        "duplicate_images_within_split": duplicates,
        "split_sha256": split_digest.hexdigest(),
        "overlay_samples": list(samples.values()),
    }


def _jsonable_scan(scan: dict[str, Any]) -> dict[str, Any]:
    hidden = {"source_identities", "source_sequences", "content_hashes"}
    return {key: value for key, value in scan.items() if key not in hidden}


def _scan_findings(
    binding: dict[str, Any],
    scans: dict[str, dict[str, Any]] | None = None,
    cross_split: dict[str, list[Any]] | None = None,
) -> tuple[list[str], list[str]]:
    """Return human-readable hard failures and non-blocking warnings."""
    failures = list(binding["hard_failures"])
    warnings: list[str] = []
    for split, scan in (scans or {}).items():
        for field, label in (
            ("unreadable_images", "unreadable images"),
            ("invalid_polygons", "invalid polygons"),
            ("effective_supervision_conflicts", "effective supervision conflicts"),
            ("source_identity_collisions", "source identity collisions"),
        ):
            if count := len(scan[field]):
                failures.append(f"{split}: {count:,} {label}")
        overlaps = scan.get(
            "raw_state_intersections", scan.get("state_overlap_violations", {})
        )
        for key, count in sorted(overlaps.items()):
            if key.endswith("_images") and count:
                pair = key.removesuffix("_images")
                area = float(overlaps.get(f"{pair}_px2", 0.0))
                warnings.append(
                    f"{split}: {int(count):,} images have raw {pair} intersection "
                    f"({area:,.6f} px^2 union area)"
                )
        if count := len(scan["duplicate_images_within_split"]):
            warnings.append(
                f"{split}: {count:,} repeated image-content records within split"
            )
    for field, label in (
        ("source_identity_collisions", "source identity collisions"),
        ("sequence_leakage", "source-sequence leakage records"),
        ("duplicate_image_sha256", "duplicate image hashes"),
    ):
        if count := len((cross_split or {}).get(field, [])):
            failures.append(f"cross-split: {count:,} {label}")
    return failures, warnings


def _render_html(path: Path, report: dict[str, Any]) -> None:
    failures = "".join(
        f"<li>{html.escape(item)}</li>" for item in report.get("hard_failures", [])
    )
    warnings = "".join(
        f"<li>{html.escape(item)}</li>" for item in report.get("warnings", [])
    )
    body = html.escape(json.dumps(report.get("inventory", {}), indent=2))
    status = report.get("status", "unknown")
    passed_binding = report["binding"]["pass"]
    gallery = []
    for split, scan in report.get("full_scan", {}).items():
        for sample in scan.get("overlay_samples", []):
            asset = html.escape(sample.get("asset", ""), quote=True)
            label = html.escape(
                f"{split} · {sample['dataset']} · {sample['state']} · {sample['category']}"
            )
            gallery.append(
                f"<figure><img loading='lazy' src='{asset}'><figcaption>{label}</figcaption></figure>"
            )
    gate_text = "PASSED" if passed_binding else "FAILED"
    gate_class = "pass" if passed_binding else "fail"
    scan_note = (
        "The exhaustive scan was not requested for this binding-only preflight."
        if status == "binding_pass"
        else "Review the machine-readable report and every stratified overlay below."
    )
    path.write_text(
        f"""<!doctype html><meta charset='utf-8'><title>Phase 1.8 production audit</title>
<style>body{{font:15px system-ui;max-width:1200px;margin:auto;padding:24px}}pre{{white-space:pre-wrap;background:#f5f5f5;padding:16px}}.fail{{color:#a00}}.pass{{color:#086b30}}.gallery{{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:16px}}figure{{margin:0}}img{{max-width:100%;height:auto}}figcaption{{overflow-wrap:anywhere}}</style>
<h1>Phase 1.8 production audit</h1><p class='{gate_class}'><strong>Binding gate: {gate_text}</strong></p>
<p><strong>Status:</strong> {html.escape(status)}. {html.escape(scan_note)}</p><h2>Hard failures</h2><ul>{failures}</ul>
<h2>Warnings</h2><ul>{warnings}</ul>
<h2>Bound inventory (all indexed records)</h2><pre>{body}</pre>
<h2>Stratified overlay samples</h2><div class='gallery'>{"".join(gallery)}</div>
<p>See <code>audit_report.json</code> for hashes and the full machine-readable contract.</p>""",
        encoding="utf-8",
    )


def run(
    project: Path,
    workspace: Path,
    output: Path,
    inventory_on_failure: bool = True,
    full_scan: bool = True,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    binding = binding_preflight(project, workspace)
    report: dict[str, Any] = {
        "schema_version": "production-dataset-audit.v2",
        "binding": binding,
        "gate_definition": {
            "hard_failures": [
                "binding mismatch",
                "unreadable image",
                "invalid polygon",
                "effective supervision-state conflict",
                "source identity collision",
                "cross-split identity, sequence, or image-content leakage",
            ],
            "warnings": [
                "raw geometric intersection resolved by positive > ignore > trusted > weak",
                "within-split repeated image content",
            ],
        },
    }
    if binding["pass"] and not full_scan:
        report["hard_failures"], report["warnings"] = _scan_findings(binding)
        report["status"] = "binding_pass"
        report["publishable"] = False
    elif binding["pass"]:
        scans = {
            split: _scan_manifest(
                workspace / f"output/annotations/proposals_{split}.json",
                workspace / "output",
                split,
                output / "assets",
            )
            for split in ("train", "val")
        }
        identity_leakage = sorted(
            scans["train"]["source_identities"] & scans["val"]["source_identities"]
        )
        sequence_leakage = sorted(
            scans["train"]["source_sequences"] & scans["val"]["source_sequences"]
        )
        cross_duplicates = sorted(
            scans["train"]["content_hashes"] & scans["val"]["content_hashes"]
        )
        report["full_scan"] = {
            split: _jsonable_scan(scan) for split, scan in scans.items()
        }
        cross_split = {
            "source_identity_collisions": identity_leakage,
            "sequence_leakage": sequence_leakage,
            "duplicate_image_sha256": cross_duplicates,
        }
        report["cross_split"] = cross_split
        report["inventory"] = {
            split: _scan_index(workspace / f"output/indexes/quad_{split}.sqlite")
            for split in ("train", "val")
        }
        report["hard_failures"], report["warnings"] = _scan_findings(
            binding, scans, cross_split
        )
        report["status"] = (
            "hard_failure" if report["hard_failures"] else "awaiting_owner_review"
        )
        report["publishable"] = not report["hard_failures"]
    elif inventory_on_failure:
        report["inventory"] = {
            split: _scan_index(workspace / f"output/indexes/quad_{split}.sqlite")
            for split in ("train", "val")
        }
        report["hard_failures"], report["warnings"] = _scan_findings(binding)
        report["status"] = "hard_failure"
        report["publishable"] = False
    else:
        report["hard_failures"], report["warnings"] = _scan_findings(binding)
        report["status"] = "hard_failure"
        report["publishable"] = False
    (output / "audit_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    _render_html(output / "index.html", report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project", type=Path, default=Path(__file__).resolve().parents[1]
    )
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--binding-only", action="store_true")
    args = parser.parse_args()
    report = run(
        args.project.resolve(),
        args.workspace.resolve(),
        args.output.resolve(),
        not args.binding_only,
        not args.binding_only,
    )
    print(
        json.dumps(
            {
                "status": report["status"],
                "hard_failures": report["hard_failures"],
                "warnings": report["warnings"],
                "output": str(args.output.resolve()),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
