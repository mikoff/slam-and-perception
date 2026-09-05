"""Product-object contract inventory and source-annotation review bundle."""

from __future__ import annotations

import csv
import hashlib
import heapq
import html
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

from PIL import Image, ImageDraw, ImageOps

from .config import Config
from .detection_conversion import GeometryError, object_bbox
from .discovery import project_root
from .progress import Progress
from .reports import read_json, write_json
from .supervisely_filter import iter_project_images
from .taxonomy import MappingResult, Taxonomy


CONTRACT_STATES = frozenset({"positive", "trusted_negative", "ignore", "drop"})
CONTRACT_SCHEMA_VERSIONS = frozenset(
    {"proposal-object-contract.candidate.v1", "proposal-object-contract.v1"}
)
AUDIT_SCHEMA_VERSION = "proposal-object-contract-audit.v1"


def _current_state(
    taxonomy: Taxonomy,
    dataset: str,
    normalized: str,
    mapping: MappingResult,
) -> str:
    trusted = {
        taxonomy.normalize(name)
        for name in taxonomy.data.get("trusted_background_categories", {}).get(
            dataset, []
        )
    }
    if normalized in trusted:
        return "trusted_negative"
    if mapping.ignore_region:
        return "ignore"
    if mapping.ignored:
        return "drop"
    return "positive"


def build_contract_inventory(
    config: Config,
    taxonomy: Taxonomy,
) -> list[dict[str, Any]]:
    """Resolve every declared source class to a current and proposed state."""
    contract = taxonomy.data.get("proposal_object_contract")
    if not isinstance(contract, dict):
        raise ValueError("taxonomy is missing proposal_object_contract")
    if contract.get("schema_version") not in CONTRACT_SCHEMA_VERSIONS:
        raise ValueError("unsupported proposal object-contract schema")
    overrides = contract.get("category_overrides", {})
    approved = contract.get("approved_category_states", {})

    rows = []
    seen_overrides: set[tuple[str, str]] = set()
    for dataset in config.datasets.values():
        source = project_root(dataset, sdk_check=False)
        classes = read_json(source / "meta.json").get("classes", [])
        for source_class in sorted(classes, key=lambda item: item["title"]):
            source_category = str(source_class["title"])
            normalized = taxonomy.normalize(source_category)
            mapping = taxonomy.map(
                dataset.name, source_category, source_class.get("description")
            )
            override = overrides.get(dataset.name, {}).get(normalized)
            if override is not None:
                seen_overrides.add((dataset.name, normalized))
                proposed_state = str(override.get("proposed_state", ""))
                if proposed_state not in CONTRACT_STATES:
                    raise ValueError(
                        f"{dataset.name}:{normalized} has invalid proposed state "
                        f"{proposed_state!r}"
                    )
                normalized_category = str(override.get("normalized_category", ""))
                if not normalized_category:
                    raise ValueError(
                        f"{dataset.name}:{normalized} has no normalized category"
                    )
            else:
                if mapping.ignored or mapping.ignore_region:
                    raise ValueError(
                        f"{dataset.name}:{normalized} is non-positive today and needs "
                        "an explicit contract override"
                    )
                proposed_state = "positive"
                normalized_category = str(mapping.canonical)
                override = {}
            approved_record = approved.get(dataset.name, {}).get(normalized)
            if contract.get("status") == "approved" and bool(
                override.get("audit_required", False)
            ):
                if not isinstance(approved_record, dict):
                    raise ValueError(
                        f"{dataset.name}:{normalized} has no approved category state"
                    )
                approved_state = str(approved_record.get("state", ""))
                if approved_state not in CONTRACT_STATES:
                    raise ValueError(
                        f"{dataset.name}:{normalized} has invalid approved state "
                        f"{approved_state!r}"
                    )
            else:
                approved_state = proposed_state
            rows.append(
                {
                    "source_dataset": dataset.name,
                    "source_category": source_category,
                    "normalized_source_category": normalized,
                    "normalized_category": normalized_category,
                    "current_state": _current_state(
                        taxonomy, dataset.name, normalized, mapping
                    ),
                    "proposed_state": proposed_state,
                    "approved_state": approved_state,
                    "rationale": str(
                        override.get("rationale", "Retained physical instance.")
                    ),
                    "audit_required": bool(override.get("audit_required", False)),
                }
            )

    declared_overrides = {
        (dataset, category)
        for dataset, categories in overrides.items()
        if dataset in config.datasets
        for category in categories
    }
    unused = declared_overrides - seen_overrides
    if unused:
        raise ValueError(f"contract overrides absent source classes: {sorted(unused)}")
    if contract.get("status") == "approved":
        expected_approved = {
            (dataset, category)
            for dataset, categories in overrides.items()
            if dataset in config.datasets
            for category, policy in categories.items()
            if policy.get("audit_required", False)
        }
        declared_approved = {
            (dataset, category)
            for dataset, categories in approved.items()
            if dataset in config.datasets
            for category in categories
        }
        if declared_approved != expected_approved:
            raise ValueError(
                "approved category states do not exactly match audited overrides: "
                f"missing={sorted(expected_approved - declared_approved)}, "
                f"extra={sorted(declared_approved - expected_approved)}"
            )
    return rows


def _stable_key(*parts: object) -> str:
    return hashlib.sha256("\0".join(map(str, parts)).encode("utf-8")).hexdigest()


def _retain_smallest(
    heap: list[tuple[int, str, dict[str, Any]]],
    row: dict[str, Any],
    count: int,
) -> None:
    rank = int(row["sample_rank"], 16)
    entry = (-rank, row["audit_id"], row)
    if len(heap) < count:
        heapq.heappush(heap, entry)
    elif rank < -heap[0][0]:
        heapq.heapreplace(heap, entry)


def collect_audit_samples(
    config: Config,
    taxonomy: Taxonomy,
    inventory: list[dict[str, Any]],
    count: int,
) -> tuple[list[dict[str, Any]], dict[tuple[str, str], int]]:
    """Select deterministic source examples, stratified by split and geometry."""
    required = {
        (row["source_dataset"], row["normalized_source_category"]): row
        for row in inventory
        if row["audit_required"]
    }
    heaps: dict[tuple[str, str, str, str], list[tuple[int, str, dict[str, Any]]]] = (
        defaultdict(list)
    )
    totals: Counter[tuple[str, str]] = Counter()

    for dataset in config.datasets.values():
        wanted = {category for source, category in required if source == dataset.name}
        if not wanted:
            continue
        source = project_root(dataset, sdk_check=False)
        progress = Progress(f"Scanning contract evidence for {dataset.name}", "images")
        for split, image_path, annotation_path in iter_project_images(source):
            if not annotation_path.exists():
                progress.add()
                continue
            annotation = read_json(annotation_path)
            for object_index, obj in enumerate(annotation.get("objects", [])):
                source_category = str(obj.get("classTitle", ""))
                normalized = taxonomy.normalize(source_category)
                key = (dataset.name, normalized)
                if normalized not in wanted:
                    continue
                totals[key] += 1
                source_annotation_id = str(
                    obj.get("id", obj.get("key", f"index-{object_index}"))
                )
                audit_id = _stable_key(
                    dataset.name,
                    split,
                    image_path.name,
                    source_annotation_id,
                    object_index,
                )[:16]
                sample_rank = _stable_key("contract-audit", audit_id)
                geometry_type = str(obj.get("geometryType", "unknown")).lower()
                policy = required[key]
                row = {
                    **policy,
                    "audit_id": audit_id,
                    "sample_rank": sample_rank,
                    "source_split": split,
                    "source_image_identity": image_path.name,
                    "source_annotation_identity": source_annotation_id,
                    "source_annotation_path": str(annotation_path),
                    "source_image_path": str(image_path),
                    "source_object_index": object_index,
                    "source_geometry_type": geometry_type,
                }
                stratum = (dataset.name, normalized, split, geometry_type)
                _retain_smallest(heaps[stratum], row, count)
            progress.add()
        progress.finish()

    by_category: dict[tuple[str, str], list[list[dict[str, Any]]]] = defaultdict(list)
    for (dataset, category, _split, _geometry), heap in sorted(heaps.items()):
        rows = [entry[2] for entry in heap]
        rows.sort(key=lambda row: (row["sample_rank"], row["audit_id"]))
        by_category[(dataset, category)].append(rows)

    selected = []
    for key in sorted(required):
        strata = by_category.get(key, [])
        category_rows = []
        while len(category_rows) < count:
            progressed = False
            for rows in strata:
                if rows and len(category_rows) < count:
                    category_rows.append(rows.pop(0))
                    progressed = True
            if not progressed:
                break
        selected.extend(sorted(category_rows, key=lambda row: row["audit_id"]))
    return selected, dict(totals)


def _object_mask(obj: dict[str, Any], image_size: tuple[int, int]) -> Image.Image:
    mask = Image.new("L", image_size, 0)
    geometry = str(obj.get("geometryType", "")).lower()
    draw = ImageDraw.Draw(mask)
    points = obj.get("points", {}).get("exterior", [])
    if geometry == "rectangle" and len(points) >= 2:
        draw.rectangle(
            [tuple(points[0]), tuple(points[1])], fill=150, outline=255, width=4
        )
    elif geometry == "polygon" and len(points) >= 3:
        polygon = [tuple(point) for point in points]
        draw.polygon(polygon, fill=110)
        draw.line(polygon + [polygon[0]], fill=255, width=4)
    elif geometry in {"line", "polyline"} and len(points) >= 2:
        draw.line([tuple(point) for point in points], fill=255, width=5)
    elif geometry in {"bitmap", "mask"}:
        import supervisely as sly

        bitmap = sly.Bitmap.from_json(obj)
        local = Image.fromarray(bitmap.data.astype("uint8") * 150, mode="L")
        mask.paste(local, (bitmap.origin.col, bitmap.origin.row), local)
    else:
        raise GeometryError(f"unsupported geometry: {geometry or 'missing'}")
    return mask


def _render_overlay(row: dict[str, Any], destination: Path) -> None:
    annotation = read_json(Path(row["source_annotation_path"]))
    obj = annotation["objects"][int(row["source_object_index"])]
    with Image.open(row["source_image_path"]) as loaded:
        image = loaded.convert("RGBA")
    mask = _object_mask(obj, image.size)
    color = Image.new("RGBA", image.size, (255, 55, 95, 0))
    color.putalpha(mask)
    rendered = Image.alpha_composite(image, color).convert("RGB")
    try:
        x1, y1, x2, y2 = object_bbox(obj)
    except GeometryError:
        points = obj.get("points", {}).get("exterior", [])
        if points:
            xs = [float(point[0]) for point in points]
            ys = [float(point[1]) for point in points]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
        else:
            x1, y1, x2, y2 = 0.0, 0.0, float(image.width), float(image.height)
    padding = max(x2 - x1, y2 - y1, 32.0) * 0.3
    crop = rendered.crop(
        (
            max(0, int(x1 - padding)),
            max(0, int(y1 - padding)),
            min(rendered.width, int(x2 + padding)),
            min(rendered.height, int(y2 + padding)),
        )
    )
    rendered.thumbnail((640, 420))
    crop = ImageOps.contain(crop, (480, 420), Image.Resampling.LANCZOS)
    canvas = Image.new(
        "RGB",
        (rendered.width + crop.width + 12, max(rendered.height, crop.height)),
        "#15171a",
    )
    canvas.paste(rendered, (0, 0))
    canvas.paste(crop, (rendered.width + 12, 0))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, "JPEG", quality=88, optimize=True)


def _write_audit_html(
    destination: Path,
    contract: dict[str, Any],
    inventory: list[dict[str, Any]],
    rows: list[dict[str, Any]],
    totals: dict[tuple[str, str], int],
    requested_count: int,
) -> None:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[(row["source_dataset"], row["normalized_source_category"])].append(row)
    policies = {
        (row["source_dataset"], row["normalized_source_category"]): row
        for row in inventory
        if row["audit_required"]
    }
    sections = []
    for (dataset, category), first in sorted(policies.items()):
        examples = grouped.get((dataset, category), [])
        images = "".join(
            f'<figure><img loading="lazy" src="assets/{row["audit_id"]}.jpg" '
            f'alt="source annotation {row["audit_id"]}"><figcaption>'
            f"{html.escape(row['source_split'])} · {html.escape(row['source_geometry_type'])} · "
            f"annotation {html.escape(row['source_annotation_identity'])}</figcaption></figure>"
            for row in examples
        )
        available = totals.get((dataset, category), 0)
        shortage = (
            f'<strong class="warn">Source contains only {available} examples; '
            f"{len(examples)} shown instead of {requested_count}.</strong>"
            if available < requested_count
            else f"{len(examples)} of {available} source annotations shown."
        )
        group_id = f"{dataset}:{category}"
        sections.append(
            f"""
<section class="policy" data-policy="{html.escape(group_id)}">
  <h2>{html.escape(dataset)} · {html.escape(first["source_category"])}</h2>
  <p>{shortage}</p>
  <table><tr><th>Normalized category</th><td>{html.escape(first["normalized_category"])}</td></tr>
  <tr><th>Current effective state</th><td>{html.escape(first["current_state"])}</td></tr>
  <tr><th>Proposed state</th><td>{html.escape(first["proposed_state"])}</td></tr>
  <tr><th>Rationale</th><td>{html.escape(first["rationale"])}</td></tr></table>
  <div class="review"><label>Decision <select class="decision"><option></option><option>ACCEPT</option><option>REJECT</option><option>EXCEPTION</option></select></label>
  <label>Approved state <select class="approved_state"><option></option><option>positive</option><option>trusted_negative</option><option>ignore</option><option>drop</option></select></label>
  <label>Exception rule / notes <input class="notes"></label></div>
  <details><summary>Show {len(examples)} source overlays</summary><div class="examples">{images}</div></details>
</section>"""
        )
    definition = html.escape(str(contract["definition"]))
    document = f"""<!doctype html><html><head><meta charset="utf-8"><title>Proposal object-contract audit</title>
<style>body{{font:14px system-ui;background:#101216;color:#eee;margin:18px}}button,select,input{{font:inherit}}.top{{position:sticky;top:0;background:#20242b;padding:12px;z-index:3}}.policy{{background:#20242b;padding:14px;margin:18px 0;border-radius:8px}}table{{border-collapse:collapse}}th,td{{text-align:left;padding:5px 12px 5px 0}}.review label{{display:block;margin:8px 0}}.review input{{width:70%}}.examples{{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:10px}}figure{{margin:0;background:#15171a;padding:8px}}figure img{{width:100%;max-height:430px;object-fit:contain}}.warn{{color:#ff9f0a}}</style></head><body>
<div class="top"><b>Candidate proposal-object contract</b> · <label>Reviewer <input id="reviewer"></label> <button id="export">Export category decisions CSV</button> <span id="progress"></span></div>
<p>{definition}</p><p>Red overlay is the exact source annotation. Each policy decision applies to the displayed source-dataset/category pair, not to one isolated image.</p>
{"".join(sections)}
<script>
const policies=[...document.querySelectorAll('.policy')],key='proposal-object-contract-audit-v1';let saved=JSON.parse(localStorage.getItem(key)||'{{}}');document.querySelector('#reviewer').value=saved.reviewer||'';
function update(){{let done=0;policies.forEach(p=>{{let id=p.dataset.policy,v=saved[id]||{{}};['decision','approved_state','notes'].forEach(k=>{{let e=p.querySelector('.'+k);if(document.activeElement!==e)e.value=v[k]||''}});if(v.decision)done++}});saved.reviewer=document.querySelector('#reviewer').value;document.querySelector('#progress').textContent=`${{done}}/${{policies.length}} decided`;localStorage.setItem(key,JSON.stringify(saved))}}
policies.forEach(p=>p.addEventListener('input',()=>{{let id=p.dataset.policy;saved[id]={{decision:p.querySelector('.decision').value,approved_state:p.querySelector('.approved_state').value,notes:p.querySelector('.notes').value}};update()}}));document.querySelector('#reviewer').addEventListener('input',update);
document.querySelector('#export').onclick=()=>{{let q=x=>'"'+String(x||'').replaceAll('"','""')+'"',lines=['source_dataset,source_category,decision,approved_state,notes,reviewer'];policies.forEach(p=>{{let [dataset,...category]=p.dataset.policy.split(':'),v=saved[p.dataset.policy]||{{}};lines.push([dataset,category.join(':'),v.decision,v.approved_state,v.notes,saved.reviewer].map(q).join(','))}});let a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.join('\n')],{{type:'text/csv'}}));a.download='object_contract_decisions_completed.csv';a.click()}};update();
</script></body></html>"""
    destination.write_text(document, encoding="utf-8")


def _write_instructions(destination: Path, report: dict[str, Any]) -> None:
    destination.write_text(
        f"""# Proposal Object-Contract Review

This bundle covers **{report["audited_category_pairs"]} disputed source/category pairs** with **{report["rendered_examples"]} source overlays**.

## Review procedure

1. Serve this directory: `python3 -m http.server 8000 --directory <bundle>`.
2. Open `http://localhost:8000/index.html` and read the product definition first.
3. For every category section, inspect the whole-image and crop overlays.
4. Choose `ACCEPT` when the proposed state is correct for the entire source category.
5. Choose `REJECT` when another single state is correct, and select that approved state.
6. Choose `EXCEPTION` when the category needs a conditional rule; state that rule precisely in notes and select the safe default state.
7. Export `object_contract_decisions_completed.csv` and preserve it beside the bundle.

`ACCEPT` means the approved state equals the proposed state. `REJECT` and `EXCEPTION` require an approved state and notes. Categories with fewer than 100 source annotations show every available example and require an explicit owner-approved source-scarcity exception.
""",
        encoding="utf-8",
    )


def _hash_files(root: Path, paths: Iterable[Path]) -> list[dict[str, Any]]:
    result = []
    for path in sorted(paths):
        relative = path.relative_to(root).as_posix()
        result.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        )
    return result


def generate_contract_audit_bundle(
    config: Config,
    taxonomy: Taxonomy,
    count: int = 100,
    destination: Path | None = None,
) -> dict[str, Any]:
    """Generate the deterministic Phase 1.1 product-policy review bundle."""
    if count < 1:
        raise ValueError("contract audit count must be positive")
    destination = (
        destination or config.reports / "proposal_object_contract_audit_bundle"
    ).resolve()
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)

    inventory = build_contract_inventory(config, taxonomy)
    samples, totals = collect_audit_samples(config, taxonomy, inventory, count)
    for row in samples:
        _render_overlay(row, destination / "assets" / f"{row['audit_id']}.jpg")

    contract = taxonomy.data["proposal_object_contract"]
    serializable_totals = {
        f"{dataset}:{category}": total
        for (dataset, category), total in sorted(totals.items())
    }
    shortages = {
        key: total for key, total in serializable_totals.items() if total < count
    }
    report = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "contract_schema_version": contract["schema_version"],
        "contract_status": contract["status"],
        "requested_examples_per_category_pair": count,
        "source_annotation_counts": serializable_totals,
        "source_scarcity_exceptions_required": shortages,
        "audited_category_pairs": sum(row["audit_required"] for row in inventory),
        "rendered_examples": len(samples),
        "source_categories_resolved": len(inventory),
        "all_source_categories_resolved": True,
        "strict_sample_count_met": not shortages,
    }
    write_json(destination / "category_inventory.json", inventory)
    write_json(destination / "audit_report.json", report)
    with (destination / "audit_instances.jsonl").open("w", encoding="utf-8") as stream:
        for row in samples:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    with (destination / "review_decisions.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "source_dataset",
                "source_category",
                "decision",
                "approved_state",
                "notes",
                "reviewer",
            ]
        )
        for row in inventory:
            if row["audit_required"]:
                writer.writerow(
                    [
                        row["source_dataset"],
                        row["normalized_source_category"],
                        "",
                        "",
                        "",
                        "",
                    ]
                )
    _write_audit_html(
        destination / "index.html", contract, inventory, samples, totals, count
    )
    _write_instructions(destination / "REVIEW_INSTRUCTIONS.md", report)
    manifest_paths = [
        path
        for path in destination.rglob("*")
        if path.is_file() and path.name != "bundle_manifest.json"
    ]
    write_json(
        destination / "bundle_manifest.json",
        {
            "schema_version": AUDIT_SCHEMA_VERSION,
            "files": _hash_files(destination, manifest_paths),
        },
    )
    return report


def _next_value(lines: list[str], label: str) -> str:
    labels = {"Decision", "Approved state", "Exception rule / notes"}
    for index, line in enumerate(lines):
        if line.strip() != label:
            continue
        for candidate in lines[index + 1 :]:
            value = candidate.strip()
            if not value:
                continue
            if value in labels or value.startswith("Show "):
                return ""
            return value
    return ""


def _notes_value(lines: list[str]) -> str:
    for index, line in enumerate(lines):
        if line.strip() != "Exception rule / notes":
            continue
        values = []
        for candidate in lines[index + 1 :]:
            value = candidate.strip()
            if value.startswith("Show "):
                break
            if value:
                values.append(value)
        return " ".join(values)
    return ""


def recover_contract_review_text(
    bundle: Path,
    review_text: Path,
    destination: Path,
) -> dict[str, Any]:
    """Recover category-level decisions from copied contract-review page text."""
    inventory = read_json(bundle / "category_inventory.json")
    expected = {
        f"{row['source_dataset']} · {row['source_category']}": row
        for row in inventory
        if row["audit_required"]
    }
    lines = review_text.read_text(encoding="utf-8").splitlines()
    header_indices = [
        (index, expected[line.strip()])
        for index, line in enumerate(lines)
        if line.strip() in expected
    ]
    reviewer = "unknown"
    for index, line in enumerate(lines):
        if "Reviewer" not in line:
            continue
        for candidate in lines[index + 1 :]:
            if candidate.strip():
                reviewer = candidate.strip()
                break
        break

    recovered = []
    errors = []
    for position, (start, policy) in enumerate(header_indices):
        end = (
            header_indices[position + 1][0]
            if position + 1 < len(header_indices)
            else len(lines)
        )
        block = lines[start + 1 : end]
        decision = _next_value(block, "Decision")
        approved_state = _next_value(block, "Approved state")
        notes = _notes_value(block)
        key = f"{policy['source_dataset']}:{policy['normalized_source_category']}"
        if decision == "ACCEPT" and not approved_state:
            approved_state = policy["proposed_state"]
        if decision not in {"ACCEPT", "REJECT", "EXCEPTION"}:
            errors.append(f"{key}: invalid or missing decision {decision!r}")
        if approved_state not in CONTRACT_STATES:
            errors.append(
                f"{key}: invalid or missing approved state {approved_state!r}"
            )
        if decision == "ACCEPT" and approved_state != policy["proposed_state"]:
            errors.append(
                f"{key}: ACCEPT state {approved_state!r} differs from proposed "
                f"{policy['proposed_state']!r}"
            )
        if decision in {"REJECT", "EXCEPTION"} and not notes:
            errors.append(f"{key}: {decision} requires notes")
        recovered.append(
            {
                "source_dataset": policy["source_dataset"],
                "source_category": policy["normalized_source_category"],
                "decision": decision,
                "approved_state": approved_state,
                "notes": notes,
                "reviewer": reviewer,
            }
        )

    recovered_keys = {
        (row["source_dataset"], row["source_category"]) for row in recovered
    }
    expected_keys = {
        (row["source_dataset"], row["normalized_source_category"])
        for row in inventory
        if row["audit_required"]
    }
    missing = [
        f"{dataset}:{category}"
        for dataset, category in sorted(expected_keys - recovered_keys)
    ]
    destination.mkdir(parents=True, exist_ok=True)
    output_csv = destination / "object_contract_decisions_recovered.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "source_dataset",
                "source_category",
                "decision",
                "approved_state",
                "notes",
                "reviewer",
            ],
        )
        writer.writeheader()
        writer.writerows(recovered)
    summary = {
        "schema_version": "proposal-object-contract-review-recovery.v1",
        "reviewer": reviewer,
        "expected": len(expected_keys),
        "recovered": len(recovered),
        "decision_counts": dict(
            sorted(Counter(row["decision"] for row in recovered).items())
        ),
        "approved_state_counts": dict(
            sorted(Counter(row["approved_state"] for row in recovered).items())
        ),
        "missing": missing,
        "errors": errors,
        "review_complete": not missing and not errors,
        "output_csv": str(output_csv),
    }
    write_json(destination / "review_recovery_summary.json", summary)
    return summary
