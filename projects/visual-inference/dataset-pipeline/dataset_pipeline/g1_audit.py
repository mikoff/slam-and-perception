"""Reproducible G1 dataset audit and human-review bundle generation."""

from __future__ import annotations

import hashlib
import html
import json
import math
import random
import re
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

from PIL import Image, ImageDraw, ImageFont, ImageOps

from .coco_export import export_all
from .coco_merge import merge_exports
from .config import Config
from .detection_conversion import convert_all, deduplicate_geometry_representations
from .reports import read_json, write_csv, write_json
from .supervisely_filter import iter_project_images
from .taxonomy import Taxonomy
from .validation import validate_all, verify_final_links


AUDIT_SCHEMA_VERSION = "quad-proposal-g1-audit.v1"
OPEN_WORLD_SCHEMA_VERSION = "quad-proposal-open-world-coco-v1"
VOC20_SEEN = frozenset(
    {
        "airplane",
        "bicycle",
        "bird",
        "boat",
        "bottle",
        "bus",
        "car",
        "cat",
        "chair",
        "cow",
        "dining_table",
        "dog",
        "horse",
        "motorcycle",
        "person",
        "potted_plant",
        "sheep",
        "couch",
        "train",
        "tv",
    }
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def hash_tree(root: Path, suffixes: set[str] | None = None) -> dict[str, Any]:
    """Hash file contents and relative paths without following directory links."""
    digest = hashlib.sha256()
    count = 0
    size = 0
    if root.exists():
        for path in sorted(item for item in root.rglob("*") if item.is_file()):
            if suffixes is not None and path.suffix.lower() not in suffixes:
                continue
            relative = path.relative_to(root).as_posix()
            file_digest = _sha256(path)
            digest.update(relative.encode("utf-8"))
            digest.update(b"\0")
            digest.update(file_digest.encode("ascii"))
            count += 1
            size += path.stat().st_size
    return {"sha256": digest.hexdigest(), "files": count, "bytes": size}


def _raw_annotation_snapshot(config: Config) -> dict[str, Any]:
    """Hash the immutable raw annotations that feed the bounded derived view."""
    snapshots: dict[str, Any] = {}
    for dataset in config.datasets.values():
        digest = hashlib.sha256()
        count = 0
        size = 0
        filtered = config.workspace_root / "intermediate" / "filtered" / dataset.name
        for split, image, _ in iter_project_images(filtered):
            path = dataset.extracted_dir / split / "ann" / f"{image.name}.json"
            if not path.exists():
                continue
            relative = path.relative_to(dataset.extracted_dir).as_posix()
            digest.update(relative.encode("utf-8"))
            digest.update(b"\0")
            digest.update(_sha256(path).encode("ascii"))
            count += 1
            size += path.stat().st_size
        snapshots[dataset.name] = {
            "sha256": digest.hexdigest(),
            "files": count,
            "bytes": size,
            "scope": "raw annotation files selected into the bounded filtered input",
        }
    return snapshots


def _derived_snapshot(config: Config) -> dict[str, Any]:
    root = config.workspace_root
    return {
        "detection_annotations": hash_tree(root / "intermediate/detection", {".json"}),
        "coco_exports": hash_tree(root / "intermediate/coco", {".json"}),
        "final_annotations": hash_tree(root / "output/annotations", {".json"}),
    }


def run_determinism_audit(
    config: Config,
    taxonomy: Taxonomy,
    workers: int,
) -> dict[str, Any]:
    """Regenerate derived data twice and compare byte-level content hashes."""
    raw_before = _raw_annotation_snapshot(config)
    passes = []
    for _ in range(2):
        convert_all(config, force=True, workers=workers)
        export_all(config, taxonomy)
        merge_exports(config, taxonomy, force=True)
        validate_all(config, taxonomy)
        passes.append(_derived_snapshot(config))
    raw_after = _raw_annotation_snapshot(config)
    links = verify_final_links(config)
    report = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "comparison": "byte-level SHA-256 of every JSON file and relative path",
        "pass_1": passes[0],
        "pass_2": passes[1],
        "derived_byte_deterministic": passes[0] == passes[1],
        "raw_annotations_unchanged": raw_before == raw_after,
        "raw_before": raw_before,
        "raw_after": raw_after,
        "final_links": links,
        "pass": passes[0] == passes[1]
        and raw_before == raw_after
        and not links.get("broken", 0),
    }
    write_json(config.reports / "g1_determinism.json", report)
    return report


def _instance_rows(manifests: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for split, manifest in manifests.items():
        for image in manifest["images"]:
            for state in ("positive", "ignore"):
                for index, record in enumerate(image.get(state, [])):
                    quad = record["quad"]
                    xs = [float(point[0]) for point in quad]
                    ys = [float(point[1]) for point in quad]
                    width = max(xs) - min(xs)
                    height = max(ys) - min(ys)
                    short, long = sorted((max(width, 1e-6), max(height, 1e-6)))
                    scale = math.sqrt(max(width * height, 0.0))
                    cx, cy = sum(xs) / 4, sum(ys) / 4
                    rx = (cx - image["width"] / 2) / max(image["width"] / 2, 1)
                    ry = (cy - image["height"] / 2) / max(image["height"] / 2, 1)
                    radial = min(math.sqrt(rx * rx + ry * ry), 1.5)
                    source_id = (
                        str(record.get("source_annotation_id", "")) or f"index-{index}"
                    )
                    key = ":".join(
                        (
                            split,
                            str(image["source_dataset"]),
                            str(image["source_image_id"]),
                            state,
                            source_id,
                        )
                    )
                    audit_id = hashlib.sha256(key.encode()).hexdigest()[:16]
                    rows.append(
                        {
                            "audit_id": audit_id,
                            "split": split,
                            "image_id": image["image_id"],
                            "file_name": image["file_name"],
                            "image_width": image["width"],
                            "image_height": image["height"],
                            "source_dataset": image["source_dataset"],
                            "source_split": image["source_split"],
                            "source_image_id": image["source_image_id"],
                            "camera_type": image.get("camera_type", "perspective"),
                            "state": state,
                            "geometry_tier": record["geometry_tier"],
                            "fit_coverage": float(record["fit_coverage"]),
                            "fit_tightness": float(record["fit_tightness"]),
                            "source_annotation_id": source_id,
                            "source_category": record.get("source_category", "unknown"),
                            "aliases": record.get("aliases", []),
                            "quad": quad,
                            "size_bin": "tiny"
                            if scale < 16
                            else "small"
                            if scale < 32
                            else "medium"
                            if scale < 96
                            else "large",
                            "aspect_bin": "extreme"
                            if long / short >= 8
                            else "slender"
                            if long / short >= 3
                            else "regular",
                            "tightness_bin": "low"
                            if record["fit_tightness"] < 0.5
                            else "medium"
                            if record["fit_tightness"] < 0.8
                            else "high",
                            "radial_bin": "center"
                            if radial < 0.35
                            else "mid"
                            if radial < 0.7
                            else "edge",
                        }
                    )
    return rows


def _account_source_instances(config: Config) -> dict[str, Any]:
    by_source: dict[str, Counter[str]] = defaultdict(Counter)
    missing_rows = []
    for dataset in config.datasets:
        filtered = config.workspace_root / "intermediate/filtered" / dataset
        detection = config.workspace_root / "intermediate/detection" / dataset
        for split, image_path, annotation_path in iter_project_images(filtered):
            filtered_data = read_json(annotation_path)
            eligible, duplicates = deduplicate_geometry_representations(
                filtered_data.get("objects", [])
            )
            detection_path = detection / split / "ann" / f"{image_path.name}.json"
            output_objects = (
                read_json(detection_path).get("objects", [])
                if detection_path.exists()
                else []
            )
            eligible_by_id = {
                str(obj.get("sourceAnnotationId", obj.get("id", index))): obj
                for index, obj in enumerate(eligible)
            }
            output_by_id = {
                str(obj.get("sourceAnnotationId", obj.get("id", index))): obj
                for index, obj in enumerate(output_objects)
            }
            counts = by_source[dataset]
            counts["source_representations"] += len(filtered_data.get("objects", []))
            counts["deduplicated_representations"] += duplicates
            counts["resolved_source_ignore"] += 0
            counts["eligible_positive_instances"] += sum(
                not bool(obj.get("ignoreRegion")) for obj in eligible
            )
            counts["source_policy_ignore_instances"] += sum(
                bool(obj.get("ignoreRegion")) for obj in eligible
            )
            for source_id, source_object in eligible_by_id.items():
                converted = output_by_id.get(source_id)
                if converted is None:
                    counts["unresolved"] += 1
                    missing_rows.append(
                        {
                            "source_dataset": dataset,
                            "split": split,
                            "image": image_path.name,
                            "source_annotation_id": source_id,
                        }
                    )
                elif source_object.get("ignoreRegion"):
                    counts["resolved_source_ignore"] += 1
                elif converted.get("ignoreRegion"):
                    counts["geometry_remainder_ignore"] += 1
                else:
                    counts["accepted_geometry"] += 1
            counts["unexpected_output"] += len(set(output_by_id) - set(eligible_by_id))
    sources = {}
    totals: Counter[str] = Counter()
    for dataset, counts in sorted(by_source.items()):
        totals.update(counts)
        eligible = counts["eligible_positive_instances"]
        sources[dataset] = {
            **dict(counts),
            "accepted_geometry_rate": counts["accepted_geometry"] / eligible
            if eligible
            else 0.0,
            "resolved_positive_rate": (
                counts["accepted_geometry"] + counts["geometry_remainder_ignore"]
            )
            / eligible
            if eligible
            else 0.0,
        }
    eligible = totals["eligible_positive_instances"]
    return {
        "definition": "eligible positives are taxonomy-retained non-ignore source objects after documented mixed-geometry deduplication; source crowds/stuff ignores are accounted separately",
        "sources": sources,
        "totals": {
            **dict(totals),
            "accepted_geometry_rate": totals["accepted_geometry"] / eligible
            if eligible
            else 0.0,
            "resolved_positive_rate": (
                totals["accepted_geometry"] + totals["geometry_remainder_ignore"]
            )
            / eligible
            if eligible
            else 0.0,
        },
        "unresolved_instances": missing_rows,
    }


def _open_world_views(
    manifests: dict[str, dict[str, Any]],
    destination: Path,
) -> dict[str, Any]:
    destination.mkdir(parents=True, exist_ok=True)
    counts: Counter[str] = Counter()
    output = {}
    for split, manifest in manifests.items():
        view = {
            "schema_version": OPEN_WORLD_SCHEMA_VERSION,
            "base_schema_version": manifest["schema_version"],
            "split": split,
            "protocol": "all-source training with COCO VOC-20 seen versus COCO-60 unseen",
            "seen_categories": sorted(VOC20_SEEN),
            "object_contract": manifest["object_contract"],
            "images": [],
        }
        for original_image in manifest["images"]:
            image = {
                **original_image,
                "positive": [],
                "ignore": [dict(row) for row in original_image["ignore"]],
            }
            for record in original_image["positive"]:
                copied = dict(record)
                is_coco = original_image["source_dataset"] == "coco_2017"
                seen = copied["source_category"] in VOC20_SEEN
                copied["seen_status"] = (
                    "seen" if seen else "unseen"
                ) if is_coco else "auxiliary"
                counts[f"{split}_{copied['seen_status']}"] += 1
                if split == "train" and is_coco and not seen:
                    copied["original_state"] = "positive"
                    copied["state"] = "ignore"
                    copied["ignore_reason"] = "withheld_open_world_category"
                    image["ignore"].append(copied)
                else:
                    image["positive"].append(copied)
            view["images"].append(image)
        path = destination / f"open_world_coco_{split}.json"
        write_json(path, view, compact=True)
        output[split] = view
    train_unseen_positive = sum(
        row.get("seen_status") == "unseen"
        for image in output["train"]["images"]
        for row in image["positive"]
    )
    return {
        "protocol": "all-source training with COCO VOC-20 seen versus COCO-60 unseen",
        "counts": dict(counts),
        "train_unseen_positive_leakage": train_unseen_positive,
        "pass": train_unseen_positive == 0
        and counts["train_unseen"] > 0
        and counts["val_unseen"] > 0,
    }


def _sample(rows: list[dict[str, Any]], count: int, seed: int) -> list[dict[str, Any]]:
    if count > len(rows):
        raise ValueError(
            f"requested {count} audit instances but only {len(rows)} exist"
        )
    groups: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        key = tuple(
            str(row[field])
            for field in (
                "source_dataset",
                "split",
                "state",
                "geometry_tier",
                "size_bin",
                "aspect_bin",
                "tightness_bin",
                "radial_bin",
            )
        )
        groups[key].append(row)
    rng = random.Random(seed)
    for values in groups.values():
        values.sort(key=lambda row: row["audit_id"])
        rng.shuffle(values)
    ordered_keys = sorted(groups)
    rng.shuffle(ordered_keys)
    selected = []
    while len(selected) < count:
        progress = False
        for key in ordered_keys:
            if groups[key] and len(selected) < count:
                selected.append(groups[key].pop())
                progress = True
        if not progress:
            break
    return sorted(selected, key=lambda row: row["audit_id"])


def _draw_overlay(
    source: Path,
    destination: Path,
    selected: dict[str, Any],
    context: list[dict[str, Any]],
) -> None:
    with Image.open(source) as loaded:
        image = loaded.convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    for row in context:
        color = (53, 199, 89, 90) if row["state"] == "positive" else (255, 159, 10, 90)
        points = [tuple(map(float, point)) for point in row["quad"]]
        draw.line(points + [points[0]], fill=color, width=2)
    points = [tuple(map(float, point)) for point in selected["quad"]]
    color = (
        (0, 215, 255, 155) if selected["state"] == "positive" else (255, 55, 95, 155)
    )
    draw.line(
        points + [points[0]], fill=color, width=max(4, round(min(image.size) / 300))
    )
    font = ImageFont.load_default(size=max(12, round(min(image.size) / 80)))
    for index, point in enumerate(points):
        radius = max(5, round(min(image.size) / 250))
        draw.ellipse(
            (
                point[0] - radius,
                point[1] - radius,
                point[0] + radius,
                point[1] + radius,
            ),
            fill=color,
        )
        draw.text(
            (point[0] + radius, point[1] + radius),
            str(index),
            fill=(255, 255, 255, 220),
            font=font,
            stroke_width=2,
            stroke_fill=(0, 0, 0, 190),
        )
    image = Image.alpha_composite(image, overlay).convert("RGB")
    xs, ys = [point[0] for point in points], [point[1] for point in points]
    padding = max(max(xs) - min(xs), max(ys) - min(ys), 32) * 0.35
    crop_box = (
        max(0, int(min(xs) - padding)),
        max(0, int(min(ys) - padding)),
        min(image.width, int(max(xs) + padding)),
        min(image.height, int(max(ys) + padding)),
    )
    crop = image.crop(crop_box)
    image.thumbnail((720, 520))
    crop = ImageOps.contain(crop, (520, 520), Image.Resampling.LANCZOS)
    canvas = Image.new(
        "RGB",
        (image.width + crop.width + 12, max(image.height, crop.height)),
        "#15171a",
    )
    canvas.paste(image, (0, 0))
    canvas.paste(crop, (image.width + 12, 0))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, "JPEG", quality=90, optimize=True)


def _coverage(rows: Iterable[dict[str, Any]]) -> dict[str, Any]:
    rows = list(rows)
    return {
        field: dict(sorted(Counter(str(row[field]) for row in rows).items()))
        for field in (
            "source_dataset",
            "split",
            "state",
            "geometry_tier",
            "size_bin",
            "aspect_bin",
            "tightness_bin",
            "radial_bin",
        )
    }


def _write_html(destination: Path, rows: list[dict[str, Any]]) -> None:
    cards = []
    for row in rows:
        metadata = " · ".join(
            html.escape(str(row[field]))
            for field in (
                "source_dataset",
                "split",
                "state",
                "geometry_tier",
                "source_category",
                "size_bin",
                "aspect_bin",
                "radial_bin",
            )
        )
        cards.append(f"""
<article class="card" data-source="{html.escape(str(row["source_dataset"]))}" data-tier="{html.escape(str(row["geometry_tier"]))}" data-state="{row["state"]}">
  <h3>{row["audit_id"]}</h3><p>{metadata}</p>
  <img loading="lazy" src="assets/{row["audit_id"]}.jpg" alt="quad overlay {row["audit_id"]}">
  <p>coverage={row["fit_coverage"]:.4f} · tightness={row["fit_tightness"]:.4f} · source annotation={html.escape(str(row["source_annotation_id"]))}</p>
  <label>Decision <select class="decision"><option></option><option>PASS</option><option>FAIL</option><option>UNCERTAIN</option></select></label>
  <label>Issue <select class="issue"><option></option><option>geometry</option><option>coverage</option><option>tightness</option><option>state</option><option>object_policy</option><option>duplicate</option><option>other</option></select></label>
  <label>Notes <input class="notes"></label>
</article>""")
    document = (
        """<!doctype html><html><head><meta charset="utf-8"><title>G1 quad audit</title>
<style>body{font:14px system-ui;background:#101216;color:#eee;margin:18px}button,select,input{font:inherit}.legend{position:sticky;top:0;background:#20242b;padding:12px;z-index:2}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:16px}.card{background:#20242b;padding:12px;border-radius:8px}.card img{width:100%;max-height:540px;object-fit:contain;background:#15171a}.card label{display:block;margin:8px 0}.card input{width:70%}.ok{color:#35c759}.warn{color:#ff9f0a}</style></head><body>
<div class="legend"><b>Selected quad:</b> <span class="ok">cyan = positive</span>, <span style="color:#ff375f">red = ignore</span>. Thin context: green positive, orange ignore. Vertex numbers show canonical order. <label>Reviewer <input id="reviewer"></label> <button id="export">Export decisions CSV</button> <span id="progress"></span></div>
<p>Inspect object-policy membership, vertex/edge fit, visible-object containment, excess background (tightness), and positive/ignore state. Open <code>REVIEW_INSTRUCTIONS.md</code> before signing off.</p><main class="grid">"""
        + "".join(cards)
        + """</main>
<script>
const cards=[...document.querySelectorAll('.card')]; const key='quad-g1-audit-v1'; let saved=JSON.parse(localStorage.getItem(key)||'{}'); document.querySelector('#reviewer').value=saved.reviewer||'';
function update(){let done=0;cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{};['decision','issue','notes'].forEach(k=>{let e=c.querySelector('.'+k);if(document.activeElement!==e)e.value=v[k]||''});if(v.decision)done++});saved.reviewer=document.querySelector('#reviewer').value;document.querySelector('#progress').textContent=`${done}/${cards.length} decided`;localStorage.setItem(key,JSON.stringify(saved))}
cards.forEach(c=>c.addEventListener('input',()=>{let id=c.querySelector('h3').textContent;saved[id]={decision:c.querySelector('.decision').value,issue:c.querySelector('.issue').value,notes:c.querySelector('.notes').value};update()}));
document.querySelector('#reviewer').addEventListener('input',update); document.querySelector('#export').onclick=()=>{let lines=['audit_id,decision,systematic_issue,notes,reviewer'];cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{},q=x=>'"'+String(x||'').replaceAll('"','""')+'"';lines.push([id,v.decision,v.issue,v.notes,saved.reviewer].map(q).join(','))});let a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/csv'}));a.download='review_decisions_completed.csv';a.click()};update();
</script></body></html>"""
    )
    destination.write_text(document, encoding="utf-8")


def _write_instructions(destination: Path, count: int) -> None:
    destination.write_text(
        f"""# G1 Visual Audit Instructions

This bundle contains a deterministic, stratified sample of **{count} instances**. It is a decision aid, not an automatic pass certificate.

## What the overlay means

- Thick cyan: selected positive proposal target.
- Thick red: selected ignore region.
- Thin green/orange: other positive/ignore regions in the same image for context.
- Vertex labels `0..3`: stored canonical clockwise order.
- Left panel: whole image. Right panel: enlarged local crop.
- `fit_coverage`: fraction of visible source geometry contained by the fitted quad; the hard floor is 0.98.
- `fit_tightness`: source geometry area divided by fitted-quad area. Low is not automatically wrong, but repeated low values may expose a loose-fit systematic error.

## Review procedure

1. Run `python3 verify_bundle.py`. Stop if any file is missing or has a mismatched SHA-256.
2. Serve the bundle locally: `python3 -m http.server 8000 --directory <bundle-directory>`.
3. Open `http://localhost:8000/index.html`; do not review only the crop.
4. Enter the reviewer name. For every card, select PASS, FAIL, or UNCERTAIN. Record the issue type and a concrete note for failures.
5. Mark FAIL when the quad misses visible object geometry, includes implausible excess background, targets the wrong physical instance, has the wrong positive/ignore state, or exposes duplicate/nested-instance policy inconsistency.
6. Use UNCERTAIN only for genuinely ambiguous source annotation; resolve those against the source annotation before gate sign-off.
7. Click **Export decisions CSV**. Preserve the downloaded file beside this bundle; do not overwrite the blank hashed template.
8. Run `python3 finalize_review.py review_decisions_completed.csv`. It rejects missing, duplicate, unknown, blank, or uncertain decisions and writes `review_summary.json` beside the CSV.
9. Group failures by `systematic_issue`, source, and geometry tier using `audit_instances.jsonl`. Any repeated conversion pattern is unresolved even if the raw failure percentage is small.

## Sign-off rule

G1 visual evidence passes only after all {count} rows have a final PASS/FAIL decision, zero UNCERTAIN rows remain, and there is no unresolved systematic conversion error. A failed isolated source annotation may be documented as a source-label defect; a repeated fitting, state, or object-policy failure requires fixing the pipeline and regenerating the bundle.

The machine report `audit_report.json` is authoritative for automated checks. `review_decisions.csv` is a blank fallback template; the browser export is the completed human evidence.
""",
        encoding="utf-8",
    )


def _write_bundle_tools(destination: Path) -> None:
    (destination / "verify_bundle.py").write_text(
        """from __future__ import annotations
import hashlib, json
from pathlib import Path
root = Path(__file__).resolve().parent
manifest = json.loads((root / "bundle_manifest.json").read_text(encoding="utf-8"))
errors = []
for row in manifest["files"]:
    path = root / row["path"]
    if not path.is_file():
        errors.append(f"missing: {row['path']}")
        continue
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != row["sha256"]:
        errors.append(f"hash mismatch: {row['path']}")
if errors:
    raise SystemExit("BUNDLE INVALID\\n" + "\\n".join(errors))
print(f"BUNDLE VALID: {len(manifest['files'])} files verified")
""",
        encoding="utf-8",
    )
    (destination / "finalize_review.py").write_text(
        """from __future__ import annotations
import csv, json, sys
from collections import Counter
from pathlib import Path
root = Path(__file__).resolve().parent
decisions_path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "review_decisions_completed.csv"
expected = {json.loads(line)["audit_id"] for line in (root / "audit_instances.jsonl").read_text(encoding="utf-8").splitlines()}
with decisions_path.open(newline="", encoding="utf-8") as stream:
    rows = list(csv.DictReader(stream))
ids = [row.get("audit_id", "") for row in rows]
counts = Counter(row.get("decision", "").strip().upper() for row in rows)
duplicates = sorted(key for key, value in Counter(ids).items() if key and value > 1)
unknown = sorted(set(ids) - expected)
missing = sorted(expected - set(ids))
complete = not (duplicates or unknown or missing or counts[""] or counts["UNCERTAIN"])
summary = {
    "expected": len(expected), "submitted": len(rows), "decision_counts": dict(counts),
    "duplicates": duplicates, "unknown": unknown, "missing": missing,
    "review_complete": complete,
    "visual_gate_candidate": complete and counts["FAIL"] == 0,
    "status": "candidate_pass" if complete and counts["FAIL"] == 0 else "requires_adjudication",
}
output = decisions_path.with_name("review_summary.json")
output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\\n", encoding="utf-8")
print(json.dumps(summary, indent=2, sort_keys=True))
if not complete:
    raise SystemExit("review is incomplete or contains unresolved decisions")
""",
        encoding="utf-8",
    )


def _bundle_manifest(destination: Path) -> dict[str, Any]:
    files = []
    for path in sorted(
        item
        for item in destination.rglob("*")
        if item.is_file() and item.name != "bundle_manifest.json"
    ):
        files.append(
            {
                "path": path.relative_to(destination).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
        )
    manifest = {"schema_version": AUDIT_SCHEMA_VERSION, "files": files}
    write_json(destination / "bundle_manifest.json", manifest)
    return manifest


def recover_review_text(
    bundle: Path, review_text: Path, output_dir: Path
) -> dict[str, Any]:
    """Recover browser review choices from a copied audit-page text dump."""
    expected = {
        row["audit_id"]: row
        for row in (
            json.loads(line)
            for line in (bundle / "audit_instances.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        )
    }
    text = review_text.read_text(encoding="utf-8")
    reviewer_match = re.search(r"Reviewer[^\S\n]*\n([^\n]+)", text)
    reviewer = reviewer_match.group(1).strip() if reviewer_match else "unknown"
    parsed = []
    for block in re.split(r"(?m)^(?=[0-9a-f]{16}$)", text):
        id_match = re.match(r"(?m)^([0-9a-f]{16})$", block)
        if not id_match:
            continue
        audit_id = id_match.group(1)
        decision_match = re.search(
            r"(?m)^Decision\s*\n(?:(PASS|FAIL|UNCERTAIN)\n)?", block
        )
        issue_match = re.search(
            r"(?m)^Issue\s*\n(?:(coverage|geometry|object_policy|state|tightness|duplicate|other)\n)?",
            block,
        )
        parsed.append(
            {
                "audit_id": audit_id,
                "decision": (
                    decision_match.group(1)
                    if decision_match and decision_match.group(1)
                    else ""
                ),
                "systematic_issue": (
                    issue_match.group(1) if issue_match and issue_match.group(1) else ""
                ),
                "notes": "",
                "reviewer": reviewer,
            }
        )
    ids = [row["audit_id"] for row in parsed]
    duplicates = sorted(key for key, value in Counter(ids).items() if value > 1)
    unknown = sorted(set(ids) - set(expected))
    missing = sorted(set(expected) - set(ids))
    decisions = Counter(row["decision"] or "BLANK" for row in parsed)
    fail_by_source: Counter[str] = Counter()
    fail_by_tier: Counter[str] = Counter()
    fail_by_issue: Counter[str] = Counter()
    for row in parsed:
        if row["decision"] != "FAIL" or row["audit_id"] not in expected:
            continue
        metadata = expected[row["audit_id"]]
        fail_by_source[str(metadata["source_dataset"])] += 1
        fail_by_tier[str(metadata["geometry_tier"])] += 1
        fail_by_issue[row["systematic_issue"] or "unspecified"] += 1
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "review_decisions_recovered.csv"
    write_csv(
        csv_path,
        ["audit_id", "decision", "systematic_issue", "notes", "reviewer"],
        parsed,
    )
    complete = not (
        duplicates or unknown or missing or decisions["BLANK"] or decisions["UNCERTAIN"]
    )
    summary = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "source_text": str(review_text.resolve()),
        "source_text_sha256": _sha256(review_text),
        "reviewer": reviewer,
        "expected": len(expected),
        "recovered": len(parsed),
        "decision_counts": dict(sorted(decisions.items())),
        "fail_by_source": dict(sorted(fail_by_source.items())),
        "fail_by_geometry_tier": dict(sorted(fail_by_tier.items())),
        "fail_by_issue": dict(sorted(fail_by_issue.items())),
        "duplicates": duplicates,
        "unknown": unknown,
        "missing": missing,
        "review_complete": complete,
        "visual_gate_candidate": complete and decisions["FAIL"] == 0,
        "status": (
            "candidate_pass"
            if complete and decisions["FAIL"] == 0
            else "requires_automated_adjudication"
        ),
        "recovered_csv": str(csv_path.resolve()),
        "note": "Copied page text preserves decisions and issue selections but not free-form input values.",
    }
    write_json(output_dir / "review_recovery_summary.json", summary)
    return summary


def generate_g1_bundle(
    config: Config,
    count: int = 300,
    destination: Path | None = None,
) -> dict[str, Any]:
    """Generate machine evidence, open-world views, and a human visual audit."""
    destination = (destination or config.reports / "g1_audit_bundle").resolve()
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    manifests = {
        split: read_json(
            config.workspace_root / f"output/annotations/proposals_{split}.json"
        )
        for split in ("train", "val")
    }
    rows = _instance_rows(manifests)
    selected = _sample(rows, count, int(config.validation["random_seed"]))
    by_image: dict[tuple[str, int], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_image[(row["split"], row["image_id"])].append(row)
    for row in selected:
        source = config.workspace_root / "output" / row["file_name"]
        _draw_overlay(
            source,
            destination / "assets" / f"{row['audit_id']}.jpg",
            row,
            by_image[(row["split"], row["image_id"])],
        )
    open_world = _open_world_views(manifests, destination / "open_world")
    accounting = _account_source_instances(config)
    split_ids = {
        split: {
            (image["source_dataset"], image["source_image_id"])
            for image in manifest["images"]
        }
        for split, manifest in manifests.items()
    }
    overlap = sorted(split_ids["train"] & split_ids["val"])
    inventory = _coverage(rows)
    sampled_coverage = _coverage(selected)
    all_sources = set(inventory["source_dataset"])
    all_tiers = set(inventory["geometry_tier"])
    sampled_complete = all_sources <= set(
        sampled_coverage["source_dataset"]
    ) and all_tiers <= set(sampled_coverage["geometry_tier"])
    trusted_count = sum(
        len(image.get("trusted_background", []))
        for manifest in manifests.values()
        for image in manifest["images"]
    )
    positive_count = sum(
        len(image["positive"])
        for manifest in manifests.values()
        for image in manifest["images"]
    )
    coverage_failures = sum(row["fit_coverage"] < 0.98 for row in rows)
    determinism_path = config.reports / "g1_determinism.json"
    determinism = (
        read_json(determinism_path)
        if determinism_path.exists()
        else {"pass": False, "status": "not_run"}
    )
    policy_path = config.reports / "g1_automatic_policy" / "policy_report.json"
    owner_policy = read_json(policy_path) if policy_path.exists() else {}
    current_hashes = {
        split: _sha256(
            config.workspace_root / f"output/annotations/proposals_{split}.json"
        )
        for split in manifests
    }
    owner_review_complete = bool(
        owner_policy.get("owner_acceptance", {}).get("accepted")
        and owner_policy.get("input_hashes") == current_hashes
        and not owner_policy.get("automatic_failures")
    )
    checks = {
        "determinism_and_raw_immutability": bool(determinism.get("pass")),
        "train_val_identity_overlap_zero": not overlap,
        "open_world_category_leakage_zero": bool(open_world["pass"]),
        "eligible_geometry_rate_at_least_0_98": accounting["totals"][
            "accepted_geometry_rate"
        ]
        >= 0.98,
        "every_eligible_instance_resolved": accounting["totals"][
            "resolved_positive_rate"
        ]
        == 1.0,
        "per_record_coverage_floor": coverage_failures == 0,
        "four_sources_and_all_geometry_tiers_sampled": sampled_complete,
        "at_least_300_visual_instances": len(selected) >= 300,
        "explicit_trusted_background_regions_present": trusted_count > 0,
        "owner_accepted_automatic_review_complete": owner_review_complete,
    }
    report = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "scope": {
            "status": "bounded acceptance sample",
            "images": sum(len(manifest["images"]) for manifest in manifests.values()),
            "records": len(rows),
            "visual_sample": len(selected),
            "warning": "This is not a full-corpus production audit if upstream filtering used --limit-images.",
        },
        "input_hashes": {
            split: _sha256(
                config.workspace_root / f"output/annotations/proposals_{split}.json"
            )
            for split in manifests
        },
        "checks": checks,
        "automated_checks_pass": all(
            value
            for key, value in checks.items()
            if key != "owner_accepted_automatic_review_complete"
        ),
        "g1_pass": all(checks.values()),
        "determinism": determinism,
        "source_accounting": accounting,
        "manifest_inventory": inventory,
        "fit_coverage_failures": coverage_failures,
        "positive_records": positive_count,
        "supervision_state": {
            "trusted_background_regions": trusted_count,
            "background_supervision_images": sum(
                image.get("background_supervision", False)
                for manifest in manifests.values()
                for image in manifest["images"]
            ),
            "interpretation": "background_supervision is true only when the manifest carries explicit trusted-background tiles for that image.",
        },
        "identity_overlap": [
            {"source_dataset": row[0], "source_image_id": row[1]} for row in overlap
        ],
        "open_world": open_world,
        "sample_coverage": sampled_coverage,
        "review": {
            "status": "complete" if owner_review_complete else "pending",
            "policy_report": str(policy_path.resolve()) if policy_path.exists() else None,
            "owner_accepted_automatic_labels": owner_review_complete,
            "instructions": "REVIEW_INSTRUCTIONS.md",
        },
    }
    write_json(destination / "audit_report.json", report)
    with (destination / "audit_instances.jsonl").open("w", encoding="utf-8") as stream:
        for row in selected:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    write_csv(
        destination / "review_decisions.csv",
        [
            "audit_id",
            "decision",
            "systematic_issue",
            "notes",
            "reviewer",
        ],
        ({"audit_id": row["audit_id"]} for row in selected),
    )
    _write_html(destination / "index.html", selected)
    _write_instructions(destination / "REVIEW_INSTRUCTIONS.md", len(selected))
    _write_bundle_tools(destination)
    _bundle_manifest(destination)
    return report
