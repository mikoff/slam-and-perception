"""Build the Phase 1.2 official-identity and crowd review bundle."""

from __future__ import annotations

import csv
import hashlib
import heapq
import html
import re
import shutil
from collections import Counter
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

from .config import Config
from .detection_conversion import object_bbox
from .reports import read_json, write_json
from .supervisely_filter import iter_project_images


def _priority(row: dict[str, Any]) -> str:
    identity = ":".join(
        (
            str(row["sourceSplit"]),
            str(row["sourceImageIdentity"]),
            str(row["sourceAnnotationId"]),
        )
    )
    return hashlib.sha256(identity.encode("utf-8")).hexdigest()


def _points(obj: dict[str, Any]) -> list[list[tuple[float, float]]]:
    if obj.get("geometryType") == "multipolygon":
        return [
            [tuple(map(float, point)) for point in segment]
            for segment in obj.get("segments", [])
        ]
    exterior = obj.get("points", {}).get("exterior", [])
    if len(exterior) == 2:
        x1, y1, x2, y2 = object_bbox(obj)
        exterior = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
    return [[tuple(map(float, point)) for point in exterior]]


def _draw(
    image_path: Path,
    destination: Path,
    objects: list[dict[str, Any]],
) -> None:
    with Image.open(image_path) as loaded:
        image = loaded.convert("RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    colors = [(0, 215, 255, 235), (255, 55, 95, 235), (52, 199, 89, 235)]
    bounds = []
    for index, obj in enumerate(objects):
        color = colors[index % len(colors)]
        for polygon in _points(obj):
            if len(polygon) >= 2:
                draw.line(polygon + [polygon[0]], fill=color, width=4)
        x1, y1, x2, y2 = object_bbox(obj)
        bounds.append((x1, y1, x2, y2))
        draw.text(
            (x1 + 3, y1 + 3),
            str(obj["sourceAnnotationId"]),
            fill=(255, 255, 255, 255),
            stroke_width=2,
            stroke_fill=(0, 0, 0, 230),
        )
    x1 = min(row[0] for row in bounds)
    y1 = min(row[1] for row in bounds)
    x2 = max(row[2] for row in bounds)
    y2 = max(row[3] for row in bounds)
    padding = max(x2 - x1, y2 - y1, 32.0) * 0.5
    crop = image.crop(
        (
            max(0, int(x1 - padding)),
            max(0, int(y1 - padding)),
            min(image.width, int(x2 + padding)),
            min(image.height, int(y2 + padding)),
        )
    )
    image.thumbnail((720, 520))
    crop.thumbnail((520, 520))
    canvas = Image.new(
        "RGB",
        (image.width + crop.width + 12, max(image.height, crop.height)),
        "#15171a",
    )
    canvas.paste(image, (0, 0))
    canvas.paste(crop, (image.width + 12, 0))
    destination.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(destination, "JPEG", quality=90, optimize=True)


def _write_html(destination: Path, rows: list[dict[str, Any]]) -> None:
    cards = []
    for row in rows:
        metadata = html.escape(
            " · ".join(
                (
                    row["kind"],
                    row["split"],
                    row["source_category"],
                    row["supervision_state"],
                    f"official IDs={','.join(row['annotation_ids'])}",
                )
            )
        )
        cards.append(
            f"""<article class="card"><h3>{row["audit_id"]}</h3><p>{metadata}</p>
<img loading="lazy" src="assets/{row["audit_id"]}.jpg" alt="identity overlay">
<label>Decision <select class="decision"><option></option><option>PASS</option><option>FAIL</option><option>UNCERTAIN</option></select></label>
<label>Issue <select class="issue"><option></option><option>identity</option><option>multipart</option><option>crowd_state</option><option>geometry</option><option>other</option></select></label>
<label>Notes <input class="notes"></label></article>"""
        )
    destination.write_text(
        """<!doctype html><html><head><meta charset="utf-8"><title>COCO identity audit</title>
<style>body{font:14px system-ui;background:#101216;color:#eee;margin:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:16px}.card{background:#20242b;padding:12px;border-radius:8px}.card img{width:100%;max-height:540px;object-fit:contain}.card label{display:block;margin:8px 0}.card input{width:70%}.bar{position:sticky;top:0;background:#20242b;padding:12px;z-index:2}</style></head><body>
<div class="bar"><label>Reviewer <input id="reviewer"></label> <button id="export">Export decisions CSV</button> <span id="progress"></span></div>
<p>Each group card shows every distinct official annotation identity with coincident source geometry. Sample cards audit ordinary, multipart, and crowd normalization.</p><main class="grid">"""
        + "".join(cards)
        + """</main><script>
const cards=[...document.querySelectorAll('.card')],key='coco-identity-audit-v1';let saved=JSON.parse(localStorage.getItem(key)||'{}');document.querySelector('#reviewer').value=saved.reviewer||'';
function update(){let done=0;cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{};['decision','issue','notes'].forEach(k=>{let e=c.querySelector('.'+k);if(document.activeElement!==e)e.value=v[k]||''});if(v.decision)done++});saved.reviewer=document.querySelector('#reviewer').value;document.querySelector('#progress').textContent=`${done}/${cards.length} decided`;localStorage.setItem(key,JSON.stringify(saved))}
cards.forEach(c=>c.addEventListener('input',()=>{let id=c.querySelector('h3').textContent;saved[id]={decision:c.querySelector('.decision').value,issue:c.querySelector('.issue').value,notes:c.querySelector('.notes').value};update()}));document.querySelector('#reviewer').addEventListener('input',update);
document.querySelector('#export').onclick=()=>{let lines=['audit_id,decision,issue,notes,reviewer'];cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{},q=x=>'"'+String(x||'').replaceAll('"','""')+'"';lines.push([id,v.decision,v.issue,v.notes,saved.reviewer].map(q).join(','))});let a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.join('\n')],{type:'text/csv'}));a.download='identity_review_completed.csv';a.click()};update();
</script></body></html>""",
        encoding="utf-8",
    )


def generate_identity_audit_bundle(
    config: Config, count: int, output: Path | None = None
) -> dict[str, Any]:
    """Render a deterministic sample plus every distinct-ID geometry group."""
    destination = (output or config.reports / "coco_identity_audit_bundle").resolve()
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    provenance = read_json(config.reports / "source_provenance.json")
    source = next(
        (row for row in provenance if row.get("dataset") == "coco_2017"), None
    )
    if source is None:
        raise ValueError("COCO source provenance is missing; run filter first")
    groups = source.get("high_multiplicity_exact_geometry_groups", [])
    group_by_identity = {
        (str(group["split"]), str(annotation_id)): index
        for index, group in enumerate(groups)
        for annotation_id in group["annotation_ids"]
    }
    found_groups: dict[int, list[tuple[Path, dict[str, Any]]]] = {}
    sample_heap: list[tuple[int, str, Path, dict[str, Any]]] = []
    project = config.workspace_root / "intermediate" / "filtered" / "coco_2017"
    for split, image_path, annotation_path in iter_project_images(project):
        annotation = read_json(annotation_path)
        for obj in annotation.get("objects", []):
            group_index = group_by_identity.get(
                (split, str(obj.get("sourceAnnotationId", "")))
            )
            if group_index is not None:
                found_groups.setdefault(group_index, []).append((image_path, obj))
            else:
                priority = _priority(obj)
                heapq.heappush(
                    sample_heap, (-int(priority, 16), priority, image_path, obj)
                )
                if len(sample_heap) > count:
                    heapq.heappop(sample_heap)
    missing_groups = sorted(set(range(len(groups))) - set(found_groups))
    if missing_groups:
        raise ValueError(
            f"identity audit could not resolve {len(missing_groups)} groups"
        )

    rows = []
    for index, group in enumerate(groups):
        members = found_groups[index]
        image_paths = {path for path, _ in members}
        if len(image_paths) != 1:
            raise ValueError(f"identity group {index} spans multiple images")
        objects = [obj for _, obj in members]
        audit_id = f"group-{index:05d}"
        _draw(
            next(iter(image_paths)), destination / "assets" / f"{audit_id}.jpg", objects
        )
        rows.append(
            {
                "audit_id": audit_id,
                "kind": "all_coincident_geometry_group",
                "split": str(group["split"]),
                "source_category": str(objects[0]["sourceCategory"]),
                "supervision_state": str(objects[0]["supervisionState"]),
                "annotation_ids": [str(obj["sourceAnnotationId"]) for obj in objects],
            }
        )
    sample_candidates = sorted(
        ((priority, image_path, obj) for _, priority, image_path, obj in sample_heap),
        key=lambda row: row[0],
    )
    for index, (_, image_path, obj) in enumerate(sample_candidates):
        audit_id = f"sample-{index:05d}"
        _draw(image_path, destination / "assets" / f"{audit_id}.jpg", [obj])
        rows.append(
            {
                "audit_id": audit_id,
                "kind": "deterministic_random_identity",
                "split": str(obj["sourceSplit"]),
                "source_category": str(obj["sourceCategory"]),
                "supervision_state": str(obj["supervisionState"]),
                "annotation_ids": [str(obj["sourceAnnotationId"])],
            }
        )
    rows.sort(key=lambda row: row["audit_id"])
    write_json(destination / "audit_instances.json", rows)
    with (destination / "review_decisions.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["audit_id", "decision", "issue", "notes", "reviewer"]
        )
        writer.writeheader()
        for row in rows:
            writer.writerow({"audit_id": row["audit_id"]})
    _write_html(destination / "index.html", rows)
    summary = {
        "schema_version": "coco-identity-audit.v1",
        "random_sample_requested": count,
        "random_sample_rendered": sum(
            row["kind"].startswith("deterministic") for row in rows
        ),
        "high_multiplicity_groups_rendered": len(groups),
        "total_cards": len(rows),
        "review_status": "pending_owner_review",
    }
    write_json(destination / "audit_report.json", summary)
    return summary


def recover_identity_review_text(
    bundle: Path, review_text: Path, output: Path
) -> dict[str, Any]:
    """Recover identity-audit choices when browser CSV saving is unavailable."""
    expected = {
        row["audit_id"]: row for row in read_json(bundle / "audit_instances.json")
    }
    text = review_text.read_text(encoding="utf-8")
    reviewer_match = re.search(r"Reviewer[^\S\n]*\n([^\n]+)", text)
    reviewer = reviewer_match.group(1).strip() if reviewer_match else "unknown"
    parsed = []
    for block in re.split(r"(?m)^(?=(?:group|sample)-\d{5}$)", text):
        id_match = re.match(r"(?m)^((?:group|sample)-\d{5})$", block)
        if not id_match:
            continue
        decision_match = re.search(
            r"(?m)^Decision\s*\n(?:(PASS|FAIL|UNCERTAIN)\n)?", block
        )
        issue_match = re.search(
            r"(?m)^Issue\s*\n(?:(identity|multipart|crowd_state|geometry|other)\n)?",
            block,
        )
        parsed.append(
            {
                "audit_id": id_match.group(1),
                "decision": decision_match.group(1)
                if decision_match and decision_match.group(1)
                else "",
                "issue": issue_match.group(1)
                if issue_match and issue_match.group(1)
                else "",
                "notes": "",
                "reviewer": reviewer,
            }
        )
    ids = [row["audit_id"] for row in parsed]
    duplicates = sorted(key for key, value in Counter(ids).items() if value > 1)
    unknown = sorted(set(ids) - set(expected))
    missing = sorted(set(expected) - set(ids))
    decisions = Counter(row["decision"] or "BLANK" for row in parsed)
    failures = [
        {**expected[row["audit_id"]], "issue": row["issue"]}
        for row in parsed
        if row["decision"] == "FAIL" and row["audit_id"] in expected
    ]
    blanks = [row["audit_id"] for row in parsed if not row["decision"]]
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "identity_review_decisions_recovered.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["audit_id", "decision", "issue", "notes", "reviewer"],
        )
        writer.writeheader()
        writer.writerows(parsed)
    complete = not (
        duplicates or unknown or missing or decisions["BLANK"] or decisions["UNCERTAIN"]
    )
    summary = {
        "schema_version": "coco-identity-review-recovery.v1",
        "source_text": str(review_text.resolve()),
        "source_text_sha256": hashlib.sha256(review_text.read_bytes()).hexdigest(),
        "reviewer": reviewer,
        "expected": len(expected),
        "recovered": len(parsed),
        "decision_counts": dict(sorted(decisions.items())),
        "failures": failures,
        "blank_decisions": blanks,
        "duplicates": duplicates,
        "unknown": unknown,
        "missing": missing,
        "review_complete": complete,
        "status": "complete" if complete else "requires_owner_resolution",
        "recovered_csv": str(csv_path.resolve()),
        "note": "Copied page text preserves decisions and issue selections but not free-form notes.",
    }
    write_json(output / "identity_review_recovery_summary.json", summary)
    return summary
