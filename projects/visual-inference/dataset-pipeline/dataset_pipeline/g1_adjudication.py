"""Automatic source-consistency adjudication for the G1 visual audit."""

from __future__ import annotations

import csv
import hashlib
import html
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

from .config import Config
from .detection_conversion import GeometryError, _source_points, _valid_source_quad
from .g1_audit import _bundle_manifest, _draw_overlay, _instance_rows
from .reports import read_json, write_csv, write_json


SCHEMA_VERSION = "quad-proposal-g1-auto-adjudication.v1"
POLICY_SCHEMA_VERSION = "quad-proposal-g1-owner-policy.v1"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _signed_area(points: list[list[float]] | list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        float(points[index][0]) * float(points[(index + 1) % len(points)][1])
        - float(points[index][1]) * float(points[(index + 1) % len(points)][0])
        for index in range(len(points))
    )


def _valid_quad(quad: list[list[float]]) -> bool:
    if len(quad) != 4 or not all(
        len(point) == 2 and all(math.isfinite(float(value)) for value in point)
        for point in quad
    ):
        return False
    turns = []
    for index in range(4):
        first, second, third = (
            quad[index],
            quad[(index + 1) % 4],
            quad[(index + 2) % 4],
        )
        turns.append(
            (second[0] - first[0]) * (third[1] - second[1])
            - (second[1] - first[1]) * (third[0] - second[0])
        )
    return _signed_area(quad) > 1e-6 and all(turn > 1e-6 for turn in turns)


def _coverage(points: list[tuple[float, float]], quad: list[list[float]]) -> float:
    inside = 0
    for x, y in points:
        distances = []
        for index in range(4):
            x1, y1 = quad[index]
            x2, y2 = quad[(index + 1) % 4]
            length = math.hypot(x2 - x1, y2 - y1)
            distances.append(
                ((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) / max(length, 1e-7)
            )
        inside += all(value >= -0.25 for value in distances) or all(
            value <= 0.25 for value in distances
        )
    return inside / max(len(points), 1)


def _source_area(geometry: str, points: list[tuple[float, float]]) -> float:
    if geometry in {"bitmap", "mask"}:
        return float(len(points))
    if geometry == "polygon" and len(points) >= 3:
        return abs(_signed_area(points))
    if not points:
        return 0.0
    xs, ys = zip(*points, strict=True)
    return (max(xs) - min(xs)) * (max(ys) - min(ys))


def _source_annotation_path(config: Config, row: dict[str, Any]) -> Path:
    return (
        config.workspace_root
        / "intermediate"
        / "filtered"
        / row["source_dataset"]
        / row["source_split"]
        / "ann"
        / f"{Path(row['source_image_id']).name}.json"
    )


def _source_object(config: Config, row: dict[str, Any]) -> dict[str, Any] | None:
    path = _source_annotation_path(config, row)
    if not path.exists():
        return None
    annotation = read_json(path)
    for index, obj in enumerate(annotation.get("objects", [])):
        source_id = str(obj.get("sourceAnnotationId", obj.get("id", index)))
        if source_id == row["source_annotation_id"]:
            return obj
    return None


def _tier_consistent(
    tier: str, geometry: str, points: list[tuple[float, float]]
) -> bool:
    if tier == "source_hbb":
        return geometry == "rectangle"
    if tier == "source_quad":
        return geometry == "polygon" and _valid_source_quad(points)
    if tier == "fitted_quad":
        return geometry in {"polygon", "bitmap", "mask"} and len(points) != 4
    if tier == "rotated_rect":
        return (
            geometry == "polygon"
            and len(points) == 4
            and not _valid_source_quad(points)
        )
    return tier == "hbb_fallback"


def _state_consistent(
    row: dict[str, Any], source: dict[str, Any], points: list[tuple[float, float]]
) -> bool:
    source_ignore = bool(source.get("ignoreRegion"))
    if row["state"] == "positive":
        geometry = str(source.get("geometryType", "")).lower()
        representable = (
            len(points) >= 2 and _source_area(geometry, points) > 0
            if geometry == "rectangle"
            else len(set(points)) >= 3
        )
        return not source_ignore and representable
    if source_ignore:
        return True
    return (
        len(points) < 3
        or _source_area(str(source.get("geometryType", "")).lower(), points) <= 0
    )


def adjudicate_row(config: Config, row: dict[str, Any]) -> dict[str, Any]:
    """Return a deterministic geometry/source-consistency verdict."""
    source = _source_object(config, row)
    checks: dict[str, bool] = {"source_record_found": source is not None}
    warnings = [
        "semantic object-policy correctness is not inferred from pixels",
        "automatic PASS means source-consistent, not necessarily visually complete",
    ]
    independent_coverage = 0.0
    recomputed_tightness = 0.0
    source_geometry = "missing"
    if source is not None:
        source_geometry = str(source.get("geometryType", "")).lower()
        try:
            points = _source_points(source)
        except GeometryError:
            points = []
        quad = [[float(value) for value in point] for point in row["quad"]]
        quad_area = abs(_signed_area(quad))
        independent_coverage = _coverage(points, quad)
        recomputed_tightness = min(
            _source_area(source_geometry, points) / max(quad_area, 1e-7), 1.0
        )
        checks.update(
            {
                "quad_is_finite_convex_clockwise": _valid_quad(quad),
                "source_geometry_containment_at_least_0_98": independent_coverage
                >= 0.98,
                "reported_coverage_matches_recomputation": abs(
                    independent_coverage - float(row["fit_coverage"])
                )
                <= 1e-6,
                "reported_tightness_matches_recomputation": abs(
                    recomputed_tightness - float(row["fit_tightness"])
                )
                <= 1e-4,
                "geometry_tier_matches_source_shape": _tier_consistent(
                    row["geometry_tier"], source_geometry, points
                ),
                "supervision_state_matches_source_policy": _state_consistent(
                    row, source, points
                ),
            }
        )
    if float(row["fit_tightness"]) < 0.5:
        warnings.append(
            "low tightness requires visual judgment; it is not an automatic failure"
        )
    if row["state"] == "ignore":
        warnings.append("ignore-state semantics require policy judgment")
    result = "PASS" if all(checks.values()) else "FAIL"
    failed = [name for name, passed in checks.items() if not passed]
    return {
        **row,
        "automatic_result": result,
        "automatic_scope": "source_geometry_and_manifest_consistency",
        "automatic_checks": checks,
        "automatic_failed_checks": failed,
        "automatic_warnings": warnings,
        "independent_fit_coverage": independent_coverage,
        "recomputed_fit_tightness": recomputed_tightness,
        "source_geometry_type": source_geometry,
    }


def _read_prior_review(path: Path | None) -> dict[str, dict[str, str]]:
    if path is None or not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        return {row["audit_id"]: row for row in csv.DictReader(stream)}


def _write_html(destination: Path, records: list[dict[str, Any]]) -> None:
    cards = []
    for row in records:
        checks = "".join(
            f'<li class="{"ok" if passed else "bad"}">{"✓" if passed else "✗"} {html.escape(name.replace("_", " "))}</li>'
            for name, passed in row["automatic_checks"].items()
        )
        warnings = "".join(
            f"<li>{html.escape(value)}</li>" for value in row["automatic_warnings"]
        )
        prior = row.get("previous_decision") or "BLANK"
        default = (
            "YES"
            if prior == row["automatic_result"]
            else "NO"
            if prior in {"PASS", "FAIL"}
            else "UNSURE"
        )
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
        cards.append(
            f"""
<article class="card auto-{row["automatic_result"].lower()}" data-auto="{row["automatic_result"]}" data-agreement="{default}" data-action="{row["training_recommendation"]}">
  <div class="verdict">Automatic: <b>{row["automatic_result"]}</b> · Previous review: <b>{prior}</b> · Training: <b>{row["training_recommendation"]}</b></div>
  <h3>{row["audit_id"]}</h3><p>{metadata}</p>
  <img loading="lazy" src="assets/{row["audit_id"]}.jpg" alt="quad overlay {row["audit_id"]}">
  <p>reported coverage={row["fit_coverage"]:.4f} · independent coverage={row["independent_fit_coverage"]:.4f} · reported tightness={row["fit_tightness"]:.4f} · recomputed tightness={row["recomputed_fit_tightness"]:.4f}</p>
  <details><summary>Automatic evidence</summary><ul>{checks}</ul><ul class="warnings">{warnings}</ul></details>
  <label>Was the automatic marking correct?
    <select class="correctness" data-default="{default}"><option>YES</option><option>NO</option><option>UNSURE</option></select>
  </label>
  <label>Short reason <select class="reason"><option></option><option>geometry</option><option>coverage</option><option>tightness</option><option>state</option><option>object_policy</option><option>source_annotation</option><option>other</option></select></label>
  <label>Optional note <input class="notes"></label>
</article>"""
        )
    destination.write_text(
        """<!doctype html><html><head><meta charset="utf-8"><title>G1 automatic adjudication</title>
<style>body{font:14px system-ui;background:#101216;color:#eee;margin:18px}button,select,input{font:inherit}.toolbar{position:sticky;top:0;background:#20242b;padding:12px;z-index:2}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:16px}.card{background:#20242b;padding:12px;border:2px solid #35c759;border-radius:8px}.card.auto-fail{border-color:#ff375f}.card img{width:100%;max-height:560px;object-fit:contain;background:#15171a}.card label{display:block;margin:8px 0}.card input{width:70%}.ok{color:#35c759}.bad{color:#ff375f}.warnings{color:#ffcc00}.verdict{font-size:16px}.hidden{display:none}textarea{width:100%;height:140px;background:#111;color:#eee}</style></head><body>
<div class="toolbar"><b>Selected overlay is translucent:</b> cyan positive, red ignore; thin green/orange are context. <button data-filter="all">All</button> <button data-filter="disagreement">Previous disagreements</button> <button data-filter="FAIL">Auto FAIL</button> <button data-filter="quarantine">Quarantine</button> <label>Reviewer <input id="reviewer"></label> <button id="export">Prepare CSV</button> <button id="copy">Copy CSV</button> <span id="progress"></span></div>
<p><b>Important:</b> automatic PASS proves consistency with source geometry and manifest policy. It does not prove that the source annotation captures the visually complete physical object. Check disagreements first.</p><main class="grid">"""
        + "".join(cards)
        + """</main><section><h2>CSV fallback</h2><p>If downloading is blocked, click Prepare CSV and copy this text.</p><textarea id="csv"></textarea></section>
<script>
const cards=[...document.querySelectorAll('.card')],key='quad-g1-auto-adjudication-v1';let saved=JSON.parse(localStorage.getItem(key)||'{}');const reviewer=document.querySelector('#reviewer');reviewer.value=saved.reviewer||'';
function update(){let done=0;cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{},s=c.querySelector('.correctness');s.value=v.correctness||s.dataset.default;c.querySelector('.reason').value=v.reason||'';c.querySelector('.notes').value=v.notes||'';if(s.value!=='UNSURE')done++});saved.reviewer=reviewer.value;localStorage.setItem(key,JSON.stringify(saved));document.querySelector('#progress').textContent=`${done}/${cards.length} resolved`}
cards.forEach(c=>c.addEventListener('input',()=>{let id=c.querySelector('h3').textContent;saved[id]={correctness:c.querySelector('.correctness').value,reason:c.querySelector('.reason').value,notes:c.querySelector('.notes').value};update()}));reviewer.addEventListener('input',update);
document.querySelectorAll('[data-filter]').forEach(b=>b.onclick=()=>{let f=b.dataset.filter;cards.forEach(c=>c.classList.toggle('hidden',!(f==='all'||(f==='disagreement'&&c.dataset.agreement!=='YES')||(f==='FAIL'&&c.dataset.auto==='FAIL')||(f==='quarantine'&&c.dataset.action!=='retain'))))});
function csv(){let q=x=>'"'+String(x||'').replaceAll('"','""')+'"',lines=['audit_id,automatic_result,previous_decision,automatic_marking_correct,reason,notes,reviewer'];cards.forEach(c=>{let id=c.querySelector('h3').textContent,v=saved[id]||{},auto=c.dataset.auto,prior=c.querySelector('.verdict').textContent.match(/Previous review: (PASS|FAIL|UNCERTAIN|BLANK)/)?.[1]||'';lines.push([id,auto,prior,v.correctness||c.querySelector('.correctness').dataset.default,v.reason,v.notes,saved.reviewer].map(q).join(','))});return lines.join('\n')}
document.querySelector('#export').onclick=()=>{let value=csv();document.querySelector('#csv').value=value;let a=document.createElement('a');a.href=URL.createObjectURL(new Blob([value],{type:'text/csv'}));a.download='automatic_adjudication_review.csv';document.body.appendChild(a);a.click();a.remove()};document.querySelector('#copy').onclick=async()=>{let value=csv();document.querySelector('#csv').value=value;try{await navigator.clipboard.writeText(value)}catch{document.querySelector('#csv').select()}};update();
</script></body></html>""",
        encoding="utf-8",
    )


def _write_verifier(destination: Path) -> None:
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
    elif hashlib.sha256(path.read_bytes()).hexdigest() != row["sha256"]:
        errors.append(f"hash mismatch: {row['path']}")
if errors:
    raise SystemExit("BUNDLE INVALID\\n" + "\\n".join(errors))
print(f"BUNDLE VALID: {len(manifest['files'])} files verified")
""",
        encoding="utf-8",
    )


def generate_adjudication_bundle(
    config: Config,
    audit_bundle: Path,
    prior_review_csv: Path | None,
    destination: Path,
) -> dict[str, Any]:
    """Generate automatic verdicts plus a lightweight correctness review."""
    manifests = {
        split: read_json(
            config.workspace_root / f"output/annotations/proposals_{split}.json"
        )
        for split in ("train", "val")
    }
    all_rows = _instance_rows(manifests)
    rows_by_id = {row["audit_id"]: row for row in all_rows}
    selected_ids = [
        json.loads(line)["audit_id"]
        for line in (audit_bundle / "audit_instances.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    prior = _read_prior_review(prior_review_csv)
    records = []
    for audit_id in selected_ids:
        record = adjudicate_row(config, rows_by_id[audit_id])
        previous = prior.get(audit_id, {}).get("decision", "")
        record["previous_decision"] = previous or "BLANK"
        record["previous_issue"] = prior.get(audit_id, {}).get("systematic_issue", "")
        record["training_recommendation"] = (
            "retain"
            if record["automatic_result"] == "PASS" and previous == "PASS"
            else "quarantine_as_ignore"
        )
        records.append(record)
    if destination.exists():
        import shutil

        shutil.rmtree(destination)
    destination.mkdir(parents=True)
    context: dict[tuple[str, int], list[dict[str, Any]]] = defaultdict(list)
    for row in all_rows:
        context[(row["split"], row["image_id"])].append(row)
    for row in records:
        _draw_overlay(
            config.workspace_root / "output" / row["file_name"],
            destination / "assets" / f"{row['audit_id']}.jpg",
            row,
            context[(row["split"], row["image_id"])],
        )
    with (destination / "adjudication_records.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in records:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    write_csv(
        destination / "automatic_correctness_review.csv",
        [
            "audit_id",
            "automatic_result",
            "previous_decision",
            "automatic_marking_correct",
            "reason",
            "notes",
            "reviewer",
        ],
        (
            {
                "audit_id": row["audit_id"],
                "automatic_result": row["automatic_result"],
                "previous_decision": row["previous_decision"],
                "automatic_marking_correct": (
                    "YES"
                    if row["previous_decision"] == row["automatic_result"]
                    else "NO"
                    if row["previous_decision"] in {"PASS", "FAIL"}
                    else "UNSURE"
                ),
            }
            for row in records
        ),
    )
    write_csv(
        destination / "quarantine_recommendations.csv",
        [
            "audit_id",
            "source_dataset",
            "source_image_id",
            "source_annotation_id",
            "source_category",
            "state",
            "geometry_tier",
            "automatic_result",
            "automatic_failed_checks",
            "previous_decision",
            "previous_issue",
            "recommendation",
        ],
        (
            {
                **row,
                "automatic_failed_checks": ";".join(row["automatic_failed_checks"]),
                "recommendation": row["training_recommendation"],
            }
            for row in records
            if row["training_recommendation"] != "retain"
        ),
    )
    auto_counts = Counter(row["automatic_result"] for row in records)
    recommendation_counts = Counter(row["training_recommendation"] for row in records)
    disagreements = sum(
        row["previous_decision"] in {"PASS", "FAIL"}
        and row["previous_decision"] != row["automatic_result"]
        for row in records
    )
    disagreement_records = [
        row
        for row in records
        if row["previous_decision"] in {"PASS", "FAIL"}
        and row["previous_decision"] != row["automatic_result"]
    ]
    report = {
        "schema_version": SCHEMA_VERSION,
        "scope": "source geometry and manifest consistency; semantic visual completeness requires reviewer confirmation",
        "records": len(records),
        "automatic_results": dict(sorted(auto_counts.items())),
        "training_recommendations": dict(sorted(recommendation_counts.items())),
        "previous_review_disagreements": disagreements,
        "disagreements_by_source": dict(
            sorted(
                Counter(row["source_dataset"] for row in disagreement_records).items()
            )
        ),
        "disagreements_by_geometry_tier": dict(
            sorted(
                Counter(row["geometry_tier"] for row in disagreement_records).items()
            )
        ),
        "disagreements_by_previous_issue": dict(
            sorted(
                Counter(
                    row["previous_issue"] or "unspecified"
                    for row in disagreement_records
                ).items()
            )
        ),
        "automatic_failures": [
            {
                "audit_id": row["audit_id"],
                "failed_checks": row["automatic_failed_checks"],
            }
            for row in records
            if row["automatic_result"] == "FAIL"
        ],
        "prior_review_csv": str(prior_review_csv.resolve())
        if prior_review_csv
        else None,
        "html": "index.html",
    }
    write_json(destination / "adjudication_report.json", report)
    (destination / "ADJUDICATION_INSTRUCTIONS.md").write_text(
        """# Automatic Adjudication Review

The automatic result checks source-record identity, supervision-state consistency, quad validity, geometry-tier compatibility, independent source containment, and recomputed coverage/tightness.

It does **not** infer whether the source annotation captures the visually complete physical object. An automatic PASS therefore means source-consistent, not semantically perfect.

Start with **Previous disagreements**. Select YES when the automatic PASS/FAIL is correct, NO when visual evidence contradicts it, and UNSURE only when the source annotation must be inspected. The initial selection mirrors the prior review so the previous work is preserved.

All overlay edges and vertex markers are translucent. Thick cyan/red is the selected positive/ignore record; thin green/orange is context.

Use **Prepare CSV**. If the browser blocks download, use **Copy CSV** or copy the visible CSV fallback text.
""",
        encoding="utf-8",
    )
    _write_html(destination / "index.html", records)
    _write_verifier(destination)
    _bundle_manifest(destination)
    return report


def apply_accepted_automatic_policy(
    config: Config,
    destination: Path,
) -> dict[str, Any]:
    """Apply the owner-accepted automatic rule to every current manifest record."""
    manifest_paths = {
        split: config.workspace_root / f"output/annotations/proposals_{split}.json"
        for split in ("train", "val")
    }
    manifests = {split: read_json(path) for split, path in manifest_paths.items()}
    rows = _instance_rows(manifests)
    adjudicated = [adjudicate_row(config, row) for row in rows]
    verdict_by_key = {
        (
            row["split"],
            int(row["image_id"]),
            row["state"],
            row["source_annotation_id"],
        ): row["automatic_result"]
        for row in adjudicated
    }
    accepted: dict[str, dict[str, Any]] = {}
    quarantined = 0
    for split, manifest in manifests.items():
        copied = json.loads(json.dumps(manifest))
        for image in copied["images"]:
            retained_positive = []
            for index, record in enumerate(image.get("positive", [])):
                source_id = str(record.get("source_annotation_id", "")) or f"index-{index}"
                verdict = verdict_by_key[(split, int(image["image_id"]), "positive", source_id)]
                if verdict == "PASS":
                    retained_positive.append(record)
                else:
                    record["state"] = "ignore"
                    record["policy_resolution"] = "automatic_failure_quarantined_as_ignore"
                    image["ignore"].append(record)
                    quarantined += 1
            image["positive"] = retained_positive
        accepted[split] = copied
    if destination.exists():
        shutil.rmtree(destination)
    (destination / "accepted_manifests").mkdir(parents=True)
    output_paths = {}
    for split, manifest in accepted.items():
        path = destination / "accepted_manifests" / f"proposals_{split}.json"
        write_json(path, manifest, compact=True)
        output_paths[split] = path
    with (destination / "full_corpus_adjudication.jsonl").open(
        "w", encoding="utf-8"
    ) as stream:
        for row in adjudicated:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    write_csv(
        destination / "full_corpus_decisions.csv",
        [
            "audit_id", "split", "source_dataset", "source_image_id",
            "source_annotation_id", "state", "automatic_result", "resolution",
        ],
        (
            {
                **row,
                "resolution": "retain" if row["automatic_result"] == "PASS"
                else "quarantine_as_ignore",
            }
            for row in adjudicated
        ),
    )
    counts = Counter(row["automatic_result"] for row in adjudicated)
    report = {
        "schema_version": POLICY_SCHEMA_VERSION,
        "scope": "every positive and ignore record in the current train/val proposal manifests",
        "owner_acceptance": {
            "accepted": True,
            "evidence": "dataset owner explicitly accepted all deterministic automatic labels in this conversation",
            "manual_review_required": False,
        },
        "policy": {
            "PASS": "retain current supervision state",
            "FAIL_positive": "move to ignore",
            "FAIL_ignore": "retain as ignore",
            "weak_unlabeled_negative_weight": 0.0,
        },
        "input_hashes": {split: _sha256(path) for split, path in manifest_paths.items()},
        "output_hashes": {split: _sha256(path) for split, path in output_paths.items()},
        "records": len(adjudicated),
        "automatic_results": dict(sorted(counts.items())),
        "quarantined_positive_records": quarantined,
        "automatic_failures": [
            {
                "audit_id": row["audit_id"],
                "failed_checks": row["automatic_failed_checks"],
            }
            for row in adjudicated
            if row["automatic_result"] == "FAIL"
        ],
        "accepted_manifests": {
            split: str(path.resolve()) for split, path in output_paths.items()
        },
    }
    write_json(destination / "policy_report.json", report)
    return report
