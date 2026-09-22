"""Build a self-contained HTML dashboard for the frozen quad audit cohort."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


UTILITY_METRICS = {
    "ar": "ar/100",
    "r50": "recall/100@0.50",
    "r75": "recall/100@0.75",
    "coverage": "coverage/object_mean/100",
    "object_fraction": "proposal/object_fraction_mean",
    "unmatched": "false/unmatched_fraction",
    "trusted_false": "false/trusted_background_fraction",
    "duplicates": "duplicates/100@0.50_fraction",
    "pair50": "pairwise/100@0.50_fraction",
    "small_r50": "recall/size_small/100@0.50",
    "thin_r50": "recall/shape_thin/100@0.50",
    "source_16_32_r50": "recall/source_short_side_[16,32)/100@0.50",
    "proposals_per_image": "proposals/100_per_image",
}


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-suppression", type=Path, required=True)
    parser.add_argument("--phase2-utility", type=Path, required=True)
    parser.add_argument("--cloud-suppression", type=Path, required=True)
    parser.add_argument("--current-diversity", type=Path, required=True)
    parser.add_argument("--cloud-diversity", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def _read(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _validate_shared_cohort(reports: list[dict[str, Any]]) -> None:
    fields = (
        "validation_manifest_sha256",
        "validation_subset_sha256",
        "source_counts",
    )
    for field in fields:
        values = [report.get(field) for report in reports]
        if any(value is None for value in values):
            raise ValueError(f"report is missing cohort field {field!r}")
        if any(value != values[0] for value in values[1:]):
            raise ValueError(f"reports disagree on cohort field {field!r}")
    image_count = sum(int(value) for value in reports[0]["source_counts"].values())
    if image_count != 800:
        raise ValueError(
            f"dashboard requires the frozen 800-image cohort, got {image_count}"
        )


def _select_metrics(metrics: dict[str, Any]) -> dict[str, float]:
    return {name: float(metrics[key]) for name, key in UTILITY_METRICS.items()}


def _sweep_groups(
    report: dict[str, Any], setting: str
) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    quad = report["quad"]
    aggregate = _select_metrics(quad["settings"][setting])
    sources = {
        name.removeprefix("source/"): _select_metrics(groups[setting])
        for name, groups in quad["groups"].items()
        if name.startswith("source/")
    }
    return aggregate, sources


def _utility_groups(
    report: dict[str, Any],
) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    groups = report["quad"]["groups"]
    aggregate = _select_metrics(groups["aggregate"])
    sources = {
        name.removeprefix("source/"): _select_metrics(metrics)
        for name, metrics in groups.items()
        if name.startswith("source/")
    }
    return aggregate, sources


def _diversity_metrics(
    report: dict[str, Any], model: str, mode: str
) -> dict[str, float]:
    metrics = report["models"][model]["modes"][mode]["metrics"]
    return {
        "pair50": float(metrics["pairwise"]["iou_ge_0.50_fraction"]),
        "pair75": float(metrics["pairwise"]["iou_ge_0.75_fraction"]),
        "cluster50_p95": float(metrics["clusters"]["iou_0.50"]["largest_size"]["p95"]),
        "cluster75_p95": float(metrics["clusters"]["iou_0.75"]["largest_size"]["p95"]),
        "grid8_cells": float(metrics["spatial"]["grid_8"]["occupied_cells"]["mean"]),
        "spatial_entropy": float(
            metrics["spatial"]["grid_8"]["normalized_entropy"]["mean"]
        ),
        "area_std": float(metrics["scale"]["log2_area_std"]["mean"]),
        "aspect_std": float(metrics["scale"]["log2_aspect_std"]["mean"]),
        "unmatched": float(metrics["unmatched"]["fraction_of_proposals"]),
        "unmatched_pair50": float(
            metrics["unmatched"]["within_unmatched"]["pairwise"]["iou_ge_0.50_fraction"]
        ),
        "unmatched_pair75": float(
            metrics["unmatched"]["within_unmatched"]["pairwise"]["iou_ge_0.75_fraction"]
        ),
        "unmatched_cluster75_p95": float(
            metrics["unmatched"]["within_unmatched"]["clusters"]["iou_0.75"][
                "largest_size"
            ]["p95"]
        ),
    }


def _dashboard_data(paths: dict[str, Path]) -> dict[str, Any]:
    reports = {name: _read(path) for name, path in paths.items()}
    _validate_shared_cohort(list(reports.values()))

    current, current_sources = _sweep_groups(
        reports["current_suppression"], "nms=0.75,score=0.20"
    )
    phase2, phase2_sources = _utility_groups(reports["phase2_utility"])
    cloud, cloud_sources = _sweep_groups(
        reports["cloud_suppression"], "nms=0.78,score=0.20"
    )

    models = {
        "cloud": {
            "label": "Selected Phase 3 cloud EMA",
            "short": "Cloud",
            "policy": "NMS 0.78 · score 0.20",
            "color": "#5eead4",
            "checkpoint_sha256": reports["cloud_suppression"]["quad"][
                "checkpoint_sha256"
            ],
            "utility": cloud,
            "sources": cloud_sources,
            "diversity": _diversity_metrics(
                reports["cloud_diversity"],
                "phase3_cloud_nms078",
                "production",
            ),
        },
        "current": {
            "label": "Current 19,995-step EMA",
            "short": "Current",
            "policy": "NMS 0.75 · score 0.20",
            "color": "#60a5fa",
            "checkpoint_sha256": reports["current_suppression"]["quad"][
                "checkpoint_sha256"
            ],
            "utility": current,
            "sources": current_sources,
            "diversity": _diversity_metrics(
                reports["current_diversity"], "quad_19995_nms075", "production"
            ),
        },
        "phase2": {
            "label": "Formal Phase 2 incumbent",
            "short": "Phase 2",
            "policy": "NMS 0.80 · score 0.20",
            "color": "#fbbf24",
            "checkpoint_sha256": reports["phase2_utility"]["quad"]["checkpoint_sha256"],
            "utility": phase2,
            "sources": phase2_sources,
            "diversity": _diversity_metrics(
                reports["current_diversity"], "old_quad", "production"
            ),
        },
    }
    policy_comparison = {
        "current": current,
        "cloud": cloud,
        "current_diversity": models["current"]["diversity"],
        "cloud_diversity": models["cloud"]["diversity"],
    }
    first = reports["current_suppression"]
    return {
        "cohort": {
            "images": sum(int(value) for value in first["source_counts"].values()),
            "source_counts": first["source_counts"],
            "manifest_sha256": first["validation_manifest_sha256"],
            "subset_sha256": first["validation_subset_sha256"],
        },
        "models": models,
        "policy_comparison": policy_comparison,
        "reports": {
            name: {"path": str(path.resolve()), "sha256": _sha256(path)}
            for name, path in paths.items()
        },
    }


def _html_document(data: dict[str, Any]) -> str:
    serialized = (
        json.dumps(data, ensure_ascii=True, separators=(",", ":"))
        .replace("</", "<\\/")
        .replace("\u2028", "\\u2028")
        .replace("\u2029", "\\u2029")
    )
    return _TEMPLATE.replace("__DASHBOARD_DATA__", serialized)


_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quad proposal comparison · Frozen 800</title>
<style>
:root{color-scheme:dark;--bg:#07111f;--panel:#0d1b2d;--panel2:#10243a;--text:#e6f0fb;--muted:#8fa7be;--line:#203a55;--good:#5eead4;--warn:#fbbf24;--risk:#fb7185;font-family:Inter,ui-sans-serif,system-ui,-apple-system,sans-serif}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 85% -10%,#173a5d 0,transparent 34rem),var(--bg);color:var(--text)}
main{width:min(1440px,calc(100% - 32px));margin:auto;padding:46px 0 70px}.eyebrow{color:var(--good);font-size:.75rem;font-weight:800;letter-spacing:.16em;text-transform:uppercase}
h1{font-size:clamp(2.2rem,5vw,4.6rem);line-height:.98;max-width:940px;margin:.65rem 0 1rem;letter-spacing:-.055em}.lede{max-width:850px;color:#b7cadc;font-size:1.08rem;line-height:1.65}
.chips,.legend{display:flex;flex-wrap:wrap;gap:9px;margin:22px 0}.chip{border:1px solid var(--line);background:#0a1828cc;border-radius:999px;padding:8px 12px;color:#bdd0e1;font-size:.82rem}.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:7px}
.verdict{display:grid;grid-template-columns:1.2fr .8fr;gap:16px;margin:30px 0}.callout{background:linear-gradient(145deg,#102b3c,#0c1c2d);border:1px solid #28506a;border-radius:18px;padding:23px}.callout.risk{background:linear-gradient(145deg,#321d2a,#171827);border-color:#613149}.callout h2{font-size:1.35rem;margin:0 0 8px}.callout p{color:#b8cada;line-height:1.55;margin:0}
section{margin-top:42px}.section-head{display:flex;align-items:end;justify-content:space-between;gap:20px;margin-bottom:15px}.section-head h2{font-size:1.55rem;margin:0}.section-head p{color:var(--muted);margin:0;max-width:680px;text-align:right}
.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.panel{background:linear-gradient(155deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:17px;padding:19px;box-shadow:0 16px 44px #0004}.panel.wide{grid-column:1/-1}.panel h3{font-size:.98rem;margin:0 0 5px}.hint{font-size:.77rem;color:var(--muted);margin-bottom:16px}
.metric{margin:14px 0}.metric-title{display:flex;justify-content:space-between;color:#bfd0df;font-size:.8rem;margin-bottom:6px}.bar-row{display:grid;grid-template-columns:86px 1fr 58px;align-items:center;gap:8px;margin:6px 0}.bar-label{font-size:.72rem;color:#9fb5c8}.track{height:9px;background:#071321;border-radius:999px;overflow:hidden}.fill{height:100%;border-radius:999px}.value{text-align:right;font-variant-numeric:tabular-nums;font-size:.76rem}
.table-wrap{overflow:auto;border:1px solid var(--line);border-radius:13px}table{width:100%;border-collapse:collapse;font-size:.79rem}th,td{padding:10px 12px;text-align:right;border-bottom:1px solid #1a334b;white-space:nowrap}th:first-child,td:first-child{text-align:left}thead{background:#0a1726;color:#9fb5c8}tbody tr:last-child td{border-bottom:0}.best{color:var(--good);font-weight:750}.bad{color:var(--risk)}
.scatter{width:100%;height:auto}.axis{stroke:#547088;stroke-width:1}.guide{stroke:#29445d;stroke-dasharray:4 5}.point-label{fill:var(--text);font-size:12px;font-weight:700}.axis-label{fill:var(--muted);font-size:11px}.point{stroke:#06101b;stroke-width:3}
.composition{display:grid;grid-template-columns:150px 1fr;gap:24px;align-items:center}.donut{width:140px;height:140px;border-radius:50%;position:relative}.donut:after{content:"800";display:grid;place-items:center;position:absolute;inset:25px;border-radius:50%;background:var(--panel);font-size:1.5rem;font-weight:800}.source-list{display:grid;gap:9px}.source-item{display:flex;justify-content:space-between;border-bottom:1px solid #1d3449;padding-bottom:7px;color:#b9ccdc}
.takeaway{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.takeaway article{border-left:3px solid var(--good);padding:3px 13px}.takeaway article:nth-child(2){border-color:var(--risk)}.takeaway article:nth-child(3){border-color:var(--warn)}.takeaway h3{margin:0 0 6px;font-size:.92rem}.takeaway p{margin:0;color:#9fb5c8;font-size:.83rem;line-height:1.5}
details{margin-top:16px;border:1px solid var(--line);border-radius:12px;padding:13px 15px;background:#091726}summary{cursor:pointer;color:#b9cedf;font-weight:700}.provenance{font:11px ui-monospace,SFMono-Regular,Menlo,monospace;color:#8da8bd;overflow-wrap:anywhere;line-height:1.55}
footer{margin-top:34px;color:#7892a8;font-size:.78rem}@media(max-width:850px){.verdict,.grid{grid-template-columns:1fr}.panel.wide{grid-column:auto}.section-head{align-items:start;flex-direction:column}.section-head p{text-align:left}.takeaway{grid-template-columns:1fr}.composition{grid-template-columns:1fr}.donut{margin:auto}}
</style>
</head>
<body><main>
<div class="eyebrow">Proposal detector · evidence dashboard</div>
<h1>The fully trained cloud quad remains the primary detector.</h1>
<p class="lede">A matched summary of the frozen 800-image audit. NMS 0.78 removes the cloud model's legacy overlap problem while preserving its localization advantage over the current 20k control.</p>
<div class="chips" id="cohort-chips"></div><div class="legend" id="legend"></div>
<div class="verdict"><article class="callout"><h2>Cloud localization wins</h2><p id="gain-copy"></p></article><article class="callout risk"><h2>Current remains the cleaner control</h2><p id="risk-copy"></p></article></div>

<section><div class="section-head"><h2>Proposal utility</h2><p>Each model uses its recorded production suppression policy. Longer bars indicate more of the named quantity; “lower is better” is called out explicitly.</p></div><div class="grid"><div class="panel" id="recall-chart"></div><div class="panel" id="quality-chart"></div><div class="panel wide" id="slice-chart"></div></div></section>

<section><div class="section-head"><h2>The operating trade-off</h2><p>Upper-left is ideal: high average recall with few unmatched proposals.</p></div><div class="panel"><svg id="tradeoff" class="scatter" viewBox="0 0 920 350" role="img" aria-label="Average recall versus unmatched proposals"></svg></div></section>

<section><div class="section-head"><h2>Overlap and diversity</h2><p>Measured on the same 800 images under the selected policies: cloud NMS 0.78 and current NMS 0.75, both at score 0.20.</p></div><div class="grid"><div class="panel" id="overlap-chart"></div><div class="panel" id="diversity-chart"></div><div class="panel wide"><h3>Selected cloud versus current control</h3><div class="hint">The policies are intentionally different and were fixed by the bounded suppression sweep.</div><div class="table-wrap"><table id="common-table"></table></div></div></div></section>

<section><div class="section-head"><h2>Source-level behavior</h2><p>The selector fixes the source mixture at 400 COCO, 160 WoodScape, 144 nuImages, and 96 BDD100K records.</p></div><div class="panel wide"><div class="table-wrap"><table id="source-table"></table></div></div></section>

<section><div class="section-head"><h2>Cohort and provenance</h2><p>Every panel is derived from reports bound to the same manifest and selected-image identity hash.</p></div><div class="grid"><div class="panel"><div class="composition"><div class="donut" id="donut"></div><div class="source-list" id="source-list"></div></div></div><div class="panel"><div class="takeaway"><article><h3>Selected primary</h3><p>The fully trained cloud EMA at NMS 0.78 provides the strongest localization and protected-slice recall.</p></article><article><h3>Retained control</h3><p>The current 20k EMA at NMS 0.75 has slightly lower overlap and fewer unmatched proposals.</p></article><article><h3>Next boundary</h3><p>Semantic crop utility belongs to the SigLIP phase; this report closes proposal-model selection.</p></article></div></div></div><details><summary>Artifact hashes and exact identities</summary><div class="provenance" id="provenance"></div></details></section>
<footer>Static report · all percentages are aggregate unless a source or slice is named · no network requests</footer>
</main>
<script type="application/json" id="dashboard-data">__DASHBOARD_DATA__</script>
<script>
const D=JSON.parse(document.getElementById('dashboard-data').textContent);const order=['cloud','current','phase2'];
const pct=v=>(100*v).toFixed(2)+'%';const num=(v,d=2)=>Number(v).toFixed(d);const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
document.getElementById('cohort-chips').innerHTML=`<span class="chip">${D.cohort.images} images</span><span class="chip">K=100 proposals</span><span class="chip">EMA weights</span><span class="chip">frozen subset ${D.cohort.subset_sha256.slice(0,10)}…</span>`;
document.getElementById('legend').innerHTML=order.map(k=>`<span class="chip"><i class="dot" style="background:${D.models[k].color}"></i>${esc(D.models[k].label)} · ${esc(D.models[k].policy)}</span>`).join('');
const cur=D.models.current.utility,cloud=D.models.cloud.utility;
document.getElementById('gain-copy').textContent=`Cloud AR@100 is ${pct(cloud.ar)} versus ${pct(cur.ar)} for current; Recall@0.75 is ${pct(cloud.r75)} versus ${pct(cur.r75)}.`;
document.getElementById('risk-copy').textContent=`Current unmatched proposals are ${pct(cur.unmatched)} versus ${pct(cloud.unmatched)} for cloud, with duplicate rates ${pct(cur.duplicates)} and ${pct(cloud.duplicates)}.`;
function chart(id,title,hint,metrics){const el=document.getElementById(id);el.innerHTML=`<h3>${esc(title)}</h3><div class="hint">${esc(hint)}</div>`+metrics.map(m=>{const max=m.max||Math.max(...order.map(k=>D.models[k].utility[m.key]),.001);return `<div class="metric"><div class="metric-title"><span>${esc(m.label)}</span><span>${m.lower?'lower is better':'higher is better'}</span></div>`+order.map(k=>{const v=D.models[k].utility[m.key];return `<div class="bar-row"><span class="bar-label">${D.models[k].short}</span><div class="track"><div class="fill" style="width:${Math.min(100,100*v/max)}%;background:${D.models[k].color}"></div></div><span class="value">${m.format==='num'?num(v):pct(v)}</span></div>`}).join('')+'</div>'}).join('')}
chart('recall-chart','Recall and coverage','Cloud leads localization; current leads broad object coverage.',[{key:'ar',label:'Average recall @100'},{key:'r50',label:'Recall @ IoU 0.50'},{key:'r75',label:'Recall @ IoU 0.75'},{key:'coverage',label:'Best-proposal object coverage'}]);
chart('quality-chart','Proposal relevance','Current is cleaner; cloud retains substantially stronger localization.',[{key:'object_fraction',label:'Mean object-area fraction'},{key:'unmatched',label:'Unmatched proposals',lower:true},{key:'duplicates',label:'Duplicate fraction',lower:true},{key:'pair50',label:'All proposal pairs IoU ≥ 0.50',lower:true}]);
chart('slice-chart','Protected recall slices','Small, thin, and 16–32 px source objects.',[{key:'small_r50',label:'Small recall @0.50'},{key:'thin_r50',label:'Thin recall @0.50'},{key:'source_16_32_r50',label:'Source short side 16–32 px recall @0.50'}]);
function scatter(){const svg=document.getElementById('tradeoff'),x0=72,y0=28,w=800,h=260,xMin=.42,xMax=.72,yMin=.10,yMax=.38;let s='';for(let i=0;i<=5;i++){const x=x0+w*i/5,y=y0+h*i/5;s+=`<line class="guide" x1="${x}" y1="${y0}" x2="${x}" y2="${y0+h}"/><text class="axis-label" x="${x}" y="${y0+h+23}" text-anchor="middle">${Math.round(100*(xMin+(xMax-xMin)*i/5))}%</text>`;s+=`<line class="guide" x1="${x0}" y1="${y}" x2="${x0+w}" y2="${y}"/><text class="axis-label" x="${x0-12}" y="${y+4}" text-anchor="end">${Math.round(100*(yMax-(yMax-yMin)*i/5))}%</text>`}s+=`<line class="axis" x1="${x0}" y1="${y0+h}" x2="${x0+w}" y2="${y0+h}"/><line class="axis" x1="${x0}" y1="${y0}" x2="${x0}" y2="${y0+h}"/><text class="axis-label" x="${x0+w/2}" y="${y0+h+48}" text-anchor="middle">Unmatched proposals → lower is better</text><text class="axis-label" transform="translate(18 ${y0+h/2}) rotate(-90)" text-anchor="middle">AR@100 → higher is better</text>`;for(const k of order){const m=D.models[k],x=x0+w*(m.utility.unmatched-xMin)/(xMax-xMin),y=y0+h*(yMax-m.utility.ar)/(yMax-yMin);s+=`<circle class="point" cx="${x}" cy="${y}" r="10" fill="${m.color}"/><text class="point-label" x="${x+15}" y="${y-10}">${esc(m.short)}</text>`}svg.innerHTML=s}scatter();
function diversityChart(id,title,items){const el=document.getElementById(id);el.innerHTML=`<h3>${esc(title)}</h3><div class="hint">${items.some(x=>x.lower)?'Direction is stated per metric.':'Higher indicates broader coverage.'}</div>`+items.map(m=>{const max=m.max||Math.max(...order.map(k=>D.models[k].diversity[m.key]),.001);return `<div class="metric"><div class="metric-title"><span>${esc(m.label)}</span><span>${m.lower?'lower is better':'higher is broader'}</span></div>`+order.map(k=>{const v=D.models[k].diversity[m.key];return `<div class="bar-row"><span class="bar-label">${D.models[k].short}</span><div class="track"><div class="fill" style="width:${Math.min(100,100*v/max)}%;background:${D.models[k].color}"></div></div><span class="value">${m.percent?pct(v):num(v)}</span></div>`}).join('')+'</div>'}).join('')}
diversityChart('overlap-chart','Redundancy and clusters',[{key:'pair50',label:'Proposal pairs IoU ≥ 0.50',lower:true,percent:true},{key:'pair75',label:'Proposal pairs IoU ≥ 0.75',lower:true,percent:true},{key:'cluster50_p95',label:'IoU-0.50 largest cluster p95',lower:true},{key:'cluster75_p95',label:'IoU-0.75 largest cluster p95',lower:true}]);
diversityChart('diversity-chart','Spatial and scale spread',[{key:'grid8_cells',label:'Occupied 8×8 cells / image'},{key:'spatial_entropy',label:'Normalized spatial entropy'},{key:'area_std',label:'Log₂ area standard deviation'},{key:'aspect_std',label:'Log₂ aspect standard deviation'}]);
const commonRows=[['AR@100','ar',true,true],['Recall @0.50','r50',true,true],['Recall @0.75','r75',true,true],['Object fraction','object_fraction',true,true],['Unmatched','unmatched',false,true],['Duplicates','duplicates',false,true],['Pairwise ≥0.50','pair50',false,true],['Grid cells','grid8_cells',true,false],['IoU-0.75 cluster p95','cluster75_p95',false,false]];
const cUtil=D.policy_comparison.current,sUtil=D.policy_comparison.cloud,cDiv=D.policy_comparison.current_diversity,sDiv=D.policy_comparison.cloud_diversity;document.getElementById('common-table').innerHTML='<thead><tr><th>Metric</th><th>Cloud</th><th>Current</th><th>Current − cloud</th></tr></thead><tbody>'+commonRows.map(([label,key,higher,percent])=>{const c=(key in cUtil)?cUtil[key]:cDiv[key],s=(key in sUtil)?sUtil[key]:sDiv[key],delta=c-s,good=higher?delta>=0:delta<=0,fmt=v=>percent?pct(v):num(v);return `<tr><td>${label}</td><td>${fmt(s)}</td><td>${fmt(c)}</td><td class="${good?'best':'bad'}">${delta>=0?'+':''}${percent?(100*delta).toFixed(2)+' pp':num(delta)}</td></tr>`}).join('')+'</tbody>';
const sourceLabels={bdd100k_images_100k:'BDD100K',coco_2017:'COCO',nuimages:'nuImages',woodscape_rgb_fisheye:'WoodScape'};const sourceKeys=Object.keys(D.cohort.source_counts);document.getElementById('source-table').innerHTML='<thead><tr><th>Source</th><th>Images</th>'+order.map(k=>`<th>${D.models[k].short} AR</th>`).join('')+order.map(k=>`<th>${D.models[k].short} unmatched</th>`).join('')+'</tr></thead><tbody>'+sourceKeys.map(source=>`<tr><td>${sourceLabels[source]||source}</td><td>${D.cohort.source_counts[source]}</td>${order.map(k=>`<td>${pct(D.models[k].sources[source].ar)}</td>`).join('')}${order.map(k=>`<td>${pct(D.models[k].sources[source].unmatched)}</td>`).join('')}</tr>`).join('')+'</tbody>';
const colors=['#5eead4','#60a5fa','#fbbf24','#fb7185'];let cursor=0,parts=[];sourceKeys.forEach((s,i)=>{const next=cursor+100*D.cohort.source_counts[s]/D.cohort.images;parts.push(`${colors[i]} ${cursor}% ${next}%`);cursor=next});document.getElementById('donut').style.background=`conic-gradient(${parts.join(',')})`;document.getElementById('source-list').innerHTML=sourceKeys.map((s,i)=>`<div class="source-item"><span><i class="dot" style="background:${colors[i]}"></i>${sourceLabels[s]||s}</span><strong>${D.cohort.source_counts[s]}</strong></div>`).join('');
document.getElementById('provenance').innerHTML=`manifest: ${D.cohort.manifest_sha256}<br>subset: ${D.cohort.subset_sha256}<br><br>`+Object.entries(D.models).map(([k,m])=>`${esc(m.label)} checkpoint: ${m.checkpoint_sha256}`).join('<br>')+'<br><br>'+Object.entries(D.reports).map(([k,r])=>`${esc(k)}<br>&nbsp;&nbsp;${esc(r.path)}<br>&nbsp;&nbsp;sha256 ${r.sha256}`).join('<br>');
</script></body></html>
"""


def main() -> None:
    args = _args()
    paths = {
        "current_suppression": args.current_suppression,
        "phase2_utility": args.phase2_utility,
        "cloud_suppression": args.cloud_suppression,
        "current_diversity": args.current_diversity,
        "cloud_diversity": args.cloud_diversity,
    }
    data = _dashboard_data(paths)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_html_document(data), encoding="utf-8")
    print(f"Wrote quad comparison dashboard to {output}")


if __name__ == "__main__":
    main()
