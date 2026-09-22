"""Create a paginated visual audit of two quad models on the frozen cohort."""

from __future__ import annotations

import argparse
import base64
from collections import Counter
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path
from typing import Any

from PIL import Image
import torch
from torch.utils.data import DataLoader, Subset

from student_detector.config import load_phase3_config
from student_detector.data import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    select_source_mixture_indices,
)
from student_detector.head import QuadDetectorOutput
from student_detector.proposal_evaluation import (
    load_proposal_model,
    valid_output_levels,
)
from student_detector.quad_data import (
    QuadProposalDataset,
    collate_quad_proposal_samples,
)
from student_detector.quad_decoder import QuadInferenceDecoder
from student_detector.quad_geometry import pairwise_quad_iou, warmup_compiled_quad_iou


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--historical-checkpoint", type=Path, required=True)
    parser.add_argument("--current-checkpoint", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-images", type=int, default=800)
    parser.add_argument("--historical-nms", type=float, default=0.90)
    parser.add_argument("--current-nms", type=float, default=0.75)
    parser.add_argument("--checkpoint-state", default="ema_model")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--log-interval", type=int, default=100)
    parser.add_argument(
        "--amp",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use CUDA FP16 network inference; geometry remains FP32",
    )
    parser.add_argument(
        "--embed-images",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Embed JPEGs in index.html; also writes a smaller index_linked.html",
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _selected_identity(dataset: QuadProposalDataset, selected: list[int]) -> str:
    encoded = json.dumps(
        [
            (dataset.records[index].source_dataset, dataset.records[index].image_id)
            for index in selected
        ],
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _image(sample: Any) -> Image.Image:
    tensor = sample.image.detach().cpu()
    mean = tensor.new_tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = tensor.new_tensor(IMAGENET_STD).view(3, 1, 1)
    pixels = ((tensor * std + mean).clamp(0, 1) * 255).byte().permute(1, 2, 0).numpy()
    return Image.fromarray(pixels, mode="RGB")


def _quad_points(quad: torch.Tensor) -> list[list[float]]:
    return [[round(float(value), 2) for value in point] for point in quad]


def _proposal_rows(
    quads: torch.Tensor, scores: torch.Tensor, best_ious: torch.Tensor
) -> list[dict[str, Any]]:
    return [
        {
            "quad": _quad_points(quad),
            "score": round(float(score), 5),
            "bestIou": round(float(best_iou), 4),
            "rank": rank + 1,
        }
        for rank, (quad, score, best_iou) in enumerate(
            zip(quads, scores, best_ious, strict=True)
        )
    ]


def _supervision_record(sample: Any, image_name: str, sequence: int) -> dict[str, Any]:
    trusted = sample.trusted_background_quads
    if trusted is None:
        trusted = sample.quads.new_empty((0, 4, 2))
    return {
        "sequence": sequence,
        "image": image_name,
        "imageId": sample.image_id,
        "source": sample.source_dataset,
        "domain": sample.domain,
        "camera": sample.camera_type,
        "groundTruth": [
            {
                "quad": _quad_points(quad),
                "size": size,
                "shape": shape,
                "tier": tier,
            }
            for quad, size, shape, tier in zip(
                sample.quads,
                sample.size_bins,
                sample.aspect_bins,
                sample.geometry_tiers,
                strict=True,
            )
        ],
        "ignored": [_quad_points(quad) for quad in sample.ignore_quads],
        "trusted": [_quad_points(quad) for quad in trusted],
    }


@torch.inference_mode()
def _infer(
    *,
    label: str,
    checkpoint: Path,
    checkpoint_state: str,
    nms_threshold: float,
    loader: DataLoader[Any],
    config: Any,
    device: torch.device,
    amp: bool,
    log_interval: int,
    output_dir: Path,
    records: list[dict[str, Any]] | None,
) -> list[dict[str, Any]]:
    model = load_proposal_model("quad", checkpoint, checkpoint_state, device)
    decoder = QuadInferenceDecoder(
        strides=config.assignment.strides,
        pre_nms_top_k=config.inference.pre_nms_top_k,
        nms_iou_threshold=nms_threshold,
        max_proposals=config.inference.max_proposals,
    )
    result = records if records is not None else []
    seen = 0
    for images, samples in loader:
        images_device = images.to(device, non_blocking=device.type == "cuda")
        autocast = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if device.type == "cuda" and amp
            else nullcontext()
        )
        with autocast:
            output = model(images_device)
        if not isinstance(output, QuadDetectorOutput):
            raise TypeError("quad model returned an unexpected output type")
        output = QuadDetectorOutput(
            quality=tuple(item.float() for item in output.quality),
            corner_offsets=tuple(item.float() for item in output.corner_offsets),
        )
        masks = tuple(
            zip(
                *(
                    valid_output_levels(
                        sample.valid_mask,
                        output.quality,
                        config.assignment.strides,
                        device,
                    )
                    for sample in samples
                )
            )
        )
        valid_masks = tuple(torch.cat(level, dim=0) for level in masks)
        detections = decoder(
            output,
            (images.shape[-2], images.shape[-1]),
            valid_masks,
        )
        for sample, detection in zip(samples, detections, strict=True):
            quads = detection.quads.float()
            scores = detection.scores.float()
            gt = sample.quads.to(device)
            best_ious = (
                pairwise_quad_iou(gt, quads).amax(dim=0)
                if gt.shape[0] and quads.shape[0]
                else scores.new_zeros((quads.shape[0],))
            )
            if records is None:
                image_name = f"images/{seen:04d}_{sample.image_id}.jpg"
                _image(sample).save(output_dir / image_name, quality=90, optimize=True)
                result.append(_supervision_record(sample, image_name, seen))
            elif result[seen]["imageId"] != sample.image_id:
                raise RuntimeError("model passes returned different image order")
            result[seen][label] = _proposal_rows(
                quads.cpu(), scores.cpu(), best_ious.cpu()
            )
            seen += 1
            if log_interval and seen % log_interval == 0:
                print(f"{label}: {seen}/{len(loader.dataset)} images", flush=True)
    if seen != len(loader.dataset):
        raise RuntimeError(f"{label} evaluated {seen}, expected {len(loader.dataset)}")
    del model
    if device.type == "cuda":
        torch.cuda.empty_cache()
    return result


def _html_document(data: dict[str, Any]) -> str:
    serialized = json.dumps(data, ensure_ascii=True, separators=(",", ":")).replace(
        "</", "<\\/"
    )
    return _HTML.replace("__AUDIT_DATA__", serialized)


def _embed_record_images(data: dict[str, Any], output_dir: Path) -> None:
    """Replace relative JPEG references with portable data URLs in-place."""
    for record in data["records"]:
        image_path = output_dir / record["image"]
        encoded = base64.b64encode(image_path.read_bytes()).decode("ascii")
        record["image"] = f"data:image/jpeg;base64,{encoded}"


_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Quad visual audit · Frozen 800</title>
<style>
:root{color-scheme:dark;--bg:#07111d;--panel:#0e1c2b;--panel2:#12263a;--line:#284158;--text:#edf5fc;--muted:#91a8bb;--gt:#4ade80;--ignore:#fbbf24;--trusted:#a78bfa;--old:#fb7185;--current:#5eead4;font-family:Inter,ui-sans-serif,system-ui,-apple-system,sans-serif}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text)}
header{padding:34px max(20px,calc((100vw - 1500px)/2));background:radial-gradient(circle at 80% 0,#173c5d,transparent 32rem),#081522;border-bottom:1px solid var(--line)}.eyebrow{color:var(--current);font-size:.73rem;font-weight:850;letter-spacing:.16em;text-transform:uppercase}h1{margin:8px 0 9px;font-size:clamp(2rem,4vw,3.8rem);letter-spacing:-.045em}.lede{color:#b8cad9;max-width:900px;line-height:1.55;margin:0}.legend{display:flex;flex-wrap:wrap;gap:13px;margin-top:18px;color:#b9cad8;font-size:.8rem}.swatch{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
.toolbar{position:sticky;top:0;z-index:20;background:#091725ee;backdrop-filter:blur(14px);border-bottom:1px solid var(--line);padding:12px max(20px,calc((100vw - 1500px)/2));display:grid;grid-template-columns:repeat(6,minmax(120px,1fr));gap:10px;align-items:end}.control{display:grid;gap:5px;color:#9fb3c4;font-size:.72rem}.control input[type=range]{width:100%;accent-color:var(--current)}select,input[type=search]{width:100%;background:#0d2031;color:var(--text);border:1px solid #315069;border-radius:8px;padding:8px}.checks{grid-column:1/-1;display:flex;flex-wrap:wrap;gap:14px;font-size:.78rem;color:#b8c9d7}.checks input{accent-color:var(--current)}
main{width:min(1500px,calc(100% - 28px));margin:22px auto 60px}.status{display:flex;align-items:center;justify-content:space-between;gap:14px;color:#9fb3c4;font-size:.82rem;margin:13px 2px}.pager{display:flex;gap:7px;align-items:center}.pager button{background:#10283c;color:var(--text);border:1px solid #315069;border-radius:8px;padding:7px 11px;cursor:pointer}.pager button:disabled{opacity:.35;cursor:default}
.cards{display:grid;gap:18px}.card{background:linear-gradient(145deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:16px;padding:14px;box-shadow:0 18px 44px #0005}.card-head{display:flex;justify-content:space-between;gap:12px;align-items:start;margin-bottom:11px}.title{font-size:.96rem;font-weight:760}.meta{color:var(--muted);font-size:.75rem;margin-top:4px}.badges{display:flex;flex-wrap:wrap;justify-content:flex-end;gap:6px}.badge{background:#071421;border:1px solid #29485f;border-radius:999px;padding:5px 8px;font-size:.68rem;color:#aac0d1}.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:11px}.view{margin:0}.view-title{display:flex;justify-content:space-between;align-items:center;font-size:.77rem;font-weight:760;margin-bottom:6px}.view-title span:last-child{font-weight:500;color:var(--muted);font-variant-numeric:tabular-nums}canvas{display:block;width:100%;aspect-ratio:1;background:#02060a;border:1px solid #315069;border-radius:8px}
.empty{padding:70px 20px;text-align:center;color:var(--muted);border:1px dashed var(--line);border-radius:14px}.footer-pager{margin-top:18px;display:flex;justify-content:center}.help{margin-top:18px;border:1px solid var(--line);background:#0a1825;border-radius:12px;padding:12px 15px;color:#9fb3c4;font-size:.78rem;line-height:1.55}kbd{background:#132b40;border:1px solid #34536b;border-radius:4px;padding:1px 5px;color:#d9e8f4}@media(max-width:1000px){.toolbar{grid-template-columns:repeat(3,1fr)}.panels{grid-template-columns:1fr}.view{max-width:650px;margin:auto;width:100%}}@media(max-width:620px){.toolbar{grid-template-columns:1fr 1fr}.card-head{display:block}.badges{justify-content:flex-start;margin-top:9px}}
</style></head><body>
<header><div class="eyebrow">Proposal detector · per-image visual audit</div><h1>Old, current, and ground truth—side by side.</h1><p class="lede">Every record in the frozen 800-image cohort. Proposal color can show the model identity or its best overlap with a labeled object; pagination keeps the audit fast without hiding any image.</p><div class="legend"><span><i class="swatch" style="background:var(--gt)"></i>ground truth</span><span><i class="swatch" style="background:var(--ignore)"></i>ignored supervision</span><span><i class="swatch" style="background:var(--trusted)"></i>trusted background</span><span><i class="swatch" style="background:var(--old)"></i>historical proposal</span><span><i class="swatch" style="background:var(--current)"></i>current proposal</span></div></header>
<div class="toolbar"><label class="control">Source<select id="source"></select></label><label class="control">Image ID<input id="search" type="search" placeholder="Search exact or partial ID"></label><label class="control">Top K <output id="topk-out">25</output><input id="topk" type="range" min="1" max="100" value="25"></label><label class="control">Historical score <output id="old-score-out">0.00</output><input id="old-score" type="range" min="0" max="1" step=".01" value="0"></label><label class="control">Current score <output id="current-score-out">0.20</output><input id="current-score" type="range" min="0" max="1" step=".01" value=".2"></label><label class="control">Page size<select id="page-size"><option>12</option><option selected>24</option><option>48</option></select></label><div class="checks"><label><input id="show-gt" type="checkbox" checked> ground truth</label><label><input id="show-ignore" type="checkbox"> ignored</label><label><input id="show-trusted" type="checkbox"> trusted background</label><label><input id="show-scores" type="checkbox"> proposal labels</label><label><input id="match-color" type="checkbox" checked> color proposals by best GT IoU</label></div></div>
<main><div class="status"><span id="status"></span><div class="pager"><button data-page="first">First</button><button data-page="prev">Previous</button><span id="page-label"></span><button data-page="next">Next</button><button data-page="last">Last</button></div></div><div class="cards" id="cards"></div><div class="footer-pager pager"><button data-page="prev">Previous</button><button data-page="next">Next</button></div><div class="help"><strong>Reading the panels.</strong> With “color by best GT IoU” enabled: green means IoU ≥ 0.50, amber means 0.10–0.50, and red means unmatched (&lt;0.10). Disable it to use model colors. Proposal labels show <kbd>rank · score · best IoU</kbd>. GT is drawn last so it remains visible above proposals.</div></main>
<script type="application/json" id="audit-data">__AUDIT_DATA__</script><script>
const D=JSON.parse(document.getElementById('audit-data').textContent),records=D.records,$=id=>document.getElementById(id);let page=0,filtered=records;const imageCache=new Map();const source=$('source');source.innerHTML='<option value="">All sources</option>'+Object.keys(D.sourceCounts).map(s=>`<option value="${s}">${s} (${D.sourceCounts[s]})</option>`).join('');
function quad(ctx,q,color,width,dash=[]){if(!q.length)return;ctx.save();ctx.beginPath();ctx.moveTo(q[0][0],q[0][1]);for(let i=1;i<q.length;i++)ctx.lineTo(q[i][0],q[i][1]);ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineJoin='round';ctx.setLineDash(dash);ctx.stroke();ctx.restore()}
function label(ctx,p,color){const text=`${p.rank} · ${p.score.toFixed(2)} · ${p.bestIou.toFixed(2)}`,x=Math.min(...p.quad.map(v=>v[0])),y=Math.max(12,Math.min(...p.quad.map(v=>v[1]))-3);ctx.font='10px ui-monospace,monospace';const w=ctx.measureText(text).width+6;ctx.fillStyle='#06101ddd';ctx.fillRect(x,y-11,w,14);ctx.fillStyle=color;ctx.fillText(text,x+3,y)}
function proposalColor(p,kind){if(!$('match-color').checked)return kind==='historical'?'#fb7185':'#5eead4';return p.bestIou>=.5?'#4ade80':p.bestIou>=.1?'#fbbf24':'#fb4f64'}
function visible(record,kind){const score=Number($(kind==='historical'?'old-score':'current-score').value),k=Number($('topk').value);return record[kind].filter(p=>p.rank<=k&&p.score>=score)}
function draw(canvas,img,record,kind){const ctx=canvas.getContext('2d');ctx.clearRect(0,0,384,384);ctx.drawImage(img,0,0,384,384);if(kind!=='reference'){for(const p of visible(record,kind)){const c=proposalColor(p,kind);quad(ctx,p.quad,c+'cc',1.4);if($('show-scores').checked)label(ctx,p,c)}}if($('show-trusted').checked)for(const q of record.trusted)quad(ctx,q,'#a78bfacc',2,[5,4]);if($('show-ignore').checked)for(const q of record.ignored)quad(ctx,q,'#fbbf24dd',2,[7,4]);if($('show-gt').checked)for(const g of record.groundTruth)quad(ctx,g.quad,'#4ade80',2.8)}
function stats(record,kind){const p=visible(record,kind),matched=p.filter(x=>x.bestIou>=.5).length,unmatched=p.filter(x=>x.bestIou<.1).length;return `${p.length} shown · ${matched} matched · ${unmatched} unmatched`}
function imageElement(path){return new Promise((resolve,reject)=>{const img=new Image();img.decoding='async';img.addEventListener('load',()=>resolve(img),{once:true});img.addEventListener('error',()=>reject(new Error('Image decode failed')),{once:true});img.src=path})}
async function decodeImage(path){if(path.startsWith('data:image/')&&typeof createImageBitmap==='function'){const comma=path.indexOf(','),binary=atob(path.slice(comma+1)),bytes=new Uint8Array(binary.length);for(let i=0;i<binary.length;i++)bytes[i]=binary.charCodeAt(i);return createImageBitmap(new Blob([bytes],{type:'image/jpeg'}))}return imageElement(path)}
function loadImage(path){if(imageCache.has(path)){const hit=imageCache.get(path);imageCache.delete(path);imageCache.set(path,hit);return hit}const promise=decodeImage(path);imageCache.set(path,promise);if(imageCache.size>72){const oldest=imageCache.keys().next().value,evicted=imageCache.get(oldest);imageCache.delete(oldest);evicted.then(img=>img.close?.()).catch(()=>{})}return promise}
function renderCard(record){const el=document.createElement('article');el.className='card';el.innerHTML=`<div class="card-head"><div><div class="title">#${record.sequence+1} · image ${record.imageId}</div><div class="meta">${record.source} · ${record.domain} · ${record.camera}</div></div><div class="badges"><span class="badge">GT ${record.groundTruth.length}</span><span class="badge">ignored ${record.ignored.length}</span><span class="badge">trusted ${record.trusted.length}</span></div></div><div class="panels">${[['reference','Ground truth'],['historical','Historical · NMS '+D.models.historical.nms.toFixed(2)],['current','Current · NMS '+D.models.current.nms.toFixed(2)]].map(([k,t])=>`<figure class="view"><div class="view-title"><span>${t}</span><span data-stats="${k}"></span></div><canvas width="384" height="384" data-kind="${k}"></canvas></figure>`).join('')}</div>`;let bitmap=null;const redraw=()=>{if(!bitmap)return;for(const canvas of el.querySelectorAll('canvas')){const kind=canvas.dataset.kind;draw(canvas,bitmap,record,kind);const stat=el.querySelector(`[data-stats="${kind}"]`);stat.textContent=kind==='reference'?`${record.groundTruth.length} GT`:stats(record,kind)}};loadImage(record.image).then(img=>{bitmap=img;redraw()}).catch(error=>{for(const canvas of el.querySelectorAll('canvas')){const ctx=canvas.getContext('2d');ctx.fillStyle='#fb7185';ctx.font='14px system-ui';ctx.fillText(error.message,18,30)}});el.redraw=redraw;return el}
function applyFilter(){const q=$('search').value.trim();filtered=records.filter(r=>(!source.value||r.source===source.value)&&(!q||String(r.imageId).includes(q)));page=0;render()}
function render(){const size=Number($('page-size').value),pages=Math.max(1,Math.ceil(filtered.length/size));page=Math.max(0,Math.min(page,pages-1));const start=page*size,shown=filtered.slice(start,start+size),cards=$('cards');cards.innerHTML='';for(const r of shown)cards.appendChild(renderCard(r));if(!shown.length)cards.innerHTML='<div class="empty">No images match these filters.</div>';$('status').textContent=`Showing ${shown.length?start+1:0}–${Math.min(start+size,filtered.length)} of ${filtered.length} images · frozen subset ${D.subsetSha.slice(0,12)}…`;$('page-label').textContent=`Page ${page+1} / ${pages}`;for(const b of document.querySelectorAll('[data-page]')){b.disabled=(['first','prev'].includes(b.dataset.page)&&page===0)||(['next','last'].includes(b.dataset.page)&&page===pages-1)}}
function redraw(){for(const card of document.querySelectorAll('.card'))card.redraw?.()}
for(const id of ['topk','old-score','current-score'])$(id).addEventListener('input',()=>{$(id+'-out').value=id==='topk'?$(id).value:Number($(id).value).toFixed(2);redraw()});for(const id of ['show-gt','show-ignore','show-trusted','show-scores','match-color'])$(id).addEventListener('change',redraw);source.addEventListener('change',applyFilter);$('search').addEventListener('input',applyFilter);$('page-size').addEventListener('change',()=>{page=0;render()});for(const b of document.querySelectorAll('[data-page]'))b.addEventListener('click',()=>{const pages=Math.max(1,Math.ceil(filtered.length/Number($('page-size').value)));if(b.dataset.page==='first')page=0;if(b.dataset.page==='prev')page--;if(b.dataset.page==='next')page++;if(b.dataset.page==='last')page=pages-1;render();scrollTo({top:document.querySelector('main').offsetTop-120,behavior:'smooth'})});render();
</script></body></html>"""


def main() -> None:
    args = _args()
    if args.validation_images < 1 or args.workers < 0:
        raise ValueError("validation images must be positive and workers nonnegative")
    if args.batch_size is not None and args.batch_size < 1:
        raise ValueError("batch size must be positive")
    if any(not 0 <= value <= 1 for value in (args.historical_nms, args.current_nms)):
        raise ValueError("NMS thresholds must be in [0, 1]")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA visual audit requested but no CUDA device is visible")
    output_dir = args.output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    (output_dir / "images").mkdir()

    config = load_phase3_config(args.config)
    annotations = config.data.quad_val_annotations or config.data.val_annotations
    dataset = QuadProposalDataset(
        annotations,
        config.data.image_root,
        config.data.index_dir / "quad_val.sqlite",
        config.data,
        config.augmentation,
        training=False,
        force_index=False,
    )
    selected = select_source_mixture_indices(
        dataset.records, config.data.source_weights, args.validation_images
    )
    subset = Subset(dataset, selected)
    loader = DataLoader(
        subset,
        batch_size=args.batch_size or config.data.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.workers > 0,
        collate_fn=collate_quad_proposal_samples,
    )
    if device.type == "cuda":
        warmup_compiled_quad_iou(device)
    records = _infer(
        label="current",
        checkpoint=args.current_checkpoint,
        checkpoint_state=args.checkpoint_state,
        nms_threshold=args.current_nms,
        loader=loader,
        config=config,
        device=device,
        amp=args.amp,
        log_interval=args.log_interval,
        output_dir=output_dir,
        records=None,
    )
    records = _infer(
        label="historical",
        checkpoint=args.historical_checkpoint,
        checkpoint_state=args.checkpoint_state,
        nms_threshold=args.historical_nms,
        loader=loader,
        config=config,
        device=device,
        amp=args.amp,
        log_interval=args.log_interval,
        output_dir=output_dir,
        records=records,
    )
    source_counts = dict(
        sorted(
            Counter(dataset.records[index].source_dataset for index in selected).items()
        )
    )
    data = {
        "schemaVersion": "visual-inference-quad-visual-comparison.v1",
        "images": len(records),
        "sourceCounts": source_counts,
        "manifestSha": _sha256(Path(annotations)),
        "subsetSha": _selected_identity(dataset, selected),
        "configSha": _sha256(args.config),
        "models": {
            "historical": {
                "checkpoint": str(args.historical_checkpoint.resolve()),
                "checkpointSha": _sha256(args.historical_checkpoint),
                "state": args.checkpoint_state,
                "nms": args.historical_nms,
            },
            "current": {
                "checkpoint": str(args.current_checkpoint.resolve()),
                "checkpointSha": _sha256(args.current_checkpoint),
                "state": args.checkpoint_state,
                "nms": args.current_nms,
            },
        },
        "records": records,
    }
    (output_dir / "audit_manifest.json").write_text(
        json.dumps(
            {key: value for key, value in data.items() if key != "records"}, indent=2
        )
        + "\n",
        encoding="utf-8",
    )
    linked_document = _html_document(data)
    (output_dir / "index_linked.html").write_text(
        linked_document,
        encoding="utf-8",
    )
    if args.embed_images:
        _embed_record_images(data, output_dir)
    (output_dir / "index.html").write_text(_html_document(data), encoding="utf-8")
    print(f"Wrote {len(records)}-image visual audit to {output_dir / 'index.html'}")


if __name__ == "__main__":
    main()
