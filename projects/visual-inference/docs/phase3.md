# Phase 3 — Train the class-agnostic proposal detector

## Objective

Phase 3 trains the horizontal-box architecture laid out and export-verified in
Phase 2. Given a 384×384 RGB image, the model should return at most 100
high-recall, class-agnostic object proposals. This phase answers only **where an
object may be**. It does not add semantic embeddings, text prompts,
open-vocabulary classification, attributes, quadrilateral regression, or
quantization.

The trained path remains:

```text
384×384 RGB
    ↓
MobileNetV4-Conv-Medium, ImageNet pretrained
    ↓
C3 / C4 / C5 at strides 8 / 16 / 32
    ↓
96-channel Lite FPN
    ↓
P3 / P4 / P5
    ↓
shared lightweight head
    ├── one-DSConv object tower → objectness
    └── two-DSConv regression tower
        ├── positive LTRB distances
        └── centerness
```

The raw network still produces fixed P3–P5 tensors. Assignment, loss target
construction, top-K selection, box decoding, clipping, and NMS stay outside
`StudentDetector.forward`, preserving the Phase-2 `torch.export` contract.

## 1. What was implemented

Phase 3 adds:

- a streaming index for the 1.62 GB merged training annotation file;
- fixed-ratio domain sampling;
- geometry-aware augmentation, mean-colour letterboxing, and ImageNet
  normalization;
- valid-image and ignore-region masks;
- the Phase-3 ATSS policy;
- configurable focal or Quality Focal Loss (QFL), uniformly weighted CIoU or
  GIoU, a stride-normalized auxiliary LTRB loss, and an optional FCOS
  centerness loss;
- a controlled second depthwise-separable block in the shared regression tower,
  added after diagnostics isolated localization as the dominant Stage-0
  ceiling;
- two-stage backbone freezing and fine-tuning;
- AdamW, square-root batch LR scaling, warm-up, cosine decay, AMP, gradient
  clipping, and EMA;
- resumable checkpoints and structured JSONL logging;
- class-agnostic AP and proposal-recall evaluation;
- real-data auditing and HBB visualization tools.

The implementation deliberately keeps HBB as the only geometry. The abandoned
Gliding-Vertex experiment is not part of this code path.

## 2. Phase-1 corrections required by training

The old derived data was unsafe for proposal training. Phase 3 therefore made
incremental corrections to the existing Phase-1 pipeline and regenerated its
outputs.

### 2.1 Duplicate geometry representations

The Dataset Ninja COCO source frequently represented one instance twice: once
as a polygon and once as a rectangle with the same class and envelope.
Converting both to HBB produced duplicate positives.

The conversion now groups objects by:

```text
(canonical class, enclosing box rounded to 1e-6)
```

It removes only matched mixed-geometry pairs and keeps the rectangle. Two
coincident rectangles are retained because they may be separate real
instances. Real regeneration removed:

| Source | Duplicates removed |
| --- | ---: |
| COCO 2017 | 998,769 |
| nuImages | 4,579 |
| BDD100K | 0 |
| WoodScape | 0 |

### 2.2 Explicit ignore regions

`__ignore_region__` previously behaved like `__ignore__` and was deleted.
Deletion would turn a visible uncertain/crowd region into negative background.

It is now exported as a reserved category with:

```json
{
  "ignore_region": true,
  "iscrowd": 1
}
```

The category is metadata, not a trainable class. The regenerated training split
contains 10,109 explicit ignore regions.

### 2.3 Source attributes

Supervisely object tags are normalized into the exported `attributes` mapping.
This preserves information such as WoodScape occlusion instead of losing it
during HBB conversion. The current baseline does not infer dataset-specific
ignore decisions merely from tag text; such a rule needs a separately reviewed
mapping.

### 2.4 Sequence-safe WoodScape split

The earlier merger hashed individual WoodScape training frames into train and
validation. That could place adjacent frames on both sides. The regenerated
split instead uses:

- original WoodScape train: 12,234 training images;
- original annotated WoodScape test: 2,766 validation images.

No per-frame random split remains.

### 2.5 Final regenerated data

| Split | Images | Annotations |
| --- | ---: | ---: |
| train | 267,800 | 4,024,644 |
| validation | 34,211 | 551,141 |

The final link audit resolved all 302,011 referenced images directly into raw
extractions and found zero broken links. It also removed 300,476 obsolete
derived symlinks left by previous split layouts; raw files were not deleted.

## 3. Dataset mixture and incomplete supervision

The available baseline uses the agreed source mixture:

| Domain | Source | Sampling weight |
| --- | --- | ---: |
| general | COCO 2017 | 50% |
| rectilinear automotive | nuImages | 18% |
| rectilinear automotive | BDD100K | 12% |
| fisheye | WoodScape | 20% |

Every batch has fixed domain quotas computed by largest remainder. Within the
30% automotive quota, nuImages and BDD are selected using their relative source
weights. Pools reshuffle and cycle independently, so dataset size does not
silently determine the mixture. The sampler requests empty-positive images at
7.5% by default.

Objects365 and Waymo are configuration TODOs because neither archive is
available locally. They are not silently substituted.

COCO supervises valid unannotated locations as background with weight 1.0.
BDD100K and nuImages supervise annotated positives, LTRB, and centerness, but
their unannotated locations receive no objectness loss. Their category-limited
labels make a hard background interpretation unsafe.

WoodScape is a controlled exception. Valid, non-ignore unannotated locations
receive a weak objectness weight of 0.05, while every assigned positive keeps
weight 1.0 and ignore/padding locations keep weight 0. This retains a small
ranking signal in the sparse fisheye domain without claiming that every
unlabelled object is background. The weight is configured per source through
`data.background_loss_weights` and can be overridden with
`--woodscape-background-weight`. A deterministic Stage-0 comparison is
required before using this policy in production; its result is reported in
Section 13.

## 4. Annotation states

Every transformed annotation becomes one of three states:

1. **Positive:** valid object used by ATSS.
2. **Ignore:** visible but unreliable region; points inside it receive no
   objectness loss unless selected by a valid positive.
3. **Invalid:** structurally unusable or less than 20% visible; it is excluded.

After transformation, a positive is changed to ignore when:

```text
area < 100 px²
OR min(width, height) < 4 px
OR visible fraction is in [0.2, 0.6)
```

Less than 20% visibility is invalid. At least 60% visibility remains positive.
This policy runs in final 384×384 coordinates, not original-image coordinates.

### Component containment

The configuration defines human, vehicle, and common component names plus
possible parent names. A component becomes ignore only when:

```text
intersection(component, parent) / area(component) > 0.8
```

A standalone shoe or wheel remains a positive. The current regenerated
annotations contain zero configured component labels, so this policy is ready
but dormant for the four-source baseline.

## 5. Preprocessing and masks

Training combines aspect-preserving resize, scale jitter in `[0.8, 1.2]`, and
translation up to 10% into one affine mapping onto a 384×384 canvas. It uses no
rotation, shear, Mosaic, MixUp, CutMix, or arbitrary vertex warping.

Photometric augmentation includes:

- horizontal flip;
- brightness, contrast, and saturation jitter;
- mild Gaussian blur;
- mild tensor noise;
- JPEG recompression.

Padding uses the ImageNet mean RGB colour, then the whole image is normalized
with the ImageNet mean and standard deviation expected by the pretrained
backbone.

The same affine operation transforms a binary valid-image mask. P3/P4/P5 masks
sample it at exact cell centres:

```text
x = (column + 0.5) × stride
y = (row    + 0.5) × stride
```

These masks affect:

- ATSS candidate selection;
- focal-loss supervision;
- negative counting;
- inference top-K.

They do **not** multiply feature tensors. An optional calibrated fisheye lens
mask can be added later; none was supplied by the current Phase-1 data, and dark
pixels are never guessed to be invalid.

## 6. Streaming index

Loading the 1.62 GB COCO JSON into every DataLoader worker is impractical.
`build_coco_sqlite_index` streams it with `ijson` into SQLite:

- images and source/domain metadata;
- HBB coordinates and explicit ignore state;
- canonical category name for component filtering;
- positive count for empty-image sampling;
- source file signature and index schema version.

The current indexes are:

| Split | Size |
| --- | ---: |
| train | about 235 MB |
| validation | about 31 MB |

Each worker opens its own read connection lazily. The index is reused only when
both its schema version and source path/size/mtime signature match.

## 7. ATSS

The detector remains FCOS-style and anchor-free. Virtual square priors exist
only for ATSS IoU calculation:

| Level | Stride | Virtual side |
| --- | ---: | ---: |
| P3 | 8 | 64 |
| P4 | 16 | 128 |
| P5 | 32 | 256 |

For each GT:

1. choose the nine nearest valid locations on each level;
2. compute candidate-prior IoUs against the GT HBB;
3. use `mean(IoU) + population_std(IoU)` as the threshold;
4. require the point to be inside the GT;
5. require the point to be within a `1.5 × stride` centre region clipped to the
   GT;
6. resolve multi-GT conflicts by greatest prior IoU.

If no normal positive survives, the fallback selects a free valid point inside
the GT, preferring ATSS candidates, highest prior IoU, and then nearest centre.
It never assigns an outside point because the ReLU head cannot emit negative
LTRB distances. A GT with no free valid inside point becomes ignore and is
reported as unrepresentable.

### Real-data audit and nuImages fallback inspection

A deterministic 300-image audit following the 50/18/12/20 source mixture found:

| Metric | Result |
| --- | ---: |
| transformed positive instances | 2,815 |
| ignore instances | 2,280 |
| positive feature locations | 13,669 |
| fallback rate per positive GT | 9.20% |
| unrepresentable rate per positive GT | 1.63% |
| mean valid-point fraction | 68.13% |

Positive locations were P3=11,222, P4=1,780, P5=667. Fallback was concentrated
in nuImages (31.94% on this slice), so the configured 2% investigation warning
is active.

Controlled diagnostics on the same sample:

| Assignment variant | Fallback | Unrepresentable |
| --- | ---: | ---: |
| agreed baseline, radius 1.5 | 9.20% | 1.63% |
| pure ATSS, no centre gate | 9.06% | 1.74% |
| old 32/64/128 priors, radius 1.5 | 5.33% | 1.21% |
| old priors, no centre gate | 5.26% | 1.21% |

The centre gate is therefore not the main cause. A second source-specific audit
inspected 1,000 positive nuImages records (8,353 valid GTs) and rendered 64
annotated examples:

| Metric | Result |
| --- | ---: |
| fallback GT | 2,525 (30.23%) |
| unrepresentable GT | 780 (9.34%) |
| persistent with all tested variants | 1,663 |
| resolved by 4×stride priors | 780 |
| resolved only by removing the centre gate | 82 |

Of the fallback instances, 2,126 were in the audit's smallest size bin. The
largest categories were car (1,091), adult pedestrian (778), bicycle (130),
truck (126), barrier (123), and traffic cone (113). Visual inspection confirms
that the dominant pattern is densely packed, very small objects after
letterboxing to 384×384, rather than a systematic centre-gate error.

The machine-readable report and overlays are under:

```text
artifacts/phase3/runs/analysis/nuimages_fallbacks/summary.json
artifacts/phase3/runs/analysis/nuimages_fallbacks/examples/
```

### Controlled prior decision

The 4×stride Stage-0 control reduced fallback from about 13.3% to 8.4%, but its
recall advantage disappeared as training progressed. At 500 steps:

| Priors | Recall@100/0.50 | Edge-small recall | P3/P4/P5 positives |
| --- | ---: | ---: | --- |
| 8×stride | 84.98% | 64.08% | 393/76/20 |
| 4×stride | 85.18% | 64.08% | 247/62/82 |

The smaller priors moved many positives from P3 to P5 without improving the
small-object slice. The current evidence therefore supports retaining
64/128/256 (8×stride) priors for the baseline. The interrupted 4× run's
`last.pt` is incomplete; only its valid `best.pt` and JSONL logs should be used.

## 8. Losses

The current baseline objective is:

```text
L = 1.0 L_QFL + 2.0 L_CIoU + 0.5 L_LTRB
```

The centerness output remains in the fixed Phase-2 model signature, but its
loss weight is zero and it is not used for baseline ranking. This preserves
checkpoint/export compatibility while testing the Generalized Focal Loss
hypothesis without an architecture change.

### Quality objectness

QFL uses `beta=2`. Background targets are zero. At an assigned positive, the
target is the detached aligned IoU between the current decoded prediction and
its GT. Objectness therefore learns proposal quality rather than a hard
foreground bit. The unreduced QFL term is multiplied by the per-location source
weight before summation:

```text
L_QFL = sum(w_i QFL_i) / max(N_positive, 1)
```

Here `w=1` for positives and dense COCO background, `w=0.05` for valid
WoodScape background, and `w=0` for padding, ignore regions, and unsupervised
BDD/nuImages background. A positive assignment always overrides an ignore or
weak-background weight.

The original sigmoid focal loss (`alpha=0.25`, `gamma=2`) remains implemented
and selectable for ablation.

### HBB regression

Predicted and target LTRB distances are decoded at positive feature points.
CIoU is averaged **uniformly over all positive ATSS samples**:

```text
L_box = sum(CIoU(predicted_box_i, target_box_i)) / N_positive
```

Centerness no longer gates regression gradients. This is important for edge and
fallback positives: every assigned location receives the same box-loss weight.
Aligned GIoU is implemented as the requested controlled alternative.

CIoU is complemented by a direct edge-distance objective:

```text
L_LTRB =
    1 / (4 N_positive)
    × sum SmoothL1(predicted_LTRB / stride,
                   target_LTRB / stride)
```

Normalization by the positive location's own P3/P4/P5 stride makes one unit
represent one feature-cell displacement on every level. Averaging over both
positives and the four coordinates keeps the configured weight independent of
positive count and coordinate count. It is graph-connected zero when a batch
has no positives.

This term does not add an inference operator or modify the exported model. In
the controlled 2,000-step Stage-0 run it raised final network recall from
88.14% to 89.13%. More importantly, oracle ranking of the learned boxes rose
from 94.47% to 97.83%, proving that learned localization now exceeds the 95%
gate. The remaining failure is score ranking, not box capacity.

The shared regression tower now contains two depthwise-separable Conv–BN–ReLU
blocks; the object tower remains one block. This keeps the same fixed output
API and quantization-friendly operators while adding one inexpensive nonlinear
refinement before LTRB prediction. On the deterministic Stage-0 run it improved
final recall@100/0.50 from 87.15% to 88.14%, edge-small recall from 64.08% to
67.61%, and the oracle-ranking localization ceiling from 93.48% to 94.47%.
That is useful evidence for retaining the block, although it did not by itself
pass the 95% gate. Checkpoints from the earlier one-block head are not
strictly load-compatible because the new block adds parameters.

The controlled GIoU run reached 83.20% recall@100/0.50 at step 500, compared
with 84.98% for CIoU under the otherwise identical protocol. CIoU therefore
remains the baseline box-overlap loss.

### Centerness

The optional auxiliary target is still standard FCOS centerness:

```text
sqrt(
    min(l,r) / max(l,r)
  × min(t,b) / max(t,b)
)
```

BCE-with-logits is evaluated only at positives when its configured weight is
nonzero. A focal + centerness control was worse than QFL at 500 Stage-0 steps:
79.45% versus 84.98% recall@100/0.50. QFL is therefore retained for the next
baseline.

Empty-positive batches produce finite, graph-connected zero regression and
centerness losses.

Normalizers are globally reduced when `torch.distributed` is initialized, so
different microbatch object counts do not change per-rank scaling.

## 9. Optimization and BatchNorm

The default schedule is:

- Stage 1: two epochs with frozen backbone weights and frozen backbone BN
  running statistics;
- Stage 2: unfreeze backbone weights for the remainder of 40 epochs;
- keep backbone BN statistics frozen throughout; BN affine parameters train
  during Stage 2;
- FPN/head BN remains in normal training mode;
- AdamW with weight decay `1e-4`;
- head/FPN reference LR `3e-4` at effective batch 64;
- backbone LR is `0.1×` the head LR;
- square-root LR scaling for actual effective batch;
- 1,000 optimizer-step warm-up, then cosine decay to 1%;
- gradient norm clipping at 10;
- EMA decay 0.9998;
- AMP only on CUDA.

The default physical batch is 10 so FPN/head BN sees more than eight examples.
On the available RTX 3060, batch 10 used about 0.85 GiB and sustained roughly
12 images/s in Stage 0. A physical-batch-50 probe fit in 3.74 GiB but sustained
only about 10–11 images/s and provided five times fewer optimizer updates for
the fixed image exposure. It is therefore not automatically the fastest
training choice. Production batch selection still needs a measured throughput
and out-of-memory sweep; “largest that fits” is not equivalent to “fastest.”

Gradient accumulation changes effective batch and LR scaling but does not
improve BN statistics. Multi-device launch/DDP wrapping is not yet provided by
the CLI, although distributed loss normalization is implemented.

## 10. Evaluation

The fixed validation decoder is:

```text
sigmoid(quality objectness)
    → mask padding before top-K
    → top 300
    → decode and clip HBB
    → class-agnostic HBB NMS at 0.6
    → at most 100 proposals
```

Metrics include:

- class-agnostic AP at IoU 0.50 and 0.75;
- recall@10/50/100 at IoU 0.50 and 0.75;
- per-domain recall@100/0.50;
- edge-small (100–256 px²), small (256–1,024 px²), medium
  (1,024–9,216 px²), and large (>9,216 px²) recall;
- fisheye centre/middle/border recall.

Model selection is the mean recall@100/0.50 across domains present in the
validation run; the overall recall is not incorrectly counted as an additional
domain. Ignore-region-overlapping detections do not become AP false positives.
The decoder uses the compiled `torchvision.ops.nms` kernel and remains outside
the exported network.

## 11. Training and debugging commands

Run tests:

```bash
uv run pytest -q tests
cd dataset-pipeline
uv run pytest -q
```

Audit a balanced transformed sample:

```bash
uv run python scripts/audit_phase3_data.py --samples 300
```

Stage-0 overfit on a fixed balanced 50-image subset:

```bash
uv run python scripts/train_phase3.py \
  --overfit-images 50 \
  --epochs 400 \
  --max-steps 2000 \
  --batch-size 10 \
  --workers 4 \
  --validation-interval 5 \
  --ltrb-weight 0.5 \
  --woodscape-background-weight 0.05 \
  --output-dir \
    artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005
```

Do not start full training until Stage 0 reaches at least 95% recall@100/0.50
on its fixed subset and fallback examples have been visually inspected.

Inspect nuImages fallback categories and overlays:

```bash
uv run python scripts/inspect_atss_fallbacks.py \
  --source nuimages \
  --samples 1000 \
  --visualize 64
```

Separate assignment, localization, and ranking ceilings:

```bash
uv run python scripts/diagnose_stage0_checkpoint.py \
  --checkpoint artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05/last.pt
```

Test BN-statistics and fixed-decoder hypotheses:

```bash
uv run python scripts/diagnose_batchnorm.py \
  --checkpoint artifacts/phase3/runs/overfit50-gpu-qfl-balanced/best.pt
uv run python scripts/diagnose_decoder_sweep.py \
  --checkpoint artifacts/phase3/runs/overfit50-gpu-qfl-balanced/best.pt
```

Dump machine-readable curve tables and PNG plots:

```bash
uv run --group inspection python scripts/plot_phase3_curves.py \
  artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05
```

Production baseline:

```bash
uv run python scripts/train_phase3.py \
  --config configs/phase3.yaml \
  --batch-size 64 \
  --workers 4 \
  --epochs 40 \
  --output-dir artifacts/phase3/runs/production-qfl-ciou-ltrb05-woodbg005-b64
```

The CLI also exposes one-variable experiment controls:

```text
--box-loss {ciou,giou}
--pure-atss
--prior-multiplier {4,8}
--tiny-area FLOAT
--tiny-min-side FLOAT
--objectness-loss {focal,quality_focal}
--ltrb-weight FLOAT
--centerness-weight FLOAT
--score-mode {objectness,objectness_x_centerness}
--woodscape-background-weight FLOAT
--freeze-backbone-epochs INT
```

Bounded smoke:

```bash
uv run python scripts/train_phase3.py \
  --device cpu \
  --batch-size 1 \
  --workers 0 \
  --batches-per-epoch 1 \
  --max-steps 1 \
  --max-val-batches 1
```

Visualize a checkpoint:

```bash
uv run python scripts/visualize_phase3.py \
  --checkpoint artifacts/phase3/runs/baseline/best.pt \
  --count 12
```

Green is GT, yellow is ignore, and red is prediction with score. For the
one-step random-head smoke checkpoint, use `--score-threshold 0` only to inspect
geometry; those proposals are not meaningful detections.

## 12. Logging and checkpoint contract

`metrics.jsonl` contains:

- total/objectness/CIoU/LTRB/centerness loss;
- positive count and GT count;
- fallback and unrepresentable counts;
- fallback rate per GT;
- positive counts for P3/P4/P5;
- both learning rates;
- all validation metrics.

`last.pt` and `best.pt` contain:

- raw model state;
- EMA model state;
- optimizer and scheduler state;
- epoch and global optimizer step;
- the complete resolved configuration;
- last metrics;
- format version, phase, and architecture identifier.

Resume at the next epoch boundary with:

```bash
uv run python scripts/train_phase3.py \
  --resume artifacts/phase3/runs/baseline/last.pt
```

## 13. Verification status

| Check | Result |
| --- | --- |
| detector tests | 36 passed |
| dataset-pipeline tests | 41 passed |
| Phase-2 fixed forward/export API | preserved |
| synthetic full forward/loss/backward | finite gradients in backbone, FPN, and head |
| zero-positive batch | finite |
| valid/ignore/positive override | tested |
| padding excluded before inference top-K | tested |
| source mixture | tested |
| sequence-safe WoodScape split | tested and regenerated |
| final raw-image links | 302,011 valid, zero broken |
| real CPU smoke | one optimizer step and one validation batch succeeded |
| CUDA runtime | PyTorch 2.6.0+cu124 on RTX 3060 / driver 580.173.02 |

The final seeded real smoke used ImageNet-pretrained MobileNetV4. It produced
finite gradients. The current uv environment already sees CUDA correctly; a
driver change did not require reinstalling PyTorch.

### Stage-0 result and production gate

The corrected deterministic 50-image subset contains exact source quotas:
25 COCO, 9 nuImages, 6 BDD100K, and 10 WoodScape images. All completed
QFL/uniform-CIoU comparisons used 2,000 optimizer steps:

| Metric | One regression block | Two blocks | + LTRB | + 0.05 WoodScape background |
| --- | ---: | ---: | ---: | ---: |
| recall@100/0.50 | 87.15% | 88.14% | 89.13% | **96.25%** |
| recall@100/0.75 | 83.00% | 84.58% | 84.78% | **92.89%** |
| median matched IoU | 0.905 | 0.902 | 0.907 | **0.910** |
| general recall | 94.48% | 95.71% | **99.39%** | **99.39%** |
| automotive recall | 96.95% | **99.24%** | **99.24%** | 98.47% |
| fisheye recall | 75.47% | 75.47% | 75.00% | **92.45%** |
| edge-small recall | 64.08% | 67.61% | 69.72% | **88.03%** |
| selection score | 0.8917 | 0.9014 | 0.9121 | **0.9677** |

The two-block run's best selection checkpoint was step 1,200 with 87.94%
recall@100/0.50; final step 2,000 improved it slightly further to 88.14%.
The extra block is retained because it improved the principal recall metric,
the edge-small slice, and the localization oracle at negligible Stage-0 memory
cost. It does not change the exported tensor contract.

The LTRB-only run peaked at step 900 with 89.53% recall@100/0.50 and a
0.91475 selection score, then finished at 89.13%. Its localization oracle
reached 97.83%, isolating score ranking as the blocker. The next run changed
only the background policy: valid WoodScape negatives received weight 0.05.
It first crossed the 95% gate at step 1,175 and continued improving; step
2,000 is both the final and best checkpoint.

The final diagnostic separates assignment, localization, and ranking:

| Diagnostic | Recall@100/0.50 |
| --- | ---: |
| network scores + predicted boxes | **96.25%** |
| oracle scores + predicted boxes | 97.43% |
| exact ATSS target boxes | 98.22% |

Ranking now costs only 1.18 percentage points relative to oracle-ranked
predicted boxes. On the difficult slices, network/oracle recall is
88.03%/91.55% for edge-small and 92.45%/94.81% for fisheye. The result supports
the intended interpretation: a small amount of valid fisheye background
supervision fixes the score ordering without treating WoodScape as a
dense-category dataset. The greater-than-95% Stage-0 gate is therefore
**passed**.

Additional controlled checks did not close the gap:

- live head BN statistics: 87.35%, identical to stored running statistics;
- live FPN + head BN statistics: 86.56%;
- pre-NMS top-K 300 through 3,024: no change;
- NMS 0.5: 88.34% recall, versus 87.35% at 0.6;
- focal + centerness at step 500: 79.45%, versus QFL 84.98%;
- 4×stride priors at step 500: 85.18%, versus 8×stride 84.98%.
- GIoU at step 500: 83.20%, versus CIoU 84.98%.

The accepted curve and diagnostic bundle is:

```text
artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005/curves/epoch_metrics.csv
artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005/curves/step_progress.csv
artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005/curves/training.png
artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005/curves/validation.png
artifacts/phase3/runs/overfit50-gpu-qfl-ciou-reg2-ltrb05-woodbg005/stage0_diagnostic_final.json
```

The LTRB-only, no-LTRB two-block, one-block CIoU, and controlled GIoU bundles
remain under
`overfit50-gpu-qfl-ciou-reg2-ltrb05/curves/`,
`overfit50-gpu-qfl-ciou-reg2/curves/`,
`overfit50-gpu-qfl-balanced/curves/` and
`overfit50-gpu-qfl-giou/curves/`, respectively.

### Production throughput selection

The real augmented pipeline was measured for 25 unfrozen-backbone optimizer
steps on the RTX 3060:

| Batch / workers | Images/s | Peak CUDA GiB |
| --- | ---: | ---: |
| 16 / 4 | 13.11 | 1.32 |
| 32 / 4 | 15.10 | 2.49 |
| 48 / 4 | 13.12 | 3.65 |
| **64 / 4** | **15.30** | **4.87** |
| 96 / 4 | 14.86 | 7.21 |
| 32 / 6 | 14.54 | 2.49 |
| 48 / 6 | 14.41 | 3.65 |
| 64 / 6 | 14.20 | 4.87 |

Batch 64 with four workers is the measured production choice. It is the
reference effective batch used by the LR configuration, has over 7 GiB of GPU
headroom, and is faster than both smaller and larger alternatives. At the
measured rate, 40 passes over 267,800 training images require roughly 194
training hours, before full-validation overhead.

## 14. Code map

| File | Responsibility |
| --- | --- |
| `student_detector/config.py` | typed Phase-3 YAML configuration |
| `student_detector/data.py` | streaming index, transforms, masks, sampler |
| `student_detector/assigner.py` | valid-aware ATSS, centre gate, fallback |
| `student_detector/targets.py` | batch target and loss-mask construction |
| `student_detector/head.py` | shared one-block object and two-block regression towers |
| `student_detector/losses.py` | focal/QFL, CIoU/GIoU, stride-normalized LTRB, optional centerness |
| `student_detector/training.py` | optimizer, schedule, BN policy, EMA, checkpoints |
| `student_detector/evaluation.py` | AP, recall, size and radial metrics |
| `student_detector/decoder.py` | masked top-K, HBB decode, NMS |
| `scripts/train_phase3.py` | training/Stage-0/smoke CLI |
| `scripts/audit_phase3_data.py` | real transformed-data and ATSS audit |
| `scripts/inspect_atss_fallbacks.py` | source-specific fallback categorization and overlays |
| `scripts/diagnose_stage0_checkpoint.py` | assignment/localization/ranking ceilings |
| `scripts/diagnose_batchnorm.py` | running-statistics versus batch-statistics control |
| `scripts/diagnose_decoder_sweep.py` | cached-forward top-K/NMS sweep |
| `scripts/plot_phase3_curves.py` | JSONL-to-CSV and PNG curve dump |
| `scripts/visualize_phase3.py` | GT/ignore/proposal overlays |
| `configs/phase3.yaml` | reproducible baseline settings |

## 15. Remaining work before Phase 3 is complete

1. ~~Inspect and categorize nuImages fallback examples.~~ Completed.
2. ~~Resolve the Stage-0 localization/ranking blocker and reach
   recall@100/0.50 above 95%.~~ Passed at 96.25% after adding 0.05-weight
   WoodScape background supervision.
3. ~~Decide whether to retain the 8×stride priors.~~ Retain them based on the
   controlled 4×/8× comparison.
4. ~~Measure the fastest physical batch.~~ Batch 64 / four workers won the
   controlled sweep. Complete the 40-epoch production CUDA job.
5. Inspect production learning curves and qualitative failure slices.
6. ~~Run the requested controlled Stage-0 GIoU/CIoU comparison.~~ CIoU won
   84.98% to 83.20% at step 500. Run pure-ATSS and tiny-threshold production
   ablations only after the Stage-0 gate and baseline production job.
7. Profile the trained random-free model on the intended Pi. No Pi connection
   or executable target is configured in this workspace yet.
8. Add a calibrated WoodScape lens-validity mask if calibration metadata
   becomes available. No calibration file is present in the current data.

Do not begin Phase 4 semantic/open-vocabulary work until these localization
criteria are satisfied.

## 16. References

1. Qin et al., [*MobileNetV4: Universal Models for the Mobile
   Ecosystem*](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/05647.pdf),
   ECCV 2024.
2. `timm`, [feature extraction and `feature_info`
   documentation](https://huggingface.co/docs/timm/feature_extraction).
3. Lin et al., [*Feature Pyramid Networks for Object
   Detection*](https://openaccess.thecvf.com/content_cvpr_2017/html/Lin_Feature_Pyramid_Networks_CVPR_2017_paper.html),
   CVPR 2017.
4. Tian et al., [*FCOS: Fully Convolutional One-Stage Object
   Detection*](https://openaccess.thecvf.com/content_ICCV_2019/html/Tian_FCOS_Fully_Convolutional_One-Stage_Object_Detection_ICCV_2019_paper.html),
   ICCV 2019; [reference implementation](https://github.com/tianzhi0549/FCOS).
5. Zhang et al., [*Adaptive Training Sample
   Selection*](https://openaccess.thecvf.com/content_CVPR_2020/html/Zhang_Bridging_the_Gap_Between_Anchor-Based_and_Anchor-Free_Detection_via_Adaptive_CVPR_2020_paper.html),
   CVPR 2020; [reference implementation](https://github.com/sfzhang15/ATSS).
6. Lin et al., [*Focal Loss for Dense Object
   Detection*](https://openaccess.thecvf.com/content_ICCV_2017/html/Lin_Focal_Loss_for_ICCV_2017_paper.html),
   ICCV 2017.
7. Zheng et al., [*Distance-IoU Loss: Faster and Better Learning for Bounding
   Box Regression*](https://ojs.aaai.org/index.php/AAAI/article/view/6999),
   AAAI 2020.
8. Loshchilov and Hutter, [*Decoupled Weight Decay
   Regularization*](https://openreview.net/forum?id=Bkg6RiCqY7), ICLR 2019.
9. PyTorch, [automatic mixed-precision
   examples](https://docs.pytorch.org/docs/stable/notes/amp_examples.html).
10. PyTorch, [`torch.export`
    documentation](https://docs.pytorch.org/docs/stable/user_guide/torch_compiler/export.html).
11. Li et al., [*Generalized Focal Loss: Learning Qualified and Distributed
    Bounding Boxes for Dense Object
    Detection*](https://proceedings.neurips.cc/paper/2020/hash/f0bda020d2470f2e74990a07a607ebd9-Abstract.html),
    NeurIPS 2020; [reference
    implementation](https://github.com/implus/GFocal).
12. Rezatofighi et al., [*Generalized Intersection over Union: A Metric and a
    Loss for Bounding Box
    Regression*](https://openaccess.thecvf.com/content_CVPR_2019/html/Rezatofighi_Generalized_Intersection_Over_Union_A_Metric_and_a_Loss_for_CVPR_2019_paper.html),
    CVPR 2019.
