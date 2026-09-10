# Correction Phase 2.1 matched architecture comparison

Status: measurement complete; owner selection not yet approved.

## Matched contract

Candidates A (HBB P3–P5), B (HBB P2–P5), and C (quad P3–P5) ran
for 400 successful optimizer updates with seed 42, 384 px inputs, batch 8,
four-step accumulation, LiteFPN, 100 warmup steps, and the corrected immutable
Phase 1 dataset. Candidate D (quad P2–P5) was omitted as agreed.

The comparison uses EMA weights and the same deterministic, source-balanced
800-image validation subset. All candidates use exact convex-polygon IoU, a
0.9 NMS threshold, and proposal budgets 10, 50, 100, and diagnostic 300. The validation
manifest SHA-256 is
`68f632d89c281923137d70e0914e706fee5d201dc7e83de51a992ec4163cecbf`.

## Results

Recall and AR are percentages. Timing is milliseconds per image on the local
RTX 3060 and is useful for experiment planning, not Raspberry Pi acceptance.
The HBB `poly NMS` column is the extra exact-polygon suppression used only to
put all candidates through the same evaluation geometry.

| Candidate | Params | R10@.50 | R50@.50 | R100@.50 | R100@.75 | AR100 | Median IoU | Forward | Native decode/NMS | Eval poly NMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A: HBB P3–P5 | 7,065,593 | 18.39 | **33.21** | **40.29** | **12.40** | **17.15** | 36.63 | 10.27 | 4.26 | 31.94 |
| B: HBB P2–P5 | 7,080,570 | 16.84 | 28.89 | 35.60 | 11.12 | 15.23 | 28.60 | 9.40 | **3.79** | 29.88 |
| C: quad P3–P5 | 7,065,881 | **20.16** | 32.94 | 40.06 | 11.44 | 16.58 | **37.23** | **7.72** | 32.30 | — |

Selected diagnostic slices at R100/IoU 0.50:

| Candidate | Radial edge | Slender | Small | Medium | Large |
|---|---:|---:|---:|---:|---:|
| A: HBB P3–P5 | **30.54** | 6.78 | 32.00 | 56.19 | **72.59** |
| B: HBB P2–P5 | 24.61 | 5.78 | 27.78 | 46.53 | **72.59** |
| C: quad P3–P5 | **30.54** | **8.56** | **32.55** | **57.39** | 67.21 |

The HBB-native diagnostic initially made P2 look attractive for the
`edge-small` slice: 11.73% versus 4.01%. That metric uses the HBB target and
rectangle evaluator, however. Under the common source-geometry evaluator P2
loses 4.69 points of R100/0.50 overall and 5.94 points on radial-edge objects.
Its common tiny-object gain is only 0.086 points. The common evaluator is the
selection evidence because it holds targets, geometry, subset, and budgets
fixed.

Quad P3 is effectively tied with HBB P3 at R100/0.50 (−0.23 points) and has a
0.60-point higher median matched IoU. HBB P3 leads AR100 by 0.57 points,
R100/0.75 by 0.96 points, and large-object recall by 5.38 points. Quad leads
R10/0.50 by 1.78 points, but its native post-processing is about 7.6 times the
HBB native decode/NMS time in this run.

## Recommendation and boundary

Use HBB P3–P5 as the provisional primary and retain quad P3–P5 as the control.
Do not promote P2 or spend a run on quad P2 from this evidence. This is a
single-seed short-run architecture screen, not a final trained-model result.

Final Phase 2 selection remains open until the owner approves K and the
false-proposal/duplicate budgets, common coordinate/export parity is complete,
and resource/export evidence is recorded. SigLIP and semantic crop experiments
remain outside the current milestone.

## Reproduction artifacts

The ignored local report is
`artifacts/phase2/short_v1/common_comparison_ema.json`. It records the config,
dataset, checkpoint hashes, global step 400 for every candidate, software
versions, device, metrics, and timings. The matched config is
`configs/benchmarks/correction_phase2_short_v1.yaml`.
The report SHA-256 is
`3c5cf5c75a3a23401f61ad9c03ab4aa7bc0d301ebeddaa9a23c98bc0d8a8877d`.
