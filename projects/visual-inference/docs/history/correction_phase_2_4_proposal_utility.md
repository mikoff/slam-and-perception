# Correction Phase 2.4 proposal utility

The fixed-policy report is produced by `scripts/evaluate_proposal_utility.py`.

Status: complete; owner selected HBB primary and quad control.

## Evaluation contract

This evaluation uses the Phase 2.1 EMA checkpoints, the same deterministic
800-image cohort as Phase 2.3, final K=100, and the owner-approved suppression
policy in `configs/benchmarks/correction_phase2_utility_v1.yaml`: HBB
NMS/score `0.70/0.30` and quad `0.80/0.20`. The cohort contains 5,227 positive
objects from 400 COCO, 160 WoodScape, 144 nuImages, and 96 BDD100K images.

Object coverage is the fraction of each labeled object's polygon covered by its
best proposal. Object fraction is the fraction of each proposal inside its
best-overlapping labeled object; its complement is labeled non-object fraction,
not certified background. A trusted-background false proposal has maximum
object IoU below 0.10, less than 50% overlap with every ignore polygon, and at
least 50% area in the disjoint trusted-background union. These polygon-area
metrics describe proposal geometry; actual crop generation and SigLIP remain
deferred by the active milestone.

## Aggregate results

All quality and budget values below are percentages. `Coverage` and `object
fraction` are polygon-area percentages at K=100.

| Geometry | AR10 | AR50 | AR100 | R10@.50 | R50@.50 | R100@.50 | R100@.75 | Coverage | Object fraction | Trusted-bg false | Unmatched | Duplicate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| HBB primary | **8.85** | **14.79** | **17.47** | **23.82** | **38.84** | **45.21** | 10.04 | **83.28** | 50.83 | **1.29** | 47.79 | **7.33** |
| Quad control | 8.56 | 14.04 | 16.55 | 22.14 | 34.97 | 41.00 | **10.98** | 70.41 | **53.05** | 1.95 | **46.55** | 10.49 |

HBB wins the fixed-budget utility balance. Quad's 0.94-point advantage at
R100/0.75 and 2.23-point object-fraction advantage do not offset HBB's
4.21-point R100/0.50 lead, 12.86-point coverage lead, lower duplicate and
trusted-background rates, or simpler post-processing.

## Source and domain results

| Geometry | Source | Objects | AR100 | R100@.50 | R100@.75 | Coverage | Trusted-bg false |
|---|---|---:|---:|---:|---:|---:|---:|
| HBB | BDD100K | 459 | **35.29** | **72.77** | 29.19 | **81.68** | **1.28** |
| Quad | BDD100K | 459 | 33.22 | 67.32 | **30.28** | 80.69 | 2.00 |
| HBB | COCO | 2,135 | **20.52** | **54.15** | 11.38 | **91.12** | 0.00 |
| Quad | COCO | 2,135 | 20.10 | 49.84 | **12.74** | 73.20 | 0.00 |
| HBB | nuImages | 526 | 19.87 | **53.23** | 10.46 | **78.48** | 0.00 |
| Quad | nuImages | 526 | **21.98** | 52.28 | **15.21** | 78.14 | 0.00 |
| HBB | WoodScape | 2,107 | **9.90** | **28.14** | **4.41** | **76.88** | **5.74** |
| Quad | WoodScape | 2,107 | 7.97 | 23.49 | 3.94 | 63.43 | 8.56 |

Automotive AR100 is effectively tied (HBB 27.06%, quad 27.22%), while HBB
leads the general and fisheye domains. WoodScape remains the weakest and most
background-sensitive domain for both heads.

## Size and shape slices

| Geometry | Slice | Objects | R100@.50 | R100@.75 |
|---|---|---:|---:|---:|
| HBB | model tiny | 1,162 | 0.00 | 0.00 |
| Quad | model tiny | 1,162 | 0.00 | 0.00 |
| HBB | model small / medium / large | 1,278 / 1,988 / 799 | 36.62 / 63.93 / 78.10 | 6.57 / 12.83 / 23.28 |
| Quad | model small / medium / large | 1,278 / 1,988 / 799 | 33.33 / 59.26 / 67.46 | 6.18 / 16.25 / 21.53 |
| HBB | thin | 1,007 | 7.65 | 0.30 |
| Quad | thin | 1,007 | **7.94** | **0.99** |

Source short-side R100/0.50 for HBB versus quad is 5.71/5.43% at 16–32px,
34.06/31.98% at 32–64px, 60.82/57.17% at 64–128px, 74.15/67.35% at
128–256px, and 83.20/63.14% at 256px or larger. Both heads score zero for the
428 retained thin-exception objects with 8–16px source short side. The short
matched runs are therefore useful for architecture selection, not acceptable
final small-object performance; this remains a retraining target.

## Background provenance and ego-body finding

The immutable validation index stores trusted background as disjoint 32-pixel
tiles named only `approved_trusted_union`. It no longer retains whether a tile
came from sky, road, structure, vegetation, or another source mask. Therefore
category-specific false rates cannot be reconstructed honestly from this
artifact. Road and vegetation are also positive categories under the approved
contract, so treating them globally as false classes would contradict Phase 1.
No validation annotation supports a license-plate rate.

Ego-body polygons are retained as an identifiable ignore category in 32 cohort
images. At least 50% of proposal area lies inside ego bodywork for 79.20% of HBB
proposals and 78.12% of quad proposals on those images. Because Phase 1 deliberately
quarantined these imperfect masks as ignore rather than trusted negative, this
is an overlap diagnostic, not a certified false-positive rate. Its magnitude is
a clear risk to recheck after full retraining and visual review.

## CPU post-processing timing

| Geometry | Neural inference | Candidate decode | Production NMS | Utility-only overlap |
|---|---:|---:|---:|---:|
| HBB | 67.12 ms/image | 0.96 ms/image | **0.30 ms/image** | 3.22 ms/image |
| Quad | 73.04 ms/image | 5.54 ms/image | 5.99 ms/image | 3.16 ms/image |

This CPU run is experiment evidence, not Raspberry Pi acceptance. Production
quad decode plus exact NMS was about nine times HBB post-processing. Utility
overlap is evaluation-only. Actual crop generation is deferred, so no crop-
generation latency is claimed.

## Decision and artifacts

- Retain HBB P3–P5 as the working primary and quad P3–P5 as the control.
- Keep the approved Phase 2.3 thresholds; Phase 2.4 found no reason to revise
  either score threshold.
- Carry explicit small/thin and WoodScape/ego-body risks into retraining gates.
- Do not claim category-specific background rates that the finalized dataset
  cannot support.

The machine-readable report is
`artifacts/phase2/short_v1/proposal_utility_ema.json`, SHA-256
`f51308fa4f99fa6a24f4b4c6c0ce9464dde8974e512b4463a10d63a4ff65f44b`.
It binds checkpoints, config, policy, validation manifest, selected cohort, all
per-domain/source metrics, size slices, definitions, unavailable rates, and
timing components.
