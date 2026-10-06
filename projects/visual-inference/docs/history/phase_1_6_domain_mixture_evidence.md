# Phase 1.6 domain-mixture evidence

Status: implementation verified; owner-approved target weights.

Owner decision: keep COCO/nuImages/BDD100K/WoodScape at 50/18/12/20 for the
corrected baseline and revisit only after measurable corrected-domain results.

## Sampling contract

The configured source target remains COCO 50%, nuImages 18%, BDD100K 12%, and
WoodScape 20%, which aggregates to general 50%, automotive 30%, and fisheye
20%. Sampling now uses cumulative largest-remainder quotas across the epoch.
Each batch receives the difference between two cumulative quotas, eliminating
the fixed rounding bias at microbatch size eight.

Every optimizer-step window and epoch writes intended and observed source and
domain counts to `events.jsonl` as `training_mixture` records. Distributed
workers reduce their observed counts before the main process records them.

| Schedule | Samples/epoch | COCO | nuImages | BDD100K | WoodScape |
|---|---:|---:|---:|---:|---:|
| Local: batch 8, 33,475 batches | 267,800 | 133,900 | 48,204 | 32,136 | 53,560 |
| Cloud: batch 128, 2,093 batches | 267,904 | 133,952 | 48,223 | 32,148 | 53,581 |

The first local eight-microbatch optimizer window contains 32/11/8/13 samples;
later windows exchange individual residual samples so the epoch total is exact.
The first cloud batch contains 64/23/15/26 samples.

## Current production-index density

These counts describe the current immutable pre-regeneration training index.
Positive density is before model-coordinate retention and does not count dense
trusted-background tiles as objects.

| Source | Images | Positive images | Positives | Mean/image | Median/image |
|---|---:|---:|---:|---:|---:|
| BDD100K | 70,000 | 69,863 | 1,286,852 | 18.38 | 17 |
| COCO | 118,287 | 117,266 | 1,054,975 | 8.92 | 5 |
| nuImages | 67,279 | 60,665 | 1,319,184 | 19.61 | 14 |
| WoodScape | 12,234 | 8,234 | 247,195 | 20.21 | 20 |

WoodScape is only 4.57% of indexed images and 6.33% of positives, so a 20%
target is deliberate fisheye oversampling. COCO is 44.17% of images but receives
50% to retain broad open-world coverage. The two automotive sources collectively
receive 30% despite representing 51.26% of images and 66.68% of positives.

## Historical failure evidence

The last min-8 quad checkpoint predates the corrected dataset. On its balanced
150-image-per-source visual audit, the object miss rate at best IoU below 0.50
was:

| Source | Evaluable objects | Misses | Miss rate |
|---|---:|---:|---:|
| BDD100K | 1,273 | 331 | 26.00% |
| COCO | 1,155 | 514 | 44.50% |
| nuImages | 1,643 | 581 | 35.36% |
| WoodScape | 0 | — | unavailable |

WoodScape had no evaluable instances because the historical validation contract
excluded its labeled data. Its reported zero recall is therefore not a 100%
failure measurement. Phase 1.4 repaired the split contract, but a corrected
model has not yet been trained. These failure rates are diagnostic context, not
grounds for optimizing the final weights.

## Recommendation for approval

Keep 50/18/12/20 for the corrected baseline. It preserves general coverage,
retains both automotive sources, and intentionally exposes the model to the
underrepresented fisheye domain. Revisit the mixture only after the corrected
validation run produces measurable per-source recall and false-proposal rates.
