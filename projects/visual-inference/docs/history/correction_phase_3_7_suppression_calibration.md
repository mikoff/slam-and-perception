# Correction Phase 3.7 suppression calibration

Status: complete; operating points owner-approved on 2026-09-10.

## Contract

The retained normalized HBB EMA checkpoint and unchanged quad-control EMA
checkpoint were evaluated on the frozen, source-balanced 800-image validation
subset. Source counts are 400 COCO, 160 WoodScape, 144 nuImages, and 96 BDD100K.
The final proposal budget is K=100 from at most 300 pre-NMS candidates.

Phase 2 already eliminated the outer threshold ranges. This bounded sweep tests
NMS IoU `0.60, 0.70, 0.80` and score `0.20, 0.30, 0.40`. Each image is inferred
once per geometry; cached candidate overlaps replay all nine settings. In
addition to recall and duplicate metrics, the sweep measures object coverage,
trusted-background and unmatched false proposals, source/domain slices, and
fixed-width score-reliability bins.

## HBB results

All percentages are absolute; `Props` is proposals per image.

| NMS / score | AR100 | R100@.50 | R100@.75 | Small R@.50 | Thin R@.50 | Props | Trusted false | Unmatched |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| .60 / .20 | 13.813 | 41.381 | 5.376 | 34.272 | 6.653 | 94.15 | 2.925 | 55.288 |
| .60 / .30 | 13.803 | 41.343 | 5.376 | 34.194 | 6.653 | 90.26 | 3.001 | 53.923 |
| .70 / .20 | 14.783 | 40.693 | 7.442 | 32.238 | 6.653 | 99.73 | 2.199 | 48.561 |
| **.70 / .30** | **14.783** | **40.693** | **7.442** | **32.238** | **6.653** | **97.99** | **2.229** | **47.833** |
| .70 / .40 | 14.593 | 40.214 | 7.346 | 30.986 | 6.554 | 86.52 | 2.150 | 43.666 |
| .80 / .20 | 14.787 | 37.899 | 8.915 | 29.499 | 5.263 | 99.87 | 1.951 | 41.917 |
| .80 / .30 | 14.787 | 37.899 | 8.915 | 29.499 | 5.263 | 98.74 | 1.966 | 41.349 |

NMS 0.80 gains only 0.004 AR100 points over 0.70 but loses 2.79 points of
R100@.50, 2.74 small-object points, and 1.39 thin-object points. NMS 0.60 gains
0.69 R100@.50 points but loses 0.98 AR100 points and materially increases false
proposals. Score 0.40 reduces unmatched proposals but loses 0.48 R100@.50 and
1.25 small-object points. At NMS 0.70, score 0.30 preserves all measured recall
from 0.20 while emitting 1.74 fewer proposals per image and lowering unmatched
false proposals by 0.73 points.

## Score reliability and domains

Scores rank object-like proposals monotonically but are not calibrated object
probabilities. At HBB NMS 0.70, the aggregate IoU>=0.50 rate rises from 1.54% in
score bin 0.30–0.40 to 7.53% at 0.50–0.60, 27.05% at 0.70–0.80, and 43.35% at
0.80–0.90. The 0.70–0.80 object rate differs by domain: 41.93% automotive,
31.57% fisheye, and 22.78% general. A score therefore remains a ranking signal,
not a universal semantic probability.

WoodScape accounts for the fisheye trusted-background false rate. Raising the
global cutoff to 0.40 reduces that rate, but the accompanying protected-recall
loss does not justify the change. Domain-specific thresholds are not introduced
without a separate deployment contract.

## Quad control and approved decision

The corrected evaluator exactly reproduces the frozen Phase 3 utility report.
The quad control remains at NMS 0.80 / score 0.20: moving to score 0.30 loses
0.52 R100@.50 points and 1.10 small-object points for reduced false proposals.
Quad remains a control and is not promoted over HBB.

Owner-approved production-policy operating points:

- HBB primary: NMS `0.70`, score `0.30`, pre-NMS 300, final K=100.
- Quad control: NMS `0.80`, score `0.20`, pre-NMS 300, final K=100.

## Provenance

- HBB checkpoint SHA-256:
  `3bb70fcbc3e81ee5cd2198d2626f570f0bb6a4b3e4e10cb82fd61497958f8063`
- Quad checkpoint SHA-256:
  `cf4d1a7228fab007317b1a9d6f2c037ccc139fa16e1c223a7d2d3a44fc8c095d`
- Validation subset SHA-256:
  `e705233ada6e63cb58c259b610417a6226dd9cf158acc7737842ecc758e15806`
- Sweep report SHA-256:
  `babb5bbb57da4f4d75c578b3f72029ec5339881e79fe42d6862d73e771530c11`
- Approved calibration contract SHA-256:
  `c8304d6938d88970e1766eed8b066f418dab34ed5e589415aa41e4d47071eb9f`

The extended sweep implementation passes 148 detector tests. Phase 3.7 is
complete.
