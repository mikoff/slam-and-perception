# Phase 1.5 small-object policy evidence

Status: owner-approved product boundary; architecture ablation pending.

Owner decision: objects and downstream raw crops must remain useful down to a
16px source short side. The coherent candidate is 384 input with P2 and a min-4
model-coordinate retention gate. This is a candidate, not yet a production
configuration: latency, memory, and false-positive ablations remain mandatory.

## Measurement contract

The audit streamed all 3,908,206 positive annotations in the current immutable
production proposal index. Each source quad was aspect-ratio letterboxed into
the current 384px model coordinates. The report distinguishes:

- annotation retention by the HBB and quad transform gates;
- exact P3/P4/P5 grid opportunity, meaning a cell center lies inside the quad;
- usable opportunity, meaning both retention and grid opportunity hold;
- the assigner's closest log-scale FPN reference before conflict resolution;
- source/model short side, polygon area, and aspect-ratio histograms;
- dataset, canonical-category, and dataset/category breakdowns;
- distance bands when attributes exist.

The current production index predates attribute preservation, so all distance
bands are explicitly `unavailable`. `proposal-manifest.v2` and SQLite schema v8
now preserve `attributes`/`attributes_json` for the regenerated audit.

## Overall comparison

| Policy simulation | Usable positives | Corpus opportunity | Dense locations vs 384/P3–P5 |
|---|---:|---:|---:|
| 384, current min-8 retained + P3 grid | 2,169,151 | 55.50% | 1.00× |
| 384, min-8 retained + P2 grid | 2,254,976 | 57.70% | 4.05× |
| 384, coherent min-4 retained + P2 grid | 3,143,738 | 80.44% | 4.05× |
| 512, min-8 retained + P3 grid | 2,630,468 | 67.31% | 1.78× |
| 640, min-8 retained + P3 grid | 2,941,321 | 75.26% | 2.78× |

Adding P2 without lowering the annotation gate gains only 2.20 percentage
points over current P3 opportunity. The coherent P2/min-4 policy has the largest
opportunity, but creates 9,216 P2 locations in addition to the current 3,024
P3–P5 locations. This increases dense head locations and negative candidates by
about 4.05× before measuring implementation-specific FPN cost.

For completeness, 2,208,850 annotations (56.52%) pass the current min-8 gate
and have a grid-center opportunity on at least one existing P3–P5 level. The
policy table uses P3 specifically because it is the finest current feature level
and therefore the relevant representability boundary for small objects.

## Per-source usable opportunity

| Source | Current 384/P3 | P2/min-4 | 512/P3 | 640/P3 |
|---|---:|---:|---:|---:|
| BDD100K | 45.43% | 77.95% | 59.82% | 69.90% |
| COCO | 82.37% | 94.43% | 89.43% | 92.73% |
| nuImages | 47.16% | 75.32% | 60.19% | 69.52% |
| WoodScape | 37.81% | 60.96% | 49.79% | 59.21% |

## Source short-side decision boundary

| Source short side | Objects | Current 384/P3 | P2/min-4 | 512/P3 | 640/P3 |
|---|---:|---:|---:|---:|---:|
| 16–24px | 655,147 | 26.4% | 96.3% | 46.5% | 80.8% |
| 24–32px | 435,082 | 50.3% | 100.0% | 95.1% | 100.0% |
| 32–48px | 521,478 | 94.9% | 100.0% | 100.0% | 100.0% |
| ≥48px | 1,104,070 | 100.0% | 100.0% | 100.0% | 100.0% |

The most affected high-volume canonical categories are car, traffic sign,
traffic light, pedestrian, traffic cone, barrier, and pole. Full category and
dataset/category histograms are available in the machine-readable report.

## Decision rubric

- If a useful downstream crop starts around 32 source pixels on its short side,
  keep 384/P3: the current architecture already provides about 95–100%
  opportunity above that boundary.
- If 24–32px objects must be reliable, 512/P3 reaches 95.1% in that band with a
  smaller dense-location multiplier than P2 or 640.
- If 16–24px objects are required, P2/min-4 provides the strongest opportunity;
  it requires a dedicated latency, memory, and false-positive ablation.
- Tiling/ROI is not simulated because its proposal merging and runtime contract
  differ materially from this always-on single-frame detector.

Artifacts:

- `data/visual-inference-datasets/reports/phase_1_5_small_object_policy/index.html`
- `small_object_policy.json` for exact summaries;
- `histograms.csv` for every scope, metric, and bin.
