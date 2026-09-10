# Correction Phase 2.3 suppression calibration

The machine report is produced by `scripts/sweep_proposal_suppression.py`.

Status: complete; thresholds owner-approved on 2026-09-06.

## Contract and definitions

The sweep uses the Phase 2.1 EMA checkpoints and the same deterministic,
source-balanced 800-image validation subset: 400 COCO, 160 WoodScape, 144
nuImages, and 96 BDD100K images. Each head produces at most 300 valid pre-NMS
candidates. The sweep evaluates NMS thresholds `0.50–0.90` and score thresholds
`0.00, 0.20, 0.30, 0.40, 0.50, 0.60` at final K=10/50/100.

HBB uses the optimized rectangle-NMS production path. Quad uses exact polygon
NMS. The sweep caches each candidate overlap matrix and replays the same stable
greedy algorithm for all settings; fixture parity tests match both production
implementations. Equal scores are ordered stably by dense index. HBB now rejects
non-finite and zero-area boxes after clipping, matching quad validity handling.

A duplicate is a final proposal whose best ground-truth match has IoU at least
0.50 and was already covered by a higher-ranked proposal. `Duplicate/all` uses
all emitted proposals as denominator. `Pair IoU≥.50` measures the fraction of
unique final proposal pairs overlapping by at least 0.50.

## Main results

All quality and redundancy values are percentages. `Props` is proposals/image.

| Geometry | Setting | AR10 | AR50 | AR100 | R10@.50 | R50@.50 | R100@.50 | R100@.75 | Props | Duplicate/all | Pair IoU≥.50 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| HBB | current `.90 / .00` | 7.42 | 14.06 | 17.15 | 18.40 | 33.19 | 40.29 | 12.42 | 100.0 | 17.58 | 7.98 |
| HBB | proposed `.70 / .30` | **8.85** | **14.79** | **17.47** | **23.82** | **38.84** | **45.21** | 10.04 | 95.0 | **7.33** | **2.81** |
| Quad | current `.90 / .00` | 7.95 | 13.72 | **16.56** | 20.15 | 32.93 | 40.02 | **11.42** | 100.0 | 14.94 | 6.69 |
| Quad | proposed `.80 / .20` | **8.56** | **14.04** | 16.55 | **22.14** | **34.97** | **41.00** | 10.98 | 99.9 | **10.49** | **4.46** |

For HBB, the proposed setting gains 4.92 points of R100/0.50 and 0.32 points of
AR100 while cutting duplicate/all by 10.25 points. It loses 2.37 points of
R100/0.75. NMS 0.60 raises R100/0.50 another 0.25 points but drops AR100 by
1.44 points and R100/0.75 by 2.87 more points, so it is past the balanced knee.

For quad, NMS 0.80 preserves AR100 within 0.01 points, gains 0.98 points of
R100/0.50, and reduces duplicate/all by 4.46 points. Quad NMS 0.70 removes more
duplicates but loses 0.40 points of AR100 and 2.75 points of R100/0.75.

The HBB score threshold 0.30 reduces output from 100.0 to 95.0 proposals/image
with only 0.11 points less R100/0.50 than score 0.00 at the same NMS. Quad score
0.30 removes 14.7 proposals/image but loses 0.80 points of R100/0.50, so the
control keeps the conservative 0.20 threshold. Phase 2.4 background-false-
proposal measurement may still revise these score thresholds.

## Runtime and artifacts

The full sweep ran on CPU because the NVIDIA driver was unavailable. CPU timing
is not deployment evidence. The earlier Phase 2.1 RTX 3060 measurement remains
the relevant experiment-planning result: native quad post-processing was about
7.6 times HBB decode/NMS at the then-current 0.90 setting.

The machine-readable sweep is
`artifacts/phase2/short_v1/suppression_sweep_ema.json`, SHA-256
`f14a667a69bd1b96aeaecf8a38e3c1d4b690840d02aef67f77f384073249c05f`.
It contains all 60 geometry/threshold results, checkpoint and validation hashes,
source counts, invalid-candidate rates, and timing components. HBB rejected no
candidates; quad rejected 3.58% of its top-300 candidates as invalid.

The proposed-setting visual audit is
`artifacts/phase2/short_v1/suppression_audit_candidate/index.html`, with JSON
SHA-256 `e250bf33526994a4723cfd1295678ff4d58fd0e95506ef4cc6d57b7557827eb6`.
It uses the same green-ground-truth/amber-proposal convention as Phase 2.2.

## Approved decision

- HBB primary: NMS `0.70`, score `0.30`, pre-NMS 300, final K=100.
- Quad control: NMS `0.80`, score `0.20`, pre-NMS 300, final K=100.
- The versioned Phase 2.4 policy records these geometry-specific settings; the
  shared training config remains geometry-neutral.
- Carry these settings into Phase 2.4 background false-proposal and domain
  utility measurement.
