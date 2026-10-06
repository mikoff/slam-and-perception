# Correction Phase 3 closeout audit

Status: Phase 3 execution complete. The frozen v1 gate returned no-go; the owner
approved a conditional v2 null-experiment path on 2026-09-10.

## Selected result

Phase 3 retains the normalized, non-mined HBB P3–P5 baseline. The selected loss
uses per-image/state reduction with objectness `1.0`, CIoU `2.0`, LTRB `0.5`,
and intentionally disabled centerness. Quad P3–P5 remains the unchanged control.
The owner-approved operating points are HBB NMS/score `0.70/0.30` and quad
`0.80/0.20`, with pre-NMS K=300 and final K=100.

The retained step-200 HBB checkpoint has AR100 14.783%, R100@0.50 40.693%,
R100@0.75 7.442%, small-object R100@0.50 32.238%, and thin-object R100@0.50
6.653% on the fixed 800-image validation subset. These values are the comparison
baseline, not evidence of improvement over a matched alternative.

## Phase summary

| Step | Result | Decision |
|---|---|---|
| 3.1 baseline reuse | Historical run lacked sufficient provenance and grouped diagnostics; a contract-bound 400-step replacement completed | Use replacement evidence |
| 3.2 loss normalization | Per-image/state reductions pass duplication, microbatch, empty-state, and reconstruction tests | Keep normalized objective |
| 3.3 gradient influence | Localization dominates every domain/size slice; quality gradients are mostly weakly aligned | Measure before changing weights |
| 3.4 coefficient calibration | Objectness 1.5 increased quality influence but lost 0.226 pp AR100 and failed general/small/16–32 px gates | Reject; retain objectness 1.0 |
| 3.5 selection contract | Frozen AR100-led policy and protected recall/false-proposal gates verified | Use for later candidates |
| 3.6 hard-negative mining | 40 conservative selectors were tested with exact exposure; candidate lost 0.174 pp AR100 and failed small/thin/trusted-background/unmatched gates | Reject mined candidate |
| 3.7 suppression calibration | Matched suppression, false-proposal, and per-domain reliability sweep completed | Approve HBB .70/.30; quad .80/.20 |

## Deliverable audit

| Required deliverable | Status | Evidence |
|---|---|---|
| Corrected batch-invariant loss reductions | pass | Phase 3.2 invariance and reconstruction tests |
| Component/shared-gradient diagnostics | pass | Phase 3.3 FPN and C5 reports at steps 100/200 |
| Reproducible coefficient calibration | pass | Phase 3.4 fixed cohorts, hashes, and matched candidate |
| Narrow loss ablation | pass | Objectness 1.5 comparison completed and rejected |
| Optional reviewed hard-negative supplement | pass | Versioned 40-selector supplement and one controlled comparison |
| Calibrated score/NMS points | pass | Phase 3.7 owner-approved matched sweep |

## Frozen stop/go gate

| Requirement | Status | Audit finding |
|---|---|---|
| Normalization tests pass across microbatch partitions | pass | Full/equal microbatch updates match within tolerance |
| No component is silently inactive | pass | Counts and effective weights are logged; centerness and weak background are intentionally zero |
| Chosen coefficients have measured gradient behavior | pass | Objectness/CIoU/LTRB coefficients are measured at FPN and C5 by slice |
| Selected loss improves the predeclared validation objective | **not met** | Neither objectness 1.5 nor mined-focus training passed; the control baseline cannot demonstrate improvement over itself |
| Critical-domain false proposals do not regress | pass for retained baseline | Regressive candidates were rejected; the selected control preserves its own frozen rates |
| Configuration and calibration evidence are versioned | **partial** | Model/data/config/report evidence is hashed, but Phase 2/3 files are uncommitted, artifacts are gitignored, and dirty run contracts do not hash/embed their patch |

The overall gate is **no-go** because every mandatory condition must pass. Four
conditions pass, objective improvement is not met, and evidence versioning is
partial. This is a governance result, not a model-quality failure: Phase 3
successfully prevented two unsupported changes from reaching full retraining,
but it did not produce the improvement required by the current Phase 4 entry
rule.

## Evidence integrity

The recorded local SHA-256 values for the Phase 3.1 replacement checkpoint/contract,
Phase 3.3 checkpoint and gradient reports, both selection decisions, the final
suppression sweep, and the selection/calibration policies were rehashed during
this audit and match their documentation. Commit preparation passes 253
detector/project tests and 79 dataset-pipeline tests; Ruff, formatting, YAML
parsing, and diff-integrity checks pass.

The training contracts record base commit
`79a9575e178bbdb7346774a0afb98e87e6859aab`, `dirty=true`, and a filename/status
snapshot, but not a hash or copy of the dirty patch. Exact byte-level training-
source reconstruction is therefore unavailable. The owner-approved v2 null-
result decision accepts this limitation; Phase 4 must launch from a clean commit.

Key immutable identities:

- selected HBB step-200 checkpoint:
  `3bb70fcbc3e81ee5cd2198d2626f570f0bb6a4b3e4e10cb82fd61497958f8063`;
- selection policy:
  `23186efcbcf9cf1387ab4877adc76846d828a1f39a0dd573d209535eb6e2beda`;
- suppression sweep:
  `babb5bbb57da4f4d75c578b3f72029ec5339881e79fe42d6862d73e771530c11`;
- approved calibration contract:
  `c8304d6938d88970e1766eed8b066f418dab34ed5e589415aa41e4d47071eb9f`.
- conditional v2 entry gate:
  `4565804abc19e8e621b6152f151ca5770f5962647849c9ecb911c07a84666be8`.

## Required owner decision

The owner accepted the null experimental result and authorized a versioned v2
entry rule. It does not claim objective improvement. Phase 4 planning is now
allowed, while contract execution and preflight remain conditional on committing
the Phase 2/3 source/config/report set and recording durable locations for
gitignored artifacts. Cloud pilot and full retraining remain separate launch
approvals.
