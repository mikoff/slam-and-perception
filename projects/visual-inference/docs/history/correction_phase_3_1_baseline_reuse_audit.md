# Correction Phase 3.1 baseline reuse audit

Status: complete. Historical reuse was rejected; the contract-bound replacement
completed at 400 optimizer steps and the owner approved subsequent Phase 3 work.

## Candidate audited

The candidate is the corrected-data HBB P3–P5 seed-42 run at 400 optimizer
steps, checkpoint
`artifacts/phase2/short_v1/hbb_p3/last.pt`. Its SHA-256 is
`aeb22da9ee5950c892a8dff0f58e54942883e55d7286896a7e6fefc8c39d1171`.
The audit reads existing checkpoint, index metadata, logs, and Phase 2 reports;
it does not retrain or modify the checkpoint.

## Gate results

| Requirement | Result | Evidence |
|---|---|---|
| Selected architecture | pass | HBB P3–P5 LiteFPN architecture ID matches the Phase 2 primary |
| Corrected immutable data | pass | train/validation signatures `dd3111…1457` / `68f632…cbf` match the prebuilt indexes and Phase 1 contract |
| Fixed budget | pass | checkpoint is exactly global step 400 |
| Seed and configured initialization | partial | checkpoint records seed 42 and `pretrained_backbone=true`, but no initial-state value hash |
| Current compatible loss | pass | QFL objectness + 2.0 CIoU + 0.5 LTRB + disabled centerness |
| Raw component logs | pass | 21 epoch rows contain objectness, CIoU, LTRB, centerness, and total |
| Weighted component reconstruction | partial | totals reconstruct within `5.99e-7`, but weighted values were not logged explicitly |
| Per-domain loss components | fail | no domain-qualified loss keys exist |
| Per-state quality components | fail | positive/trusted/weak/ignore contributions were not logged separately |
| Full run provenance | fail | the checkpoint's `run_contract` is `{}` |
| Frozen proposal evaluation | pass | Phase 2.4 evaluates the EMA state with approved HBB NMS/score `0.70/0.30` |
| Checkpoint load/export integrity | pass | raw and EMA weights load strictly; FP32 PT2 export/reload parity is exact |

The config is content-addressed as
`9238bddc48614df185ac3750fa06f4924905d264745f30c37010464e4448a12f`.
The training log spans steps 20 through 400; batch-level progress and checkpoint
state establish that the bounded run completed, including its interrupted final
epoch. This makes the result useful historical baseline evidence.

## Why strict reuse fails

Phase 3 compares loss formulations. A baseline without a captured source,
environment, manifest, model-signature, and initial-state contract cannot prove
that later differences come only from the loss. Aggregate raw components also
cannot answer whether quality loss is driven by positives, trusted background,
or a particular domain. Reconstructing weighted totals proves the arithmetic,
but does not recover those missing observations.

Post-hoc diagnostics on the final EMA checkpoint would describe that checkpoint,
not the training trajectory or its exact starting state. They are still useful,
but are not equivalent to the Phase 3.1 training baseline required for controlled
Phase 3.2 comparisons.

## Replacement baseline contract

Run one HBB-only 400-step replacement with the already approved P3–P5
architecture, corrected dataset, seed 42, current loss, schedule, and validation
cohort. Do not retrain the quad control. The implementation now:

1. capture the resolved run contract and initial raw model-state SHA-256;
2. add raw and coefficient-weighted component logging;
3. split quality loss into positive, trusted-background, weak, and ignored counts
   and contributions;
4. aggregate the same components by source/domain without changing loss math;
5. retain the approved HBB evaluation settings only for final proposal metrics;
6. compare the replacement result with the historical 400-step HBB evidence.

This is the smallest rigorous remedy. It changes instrumentation and provenance,
not the baseline objective. The versioned launch config is
`configs/benchmarks/correction_phase3_baseline_v1.yaml`. It matches the
historical HBB contract in all data, augmentation, architecture, loss,
inference, and optimization settings. Only its output directory and operational
100-step checkpoint interval differ.

## Instrumentation verification

An eight-image deterministic batch selected with the production source weights
passed through the corrected dataset, EMA HBB checkpoint, target builder, and
loss on 2026-09-06. Its 16,284 valid feature points partitioned exactly into 359
positive, 3,869 trusted-background, 12,017 weak-background, and 39 ignored
points. Weighted components reconstructed total loss within `1.2e-7`; effective
state contributions reconstructed objectness exactly; every source and domain
aggregate reconciled with its overall component.

The complete `tests/student_detector` suite passes: 124 tests. Ruff also passes.

## Replacement result

The local RTX 3060 run completed cleanly at step 400. The best EMA checkpoint is
the final checkpoint. Values below use the same 800-image validation cohort;
deltas are replacement minus historical.

| Metric | Replacement | Historical | Delta |
|---|---:|---:|---:|
| Selection score | 0.402632 | 0.401683 | +0.000949 |
| Recall 100 @ 0.50 | 0.388326 | 0.386955 | +0.001370 |
| Recall 100 @ 0.75 | 0.135380 | 0.132502 | +0.002878 |
| AP @ 0.50 | 0.045002 | 0.042730 | +0.002272 |
| Median matched IoU | 0.379306 | 0.373847 | +0.005459 |
| Final training loss | 1.275440 | 1.285717 | -0.010277 |

The trajectories are numerically identical through step 99 and diverge slightly
after the replacement's additional step-100 checkpoint. The final proposal
metrics remain close and slightly favor the replacement; this is suitable as a
bounded GPU baseline, not evidence for a loss improvement.

All 21 epoch rows are finite. Across the full run, weighted components reconstruct
total loss within `6.56e-7`, state contributions reconstruct objectness within
`4.92e-8`, and source/domain groups reconstruct overall components within
`3.34e-7`.

Artifacts:

- `last.pt` SHA-256: `8af9fa38ded6975f116ce3e8b74bb1fd3206681b2571ce86b28b672070f182fd`
- `run_contract.json` SHA-256: `fba7b9e34abc259bb2c2b22c703fa603db75cf8066fe93f73a35bb003629d759`
- initial model state SHA-256: `7af6a0f21eb179b673d96290fcd41f6259f64efb512338d45a521208b8afa244`

Phase 3.1 is complete; the owner subsequently approved Phase 3.2.
