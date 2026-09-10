# Proposal detector roadmap

The current milestone ends with a retrained, evaluated class-agnostic detector
producing HBB or quadrilateral proposals. Pause after reviewing those outputs.
SigLIP integration, embeddings, distillation, downstream classification/retrieval
evaluation, and final geometry selection based on semantic crop utility are deferred.

## How the plans fit together

The original project phases and the correction work packages use different
numbering. Existing filenames and experiment IDs retain their original meaning.

| Original project document | Role | Correction-plan work |
|---|---|---|
| Dataset pipeline documentation | Prepare supervised data | Correction Phase 1: policy, regeneration, audit, publication and clean-worker checks |
| [phase2.md](phase2.md) | Detector architecture implementation reference | Correction Phase 2: extend and compare proposal architectures |
| [phase3.md](phase3.md) | Proposal training and evaluation | Correction Phases 3–4: loss validation, controlled retraining and proposal acceptance |
| [phase3_bounded_benchmark_v1.md](phase3_bounded_benchmark_v1.md) | Historical bounded experiment and reproduction record | Evidence to retain; not the corrected-data comparison or an acceptance result for it |

The detailed execution checklist is the
[correction and retraining plan](../../../.specs/proposal-detector-correction-and-retraining-plan.md).
Use “correction Phase 2” when referring to its architecture work package.
Do not interpret it as replacing the implemented original project Phase 2.

## Current execution sequence

1. Local correction Phase 1 loader gate completed on 2026-09-05: both readers
   passed train/validation production-index checks in parent and spawned-worker
   processes, with peak RSS below 1 GiB. Remote archive download/hash checks and
   clean-worker portability remain deferred to the first normal cloud staging.
2. The seed-42, 400-update matched screen is complete for HBB P3–P5, HBB P2–P5,
   and quad P3–P5. The common 800-image exact-polygon evaluation supports HBB
   P3–P5 as the provisional primary, quad P3–P5 as the retained control, and no
   quad-P2 run. See the [Phase 2.1 report](correction_phase_2_1_matched_architecture_comparison.md).
3. The owner accepted the common proposal contract: K=100, stable dense-location IDs,
   four-point HBB/quad geometry, and source-coordinate inversion. The
   [Phase 2.2 contract and audit](correction_phase_2_2_proposal_contract.md) are
   the downstream interface.
4. The corrected-data [Phase 2.3 suppression sweep](correction_phase_2_3_suppression_calibration.md)
   produced owner-approved HBB NMS/score `0.70/0.30` and quad `0.80/0.20`.
5. The [Phase 2.4 utility evaluation](correction_phase_2_4_proposal_utility.md)
   retains HBB as primary and quad as control; the owner accepted that decision.
6. The [Phase 2.5 deployment gate](correction_phase_2_5_deployment_feasibility.md)
   passes FP32 PT2 export/reload parity for both candidates. Physical RPi and
   INT8/ExecuTorch measurements remain deferred release evidence.
7. The [Phase 3.1 reuse audit](correction_phase_3_1_baseline_reuse_audit.md)
   found missing HBB run provenance and domain/state loss logs. The replacement
   contract-bound HBB baseline completed at 400 steps with verified diagnostics.
8. [Phase 3.2](correction_phase_3_2_loss_normalization.md) now reduces the HBB
   loss per image and passes duplication, microbatch, empty-state, production,
   and full-suite gates; the subsequent normalized calibration run completed.
9. [Phase 3.3](correction_phase_3_3_gradient_influence.md) instrumentation and
   normalized-run step-100/step-200 calibration pass. Localization dominates
   every measured slice; the subsequent Phase 3.4 search is complete.
10. [Phase 3.4](correction_phase_3_4_coefficient_calibration.md) rejects the
   matched objectness-1.5 candidate after production-policy utility regresses.
   Retain weight 1.0 and skip the upper 2.0 control.
11. [Phase 3.5](correction_phase_3_5_selection_contract.md) freezes the
   owner-approved AR100-led pass/fail/inconclusive policy with recall and
   false-proposal guardrails.
12. [Phase 3.6](correction_phase_3_6_hard_negative_mining.md) produced a
   train-only capped candidate set and a 40-selector conservative supplement.
   The controlled 200-step comparison used one same-source focus image per
   optimizer window and failed the frozen selection gates. Retain the normalized
   non-mined HBB baseline; do not calibrate or promote the mined candidate.
13. [Phase 3.7](correction_phase_3_7_suppression_calibration.md) completed a
   matched 800-image suppression, false-proposal, and per-domain reliability
   sweep. The owner approved retaining HBB `.70/.30` and quad `.80/.20`.
14. The [Phase 3 closeout audit](correction_phase_3_closeout_audit.md) marks all
   deliverables complete but the Phase 4 entry gate no-go: no tested candidate
   improves the frozen objective. The owner accepted this null result through a
   conditional v2 gate; commit and artifact-reference checks remain.
15. Follow the supervised [Phase 4 execution plan](correction_phase_4_execution_plan.md):
   commit/evidence boundary, frozen contract, local preflight, separately
   approved cloud pilot/full run, selection, proposal audit, and packaging.
16. Review fixed-budget recall, size/domain slices, missed objects, background
   false proposals and duplicates; verify coordinate/export parity and package
   reproducible checkpoint/config/data references. Stop for owner review.

The production proposal budget is K=100. Acceptable recall, false-proposal, and
duplicate limits must be agreed before model selection. No SigLIP model is
needed to reach this stopping point. Retain proposal scores, geometry and source
coordinates so a later semantic stage can consume frozen outputs.

## Deferred gates

Semantic crop generation and SigLIP quality tolerances belong to a later milestone.
Physical RPi production acceptance and rollout also remain release gates, not a
claim made by the proposal-training milestone. Early export checks and available
hardware measurements still inform architecture feasibility; desktop timing is
not proof of target-device readiness.

Historical benchmark numbers, hashes and schedules remain historical. New
corrected-data experiments need their own versioned contracts and results. The
old HBB/quad schedule mismatch must not be presented as a matched comparison.
