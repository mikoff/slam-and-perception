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
2. Next, prepare and compare proposal architectures using matched short runs. Keep
   HBB and quad controls, and test P2/min-4 because the approved source short-side
   requirement is 16px. P2 is a candidate, not a measured winner. Quad P2 is
   optional and needs a proposal-level justification before extra runs.
3. Choose a working architecture for retraining using proposal metrics, resource
   cost and visual evidence. This is provisional for the eventual semantic system.
   Validate loss normalization and tune only within the approved bounded search.
4. Run controlled full retraining, calibrate score/NMS thresholds, and evaluate
   raw and EMA checkpoints on the corrected validation set.
5. Review fixed-budget recall, size/domain slices, missed objects, background
   false proposals and duplicates; verify coordinate/export parity and package
   reproducible checkpoint/config/data references. Stop for owner review.

The proposal budget K and acceptable recall/false-proposal/duplicate limits must
be agreed before model selection. No SigLIP model or crop-rate requirement is
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
