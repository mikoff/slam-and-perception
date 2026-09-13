# Correction Phase 4 execution plan

Status: Phase 4.0 through the
[Phase 4.2a promotion contract](correction_phase_4_2a_promotion_contract.md) are
complete. Phase 4.3 is waiting for a separate owner launch approval.

## Goal and stopping point

Train and package one final HBB P3–P5 proposal-detector candidate under the
corrected dataset, normalized loss, and approved proposal policy. Compare it
against the frozen Phase 3 baseline and retained quad control. Stop after the
owner accepts or rejects the proposal outputs and export package.

SigLIP integration, semantic crop utility, INT8 work, physical RPi acceptance,
and production rollout remain deferred. Quad is an evaluation control and will
not be retrained without a new proposal-level justification.

## Frozen inputs

| Contract item | Value |
|---|---|
| Dataset | `phase3-production-bg-policy-v2-2026-08-29` |
| Primary geometry | HBB P3–P5 LiteFPN |
| Control | Existing quad P3–P5 EMA checkpoint |
| Input | FP32 `[B, 3, 384, 384]`; FP16 training on CUDA |
| Source short-side policy | 16 px |
| Loss | per-image/state QFL 1.0 + CIoU 2.0 + LTRB 0.5; centerness 0 |
| HBB inference | pre-NMS 300, NMS 0.70, score 0.30, final K=100 |
| Quad inference | pre-NMS 300, NMS 0.80, score 0.20, final K=100 |
| Selection baseline | Phase 3 step-200 EMA, SHA-256 `3bb70f…f8063` |
| Selection rule | frozen `proposal-selection-policy.v1` |

## Supervised execution steps

### 4.0 Commit and evidence boundary

Complete. The evidence set is immutable and remotely verified; the manifest was
uploaded last.

- Use the verified [Phases 2–3 commit review](correction_phase_2_3_commit_review.md).
- Review and commit the complete Phase 2/3 source, tests, configs, and reports.
- Record the clean source commit and `uv.lock` hash.
- Build a small manifest for required gitignored checkpoints/reports, including
  local path, size, SHA-256, and durable S3 key.
- Verify remote metadata without downloading already verified local payloads.
- Do not launch from a dirty tree; the Phase 3 status-only dirty record is not an
  acceptable substitute for an exact Phase 4 source identity.

Stop for owner confirmation before Phase 4.1.

### 4.1 Freeze the full-run contract

Approved. The machine-readable recipe and contract fix the budget, cadence,
seed policy, Packet path, and ceilings.

Create one immutable config and run contract. Resolve the remaining decisions:

- initialization: recommended weights-only initialization from the selected
  normalized HBB checkpoint, resetting optimizer, scheduler, scaler, EMA, step,
  sampler, and run identity;
- exact optimizer-step budget, warm-up, validation/checkpoint cadence, and
  early-stop patience;
- primary seed and whether a second seed is required only for a near-boundary
  result;
- cloud GPU type, cost/runtime ceiling, W&B identity, and immutable S3 prefix.

The contract must reject a dirty source tree and incompatible full-state resume.
Stop for owner approval of initialization, budget, cost, and stopping rules.

### 4.2 Run local preflight

Complete. All required gates passed; details and measured evidence are recorded
in the [local preflight report](correction_phase_4_2_local_preflight.md). No
Packet host or other cloud resource was launched.

Run, in order:

1. static schema/hash/config checks;
2. source-balanced loader smoke covering positive, ignore, trusted, and weak
   states;
3. tiny mixed-domain overfit and decoded-target recovery;
4. equivalent-effective-batch update comparison;
5. two-step FP16/EMA/checkpoint/validation smoke on the local RTX 3060;
6. safe-resume continuity and incompatible-contract rejection.

The AI reviews target/proposal overlays using the conservative default strategy
and reports exceptions; the owner is not required to inspect every image. Stop
on any failed check and ask for supervision before remediation or cloud work.

### 4.2a Freeze the HBB-versus-quad promotion contract

Complete. Quad is the incumbent and HBB is the challenger. HBB must pass the
existing comparison against the retained HBB baseline and the versioned
[quad-incumbent promotion policy](correction_phase_4_2a_promotion_contract.md).
Deployment speed is report-only and cannot override a quality failure. The
current HBB correctly fails this new gate, so no production switch has occurred.

### 4.3 Run a bounded cloud pilot

The pilot exercises the HBB challenger but does not replace the quad incumbent.
Promotion later requires both the HBB-baseline policy and the Phase 4 quad-
incumbent policy to pass. GitHub `mode=pilot` is machine-capped at 2,000
successful optimizer steps and 500-step checkpoints; `production` is not used.

- Stage or verify the immutable dataset and initialization artifact.
- Run enough steps to cross warm-up and at least one validation/checkpoint event.
- Interrupt once at a safe checkpoint, resume, and verify continuity.
- Verify W&B telemetry, S3 checkpoint/manifest hashes, throughput, VRAM, and
  projected full-run cost.

This requires a separate owner launch approval. The pilot does not authorize the
full run. Present a pass/fail table and revised cost/runtime estimate.

### 4.4 Run full HBB retraining

- Launch exactly the approved contract; do not retrain quad or reopen Phase 3
  loss/architecture search.
- Monitor source/domain mixture, state counts, raw/weighted components, learning
  rate, gradient/scaler state, skipped steps, throughput, memory, raw/EMA
  validation, proposal count, duplicates, and false-proposal slices.
- Maintain latest resumable, previous resumable, best-primary, and
  best-constraint checkpoints with hashes.
- Intervene only for predefined OOM, non-finite, data corruption, checkpoint,
  validation-collapse, false-proposal, runtime, or cost conditions.

This requires a separate full-run approval. Report progress without launching
unplanned replacement runs.

### 4.5 Select the checkpoint

- Evaluate every scheduled validation checkpoint under the frozen selection
  policy against the Phase 3 baseline.
- Prefer EMA only when it clears the constraints; retain raw and EMA evidence.
- A candidate must improve AR100 by the frozen margin and pass every protected
  recall/domain/false/duplicate gate. Otherwise retain the Phase 3 baseline.
- Use a second seed only for a near-boundary/inconclusive result, not to rescue a
  clear failure.

Stop for owner confirmation of the selected or rejected checkpoint.

### 4.6 Final proposal audit

- Evaluate selected raw/EMA states on the full hash-bound validation set.
- Report aggregate, source/domain, size, aspect, 16 px, trusted-background,
  unmatched, duplicate, and ego-body metrics.
- Generate a balanced diagnostic bundle. The AI performs the default visual
  review and produces categorized counts/examples; owner review is limited to
  ambiguous exceptions and the final recommendation.
- Recheck HBB/quad coordinate, stable-ID, K=100, and transform-inversion parity.

SigLIP and semantic crop evaluation are explicitly outside this step.

### 4.7 Export, package, and stop

- Export the selected FP32 PT2 raw-output graph and verify eager/export parity.
- Package weights, raw/EMA identity, preprocessing, decoding/NMS thresholds,
  data/code/config hashes, validation and visual reports, runtime requirements,
  limitations, and rollback artifact.
- Upload the manifest last, verify remote size/hash metadata, and retain the
  Phase 3 baseline for rollback.

Stop after owner proposal-level acceptance. INT8, SigLIP, RPi measurement, and
deployment canary require a later plan and separate approval.

## Completion gate

Phase 4 is complete only when:

- a clean immutable training contract and durable artifact manifest exist;
- local preflight and cloud resume/checkpoint paths pass;
- the full run ends under the approved budget or a predefined stop condition;
- the chosen checkpoint passes the frozen proposal-selection policy, or the
  baseline is explicitly retained after a valid failure;
- full validation and AI-led visual audit are reviewed by the owner;
- export parity passes and both selected and rollback artifacts are recoverable.
