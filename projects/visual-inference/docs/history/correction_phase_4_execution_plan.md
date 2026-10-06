# Correction Phase 4 execution plan

Status: complete with baseline retained. The supervised local HBB and quad
pilots failed the frozen utility gates, so no cloud/full run or replacement
package is authorized. See the [Phase 4 closeout](correction_phase_4_closeout.md).

## Goal and stopping point

Test whether matched retraining can replace the retained quad P3–P5 proposal
detector under the corrected dataset and approved utility policy. A valid frozen
gate failure retains the incumbent and closes the branch without a new package.

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

### 4.3 Run bounded matched pilots

Complete locally after the requested Packet GPUs were unavailable. HBB and quad
used the same seed, production data, effective batch 128, 2,000-step budget,
schedule, augmentation, and 500-step checkpoint cadence. Both warm-started the
explicit EMA state from their approved geometry-specific checkpoint.

- Exact dataset, schema-8 index, source commit, initialization-state, and
  checkpoint hashes passed before CUDA setup.
- HBB and quad completed 2,000 successful steps and full raw/EMA validation.
- Quad resumed deterministically from step 2,000 and stopped at step 5,000 with
  optimizer, scheduler, scaler, EMA, sampler, and RNG continuity.
- Frozen 800-image utility and bounded suppression reports bind every decision.

HBB failed proposal-object-fraction and unmatched gates. Quad improved the
incumbent's AR but failed unmatched. The owner approved bounded suppression
calibration and a deterministic quad continuation to a 5,000-step stop.

### 4.4 Run full HBB retraining

Not launched. HBB's 2,000-step frozen result is a clear gate failure, and the
matched quad continuation worsened its protected metrics by step 5,000. The
predefined false-proposal stop condition therefore prohibits either unchanged
recipe from continuing to 20,000 steps.

### 4.5 Select the checkpoint

Complete. HBB 2k fails against both required references. Quad 2k fails unmatched
by +7.17 pp versus the incumbent; quad 5k fails proposal object fraction by
-2.39 pp and unmatched by +10.55 pp. These are clear failures, so seed 43 is not
authorized and the original quad EMA remains selected.

### 4.6 Final proposal audit

Closed by baseline retention. The incumbent already passed the Phase 2 utility,
diagnostic, coordinate, K=100, and export-parity audits. Phase 4 adds no new
production artifact requiring another visual/export acceptance cycle.

### 4.7 Export, package, and stop

No replacement export or upload is needed. Retain the previously evaluated quad
artifact and rollback evidence. The versioned closeout manifest binds the local
pilot checkpoints/reports and records the intermediate-checkpoint retention
limitation. Any later model change starts under a new contract.

## Completion gate

Phase 4 is complete because:

- a clean immutable training contract and durable artifact manifest exist;
- local preflight and deterministic resume/checkpoint paths pass; cloud training
  was not entered after the local stop condition fired;
- the bounded runs ended under the predefined false-proposal stop condition;
- the chosen checkpoint passes the frozen proposal-selection policy, or the
  baseline is explicitly retained after a valid failure;
- the retained incumbent's prior validation, visual audit, and export parity
  remain authoritative; no rejected candidate is presented as production-ready;
- the incumbent remains recoverable and all new evidence hashes are recorded.
