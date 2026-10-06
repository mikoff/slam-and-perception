# Visual-inference code audit

Audit date: 2026-09-25

Scope: the tracked source, scripts, tests, configuration, documentation, and
checkpoint artifacts under projects/visual-inference. No project files were
deleted. Generated runs and checkpoints were treated as evidence, not as source
code.

## Executive result

The current quad detector is a real, connected implementation. Its production
path is:

    image [B,3,384,384]
      -> MobileNetV4Backbone (C3/C4/C5, strides 8/16/32)
      -> LiteFPN (P3/P4/P5, 96 channels)
      -> SharedQuadProposalHead
           -> quality logits [B,1,H,W]
           -> signed corner offsets [B,8,H,W]
      -> decode_quad_offsets
      -> validity filtering + canonicalization + polygon NMS
      -> up to K=100 quad proposals

The main training entry point is
[scripts/train_quad_proposals.py](../scripts/train_quad_proposals.py), which
constructs QuadProposalDetector and trains it through
[student_detector/quad_training.py](../student_detector/quad_training.py).
The cloud dispatcher also selects this script for geometry=quad
([scripts/cloud/run_training.py:253](../scripts/cloud/run_training.py:253)).

## Cleanup status

The low-risk cleanup from this audit has been applied:

- stale references to deleted Phase-2 scripts were removed or replaced with
  maintained quad tests and diagnostics;
- the broken cloud-training link was removed;
- unused quad compatibility aliases and the redundant polygon-NMS wrapper were
  removed;
- the canonical quad model, output, decoder, and dense-decoder API is exported
  from student_detector;
- rectification was retained as an explicitly optional, test-backed downstream
  utility because its future use is not decided.

## Checkpoints to keep

These are the important weights for quad detection. Keep all of them; the first
two are the useful inference pair and last.pt is needed for deterministic
training continuation.

| Role | Checkpoint | State | Step | SHA-256 | Use |
| --- | --- | --- | ---: | --- | --- |
| Selected primary | artifacts/phase3/checkpoints/best_ema.pt | ema_model | 83,684 | deee0ce3919ffdc0f6c68d675896efe55831327a3c22cd605fc8b71264f3c71f | Use for quad proposals. Apply score >=0.20, polygon NMS IoU 0.78, maximum 100. |
| Phase 4 control | artifacts/phase4/local_quad_p3_seed42_step20000_v1/best_ema.pt | ema_model | 19,995 | c9b00311f8758072299ec6437a043091e902b818d8481bf71b8e4de239183c2b | A/B control. Apply score >=0.20, polygon NMS IoU 0.75, maximum 100. |
| Resume-only | artifacts/phase4/local_quad_p3_seed42_step20000_v1/last.pt | full state | 19,995 | 5e4ad13d32c043d358ae4d3fc0d7f2d7c066eb39e78c2a71440df07fccd20cf3 | Resume training; do not use as the selected inference artifact. |

The selection is owner-approved in
[correction_phase4_quad_selection_v2.json](../configs/benchmarks/correction_phase4_quad_selection_v2.json)
and summarized in
[correction_phase_4_closeout.md](correction_phase_4_closeout.md).
The checkpoint's embedded config contains the model's candidate decoder
settings; the score threshold and final operating points are an external
selection policy. Calling QuadInferenceDecoder directly therefore returns ranked
candidates, not necessarily only candidates above the production score
threshold. Apply the threshold in the proposal contract or equivalent
post-processing path.

The pilot, step-10k, step-15k, Phase 2, and older phase3/open_world quad
checkpoints are historical experiment evidence. Retain them for reproducibility
until the corresponding reports are intentionally archived, but do not choose
them for a new quad detector integration.

## Main NN checkpoints to inspect

If by “checkpoints” you mean internal model stages rather than weight files,
these are the useful inspection points:

| Stage | Code | What it tells you |
| --- | --- | --- |
| C3/C4/C5 | [backbone.py:53](../student_detector/backbone.py:53) | Backbone features at strides 8/16/32. C3 preserves localization detail; C5 supplies context. |
| P3/P4/P5 | [neck.py:28](../student_detector/neck.py:28) | Fused features consumed by the quad head. These are the primary feature maps to visualize. |
| Quality logits | [head.py:86](../student_detector/head.py:86) | Per-location proposal quality before sigmoid. |
| Corner offsets | [head.py:112](../student_detector/head.py:112) | Eight signed coordinates for four corners relative to each grid point. |
| Dense decoded quads | [quad_decoder.py:53](../student_detector/quad_decoder.py:53) | All grid-location quads before ranking, validity filtering, and NMS. Best for diagnosing localization failures. |
| Final proposals | [quad_decoder.py:98](../student_detector/quad_decoder.py:98) | Valid, canonicalized, polygon-NMS-filtered proposals with pyramid-level and location identity retained. |
| Assigned training locations | [quad_assigner.py:35](../student_detector/quad_assigner.py:35) | Whether each ground-truth quad is representable by the P3-P5 grid and which locations train it. |
| Quad losses | [quad_losses.py:49](../student_detector/quad_losses.py:49) | Quality, cyclic/reversed corner, optional GWD, and validity terms. |

For a practical debugging sequence, inspect P3-P5 first, then quality maps,
then dense decoded quads, and only then final NMS output. This separates feature
failure, ranking failure, localization failure, invalid geometry, and suppression
failure.

## High-confidence unused or obsolete items

### 1. Resolved stale Phase-2 documentation references

The deleted Phase-2 scripts are no longer referenced by active documentation.
The Phase-2 document is marked historical, its obsolete commands are removed,
and current quad diagnostics are linked instead.

### 2. Resolved broken cloud documentation link

The README now points only to the active cloud training runbook.

### 3. Rectification module is test-only

student_detector/rectification.py has no production import; its only repository
consumer is tests/student_detector/test_rectification.py. It is a valid
homography/patch utility, but it is not part of quad detection, training,
evaluation, or deployment. It is retained as an explicitly labelled optional
post-processing module until the downstream crop/rectification decision is made.

### 4. Resolved unused compatibility symbols

The unused QuadStudentDetector, QuadDetector, InferenceQuadDecoder, and
class_agnostic_polygon_nms symbols were removed. Repository callers now use the
canonical QuadProposalDetector, QuadInferenceDecoder, and polygon_nms names.

## Old or scope-specific code that is not currently dead

These areas are not on the selected quad-inference path, but they still have
tests, scripts, or benchmark evidence. They should be archived only after the
corresponding project decision is made.

| Area | Code | Current status |
| --- | --- | --- |
| HBB detector | StudentDetector, assigner.py, targets.py, losses.py, decoder.py, evaluation.py, training.py, and scripts/train_phase3.py | Still used for HBB baselines, controls, initialization, comparisons, and tests. Remove only if HBB is formally retired. |
| P2 candidate | P2LiteFPN, StubP2Backbone, the P2 branch in StudentDetector, and scripts/train_phase3.py --p2 | Experiment/ablation path. Not used by the selected P3-P5 quad checkpoints. Keep for the small-object decision; otherwise archive together. |
| AttnRes candidate | AttnResLiteFPN, configs/phase3_attnres*.yaml, and benchmark tooling | Valid A/B architecture, but the selected checkpoint architecture is mobilenetv4_conv_medium_lite_fpn96_quad_dqco. Archive after the architecture comparison is no longer needed. |
| Proposal audits | proposal_contract.py, proposal_evaluation.py, proposal_utility.py, proposal_diversity.py, suppression.py, and the related scripts | Active evaluation and selection infrastructure. It is not training code, but it is required to reproduce the Phase 4 selection decision. |
| Cloud/data pipeline | dataset-pipeline/ and scripts/cloud/ | Active operational code. The nested project must be tested in its own environment. |
| Generated artifacts | artifacts/, runs/, wandb/, caches, and visual_inference.egg-info/ | Not source code. Keep selected evidence/checkpoints; archive or regenerate the rest according to retention policy rather than deleting individual files ad hoc. |

The many standalone audit/benchmark/visualization scripts with no textual caller
are CLI entry points, not necessarily dead functions. Their lack of imports is
expected because they are invoked manually or by CI/cloud workflows. A future
cleanup should add a small script registry and an “active / historical” label
instead of deleting them solely from import counts.

## API and documentation cleanup recommendations

1. Completed: README.md and the maintained inspection guide now make the
   quad path, selected checkpoints, and P3-P5 flow primary.
2. Completed: the Phase-2 inspection document is marked historical and all
   references to deleted scripts were removed.
3. Decide whether rectification.py is a planned downstream capability. It is
   currently retained as an optional, test-backed utility.
4. Export the canonical quad public API from student_detector/__init__.py.
   This cleanup now exposes the model, output, decoder, and dense-decoder entry
   points from the package root.

5. Keep the selected checkpoint and the Phase 4 control under a named retention
   policy. Do not use last.pt for ordinary inference.

## Verification performed

- Ruff unused-name/import/error checks: passed.
- Quad-focused tests: 54 passed.
- Dataset-pipeline tests: passed in the nested project environment.
- Main project tests: 298 passed; one spawn-worker test timed out because this
  restricted environment cannot start PyTorch's torch_shm_manager (Operation
  not permitted), not because of a model assertion failure.
- Both retained EMA checkpoints loaded strictly into QuadProposalDetector and
  produced P3/P4/P5 maps (48x48, 24x24, 12x12) and decoded 100 quads on a
  384x384 smoke input.
- Cloud configuration/policy verification: passed.

