# Correction Phases 2–3 commit review

Status: commit candidate verified; no commit or staging was performed.

## Scope

The change set contains the corrected HBB/quad proposal contract, P2 screening,
deployment parity, per-image/state loss normalization, gradient diagnostics,
selection policy, one rejected coefficient candidate, one rejected mined-focus
candidate, final suppression/reliability calibration, and the conditional Phase
4 entry contract and execution plan.

Large experiment outputs remain under gitignored `artifacts/`. Versioned configs
and reports contain their SHA-256 identities. Unrelated repository-root files
`-C`, `1.html`, and `26-Week-Plan-FINAL-v4 (1).odt` are outside this commit.

## Verification

| Check | Result |
|---|---|
| Ruff lint, full visual-inference tree | pass |
| Ruff format, changed Python files only | pass; 12 changed files normalized |
| Detector/project tests excluding nested package | 253 passed |
| Dataset-pipeline tests in its own locked environment | 79 passed |
| Diff whitespace/conflict-marker check | pass |
| Phase 3 evidence rehash | seven key identities match |
| Phase 3 candidate decisions | objectness 1.5 fail; mined-focus fail |
| Architecture status | four required sections, 80 lines |

The repository-wide formatter would rewrite unrelated historical files, so only
the 27 Python files in this change were checked and the 12 nonconforming changed
files were formatted. The dataset-pipeline dependency warnings are upstream
Supervisely/Pydantic deprecations and do not fail its suite.

Known provenance limitation: Phase 3 run contracts hash model/data/config/report
artifacts and record base commit `79a9575…`, but their dirty-worktree record is a
status listing rather than a patch hash. The exact training source snapshot is
not byte-reconstructable. The owner accepted this for the null-result closeout;
Phase 4 must use a clean commit and must not inherit this limitation.

## Usage and deletion review

All new library modules are imported by an executable path and/or focused tests.
All new CLI entry points now have a reproducing report reference. The obsolete
Python dataset uploader is absent; publication uses the documented native
tar/pigz/AWS stream. No additional deletion is supported by current evidence.

One non-blocking refactor candidate remains: seven evaluation helpers are
duplicated between `scripts/evaluate_proposal_utility.py` and
`scripts/sweep_proposal_suppression.py` (`_load_model`, `_valid_levels`,
`_candidate_tensors`, `_source_size_bands`, `_ego_source_quads`,
`_transform_ego`, and `_merge_metrics`). A later change could move candidate
loading/geometry/region preparation into a shared `student_detector` module.
Doing that now would enlarge the already evidence-bound commit and risk changing
evaluation behavior, so it is intentionally deferred. No other high-payoff,
behavior-preserving refactor is justified before this commit.

## Suggested commit boundary

From the repository root:

```bash
git add .specs/proposal-detector-correction-and-retraining-plan.md \
  projects/visual-inference
git diff --cached --check
git status --short
git commit -m "feat(visual-inference): complete corrected proposal phases 2 and 3"
```

After committing, record the clean commit hash and create/upload the Phase 3
gitignored-artifact manifest before beginning Phase 4.1.
