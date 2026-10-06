# Phase 1 commit review

Proposed commit: `feat(visual-inference): correct proposal supervision and publish Phase 1 dataset`

Status: the reviewed source was committed and published, and the owner-approved
local HBB/quad loader gate passed. The staged taxonomy mapping and final status
evidence still need a follow-up commit. Clean-worker archive portability is
deferred to the first normal cloud staging.

## Scope

The reviewed commit included the modified and new visual-inference source,
configs, dependency locks, tests, and Phase 1 evidence documents. This follow-up
only needs the approved taxonomy mapping and final Phase 1 status evidence.

The change covers approved category/supervision policy, official COCO identity,
sequence-safe WoodScape validation, bounded-memory generation and auditing,
shared HBB/quad augmentation, source-mixture accounting, and immutable dataset
publication with backward-compatible v2 archive verification.

The scope also includes the owner-requested documentation alignment: original
project phase names stay intact, the correction plan is mapped to them, and the
current milestone explicitly stops after proposal retraining/evaluation with
SigLIP-dependent gates deferred. Historical benchmark results remain unchanged.

## Review cleanup

- Validate SHA-256 values as hexadecimal rather than checking length alone.
- Reject all tar member types except regular files and directories.
- Exercise v2 missing-index, wrong extracted-size, same-size archive-corruption,
  and malformed-hash failures. Historical v1 staging tests remain intact.
- Fix native-upload documentation to wait for the hash process, clean up its
  FIFO, stop on pipeline errors, and explicitly dereference hard links for
  future uploads. The already published archive is unchanged.
- Remove five unused imports and name centerness distances explicitly.
- Apply the existing quad reader's lazy, process-local, read-only SQLite pattern
  to HBB, including pickle-safe connections, shared epochs for persistent workers,
  and the cloud prebuilt-index requirement.

## Deletions and retained code

Delete `scripts/cloud/upload_dataset_bundle.py` and its dedicated test, as already
approved. Native streaming replaces the uploader.

Retain `build_dataset_bundle.py`: it remains the documented small-fixture/v1
builder and has an actual staging round-trip test. Retain the audit and diagnostic
scripts: they reproduce the Phase 1 evidence and are referenced by tests or CLI
workflows. No further whole-module deletion is justified by the review.

Do not include or delete these unrelated/unclassified working-tree files:
repository-root `-C`, `1.html`, `26-Week-Plan-FINAL-v4 (1).odt`,
`.specs/proposal-detector-correction-and-retraining-plan copy.md`, and project
file `]`. Odd filenames alone do not establish that their contents are disposable.

## Remaining gates and limitations

- HBB now fetches annotation rows per image, following the quad reader. Both
  production train/validation readers passed parent and spawned-worker checks.
  Peak process RSS across the smoke was 981.7 MiB; initialization took up to
  188.85 seconds because the manifest signature is streamed during reader setup.
- The published archive's SHA-256 has been captured locally; remote size and
  manifest bytes have been checked. Actual remote archive hashing, native-tar
  compatibility, and extracted-size reconciliation remain deferred to the first
  normal cloud staging. Local source symlinks do not prove archive portability.
- The v2 `dataset_contract_sha256` is a provenance reference. Staging validates
  its format, but does not retrieve or independently verify the referenced
  Phase 1 contract document. The archive SHA-256 is the transfer integrity check.
- Approved 16px source-object support still requires the Phase 2 P2/min-4
  architecture ablation; existing P3 configs are not evidence of that support.
- Keep published dataset/manifest/audit bytes immutable. Source cleanup changes
  current tool hashes; the recorded generation inventory describes the code
  that produced the dataset, not the final cleanup commit.

## Verification

Final results: 205 detector/cloud tests and 79 pipeline tests passed. Ruff and
`git diff --check` passed.

Run detector/cloud tests using `uv run pytest -q tests` from visual-inference,
and pipeline tests using `uv run pytest -q` from dataset-pipeline. They have
separate environments; recursively collecting both from the parent uses the
wrong dependency environment for pipeline tests.

The HBB regression includes a real persistent spawned worker. The sandbox blocks
PyTorch's shared-memory manager; the focused and full reader suites passed with
that OS restriction lifted. The subsequent production-corpus smoke also passed
all four train/validation HBB/quad cases with spawned workers; exact results are
recorded in `phase_1_9_local_loader_smoke.json`.

Ruff checks cover `student_detector`, `scripts`, `tests`, and pipeline code/tests.
Third-party Supervisely/Pydantic deprecation warnings remain in pipeline tests.
No cloud worker, training run, or full archive download was launched as part of
the local loader verification.
