# Cloud training refactor verification checklist

This is the acceptance record for the cloud-training simplification. Checked
items have executable local evidence. Provider canaries remain unchecked until
the protected GitHub environment has credentials and GPU capacity.

## Implemented

- [x] Replace provider-specific training daemons with one dstack task and one
  validated Python training entrypoint.
- [x] Use dstack's native RunPod backend and a bounded Packet.ai SSH-fleet
  bridge with durable desired state, reconciliation, and orphan cleanup.
- [x] Pin GitHub actions, uv, dstack, and the CUDA container image digest.
- [x] Restrict dispatch to typed provider/GPU/config/dataset/mode inputs and
  remove arbitrary remote arguments.
- [x] Retry capacity/host interruptions, but do not retry application failures.
- [x] Add a CI workflow for lock sync, critical lint, formatting, unit tests,
  cloud policy validation, and frozen benchmark-contract validation.
- [x] Replace mutable dataset sync with immutable archive/manifest staging,
  SHA-256 validation, safe extraction, disk preflight, and atomic publication.
- [x] Add a producer-side bundle command and require prebuilt SQLite indexes.
- [x] Load annotation rows lazily from per-process read-only SQLite connections;
  remove DuckDB and whole-table annotation loading.
- [x] Serialize S3 checkpoint uploads, publish manifests after data, retain two
  recovery candidates, verify downloads, and fall back from corrupt latest data.
- [x] Save full deterministic resume state in `last.pt`; make best checkpoints
  weights-only and explicitly label raw versus EMA state.
- [x] Handle termination at a safe batch boundary and resume incomplete epochs
  without double-advancing scheduler, EMA, or global step.
- [x] Consolidate training telemetry under Accelerate/W&B plus local JSONL with
  provenance, phase durations, disk space, and checkpoint queue depth.
- [x] Run raw and EMA validation in one loader pass, retain geometry on the
  accelerator, and reduce production validation cadence to every five epochs
  plus the final epoch.
- [x] Vectorize primary quad assignment and offset construction.
- [x] Remove obsolete autotune, cloud provider wrappers/daemon, batch benchmark,
  janitor workflow, direct TensorBoard training path, and their dead tests.
- [x] Preserve the frozen benchmark recipe and add an exact four-decimal gate
  across every seed, neck, state, and primary metric.
- [x] Add an on-demand real-data GPU batch preflight with isolated OOM handling,
  throughput-plateau selection, VRAM headroom, GitHub dispatch, and S3 report.
- [x] Refresh `ARCHITECTURE.md`, the cloud runbook, dependencies, and lockfile.

## Local verification evidence

- [x] `uv sync --frozen --group dev` succeeds from the committed lock.
- [x] Critical Ruff rules pass across `student_detector`, `scripts`, and tests.
- [x] All changed cloud/runtime files pass `ruff format --check`.
- [x] Cloud policy validator accepts the workflows and dstack task.
- [x] Test suite passes: **101 passed**.
- [x] Frozen bounded-v1 benchmark lock verifies.
- [x] Existing refactor reproduction contains all expected identities and all
  **48 primary metric gates match the frozen reference to four decimals**.
- [x] `git diff --check` reports no whitespace errors.

## External acceptance gates

- [ ] Configure and protect the `cloud-production` GitHub environment.
- [ ] Enable S3 versioning/lifecycle and publish a new immutable dataset bundle.
- [ ] Run a RunPod smoke task and force one host interruption; verify S3 resume,
  stable W&B history, and automatic cleanup.
- [ ] Run a Packet smoke task and force one host interruption; verify reconciler
  replacement, S3 resume, and orphan cleanup.
- [ ] Force one user-code failure on each provider; verify it is not retried.
- [ ] Run the current-code GPU benchmark matrix (seeds 42/43/44 × LiteFPN and
  AttnRes, raw and EMA) and archive the four-decimal comparison evidence.
- [ ] Profile a production-size validation set for GPU utilization and peak VRAM;
  if retained evaluation tensors are material, replace them with streaming metric
  accumulators without changing the frozen metric gate.
