# Module Purpose & Boundaries

This project owns the approved proposal-object contract and trains class-agnostic HBB and quad detectors. HBB supports dense P3–P5 and P2–P5 outputs; quad uses P3–P5.
GitHub Actions dispatches cloud work; dstack owns cloud tasks. The Packet bridge provisions fresh hosts and registers attempt-specific SSH fleets.

# Technical Contracts & Interfaces

- Input tensors are FP32 `[B, 3, H, W]`, with dimensions divisible by 32.
- Accelerate owns device placement, FP16, accumulation, tracker dispatch, and
  distributed synchronization; the training runtime owns checkpoint lifecycle.
- `DATASET_ID` identifies an immutable S3 prefix with an archive-level hash manifest.
- Production bundles use native tar/pigz/AWS streaming while hashing and
  dereferencing symlinks; the manifest uploads only after archive verification.
- Phase 4 launch fails before GPU setup unless the exact dataset/cloud-contract
  hashes, schema-8 indexes, manifest bindings, and initialization hash match.
- HBB/quad workers lazily reopen SQLite read-only per process and fetch one
  image's annotations; shared epoch tensors propagate to persistent workers.
- HBB/quad share `proposal-manifest.v2` and an immutable decoded K=100 record
  with stable IDs, inverse-letterboxed geometry, and deterministic score ties.
- FP32 deployment artifacts are flattened raw-output PT2 graphs; decoding and
  geometry-specific NMS remain outside the exported network.
- Both geometry adapters consume one sampled affine/photometric policy; validation
  is letterbox-only and effective bounds/exclusions are recorded in run metadata.
- Phase 4 local runs use FP16 `[B=16, 3, 384, 384]` with accumulation 8;
  the 20,000-step quad continuation is prepared but has not been launched.
- Full `last.pt` checkpoints contain deterministic resume state. Best model
  checkpoints are weights-only and declare whether raw or EMA weights won.
- A weights-only warm start resets optimizer, scheduler, and EMA; full resume
  restores all state and takes precedence when auto-resume finds a checkpoint.
- Quality recipes starting at centerness use the staged curriculum; explicit
  blend/geometry recipes stay fixed, including the mature-EMA min-8 warm start.
- Run contracts bind source/data/config/project Git state; resumes require descendant source.

# Active Design Patterns & Decisions

- Console and JSONL use millisecond UTC; W&B metrics are namespaced.
- SIGINT/SIGTERM checkpoints at safe batch boundaries for deterministic resume.
- Source sampling uses cumulative residual quotas; intended/observed source and
  domain counts are reduced across workers and logged per optimizer window/epoch.
- A single bounded background uploader serializes S3 checkpoint writes and is
  flushed before tracker shutdown; local atomic writes precede remote upload.
- Validation computes each image's polygon overlaps on the accelerator, then
  folds them into counters, compact CPU diagnostics, and fixed-memory score
  histograms. No validation-long proposal or ground-truth tensors are kept.
- NMS audits promote FP16 outputs to FP32 geometry; interactive visualization
  defaults to NMS 0.7/score 0.4, supports overrides and per-source sampling.
- Quad raw and EMA states share one validation traversal; HBB also reports both
  states, each with an independent streaming metric accumulator.
- Production validates every five epochs and at completion. Frozen benchmark
  recipes validate every epoch so their comparison contract is unchanged.
- `batch_preflight` tests real benchmark samples in isolated FP16 subprocesses,
  recommends the smallest batch within 95% of peak throughput with 15% VRAM
  headroom, stops on the first OOM/error/timeout, and uploads its report to S3.
- Assignment and positive offset construction are vectorized; only the rare
  no-candidate fallback remains per-object.
- Shared proposal-evaluation helpers prepare models, masks, ego regions, and
  slice labels; scripts retain fixed-policy versus grid-specific aggregation.
- Diversity audits cover `[N,4,2]` overlap clusters, spatial/scale diversity, and unmatched isolation.
- The Phase 2 quad EMA stays incumbent; 15k passed all gates at candidate NMS 0.75.
- Continuation contracts hash-bind parent checkpoints and gate reports; later stages need owner approval.
- Future ranking/background-loss work needs a new contract and step-named artifacts; it is not a Phase 4 resume.

# Local Constraints & Gotchas

- Validation score quantiles use 65,536-bin `[0,1]` histograms; memory is
  independent of dataset size and absolute quantile error is below one bin.
- A dataset ID is immutable. Publish changed bytes under a new ID instead of
  replacing an existing S3 prefix or local staged directory.
- Production images are local symlinks and exceed archive scratch space; use the
  runbook's native tar/pigz/pv/AWS pipeline, never a local production archive.
- Phase 4 HBB aborts on AMP skips; staged quad recovery permits at most four
  logged skips, and advances scheduler/EMA/global step only on successful updates.
- Packet provisioning requires its registered SSH key and explicit owner cost check.
- `packet_host_bootstrap.sh` owns Packet host mutation; the Python bridge injects
  keys/versions, sets Docker's 32G shm default, then verifies before dstack.
- Task submissions bind run IDs, tags, commits, and GPU resources; Packet rotates
  valid placements, targets a fresh attempt-specific fleet, and adopts no host.
- GitHub verifies dataset reads and checkpoint-prefix write/read, not S3 settings.
- Local HBB and quad use 16 × 8. The isolated 15k→20k stage keeps the exhausted
  four-skip ceiling; any new AMP skip aborts. Total unmatched remains diagnostic.
