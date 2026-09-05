# Module Purpose & Boundaries

This project owns the approved proposal-object contract and trains class-agnostic
HBB and quad detectors. Dense P3-P5 outputs exclude decoding, NMS, and metrics;
LiteFPN/AttnResLiteFPN are quad variants; `docs/proposal_detector_roadmap.md` sets scope.

GitHub Actions dispatches cloud work; dstack owns RunPod tasks. The Packet bridge
provisions hosts and registers dstack SSH fleets using the same task contract.

# Technical Contracts & Interfaces

- Input tensors are FP32 `[B, 3, H, W]`, with dimensions divisible by 32.
- Accelerate owns device placement, FP16, accumulation, tracker dispatch, and
  distributed synchronization; the training runtime owns checkpoint lifecycle.
- `DATASET_ID` identifies an immutable S3 prefix with an archive-level hash manifest.
- Production bundles use native tar/pigz/AWS streaming while hashing and
  dereferencing symlinks; the manifest uploads only after archive verification.
- Cloud datasets must contain prebuilt `indexes/quad_train.sqlite` and
  `indexes/quad_val.sqlite`; a cloud worker may not silently rebuild an index.
- HBB/quad workers lazily reopen SQLite read-only per process and fetch one
  image's annotations; shared epoch tensors propagate to persistent workers.
- HBB/quad share `proposal-manifest.v2`; state is fixed before geometry encoding.
- Phase 1.8 hard-fails effective state conflicts/leakage; raw precedence overlaps warn.
- Both geometry adapters consume one sampled affine/photometric policy; validation
  is letterbox-only and effective bounds/exclusions are recorded in run metadata.
- RTX 3060 uses FP16 `[B=8, 3, 384, 384]`, eight-step accumulation, and 5-epoch validation.
- Full `last.pt` checkpoints contain deterministic resume state. Best model
  checkpoints are weights-only and declare whether raw or EMA weights won.
- A weights-only warm start resets optimizer, scheduler, and EMA; full resume
  restores all state and takes precedence when auto-resume finds a checkpoint.
- Quality recipes starting at centerness use the staged curriculum; explicit
  blend/geometry recipes stay fixed, including the mature-EMA min-8 warm start.
- S3 manifests retain latest plus two immutable SHA-256 recovery candidates.
- Run contracts bind source/data/config; parent resumes require descendant source.

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
- Raw and EMA states share one dataloader traversal and target construction
  pass; each state owns an independent streaming metric accumulator.
- Production validates every five epochs and at completion. Frozen benchmark
  recipes validate every epoch so their comparison contract is unchanged.
- `batch_preflight` tests real benchmark samples in isolated FP16 subprocesses,
  recommends the smallest batch within 95% of peak throughput with 15% VRAM
  headroom, stops on the first OOM/error/timeout, and uploads its report to S3.
- Assignment and positive offset construction are vectorized; only the rare
  no-candidate fallback remains per-object.
- Frozen bounded-v1 results are accepted only when every seed/state primary
  metric matches the reference to four decimal places.
- Phase 3 HBB and quad recipes share proposal manifests; historical benchmarks stay frozen.
- `analyze_small_object_policy.py` streams model-coordinate policy evidence from SQLite.

# Local Constraints & Gotchas

- The cloud gate is seeds 42/43/44 for LiteFPN and AttnResLiteFPN; CPU cannot certify it.
- Validation score quantiles use 65,536-bin `[0,1]` histograms; memory is
  independent of dataset size and absolute quantile error is below one bin.
- A dataset ID is immutable. Publish changed bytes under a new ID instead of
  replacing an existing S3 prefix or local staged directory.
- Production images are local symlinks and exceed archive scratch space; use the
  runbook's native tar/pigz/pv/AWS pipeline, never a local production archive.
- The v2 candidate is published; Phase 1.9 clean-worker staging/smoke is pending.
- RunPod needs a configured dstack backend; Packet needs its registered SSH key.
- `packet_host_bootstrap.sh` owns Packet host mutation; the Python bridge injects
  keys/versions, sets Docker's 32G shm default, then verifies before dstack.
- Task submissions render concrete run IDs, tags, commits, and GPU resources;
  Packet targets its unique attempt-specific fleet.
- Packet rotates valid pool/region placements and never adopts existing servers.
- GitHub verifies dataset reads and checkpoint-prefix write/read, not S3 settings.
- Versioning aids manual recovery; automatic resume does not read prior versions.
