# Module Purpose & Boundaries

This project trains class-agnostic HBB and quad proposal detectors. The exported
network ends at dense P3-P5 outputs; decoding, polygon NMS, and metrics are kept
outside the model. HBB is a comparison control, while LiteFPN and AttnResLiteFPN
are the production quad variants.

GitHub Actions dispatches cloud work but never owns a GPU directly. dstack owns
RunPod tasks. Packet.ai instances are provisioned by the small desired-state
bridge in `scripts/cloud/packet_bridge.py`, registered as a dstack SSH fleet,
and then run through the same dstack task contract.

# Technical Contracts & Interfaces

- Input tensors are FP32 `[B, 3, H, W]`, with dimensions divisible by 32.
- Accelerate owns device placement, FP16, accumulation, tracker dispatch, and
  distributed synchronization; the training runtime owns checkpoint lifecycle.
- `DATASET_ID` identifies an immutable S3 prefix containing `dataset.tar.gz`
  and a v1 manifest with byte size and SHA-256 for the archive and every file.
- Large production bundles stream directly to S3 while hashing archive bytes;
  file symlinks are dereferenced into regular tar members. The manifest is
  uploaded only after the archive command completes successfully.
- Cloud datasets must contain prebuilt `indexes/quad_train.sqlite` and
  `indexes/quad_val.sqlite`; a cloud worker may not silently rebuild an index.
- Spawned CUDA workers use file-system tensor sharing, lazily reopen SQLite
  read-only, and never load the full annotation table.
- Full `last.pt` checkpoints contain deterministic resume state. Best model
  checkpoints are weights-only and declare whether raw or EMA weights won.
- S3 checkpoint manifests follow immutable object uploads and retain the latest
  plus two SHA-256 recovery candidates; transport errors fail closed.
- Run contracts bind checkpoints to source, data, config, phase, and architecture.

# Active Design Patterns & Decisions

- Training reports structured JSONL locally and namespaced `train/`, `val_raw/`,
  and `val_ema/` metrics through Accelerate's W&B tracker.
- SIGINT/SIGTERM is handled at a safe batch boundary. An incomplete-epoch
  checkpoint records batch position so restarted work skips completed batches.
- A single bounded background uploader serializes S3 checkpoint writes and is
  flushed before tracker shutdown; local atomic writes precede remote upload.
- Validation computes each image's polygon overlaps on the accelerator, then
  immediately folds them into counters and compact one-dimensional CPU
  diagnostics. No validation-long proposal or ground-truth tensors are kept.
- Raw and EMA states share one dataloader traversal and target construction
  pass; each state owns an independent streaming metric accumulator.
- Production validates every five epochs and at completion. Frozen benchmark
  recipes validate every epoch so their comparison contract is unchanged.
- `batch_preflight` tests real benchmark samples in isolated FP16 subprocesses,
  recommends the smallest batch within 95% of peak throughput with 15% VRAM
  headroom, stops on the first OOM/error/timeout, and uploads its report to S3.
- Assignment and positive offset construction are vectorized; only the rare
  no-candidate fallback remains per-object.
- Packet cleanup is exact-name; disk derives from manifests; dstack status uses `run_spec.run_name`.
- dstack clones to `/dstack/repo`; tasks run in the project subdirectory.
- Packet allows 15m image pull plus 1m stable execution; running tasks survive local gate errors.
- The reconciler retries interrupted hosts and cleans old orphans.
- Frozen bounded-v1 results are accepted only when every seed/state primary
  metric matches the reference to four decimal places.

# Local Constraints & Gotchas

- The full cloud/GPU matrix is seeds 42/43/44 for LiteFPN and AttnResLiteFPN;
  CPU-only development can verify contracts but cannot certify this gate.
- Validation score quantiles retain scalar FP32 scores on CPU for exact results;
  their host-memory cost scales with feature-point count, not proposal geometry.
- A dataset ID is immutable. Publish changed bytes under a new ID instead of
  replacing an existing S3 prefix or local staged directory.
- Production images are local symlinks into raw sources and exceed available
  archive scratch space; use `upload_dataset_bundle.py`, not a local tar file.
- RunPod needs a configured dstack backend; Packet needs its registered SSH key.
- `packet_host_bootstrap.sh` owns Packet host mutation; the Python bridge injects
  keys/versions, sets Docker's 32G shm default, then verifies before dstack.
- All task submissions render concrete run IDs, tags, repo commits, and GPU
  resources before dstack; Packet targets its unique attempt-specific fleet.
- Packet rotates valid pool/region placements; it never combines them freely.
- Packet servers are never adopted; ambiguous launches are only destroyed.
- A 30-minute lease prevents races. After expiry, started tasks are monitored;
  otherwise resources are replaced. dstack checks out the recorded commit.
- GitHub verifies S3 versioning, multipart cleanup, and seven-day retention
  before submission; W&B artifact links are best-effort observability only.
