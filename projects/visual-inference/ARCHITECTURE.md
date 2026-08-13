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
- Each worker lazily opens its SQLite index read-only and fetches annotations per
  image. The full annotation table is never loaded into process memory.
- Full `last.pt` checkpoints contain deterministic resume state. Best model
  checkpoints are weights-only and declare whether raw or EMA weights won.
- S3 checkpoint manifests are published only after immutable object upload and
  contain the latest plus two previous recovery candidates with SHA-256 hashes.
- Run contracts bind checkpoints to source commit, dataset ID and manifest,
  config path, phase, and model architecture. Mismatches fail closed.

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
  headroom, and uploads its report to S3. It never edits a training recipe.
- Assignment and positive offset construction are vectorized; only the rare
  no-candidate fallback remains per-object.
- dstack retries capacity failures and host interruptions, not application
  errors. The Packet reconciler replaces interrupted hosts with a bounded retry
  count and cleans orphaned instances after a grace period.
- Frozen bounded-v1 results are accepted only when every seed/state primary
  metric matches the reference to four decimal places.

# Local Constraints & Gotchas

- The full cloud/GPU matrix is seeds 42/43/44 for LiteFPN and AttnResLiteFPN;
  CPU-only development can verify contracts but cannot certify this gate.
- Validation score quantiles retain scalar FP32 scores on CPU for exact results;
  their host-memory cost scales with feature-point count, not proposal geometry.
- Exact polygon NMS remains a measurable inference cost and should be profiled
  separately.
- A dataset ID is immutable. Publish changed bytes under a new ID instead of
  replacing an existing S3 prefix or local staged directory.
- Production images are local symlinks into raw sources and exceed available
  archive scratch space; use `upload_dataset_bundle.py`, not a local tar file.
- RunPod needs a configured dstack backend. Packet also needs its API key, SSH
  public/private key pair, and the scheduled reconciler to remain enabled.
- W&B is observability, not checkpoint durability. S3 versioning and lifecycle
  policy are still required for recovery from accidental object replacement.
- Historical runs used different manifests and PyTorch versions; do not compare
  them as paired ablations. The lock and run contract are part of the result.
- TensorBoard remains only in the optional inspection dependency group; training
  has one reporting path and does not write parallel TensorBoard event streams.
