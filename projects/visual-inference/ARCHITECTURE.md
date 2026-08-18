# Module Purpose & Boundaries

This project trains class-agnostic HBB and quad proposal detectors. Dense P3-P5
outputs exclude decoding, polygon NMS, and metrics. HBB is a comparison control;
LiteFPN and AttnResLiteFPN are the production quad variants.

GitHub Actions dispatches cloud work; dstack owns RunPod tasks. The Packet bridge
provisions hosts and registers dstack SSH fleets using the same task contract.

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
- The RTX 3060 min-8 recipe uses FP16 `[B=8, 3, 384, 384]` plus eight-step
  accumulation; its 20-epoch EMA fine-tune uses `1e-4`/`1e-5` LR and 5-epoch validation.
- Full `last.pt` checkpoints contain deterministic resume state. Best model
  checkpoints are weights-only and declare whether raw or EMA weights won.
- A weights-only warm start resets optimizer, scheduler, and EMA; full resume
  restores all state and takes precedence when auto-resume finds a checkpoint.
- Quality recipes starting at centerness use the staged curriculum; explicit
  blend/geometry recipes stay fixed, including the mature-EMA min-8 warm start.
- S3 manifests retain latest plus two immutable SHA-256 recovery candidates.
- Run contracts bind source/data/config; parent resumes require descendant source.

# Active Design Patterns & Decisions

- Training console messages and JSONL records carry millisecond UTC timestamps;
  W&B metrics are namespaced and dataset references are observability-only links.
- SIGINT/SIGTERM is handled at a safe batch boundary. An incomplete-epoch
  checkpoint records batch position so restarted work skips completed batches.
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
- Packet cleanup is exact-name; disk derives from manifests; dstack status uses
  `run_spec.run_name`. Tasks clone to `/dstack/repo` and run in the project.
- Frozen bounded-v1 results are accepted only when every seed/state primary
  metric matches the reference to four decimal places.

# Local Constraints & Gotchas

- The cloud gate is seeds 42/43/44 for LiteFPN and AttnResLiteFPN; CPU cannot certify it.
- Validation score quantiles use 65,536-bin `[0,1]` histograms; memory is
  independent of dataset size and absolute quantile error is below one bin.
- A dataset ID is immutable. Publish changed bytes under a new ID instead of
  replacing an existing S3 prefix or local staged directory.
- Production images are local symlinks into raw sources and exceed available
  archive scratch space; use `upload_dataset_bundle.py`, not a local tar file.
- RunPod needs a configured dstack backend; Packet needs its registered SSH key.
- `packet_host_bootstrap.sh` owns Packet host mutation; the Python bridge injects
  keys/versions, sets Docker's 32G shm default, then verifies before dstack.
- All task submissions render concrete run IDs, tags, repo commits, and GPU
  resources before dstack; Packet targets its unique attempt-specific fleet.
- Packet rotates valid pool/region placements and never adopts existing servers.
- A 30-minute lease prevents races. Pre-submission failures may be replaced;
  accepted or ambiguous tasks wait for an explicit terminal dstack result.
- GitHub verifies dataset reads and checkpoint-prefix write/read, not S3 settings.
- Versioning aids manual recovery; automatic resume does not read prior versions.
