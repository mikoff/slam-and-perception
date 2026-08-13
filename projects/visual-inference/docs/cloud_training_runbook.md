# Cloud training runbook

The production path is GitHub Actions → dstack → RunPod, or GitHub Actions →
Packet desired-state bridge → dstack SSH fleet. Both paths execute the same
digest-pinned container, immutable dataset staging, Accelerate training command,
W&B logging, and verified S3 checkpoint transport.

## One-time setup

Create a protected GitHub environment named `cloud-production`. Add these
environment secrets:

| Secret | Used by |
|---|---|
| `DSTACK_TOKEN`, `DSTACK_SERVER_URL`, `DSTACK_PROJECT` | dstack control plane |
| `S3_BUCKET`, `S3_ENDPOINT_URL` | datasets, checkpoints, Packet state |
| `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`, `AWS_DEFAULT_REGION` | S3 API |
| `WANDB_API_KEY`, `WANDB_PROJECT`, `WANDB_ENTITY` | experiment tracking |
| `PACKET_API_KEY` | Packet provisioning |
| `PACKET_SSH_PRIVATE_KEY` | Packet SSH fleet; matching public key is registered in Packet |
| `PACKET_STORAGE_BLOCK_ID` | optional persistent Packet volume |

RunPod credentials belong in the dstack server/backend configuration, not the
workflow. Packet secrets are only needed when Packet is selected. Apply least
privilege, require reviewers for the environment, and rotate credentials after
any log or runner exposure.

Enable S3 versioning. Add lifecycle rules that retain checkpoint objects long
enough for recovery and eventually expire abandoned multipart uploads. W&B is
not a backup: S3 is the durable source for resume state.

## Publish an immutable dataset

The source directory must already contain `indexes/quad_train.sqlite` and
`indexes/quad_val.sqlite`. Small datasets can be bundled locally:

```bash
cd projects/visual-inference
uv run python scripts/cloud/build_dataset_bundle.py \
  --root /absolute/path/to/dataset \
  --dataset-id phase3-2026-08-09 \
  --output /tmp/phase3-2026-08-09
aws s3 cp /tmp/phase3-2026-08-09/dataset.tar.gz \
  s3://$S3_BUCKET/datasets/phase3-2026-08-09/dataset.tar.gz \
  --endpoint-url "$S3_ENDPOINT_URL"
aws s3 cp /tmp/phase3-2026-08-09/dataset-manifest.json \
  s3://$S3_BUCKET/datasets/phase3-2026-08-09/dataset-manifest.json \
  --endpoint-url "$S3_ENDPOINT_URL"
```

Never overwrite a published dataset ID. The worker validates archive size and
SHA-256, rejects unsafe tar members, verifies every extracted file, and only then
atomically publishes the local directory. A matching staged manifest is reused.
Before GPU submission, GitHub downloads and validates only the small S3 manifest.
The full archive is downloaded once, directly onto the selected GPU worker.

Production images are symlinked and the dereferenced archive is larger than the
available local scratch disk. Stream it directly to S3; the command hashes every
file, dereferences image links into regular tar members, hashes the archive
stream, saves a local manifest, and publishes that manifest last:

```bash
uv run --group cloud python scripts/cloud/upload_dataset_bundle.py \
  --root ../../data/visual-inference-datasets/output \
  --dataset-id <new-immutable-id> \
  --bucket "$S3_BUCKET" \
  --endpoint "$S3_ENDPOINT_URL" \
  --manifest-output ../../data/visual-inference-datasets/cloud-bundles/<new-immutable-id>/dataset-manifest.json
```

## Dispatch and monitor

In GitHub Actions, run **Submit Cloud Training** and select provider, exact GPU,
config, immutable dataset ID, and a run mode. Arbitrary training arguments are
deliberately unavailable. Each dispatch receives a stable
`vi-<actions-run>-<attempt>` run ID that also identifies W&B and S3 state.

### GPU batch preflight

Run `batch_preflight` once for each GPU and model configuration before creating
a GPU-specific production recipe. Select the small immutable bounded-benchmark
dataset (for example `phase3-bounded-v1-2026-08-09`), not the production bundle.
The bundle must retain the production layout: `annotations/proposals_train.json`,
benchmark images, and both prebuilt SQLite indexes.

The default candidates are `16,32,64,96,128`. Each candidate runs in a fresh
process for 10 warm-up and 50 measured complete optimizer steps using FP16,
real augmentation, target construction, loss, backward, optimizer, and EMA.
OOM candidates are safely discarded. The recommendation is the smallest batch
within 95% of the best throughput that retains at least 15% reserved-memory
headroom.

Run separate preflights for `configs/phase3.yaml` and
`configs/phase3_attnres.yaml`; their memory and throughput profiles differ. Read
the live table in dstack logs or download the durable JSON report from:

```text
s3://$S3_BUCKET/runs/<run-id>/batch-preflight.json
```

The preflight never edits a recipe. Commit the recommended batch in a new
versioned config, run the bounded quality benchmark, and only then use that
config for production. Maximum fitting batch is not an accuracy result.

Run `smoke` first for every new image, provider, GPU family, or dataset. It runs
five optimizer steps and two validation batches. Use `production` only after the
smoke task completes and its checkpoint is present in S3.

Monitor all three surfaces:

- GitHub Actions: submission or provisioning failures.
- dstack: task scheduling, host status, stdout/stderr, and termination.
- W&B: namespaced metrics, system utilization, validation duration, checkpoint
  save duration, free disk, and upload queue depth.

The Packet reconciler runs every 15 minutes. It adopts the exact named instance,
replaces interrupted/missing hosts up to three attempts, and does not retry a
training application error. It preserves the original source commit even when
the reconciler workflow later runs from a newer checkout.

## Failure and recovery procedure

First classify the failure:

| Failure | Expected action |
|---|---|
| no capacity | dstack retries within the task duration |
| provider interruption / missing Packet host | dstack or Packet reconciler replaces host |
| OOM, bad data, assertion, user-code exception | no automatic retry; fix and dispatch a new run |
| GitHub submission interruption | rerun the same workflow attempt; stable state is adopted |
| corrupt newest checkpoint | auto-resume verifies hashes and falls back to an older manifest entry |
| contract mismatch | publish/use the correct dataset/config/commit; never force-load it |

On a replacement host, `--resume-mode auto` downloads the newest verified full
checkpoint from S3. Incomplete epochs resume at the recorded batch. Scheduler,
EMA, and global step advance only after a successful optimizer step.

Do not delete an instance merely because GitHub submission ended: dstack or the
Packet reconciler owns cleanup. For Packet, inspect desired state beneath
`s3://$S3_BUCKET/control/packet-runs/` before manual intervention.

## Validation and benchmark release gate

Production validation runs every five epochs plus the final epoch. Raw and EMA
models use a single validation-loader traversal and keep IoU evaluation on the
accelerator, preventing the old CPU-validation/GPU-idle phase.

Before accepting the refactor, run the frozen matrix on GPU for seeds 42, 43,
and 44 with both `phase3_bounded_v1` LiteFPN and AttnRes recipes. Compare each
candidate summary to the locked reference:

```bash
uv run python scripts/run_limited_benchmark.py \
  --summarize-only \
  --output-root artifacts/phase3/benchmarks/<candidate> \
  --compare-to artifacts/phase3/benchmarks/bounded_v1/summary.json
```

Every raw and EMA primary metric (AR100, R50, R75, median IoU) must match to four
decimal places. Preserve the candidate summary, run contracts, source commit,
dataset manifest, and W&B links as the release evidence.
