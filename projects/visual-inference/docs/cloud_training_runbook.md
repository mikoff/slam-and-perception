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
| `PACKET_SSH_PRIVATE_KEY` | Private key used by dstack; its public half must already be registered in Packet |
| `PACKET_SSH_KEY_ID` | ID of that registered Packet key; legacy `CLOUD_SSH_KEY_ID` is also accepted |
| `PACKET_STORAGE_BLOCK_ID` | optional persistent Packet volume |

RunPod credentials belong in the dstack server/backend configuration, not the
workflow. Packet secrets are only needed when Packet is selected. Apply least
privilege, require reviewers for the environment, and rotate credentials after
any log or runner exposure.

Add the public half of `PACKET_SSH_PRIVATE_KEY` once in Packet's **SSH Keys**
settings, then save its Packet key ID as `PACKET_SSH_KEY_ID`. The bridge verifies
that the ID and private key agree whenever Packet exposes key metadata. It can
also discover the key from Packet's full key, SHA-256 fingerprint, or key
preview. It deliberately does not create keys through Packet's API because the
live endpoint's validation contract currently disagrees with its OpenAPI schema.

Enable S3 versioning. Add an enabled lifecycle rule applying to `runs/` that
aborts incomplete multipart uploads. If that rule expires current objects or
noncurrent versions, it must retain them for at least seven days and must not
use a fixed expiration date. Every GitHub dispatch verifies these properties
before allocating a GPU. W&B is not a backup: S3 is the durable source for
resume state.

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
Progress is logged every ten steps. An OOM, application error, or 15-minute
candidate timeout terminates the entire worker process group and skips larger
batches. Errors and timeouts invalidate the recommendation; an OOM is a valid
upper capacity boundary. CUDA DataLoader workers use `spawn` instead of Linux's
unsafe post-CUDA `fork` and exchange tensors through file-system shared memory;
preflight workers are non-persistent so a completed or failed candidate exits
cleanly. Error entries include the full Python traceback.
The recommendation is the smallest batch within 95% of the best throughput that
retains at least 15% reserved-memory headroom.

Run separate preflights for `configs/phase3.yaml` and
`configs/phase3_attnres.yaml`; their memory and throughput profiles differ. Read
the live table in dstack logs or download the durable JSON report from:

```text
s3://$S3_BUCKET/runs/<run-id>/batch-preflight.json
```

The preflight never edits a recipe. Commit the recommended batch in a new
versioned config, run the bounded quality benchmark, and only then use that
recipe for production. Reports containing candidate failures are uploaded too,
so the S3 object remains the durable diagnostic even when the task exits nonzero.
Maximum fitting batch is not an accuracy result.

Run `smoke` first for every new image, provider, GPU family, or dataset. It runs
five optimizer steps and two validation batches. Use `production` only after the
smoke task completes and its checkpoint is present in S3.

Monitor all three surfaces:

- GitHub Actions: submission or provisioning failures.
- dstack: task scheduling, host status, stdout/stderr, and termination.
- W&B: namespaced metrics, system utilization, validation duration, checkpoint
  save duration, free disk, and upload queue depth.

The locked cloud group includes `wandb[aws]` so W&B can resolve the immutable
S3 dataset reference. For an S3-compatible provider, the worker maps
`S3_ENDPOINT_URL` to W&B's `AWS_S3_ENDPOINT_URL`. Failure to attach that
auxiliary dataset link is recorded and printed but does not abort training;
failure to initialize the W&B run itself remains fatal.

The Packet reconciler runs every 15 minutes. Every attempt receives a unique
Packet instance and dstack fleet name. A failed or stale provisioning attempt is
destroyed; recovery creates fresh capacity up to the three-attempt limit. The
bridge never reconnects to a half-provisioned server. It does continue monitoring
an already accepted dstack task, avoiding duplicate submission if the GitHub job
stopped immediately after dstack accepted it. Training application errors are
not retried.

The original source commit remains fixed even when a newer reconciler workflow
runs later: dstack receives the public repository URL and the exact recorded
commit hash. A 30-minute provisioning lease prevents the reconciler from racing
a still-active submission job.

Packet fleet registration remains attached to the GitHub submission job.
`scripts/cloud/packet_host_bootstrap.sh` installs the selected public key for
`root` and `ubuntu`, enables SSH TCP forwarding, grants passwordless sudo to
`ubuntu`, and installs Docker Engine 29.6.1 and NVIDIA Container Toolkit
1.19.1-1 from their signed apt repositories. It writes `running`, `ready`, or
`failed:<code>` to
`/var/lib/visual-inference-bootstrap/status` and detailed output to
`/var/log/visual-inference-bootstrap.log`.

Packet bootstrap also sets Docker's host-wide `default-shm-size` to `32G` and
validates the merged daemon configuration before restart. The dstack task still
requests `resources.shm_size: 32GB`. As a final effective-runtime check, every
task—batch preflight, smoke, or production—requires at least 16 GiB at
`/dev/shm` before installing dependencies or staging data. This prevents
PyTorch DataLoader failures even if either orchestration layer drops its setting.
Production training uses the same `spawn` and file-system sharing policy as
preflight, while retaining pinned-memory transfers and per-process read-only
SQLite connections.

Before creating a fleet, the bridge waits for Packet's active SSH endpoint and
then streams the bootstrap through that exact endpoint with passwordless sudo.
Progress is printed for every installation phase and execution is bounded at 15
minutes. Packet's `startup_script` launch field is not relied on because live
provisioning did not execute it. After bootstrap, the bridge verifies SSH keys,
passwordless sudo, systemd, Docker, NVIDIA runtime, and a visible GPU—the same
prerequisites documented for dstack SSH fleets. Failures include the last 100
remote log lines in GitHub and terminate the Packet instance without waiting for
dstack's opaque timeout.

After that proof, fleet registration remains attached to GitHub and training is
submitted detached to the fresh one-node attempt fleet. The job then polls dstack
for up to 15 minutes while scheduling and pulling the pinned CUDA image. Once
`running`, the task gets a separate one-minute stability window; time spent
pulling cannot consume that window. A missing, stuck, or terminal run fails
GitHub with its dstack status message, stops the run, and removes idle Packet
capacity. A task positively observed as `running` is preserved if only the local
startup gate fails, so a wrapper timing bug cannot destroy healthy paid work.
The GitHub submission job has a 45-minute outer bound so its own timeout cannot
cut off the bounded host-bootstrap and image-pull phases.
The pinned dstack JSON identifies runs through `run_spec.run_name`, not a
top-level `name`; the bridge follows that schema when polling.
The immutable monorepo commit is cloned explicitly to `/dstack/repo`, and the
task runs from `/dstack/repo/projects/visual-inference`. Its first command checks
for `pyproject.toml` and `uv.lock`. GitHub requires the run to remain `running`
for one minute, so immediate checkout/install failures cannot produce a green
submission job.

Both providers submit through the Python task renderer. It replaces the static
task defaults with the concrete run ID, run tag, immutable repository commit,
and GPU request before invoking dstack. RunPod requests the selected model (for
example `L40S:1`). Packet verifies 8 CPUs and 32 GiB RAM directly over SSH.
Required free space is derived from the selected immutable manifest as archive
bytes plus extracted-file bytes plus a 64 GiB container/checkpoint reserve, and
is checked on Docker's actual filesystem. It then uses permissive dstack matching
for its unique fleet: one GPU, 16 GiB RAM, minimal CPU/disk, any spot
classification, and no marketplace price ceiling. An offer probe reports the
rejecting stage.
`${{ env.* }}` is deliberately forbidden in `cloud-training.dstack.yml` because
dstack does not support arbitrary environment interpolation in resources/tags.

For Packet, the dispatch label `RTX6000` means the 96 GB NVIDIA RTX PRO 6000
Blackwell; it cannot select the older RTX 6000 Ada. A pool identifies the product
or host shape—`premium2` is not a location. The bridge resolves that pool once,
then separately discovers compatible region IDs from Packet's launch options.
It resolves each pool's authoritative `region_id` against the global region
catalog and passes that pair explicitly. Fresh attempts rotate complete available
pool/region placements. If a pool has no `region_id`, only regions explicitly
linked through its product/service may be tried; the unrelated global catalog is
never treated as compatible. With no linked region metadata, the bridge reports
that limitation and uses Packet's documented automatic region selection.

Instance and fleet names end in the attempt number, for example
`vi-packet-<run-id>-2` and `packet-<run-id>-2`. If Packet's launch response is
ambiguous, the bridge may search that exact name only to terminate it. It never
adopts the result for training. Orphan cleanup remains a backstop for abrupt
runner termination.

The initial GitHub submission stays attached through at most three fresh Packet
attempts, waiting 60 then 120 seconds between retryable failures. Instance
creation POSTs are never replayed directly: after a timeout, HTTP 429, or HTTP
5xx, the bridge first proves that the exact failed attempt was removed. If that
cleanup cannot be confirmed, it fails closed instead of risking duplicate paid
capacity. The scheduled reconciler remains the fallback if GitHub itself stops.

## Failure and recovery procedure

First classify the failure:

| Failure | Expected action |
|---|---|
| no capacity | dstack retries within the task duration |
| Packet launch HTTP 429/5xx/timeout | GitHub cleans the exact attempt and retries fresh capacity up to three total attempts |
| provider interruption / missing Packet host | dstack or Packet reconciler replaces host |
| Packet bootstrap/preflight failure | read the remote log tail or named missing prerequisite in GitHub; Packet attempt is terminated |
| direct preflight passes but dstack stays pending | GitHub fails after 15 minutes with the dstack reason, stops the run, and removes Packet capacity; inspect the run and fleet events |
| OOM, bad data, assertion, user-code exception | no automatic retry; fix and dispatch a new run |
| GitHub interruption before dstack accepts the task | stale attempt is destroyed; reconciler creates fresh capacity |
| GitHub interruption after dstack accepts the task | reconciler monitors the existing dstack task without reconnecting to the server |
| corrupt newest checkpoint | auto-resume verifies hashes and falls back to an older manifest entry |
| S3 authentication, timeout, or service failure | fail before training; only an authoritative missing-object response permits a fresh run |
| W&B dataset artifact reference failure | record and print the observability error; continue because the immutable dataset contract is already captured |
| contract mismatch | publish/use the correct dataset/config/commit; never force-load it |

On a replacement host, `--resume-mode auto` downloads the newest verified full
checkpoint from S3. Incomplete epochs resume at the recorded batch. Scheduler,
EMA, and global step advance only after a successful optimizer step.

Do not delete an instance merely because GitHub submission ended: the 30-minute
lease allows the submission to finish, after which the Packet reconciler owns
cleanup or task monitoring. Inspect desired state beneath
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
