# Frozen Phase-3 Bounded Benchmark

The complete reproduction record, expected metrics, and interpretation live in
[`docs/phase3_bounded_benchmark_v1.md`](../../docs/phase3_bounded_benchmark_v1.md).

`phase3_bounded_v1.yaml` is the shared training recipe for both necks. The
runner overrides only `neck_type`, `schedule.seed`, and `output_dir`.

`phase3_bounded_v1.lock.json` pins the YAML, compact G1 train/validation
manifests, historical `hbb_control_v1` checkpoint, and its same-manifest raw
evaluation report by byte size and SHA-256. `run_limited_benchmark.py` refuses
to train if any locked input drifts. The lock also fixes batch 20, 19 batches
per epoch, 40 epochs, and 760 optimizer updates.

The lock makes the local benchmark immutable, not portable: source images stay
in the gitignored `data/visual-inference-datasets/output` workspace. Archive
that data directory separately if the benchmark must survive workstation loss.

Run from `projects/visual-inference`:

```bash
uv sync --frozen --group cloud --inexact
uv run --no-sync python scripts/run_limited_benchmark.py --verify-only
uv run --no-sync python scripts/run_limited_benchmark.py --seeds 42,43,44
```

The explicit `cloud` group preserves the repository's AWS/W&B tools, while
`--inexact` leaves unrelated packages already in the environment intact. After
syncing, `--no-sync` prevents benchmark launches from reconciling and pruning
the environment again. When migrating an existing environment from CUDA 12 to
CUDA 13, shared NVIDIA package directories may need one repair sync:

```bash
uv sync --frozen --group cloud --inexact \
  --reinstall-package nvidia-nccl-cu13 \
  --reinstall-package nvidia-nvshmem-cu13 \
  --reinstall-package nvidia-cusparselt-cu13
```

With two GPUs, run LiteFPN on GPU 0 and AttnRes on GPU 1 concurrently for each
seed:

```bash
uv run --no-sync python scripts/run_limited_benchmark.py \
  --seeds 42,43,44 --parallel-neck-devices 0,1
```

The output matrix is `artifacts/phase3/benchmarks/bounded_v1/<neck>/seed_<n>`.
Each run contains a resolved `run_contract.json`, raw/EMA checkpoints, and
metrics. The root `summary.json` and printed tables contain per-run values,
mean/standard deviation by neck/state, paired AttnRes-minus-Lite deltas, and
raw Quad-minus-HBB deltas. HBB control v1 trained for 1,240 updates, so it is a
same-validation deployment baseline rather than a matched 760-step ablation.
