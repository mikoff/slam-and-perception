# Phase-3 Bounded Benchmark v1 Reproduction Record

This document is the canonical record for regenerating the completed bounded
LiteFPN/AttnRes experiment and comparing it with `hbb_control_v1`. Do not edit
the v1 recipe to improve a result; create a new benchmark version instead.

## What was measured

| Contract item | Frozen value |
|---|---:|
| Training images | 364 |
| Validation images | 36 |
| Input tensor | `[B, 3, 384, 384]`, normalized FP32 |
| Quad variants | `lite`, `attn_res` |
| Seeds | 42, 43, 44 |
| Global batch | 20 |
| Batches per epoch | 19 |
| Epochs | 40 |
| Optimizer updates | 760 |
| Backbone frozen | First 2 epochs |
| Training precision | Accelerate FP16 on CUDA |
| Configured warmup | 1,000 updates |
| Effective warmup | 760 updates (clamped to total steps) |

The annotation inputs are content-addressed:

| Input | SHA-256 |
|---|---|
| Benchmark YAML | `403a72de5bfae192d52be625071786fdcab28037cec7f639c6484052888a5a7f` |
| Training manifest | `bbf5592dcb8138cbeb01e6a2c6fc2485e9a1dcc257effbb10064136086db1e25` |
| Validation manifest | `25ad4e80b7ec8ab4c952d96a033ad13b21d27fe2b17e8ea256ec090bbff7c938` |

The runner also verifies the locked HBB checkpoint and raw evaluation report.
That report uses the same validation-manifest hash and exact polygon IoU.

## Environment preparation

Run from the project directory. The cloud group preserves AWS/W&B tooling,
`--inexact` does not prune unrelated installed packages, and later commands use
`--no-sync` so launching the benchmark cannot reconcile the environment.

```bash
cd /home/sashamikoff/git/skillup/projects/visual-inference
uv sync --frozen --group cloud --inexact

uv run --no-sync python -c \
  "import torch; print(torch.__version__, torch.version.cuda); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')"
```

The recorded dependency lock targets PyTorch `2.13.0+cu130`. If an in-place
CUDA 12-to-13 migration leaves NVIDIA package metadata but removes shared
libraries, use the repair command in the benchmark README before continuing.

## Verify before training

```bash
uv run --no-sync python scripts/run_limited_benchmark.py --verify-only

uv run --no-sync python scripts/run_limited_benchmark.py \
  --seeds 42,43,44 \
  --dry-run
```

Verification must print `Verified frozen benchmark phase3_bounded_v1`. The dry
run must print six launches: two necks for each of the three seeds.

## Launch the benchmark

One GPU, sequential LiteFPN and AttnRes runs:

```bash
uv run --no-sync python scripts/run_limited_benchmark.py \
  --seeds 42,43,44 \
  --skip-existing
```

Two GPUs, one neck per GPU for each seed:

```bash
uv run --no-sync python scripts/run_limited_benchmark.py \
  --seeds 42,43,44 \
  --parallel-neck-devices 0,1 \
  --skip-existing
```

To distribute each model across two GPUs instead, run:

```bash
CUDA_VISIBLE_DEVICES=0,1 uv run --no-sync python \
  scripts/run_limited_benchmark.py \
  --seeds 42,43,44 \
  --num-processes 2 \
  --skip-existing
```

Do not combine `--parallel-neck-devices` with `--num-processes 2`.
`--skip-existing` treats a run with `last.pt` as complete; inspect its
`run_contract.json` before reusing results copied from another machine.

## Rebuild and save the comparison tables

```bash
uv run --no-sync python scripts/run_limited_benchmark.py \
  --seeds 42,43,44 \
  --summarize-only \
  | tee artifacts/phase3/benchmarks/bounded_v1/comparison_table.txt
```

The JSON source of the tables is
`artifacts/phase3/benchmarks/bounded_v1/summary.json`. Each run directory is
`artifacts/phase3/benchmarks/bounded_v1/<neck>/seed_<n>/` and contains:

```text
last.pt
best.pt
best_raw.pt
best_ema.pt
quad_metrics.jsonl
run_contract.json
tensorboard/
```

## Recorded final-epoch results

| Model/state | AR@100 | R@50 | R@75 | Median IoU |
|---|---:|---:|---:|---:|
| HBB control v1, raw fixed | **0.1164** | **0.3022** | 0.0709 | **0.2332** |
| LiteFPN, raw mean | 0.1052 ± 0.0102 | 0.2786 ± 0.0206 | 0.0684 ± 0.0184 | 0.2197 ± 0.0263 |
| LiteFPN, EMA mean | 0.1056 ± 0.0083 | 0.2811 ± 0.0057 | 0.0609 ± 0.0078 | 0.2253 ± 0.0163 |
| AttnRes, raw mean | 0.0978 ± 0.0053 | 0.2550 ± 0.0219 | **0.0721 ± 0.0155** | 0.2213 ± 0.0210 |
| AttnRes, EMA mean | 0.0981 ± 0.0006 | 0.2525 ± 0.0219 | 0.0659 ± 0.0057 | 0.2304 ± 0.0298 |

The summary reads final metrics from `last.pt`. For deployment checkpoint
inspection, the selected best raw means were:

| Model | AR@100 | R@50 | R@75 | Median IoU |
|---|---:|---:|---:|---:|
| HBB control v1 | **0.1164** | **0.3022** | 0.0709 | 0.2332 |
| LiteFPN best raw | 0.1122 | 0.2910 | 0.0709 | **0.2384** |
| AttnRes best raw | 0.1036 | 0.2674 | **0.0808** | 0.2218 |

Best-checkpoint means are descriptive because each epoch was selected on the
same small validation set. Use the final-epoch table for the least selected
comparison and reevaluate a chosen production checkpoint once on a separate
holdout before promotion.

## Interpretation and production status

- LiteFPN is the preferred quad neck. It beats AttnRes on mean AR@100 and R@50;
  best-raw LiteFPN also beats best-raw AttnRes on AR for all three paired seeds.
- AttnRes improves strict R@75 in some runs but does not justify its additional
  complexity or its general-recall regression.
- HBB remains the safest overall-recall production reference. Best-raw LiteFPN
  is close, ties HBB at R@75, and slightly exceeds its median matched IoU.
- HBB is one fixed run, not a variance estimate. It trained with batch 12,
  31 batches per epoch, 1,240 updates, and 100 warmup steps.
- The v1 quad runs trained with 39% fewer updates and spent the complete run in
  warmup. Therefore HBB-versus-Quad is a deployment comparison, not a matched
  optimization-budget ablation.

## Reproducibility boundary

The lock and run contracts detect configuration, annotation, checkpoint, code
state, and environment drift. They do not make CUDA kernels bit-deterministic
across GPU architectures, drivers, or PyTorch releases. Compare regenerated
runs at the reported metric precision and inspect paired-seed direction rather
than requiring byte-identical checkpoints.

Source images live under the gitignored dataset workspace. Their paths are
referenced by locked manifests, but individual image bytes are not currently
hashed. Archive the entire dataset workspace together with the repository
commit, `uv.lock`, benchmark outputs, and run contracts for disaster recovery.

## Required follow-up benchmark

Preserve v1 unchanged. A schedule-matched v2 should train HBB, LiteFPN quad,
and AttnRes quad with seeds 42/43/44, batch 12, 31 batches per epoch, 40 epochs,
1,240 optimizer updates, and 100 warmup steps. That is the appropriate gate for
a final HBB-versus-Quad production decision.
