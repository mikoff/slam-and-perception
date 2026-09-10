# Correction Phase 2.5 deployment feasibility

Status: early deployment gate complete; HBB primary and quad control export
successfully. Physical Raspberry Pi and INT8 runtime acceptance remain deferred.

## Boundary and method

The current implemented deployment boundary is a fixed-shape FP32
`torch.export` PT2 graph at `[1, 3, 384, 384]`. It emits raw P3–P5 tensors;
score filtering, coordinate decoding, clipping, NMS, and source-coordinate
inversion remain outside the graph. ExecuTorch lowering and INT8 calibration do
not yet exist and are not implied by this result.

Each Phase 2.1 EMA candidate was loaded strictly, wrapped only to flatten its
named output into ordinary tensors, exported, saved, loaded from disk, and run
again on the same seeded input. Eager-to-export and eager-to-reload comparisons
require matching tensor shapes and `atol=rtol=1e-5`. Convolution and linear MACs
are counted by executed module hooks; resize, additions, and activations are
excluded. CPU measurements use isolated processes, FP32, batch one, four
threads, 10 warmups, and 50 timed iterations.

## Export and static cost

| Candidate | Status | Raw tensors | Parameters | FP32 parameter bytes | Conv/linear MACs | PT2 size | Graph nodes | Reload max abs error |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| HBB P3–P5 primary | pass | 9 | 7,065,593 | 28,262,372 | 2.543G | 31,811,340 B | 706 | **0.0** |
| Quad P3–P5 control | pass | 6 | 7,065,881 | 28,263,524 | 2.544G | 31,777,740 B | 688 | **0.0** |

Both exports are structurally and numerically viable. Quad has only 288 more
parameters and 0.87M more counted MACs, so raw-network static cost does not
meaningfully distinguish them. The PT2 archives are approximately 30.3 MiB and
remain unquantized.

## Available-host runtime and memory

Host: Intel Core i5-8350U, x86-64, four physical/eight logical CPUs, PyTorch
2.13.0, no working CUDA driver. Process RSS includes PyTorch/runtime overhead;
the isolated peak delta is more useful than interpreting it as model-only RAM.

| Candidate | Raw median | Raw p90 | Median throughput | Peak RSS | Peak delta from process start |
|---|---:|---:|---:|---:|---:|
| HBB primary | **65.78 ms** | **69.72 ms** | **15.20 images/s** | 887.54 MiB | 249.60 MiB |
| Quad control | 69.21 ms | 79.55 ms | 14.45 images/s | 882.84 MiB | 244.82 MiB |

Phase 2.4 measured approved-threshold CPU post-processing separately: HBB
candidate decoding plus production NMS was 1.25 ms/image, versus 11.53 ms/image
for quad exact decoding/NMS. Utility-only overlap accounting is excluded.

The earlier matched RTX 3060 run remains useful experiment-planning evidence:
HBB/quad raw forward measured 10.27/7.72 ms per image. At the older NMS=0.90
settings, native decode/NMS measured 4.26/32.30 ms. The CUDA driver is currently
unavailable, so these historical numbers were not rerun with the approved
thresholds and are not presented as current GPU acceptance.

## Unavailable evidence

- The matched short runs did not record peak training VRAM.
- No Raspberry Pi is attached, so target latency, memory, temperature,
  throttling, and power remain unmeasured.
- No INT8/PT2E/ExecuTorch artifact exists; FP32 PT2 success is not proof of
  backend delegation or quantized accuracy.
- Desktop x86 and RTX 3060 measurements are not substitutes for physical-target
  evidence. Under the active milestone, that release gate is deferred rather
  than silently passed.

## Decision

Phase 2 supports HBB P3–P5 as the primary architecture and quad P3–P5 as the
retained regression control. Both pass the current export boundary. HBB is the
better proposal-stage deployment choice because it won Phase 2.4 utility and
its geometry-specific post-processing is substantially cheaper. Proceeding to
loss validation does not promote this FP32 graph to Raspberry Pi production.

## Reproduction and artifacts

```bash
uv run python scripts/benchmark_deployment_candidate.py \
  --kind hbb --checkpoint artifacts/phase2/short_v1/hbb_p3/last.pt \
  --checkpoint-state ema_model \
  --artifact artifacts/phase2/deployment_v1/hbb_p3_ema_fp32.pt2 \
  --report artifacts/phase2/deployment_v1/hbb_p3_ema_fp32.json

uv run python scripts/benchmark_deployment_candidate.py \
  --kind quad --checkpoint artifacts/phase2/short_v1/quad_p3/last.pt \
  --checkpoint-state ema_model \
  --artifact artifacts/phase2/deployment_v1/quad_p3_ema_fp32.pt2 \
  --report artifacts/phase2/deployment_v1/quad_p3_ema_fp32.json
```

| Artifact | SHA-256 |
|---|---|
| HBB PT2 | `c980ec8ca7563bfd51cb864cd06cd1682aa0ac0ab40d924ea895f26d5d272136` |
| HBB report | `3cb0445fde836a177ca8174ed09e593d319e83e5c2368bf14a64f0e2253ca319` |
| Quad PT2 | `fda1cd55423900c8da2d5d7439bb8a8ba03a9627daea6266077c63b64e5b05ff` |
| Quad report | `a3c61b15acbcc04f25a21570588d8adff1681c1463d3b9b5e789793244b64483` |
