# Correction Phase 4.2 local preflight

Status: passed on 2026-09-13. No Packet host or other cloud resource was
launched.

## Result

| Gate | Evidence | Result |
|---|---|---|
| Contract and cloud wiring | Recipe/contract hashes match the approved document; tests cover HBB command selection, exact initialization and dataset hashes, Packet 12-hour rendering, and resume rules | Pass |
| Production reader | Prebuilt schema-6 SQLite indexes opened read-only without rebuilding the 9 GB manifests; 267,800 training records and all four sources loaded | Pass |
| State-balanced loader | A real batch contained positive, ignore, trusted-background, and weak-background targets | Pass |
| Deterministic overfit | Four fixed images, 20/20 successful steps; loss fell from 1.259099 to 0.765885 (39.2%); raw AR100@0.50 reached 0.54545 | Pass |
| Effective-batch equivalence | Full-batch and partitioned-microbatch loss and one-step parameter updates match | Pass |
| CUDA FP16, EMA, and validation | RTX 3060 smoke completed consecutive optimizer updates with initial scale 8192, checkpointed, and evaluated raw and EMA states | Pass |
| Crash-safe resume | Mid-epoch checkpoint at step 2/batch 2 restored optimizer, scaler, EMA, sampler position, and identity; it advanced to step 3/batch 3 and saved/evaluated both states | Pass |
| Fail-closed contracts | Tests reject a missing required EMA state, an unapproved cloud provider/GPU, and invalid cross-run resume | Pass |
| Automated suites | Main project: 268 passed; dataset pipeline: 79 passed; spawned-worker test: 1 passed outside the filesystem sandbox | Pass |

## Remediations frozen by the preflight

- HBB accepts the published proposal index schema 6 only through a scoped
  compatibility list; quad keeps its stricter current-schema requirement.
- The HBB fixed-overfit path now repeats the selected fixed subset instead of
  falling back to the production source sampler.
- Phase 4 starts FP16 scaling at 8192. A skipped optimizer step is a hard error,
  so the 20,000 successful-step and 2,560,000-sample budgets cannot silently
  drift.
- Warm-start loading explicitly requires `ema_model`; it cannot follow the
  source checkpoint's `selected_state=model` metadata by accident.

## Boundary

This establishes local execution readiness only. Before Phase 4.3, the owner
must record the current Packet RTX 4090 account rate and confirm that 12 hours
projects to no more than $12. The bounded pilot requires a separate launch
approval and must prove remote checkpoint transport and resume before the full
training run can be authorized.
