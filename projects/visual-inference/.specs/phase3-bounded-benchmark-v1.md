## Problem Statement

The Phase-3 proposal architectures could not be compared reliably because
historical runs mixed necks, checkpoint states, manifests, batch schedules,
optimizer-step counts, and evaluation behavior. A completed experiment also
needed enough provenance to be rerun and interpreted after the original shell
session and environment were gone.

## Solution

Provide one frozen bounded benchmark that trains LiteFPN and AttnResLiteFPN as
paired seeded runs, verifies all locally available inputs before launch,
records a run contract beside every checkpoint, and renders results alongside
the fixed HBB deployment baseline. Preserve the completed v1 measurements and
their limitations instead of retroactively changing the recipe.

## Implementation Decisions

- The benchmark contract fixes 364 training images, 36 validation images,
  three seeds, two quad necks, 40 epochs, and 760 optimizer updates.
- LiteFPN and AttnResLiteFPN are independent models; one model contains exactly
  one neck and the paired runner changes only neck, seed, and output location.
- Accelerate owns device placement, FP16, accumulation, and distributed
  synchronization while preserving the global batch and update count.
- Checkpoints are architecture-specific and load strictly. Raw and EMA states
  have separate best checkpoints and validation records.
- The runner verifies byte size and SHA-256 for the configuration, annotation
  manifests, HBB checkpoint, and HBB evaluation report before doing work.
- Every run contract records the resolved configuration, manifest and package
  lock hashes, model signature, Git state, runtime, world size, and step count.
- Summary output contains per-run metrics, mean and sample deviation, paired
  AttnRes-minus-Lite deltas, and raw Quad-minus-HBB deltas.
- HBB control v1 is a fixed same-validation deployment reference, not a matched
  training ablation: it used 1,240 updates and 100 warmup steps.
- Bounded v1 intentionally remains immutable. Its quad runs used 760 updates
  and an effective 760-step warmup, so a corrected schedule belongs in v2.
- Source image files remain external and gitignored. The manifests are locked,
  but complete workstation-loss recovery requires separately archiving the
  referenced image dataset.

## Testing Decisions

- The highest acceptance seam is the benchmark runner: verification must reject
  missing, resized, or rehashed frozen inputs before training starts.
- Dry-run output must expose the exact Accelerate command for every requested
  neck and seed without constructing data indexes or launching training.
- Training tests cover strict checkpoint contracts, resume state, and one
  optimizer/EMA/scheduler update per synchronized accumulation boundary.
- Summary reconstruction must work from completed checkpoint directories and
  include the locked HBB row without retraining or partially loading a model.
- A CPU Accelerate smoke run is sufficient for orchestration behavior; actual
  CUDA/FP16 and multi-GPU throughput must be checked on the target host.

## Out of Scope

- Claiming bit-identical CUDA results across GPU models or PyTorch versions.
- Treating the single historical HBB run as a three-seed statistical sample.
- Mutating v1 to correct its warmup or optimizer budget.
- Archiving or redistributing the external source-image dataset.
- Selecting AttnRes for production without a later schedule-matched win.

## Further Notes

The completed v1 evidence selects LiteFPN over AttnRes for the quad path, while
HBB remains the safest overall-recall production reference. The next scientific
comparison should freeze a new version with batch 12, 31 batches per epoch,
1,240 updates, 100 warmup steps, and paired HBB/LiteFPN/AttnRes seeds.
