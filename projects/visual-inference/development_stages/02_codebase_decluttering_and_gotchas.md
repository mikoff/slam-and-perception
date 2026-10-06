# Codebase Decluttering Blueprint & Engineering Gotchas

**Status:** Proposed & Approved Strategy  
**Date:** 2026-10-06  
**Scope:** Full code, script, documentation, configuration, and artifact inventory of `projects/visual-inference/`  
**Target State:** Lean, maintainable codebase retaining only reusable primitives for Phase 5.0 (SAM Discovery, Dual-Geometry Student, Raspberry Pi 5 Export)

---

## Part 1: Strategic Evaluation — Is Decluttering Reasonable?

### The Short Answer: **Yes, absolutely — but staged in the right order.**

The `visual-inference` sub-project has accumulated over **25,000 lines of code across 80+ Python files**, 45 documentation fragments, and 17 GB of historical checkpoints across Phases 1 through 4. Over 70% of these files are historical one-off sweeps, dead-end ablation artifacts (like hard-negative mining), or gate audit scripts that will never be run again.

Carrying this technical debt imposes real costs:
1. **Cognitive Overhead:** Sifting through 35 standalone scripts where 25 are historical sweep artifacts creates confusion for both humans and AI agents.
2. **False Dependency Chains:** Obsolete abstractions (e.g. `rectification.py`, `gradient_diagnostics.py`, `hard_negative_mining.py`) appear in type definitions and tests despite having zero production consumers.
3. **Fragile Maintenance:** Refactoring core models triggers test failures in obsolete historical test suites that no longer reflect project goals.

### The Staged Execution Rule:
- **Do NOT** execute a destructive blanket deletion of everything outside `development_stages/` today. Doing so would destroy critical, high-value components that Phase 5 directly depends on:
  - The MobileNetV4 backbone (`backbone.py`)
  - The P2-LiteFPN neck (`neck.py`)
  - The SigLIP teacher inference engine (`siglip.py`)
  - The contextual crop extraction module (`crop_extractor.py`)
  - Tested geometric primitives (`geometry.py`, `quad_geometry.py`)
  - The Phase 4.0 validation harness (`run_phase4_0_teacher_baselines.py`, `build_phase4_0_visual_audit.py`)
- **DO** follow a staged pruning plan:
  - **Tier 1 (Immediate Removal):** Purge dead sweeps, hard-negative mining routines, microfixtures, and obsolete historical audit scripts (saving ~8,000 lines immediately).
  - **Tier 2 (Phase 5 Refactor):** Replace bloated training/assigner files with lean Task-Aligned implementations.
  - **Tier 3 (Final Archival):** Archive legacy Phase 1–3 docs and sweep configs into an archive folder.

---

## Part 2: Institutional Knowledge & Engineering Gotchas

Before pruning historical code, we extract and record the critical lessons learned and failure modes discovered across Phases 1 through 4. These gotchas must govern all future development.

### Gotcha 1: In-Memory Image Buffering & RAM OOM
- **The Pitfall:** Attempting to pre-load or cache raw uncompressed image arrays into Python memory/RAM during dataset initialization.
- **The Math:** A single uncompressed $1920 \times 1080$ RGB `uint8` image occupies $\approx 6.22\text{ MB}$. Buffering 10,000 images in memory requires **$>62\text{ GB}$ of RAM**, instantly causing Linux OOM kills or severe kernel swapping.
- **The Rule:** Always use **lazy streaming I/O**. Load images on-demand from disk via `PIL.Image.open()` or memory-mapped index records. Cache only lightweight vector embeddings or serialized annotation metadata (JSON/RLE), never raw pixel arrays.

### Gotcha 2: PyTorch Multiprocessing & Shared Memory (`torch_shm_manager`)
- **The Pitfall:** Configuring PyTorch `DataLoader(..., num_workers=4, ...)` in containerized, restricted Linux, or sandbox environments.
- **The Error:** PyTorch's inter-process worker communication relies on POSIX shared memory (`/dev/shm`). In sandboxed or Docker environments without allocated IPC permissions, spawning worker processes crashes with:
  `RuntimeError: DataLoader worker (pid XXX) is killed by signal: Operation not permitted` or hangs on `torch_shm_manager`.
- **The Rule:** In local development and restricted environments, keep `num_workers = 0` (in-process loading). Only enable multi-worker loading on dedicated workstations where `/dev/shm` is verified (`df -h /dev/shm`).

### Gotcha 3: The Polygon NMS $O(N^2)$ CPU Bottleneck
- **The Pitfall:** Running exact polygon clipping (Sutherland-Hodgman) directly across hundreds of raw quad candidates on CPU.
- **The Math:** Exact convex polygon intersection in pure Python/PyTorch involves pairwise line-segment intersection and winding checks. Computing an exact $300 \times 300$ polygon IoU matrix takes **$>80\text{ ms}$ on CPU**, making real-time edge execution impossible.
- **The Rule:** Always use **two-stage suppression**. Pre-filter polygon candidates using their axis-aligned bounding box envelopes with compiled `torchvision.ops.nms` at IoU 0.50 (which takes $<0.3\text{ ms}$). This eliminates 85–90% of candidates before running exact polygon NMS on the small remaining subset.

### Gotcha 4: The 0.0 Weak-Negative Weight Trap (Background Hallucination)
- **The Pitfall:** Setting `weak_negative_weight: 0.0` in the loss function to prevent penalizing unannotated objects in open-world settings.
- **The Consequence:** COCO 2017 contains **zero** trusted background annotations; 100% of its background is classified as "weak negative". With weight 0.0, the model received **zero negative gradient across 50% of the training dataset**. Quality scores on background textures floated uncontrollably high, producing noisy false positives.
- **The Rule:** Never set weak negative weight to 0.0. Maintain a non-zero regularization floor (e.g., $0.15$–$0.25$) or use continuous Quality Focal Loss where all non-assigned locations are supervised toward 0.

### Gotcha 5: Stride-8 "Unrepresentable" Small Object Dropout
- **The Pitfall:** Training an anchor-free detector on small images ($384 \times 384$) with the finest feature stride at P3 (stride 8).
- **The Consequence:** Distant pedestrians ($25\text{ px}$ tall in 1080p) shrink to $<6\text{ px}$ tall and $<2\text{ px}$ wide at 384x384. A stride-8 grid cell is $8 \times 8\text{ px}$. When no grid point lands inside the pedestrian, the assigner silently marks the object as `unrepresentable` and discards it from positive supervision. This resulted in **0.0% recall on tiny objects and 1.6% recall on small objects**.
- **The Rule:** Small-object perception strictly requires **P2 (stride 4)** or an input resolution of at least $512 \times 512$ so that objects enclose multiple feature cells.

### Gotcha 6: The Centerness Score Disconnect & Proposal Stacking
- **The Pitfall:** Training proposal quality scores against static geometric Gaussian centerness rather than predicted box IoU.
- **The Consequence:** Anchors near the centroid achieve quality targets $\approx 1.0$ regardless of whether their corner regressions are tight or sloppy. The score landscape becomes a flat plateau, causing all 18 ATSS positive anchors to survive post-processing as redundant, stacked proposals.
- **The Rule:** Use **Task-Aligned IoU targets** ($t = \text{predicted\_IoU}$). If an anchor predicts a sloppy box, its score target is forced to zero, ensuring a single sharp peak at the optimal anchor.

### Gotcha 7: Symlink Paths & Cwd Resolution
- **The Pitfall:** Relying on relative paths (e.g. `./data` or `../artifacts`) in Python scripts.
- **The Consequence:** Scripts fail depending on whether the user invokes them from repo root (`/home/sashamikoff/git/skillup`) or sub-project root (`projects/visual-inference`).
- **The Rule:** Always anchor paths to `Path(__file__).resolve().parents[...]` or resolve explicit environment variables (e.g. `${PHASE2_DATASET_ROOT}`).

### Gotcha 8: Checkpoint State Dictionaries (Full State vs. EMA Weights)
- **The Pitfall:** Confusing training resume checkpoints with inference deployment checkpoints.
- **The Consequence:** `last.pt` contains full optimizer states, gradient scaler states, and EMA shadow weights ($>120\text{ MB}$). Loading `last.pt` directly into `QuadProposalDetector.load_state_dict()` fails with missing/unexpected key errors unless nested keys are parsed.
- **The Rule:** Deploy and evaluate only `best_ema.pt` (which contains clean, unnested weights). Reserve `last.pt` exclusively for deterministic training resumption.

### Gotcha 9: Downstream SigLIP Latency Bottleneck on Raspberry Pi 5
- **The Pitfall:** Forwarding 50 to 100 proposals from Stage 1 into Stage 2 (SigLIP ViT-B).
- **The Math:** Running `google/siglip-base-patch16-224` (86M parameters) on a Raspberry Pi 5 CPU takes **$100$ to $180\text{ ms}$ per single crop**. Forwarding 100 crops requires **10 to 18 seconds per frame**.
- **The Rule:** On edge hardware, Stage 1 must be aggressively deduplicated via NMS at IoU 0.45–0.50, and the proposal budget must be capped at **$K \le 8$ to $12$ top candidate crops**.

---

## Part 3: Comprehensive File Audit & Disposition Table

Every file in `projects/visual-inference/` is evaluated below under four categories:
- **`[KEEP - CORE]`**: Essential foundation, active component, or required for Phase 5.
- **`[KEEP - TRANSITION]`**: Needed until the Phase 5 replacement is implemented; delete once Phase 5 is operational.
- **`[STREAMLINE]`**: Valuable logic, but bloated or over-engineered; refactor into a compact version.
- **`[DELETE - OBSOLETE]`**: Dead-end sweep, historical audit, or unused legacy code; safe to delete immediately.

### 3.1 `student_detector/` Directory (45 Files, 12,347 Lines)

| File | Lines | Role / Functionality | Recommendation | Action & Rationale |
|---|---:|---|---|---|
| `backbone.py` | 185 | MobileNetV4 trunk (C2, C3, C4, C5 taps) | **`[KEEP - CORE]`** | Essential for Phase 5 edge detector. Clean, modular, well-tested. |
| `neck.py` | 173 | LiteFPN & P2LiteFPN feature pyramids | **`[KEEP - CORE]`** | Essential. Contains `P2LiteFPN` needed for stride-4 pedestrian recall. |
| `crop_extractor.py` | 159 | Contextual crop extraction (`margin=0.15`) | **`[KEEP - CORE]`** | Essential bridge to SigLIP. Winning crop policy from Phase 4.0. |
| `siglip.py` | 247 | SigLIP model wrapper, embeddings, text banks | **`[KEEP - CORE]`** | Essential bridge to SigLIP. Pinned revision, tested, clean. |
| `geometry.py` | 109 | Grid points, LTRB decode, box IoU | **`[KEEP - CORE]`** | Essential geometric math primitives. |
| `quad_geometry.py` | 475 | Polygon area, canonicalization, polygon NMS | **`[KEEP - CORE]`** | Essential for dual-geometry extraction and quad support. |
| `ontology.py` | 205 | Base vs novel evaluation taxonomy | **`[KEEP - CORE]`** | Active evaluation ontology for Phase 4 & Phase 5. |
| `semantic_evaluation_data.py` | 231 | Benchmark cohort data loader | **`[KEEP - CORE]`** | Used by Phase 4.0 teacher baselines and Phase 5 pilot. |
| `model.py` | 107 | Top-level detector container | **`[KEEP - CORE]`** | High-level module boundary. |
| `decoder.py` | 188 | HBB Inference decoder | **`[KEEP - CORE]`** | Clean torchvision NMS post-processor for HBB. |
| `quad_decoder.py` | 221 | Quad inference decoder | **`[KEEP - CORE]`** | Post-processor for quad proposals. |
| `head.py` | 123 | FCOS & Quad prediction heads | **`[KEEP - TRANSITION]`** | Will be updated in Phase 5 to incorporate Task-Aligned IoU head. |
| `losses.py` | 419 | CIoU, GIoU, Quality Focal Loss | **`[STREAMLINE]`** | Keep CIoU and QFL; prune unused LTRB Smooth-L1 branches. |
| `quad_losses.py` | 304 | Corner loss, GWD, validity loss | **`[KEEP - TRANSITION]`** | Keep for quad training; contains unused IoU target mode. |
| `assigner.py` | 262 | ATSS assigner (HBB) | **`[KEEP - TRANSITION]`** | Replace in Phase 5 with Task-Aligned Assigner (TAL). |
| `quad_assigner.py` | 249 | ATSS assigner (Quad) | **`[KEEP - TRANSITION]`** | Replace in Phase 5 with Task-Aligned Assigner (TAL). |
| `targets.py` | 264 | Training target builder (HBB) | **`[KEEP - TRANSITION]`** | Replace in Phase 5 with lean target builder. |
| `quad_targets.py` | 175 | Training target builder (Quad) | **`[KEEP - TRANSITION]`** | Replace in Phase 5 with lean target builder. |
| `config.py` | 360 | Model and training dataclasses | **`[STREAMLINE]`** | Simplify; remove legacy parameters from deleted phases. |
| `data.py` | 1060 | Monolithic multi-source dataset loader | **`[STREAMLINE]`** | 1060 lines is too heavy. Streamline to focus on SAM JSON format. |
| `quad_data.py` | 388 | Quad dataset loader | **`[STREAMLINE]`** | Unify with `data.py` under the new dual-geometry JSON schema. |
| `augmentation.py` | 246 | Geometric & photometric augmentations | **`[KEEP - CORE]`** | Fully functional, test-backed albumentations/torch pipeline. |
| `evaluation.py` | 211 | COCO AR and proposal evaluator | **`[KEEP - CORE]`** | Standard metrics harness. |
| `quad_evaluation.py` | 434 | Quad polygon evaluation metrics | **`[KEEP - CORE]`** | Standard metrics harness for polygon geometry. |
| `checkpoints.py` | 139 | Safe weight loading & EMA unpacking | **`[KEEP - CORE]`** | Reliable checkpoint serialization. |
| `training.py` | 349 | HBB training loop | **`[STREAMLINE]`** | Merge HBB and Quad training into one clean script. |
| `quad_training.py` | 462 | Quad training loop | **`[STREAMLINE]`** | Merge with `training.py`. |
| `training_runtime.py` | 813 | Cloud/distributed training harness | **`[STREAMLINE]`** | Extremely verbose (813 lines). Prune cloud-specific boilerplate. |
| `training_optimization.py` | 155 | Optimizer & LR scheduler setup | **`[KEEP - CORE]`** | Standard AdamW / cosine decay setup. |
| `training_reporting.py` | 364 | Logging & WandB metrics emission | **`[STREAMLINE]`** | Simplify to standard Python logging + optional WandB. |
| `deployment.py` | 124 | TorchScript / ONNX export utilities | **`[KEEP - CORE]`** | Needed for Raspberry Pi 5 export. |
| `hard_negative_mining.py` | 132 | Phase 3.6 hard negative miner | **`[DELETE - OBSOLETE]`** | Dead-end experiment from Phase 3.6. Not used in production. |
| `hard_negative_promotion.py`| 145 | Phase 3.6 hard negative promoter | **`[DELETE - OBSOLETE]`** | Dead-end experiment from Phase 3.6. Not used in production. |
| `gradient_diagnostics.py` | 151 | Phase 3.3 gradient influence study | **`[DELETE - OBSOLETE]`** | Historical research code. Never called in active paths. |
| `rectification.py` | 135 | Homography patch extraction | **`[DELETE - OBSOLETE]`** | Confirmed test-only artifact with no consumers. |
| `proposal_contract.py` | 190 | Phase 2 proposal contract validator | **`[DELETE - OBSOLETE]`** | Superseded by Phase 4.0 & 5.0 contracts. |
| `proposal_utility.py` | 293 | Phase 2.4 proposal utility audits | **`[DELETE - OBSOLETE]`** | Historical audit code. |
| `proposal_diversity.py` | 355 | Phase 3 proposal diversity audits | **`[DELETE - OBSOLETE]`** | Historical audit code. |
| `proposal_evaluation.py` | 138 | Phase 2 proposal evaluator | **`[DELETE - OBSOLETE]`** | Superseded by `evaluation.py`. |
| `suppression.py` | 164 | Phase 3.7 suppression sweep logic | **`[DELETE - OBSOLETE]`** | Historical sweep logic that caused proposal stacking. |
| `selection_policy.py` | 208 | Phase 3 selection policy checks | **`[DELETE - OBSOLETE]`** | Superseded. |
| `provenance.py` | 185 | Git/hash metadata tracking | **`[DELETE - OBSOLETE]`** | Over-engineered bookkeeping. |
| `phase4_identity.py` | 373 | Phase 4 strict identity hashing | **`[STREAMLINE]`** | Retain only the dataset slice hashing utility. |
| `quad_microfixture.py` | 292 | Synthetic test fixture | **`[DELETE - OBSOLETE]`** | Move into `tests/` directory or delete. |
| `checkpoint_transport.py`| 358 | Packet bridge transfer utilities | **`[STREAMLINE]`** | Cloud bridge specific; keep minimal transfer logic. |

---

### 3.2 `scripts/` Directory (35 Files, 12,467 Lines)

| Script | Lines | Role / Functionality | Recommendation | Action & Rationale |
|---|---:|---|---|---|
| `run_phase4_0_teacher_baselines.py` | 753 | Phase 4.0 baseline evaluation runner | **`[KEEP - CORE]`** | Active baseline tool used for 800-image benchmark. |
| `build_phase4_0_visual_audit.py` | 1374 | Interactive HTML dashboard builder | **`[KEEP - CORE]`** | Essential visualization tool for proposal auditing. |
| `train_quad_proposals.py` | 599 | Main quad training entry point | **`[KEEP - TRANSITION]`** | Keep until Phase 5 training script is implemented. |
| `train_phase3.py` | 639 | Legacy HBB training entry point | **`[KEEP - TRANSITION]`** | Keep until Phase 5 training script is implemented. |
| `visualize_quad_comparison.py` | 444 | Visual comparison tool | **`[KEEP - CORE]`** | Useful for visual verification of proposals. |
| `visualize_quad_validation.py` | 485 | Visual inspection CLI | **`[KEEP - CORE]`** | Useful for visual verification of proposals. |
| `mine_hard_negatives.py` | 614 | Phase 3.6 hard negative mining CLI | **`[DELETE - OBSOLETE]`** | Dead-end experiment; safe to delete immediately. |
| `promote_hard_negatives.py` | 72 | Phase 3.6 hard negative promoter CLI | **`[DELETE - OBSOLETE]`** | Dead-end experiment; safe to delete immediately. |
| `audit_hard_negative_exposure.py` | 111 | Phase 3.6 exposure auditor | **`[DELETE - OBSOLETE]`** | Dead-end experiment; safe to delete immediately. |
| `sweep_quad_nms.py` | 388 | NMS threshold sweep script | **`[DELETE - OBSOLETE]`** | Historical sweep that produced flawed 0.78 threshold. |
| `sweep_proposal_suppression.py` | 517 | Suppression calibration sweep | **`[DELETE - OBSOLETE]`** | Historical sweep; safe to delete immediately. |
| `measure_gradient_influence.py` | 376 | Gradient influence ablation CLI | **`[DELETE - OBSOLETE]`** | One-off historical research script. |
| `audit_proposal_diversity.py` | 307 | Proposal diversity audit CLI | **`[DELETE - OBSOLETE]`** | Historical audit; safe to delete immediately. |
| `audit_phase2_proposal_contract.py`| 286 | Phase 2 contract audit CLI | **`[DELETE - OBSOLETE]`** | Obsolete Phase 2 artifact. |
| `audit_phase3_data.py` | 131 | Phase 3 data audit CLI | **`[DELETE - OBSOLETE]`** | Obsolete Phase 3 artifact. |
| `audit_production_dataset.py` | 714 | Ingestion audit CLI | **`[DELETE - OBSOLETE]`** | Obsolete Phase 1 artifact. |
| `audit_quad_gate_readiness.py` | 140 | Gate readiness auditor | **`[DELETE - OBSOLETE]`** | Obsolete Phase 3 artifact. |
| `audit_augmentation_policy.py` | 315 | Augmentation sweep auditor | **`[DELETE - OBSOLETE]`** | Obsolete Phase 1 artifact. |
| `analyze_small_object_policy.py` | 566 | Small-object diagnostic CLI | **`[DELETE - OBSOLETE]`** | Findings already integrated into audit & Phase 5. |
| `benchmark_batch_size.py` | 467 | Batch size benchmarking CLI | **`[DELETE - OBSOLETE]`** | One-off hardware tuning script. |
| `benchmark_deployment_candidate.py`| 220 | Deployment latency benchmark | **`[STREAMLINE]`** | Adapt into an edge RPi benchmark script. |
| `benchmark_quad_quality_targets.py`| 163 | Quality target comparison | **`[DELETE - OBSOLETE]`** | Findings already integrated into audit report. |
| `build_dataset_contract.py` | 210 | Legacy contract generation | **`[DELETE - OBSOLETE]`** | Superseded. |
| `build_quad_comparison_dashboard.py`| 305 | Legacy HTML dashboard generator | **`[DELETE - OBSOLETE]`** | Superseded by `build_phase4_0_visual_audit.py`. |
| `compare_open_world_proposals.py` | 342 | Open-world proposal comparison | **`[DELETE - OBSOLETE]`** | Obsolete Phase 3.7 script. |
| `create_quad_g6_fixture.py` | 24 | Test fixture creator | **`[DELETE - OBSOLETE]`** | Test artifact. |
| `diagnose_production_audit.py` | 416 | Ingestion diagnostics | **`[DELETE - OBSOLETE]`** | Obsolete Phase 1 script. |
| `diagnose_quad_checkpoint.py` | 135 | Checkpoint inspection CLI | **`[STREAMLINE]`** | Simplify to 30-line weight inspect tool. |
| `evaluate_proposal_utility.py` | 414 | Phase 2 proposal utility CLI | **`[DELETE - OBSOLETE]`** | Obsolete Phase 2 script. |
| `evaluate_quad_g6.py` | 201 | Phase 3 G6 benchmark script | **`[DELETE - OBSOLETE]`** | Obsolete Phase 3 script. |
| `evaluate_selection_candidate.py` | 72 | Selection candidate evaluator | **`[DELETE - OBSOLETE]`** | Obsolete Phase 3 script. |
| `run_limited_benchmark.py` | 524 | Limited benchmark script | **`[DELETE - OBSOLETE]`** | Superseded by Phase 4.0 baselines. |
| `visualize_phase3.py` | 142 | Phase 3 visualization script | **`[DELETE - OBSOLETE]`** | Superseded. |

---

### 3.3 `docs/` Directory (45 Markdown Files)

The `docs/` folder contains extensive records of historical milestones. 
- **`[KEEP - ACTIVE]`**:
  - `code_audit.md` (active cleanup notes)
  - `visual_inference_system_audit.md` (the core system audit)
- **`[ARCHIVE to docs/history/]`**:
  - All 43 other files: `phase_1_*`, `phase2.md`, `phase3.md`, `correction_phase_*`, `cloud_training_runbook.md`, etc.
  - Moving these to `docs/history/` immediately cleans up the top-level documentation tree while keeping git history intact.

---

### 3.4 Summary Impact of Proposed Decluttering

| Section | Current Files | Current Lines | Files Recommended for Immediate Deletion | Lines Removed | Retained Core / Transition Files |
|---|---:|---:|---:|---:|---:|
| `student_detector/` | 45 | 12,347 | **8 files** | **~1,700 lines** | 37 files (to be streamlined to ~15) |
| `scripts/` | 34 | 12,467 | **24 files** | **~8,800 lines** | 10 active scripts |
| `docs/` | 45 | ~6,000 | **43 files** (Archived to `docs/history/`) | ~5,500 lines moved | 2 active documents |
| **Total** | **124** | **~30,800** | **32 files deleted + 43 archived** | **~16,000 lines removed/archived** | **Clean, focused workspace** |

---

## Part 4: Recommended 3-Step Execution Plan

1. **Step 1: Immediate Safe Purge (Zero Risk):**
   - Delete the 24 obsolete scripts in `scripts/` (hard-negative mining, historical sweeps, Phase 2/3 one-off CLIs).
   - Delete the 8 obsolete files in `student_detector/` (`rectification.py`, `gradient_diagnostics.py`, `hard_negative_*.py`, `suppression.py`, `proposal_diversity.py`, `selection_policy.py`).
   - Move historical phase markdown files into `docs/history/`.
   - Run tests to confirm zero regressions on active paths.
2. **Step 2: Implement Phase 5 Clean Core:**
   - Create `scripts/run_sam_pilot_discovery.py` to extract dual-geometry pseudo-labels on the 800-image cohort.
   - Refactor `student_detector/` to use a single clean training script with Task-Aligned IoU loss and P2 feature stride.
3. **Step 3: Retire Legacy Duplicates:**
   - Once the Phase 5 student detector is trained and verified on the 800-image cohort, retire the legacy ATSS assigners (`assigner.py`, `quad_assigner.py`) and legacy quad heads.

---

## Part 5: Execution Log — Phase 4 Purge & Clean Slate Achieved

On 2026-10-06, the second wave of decluttering was executed to completely retire the flawed "correction_phase4" pipeline and clear the deck for Phase 5.0:

1. **`configs/` Cleared Completely:**
   - All `correction_phase4_*.yaml` configs, `phase3.yaml`, and benchmark contracts in `configs/benchmarks/` were moved to `docs/history/configs/`.
   - `configs/` is now pristine (retaining only `.gitkeep`) ready for clean Phase 5 configuration.
2. **`scripts/` Cleared Completely:**
   - Deleted legacy training entrypoints (`train_quad_proposals.py`, `train_phase3.py`).
   - Deleted obsolete diagnostics and viewers (`diagnose_quad_checkpoint.py`, `visualize_quad_validation.py`, `benchmark_deployment_candidate.py`).
   - Deleted entire cloud training infrastructure (`scripts/cloud/` including `packet_bridge.py`, `dataset_staging.py`, `run_training.py`, etc., and `cloud-training.dstack.yml`).
   - Archived the one-off Phase 4 audit runners (`run_phase4_0_teacher_baselines.py`, `build_phase4_0_visual_audit.py`) into `docs/history/scripts/`.
   - `scripts/` now contains only `__init__.py`, ready for `scripts/run_sam_pilot_discovery.py`.
3. **`tests/` Streamlined:**
   - Pruned 9 obsolete test files that tested the retired scripts and configs (`test_phase4_training_contract.py`, `test_train_quad_initialization.py`, `test_train_hbb_initialization.py`, `test_cloud_entrypoint.py`, `test_dataset_staging.py`, `test_packet_bridge.py`, `test_checkpoint_transport.py`, `test_build_streamed_dataset_manifest.py`, `test_visualize_quad_validation.py`).
   - Renamed `test_phase4_0_components.py` to `test_teacher_and_crop_components.py`.
   - Test suite status: **127 passed in ~20s (100% green)**.
   - Lint status: **Ruff 0 errors**.

