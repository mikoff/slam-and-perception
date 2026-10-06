# Visual Inference Development Stages

This directory serves as the structured architectural ledger for the visual inference pipeline, documenting the evolution of models, audits, decisions, and future implementation contracts.

---

## Stages Overview

| Stage | Document | Focus & Key Findings | Status |
|---|---|---|---|
| **Preliminary Studies** | [**`00_preliminary_studies_system_audit.md`**](file:///home/sashamikoff/git/skillup/projects/visual-inference/development_stages/00_preliminary_studies_system_audit.md) | Deep system audit identifying the mathematical root causes of proposal clustering (NMS 0.78, ATSS 18-anchor assignment, flat centerness target) and missed detections (taxonomy gap, 0.0 weak-negative weight, stride-8 small object ceiling). | **Completed** |
| **HBB vs Quads Architecture Audit** | [**`hbb_quads_tested_archs_and_approaches.md`**](file:///home/sashamikoff/git/skillup/projects/visual-inference/development_stages/hbb_quads_tested_archs_and_approaches.md) | Comprehensive audit comparing Baseline (ATSS), Recipe 1 (Decoupled Centerness & Part-Aware NMS), and Recipe 2 (Dual-Head TAL + GWD) across HBB and Quads. Empirically validates HBB superiority (+6.5% fisheye recall over Quads) and includes paper citations. | **Active & Authoritative** |

---

## Historical Context & Prior Phases

- **Phase 1 (Data & Infrastructure):** Multi-source ingestion (COCO, nuImages, BDD100k, WoodScape), taxonomy mapping, and initial data verification (`docs/phase_1_*`).
- **Phase 2 (Architecture Exploration):** MobileNetV4 backbone selection, LiteFPN neck, and initial proposal contracts (`docs/phase2.md`).
- **Phase 3 (Suppression Calibration):** FCOS-style heads, direct-corner loss, bounded NMS sweeps at IoU 0.70–0.80 targeting COCO AR@100 (`docs/phase3.md`, `docs/correction_phase_3_7_suppression_calibration.md`).
- **Phase 4.0 (Teacher Baselines):** SigLIP distillation baseline on 800 images; identified contextual source crops (`margin=0.15`) as winning policy (35.88% Top-1) and measured the 0.0% tiny / 1.6% small object recall ceiling (`artifacts/phase4/baselines/phase4_0_teacher_baseline_report.md`).
- **Phase 5.0 (Open-World Discovery & Edge RPi Target):** The current active stage, transitioning from closed-world supervised training to foundation-model pseudo-labeling with SAM and dual-geometry edge students.
