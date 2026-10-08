#!/usr/bin/env bash
# run_two_stage_pipeline.sh: End-to-End Orchestrator for MobileNetV4 Two-Stage Training & Audit.
set -euo pipefail

STAGE="${1:-all}"
DEVICE="${2:-cuda}"
BATCH_SIZE="${3:-64}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

echo "================================================================================"
echo "MobileNetV4 P2-HBB Two-Stage Pipeline Orchestrator"
echo "Stage Selection : ${STAGE} (options: all | stage1 | stage2 | dashboard)"
echo "Device          : ${DEVICE}"
echo "Batch Size      : ${BATCH_SIZE}"
echo "Working Dir     : ${ROOT_DIR}"
echo "================================================================================"

run_stage1() {
    echo ""
    echo ">>> [STAGE 1/3] Self-Supervised Discovery Pre-Training (15 Epochs on 10k SAM Dataset)..."
    uv run python scripts/train_student_pilot.py \
        --geometry hbb \
        --recipe p2_baseline \
        --epochs 15 \
        --batch-size "${BATCH_SIZE}" \
        --lr 0.0020 \
        --pseudo-labels artifacts/pseudo_labels/sam_discovery.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir artifacts/train_10k_hbb \
        --device "${DEVICE}"
    echo ">>> Stage 1 Complete -> Checkpoint: artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"
}

run_stage2() {
    echo ""
    echo ">>> [STAGE 2/3] Stage-2 Fine-Tuning on Human Ground Truth (5 Epochs with L1+CIoU Tightening)..."
    uv run python scripts/finetune_student_gt.py \
        --init-checkpoint artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt \
        --cohort-db artifacts/pseudo_labels/sam_discovery.sqlite \
        --gt-index /home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir artifacts/finetune_10k_hbb \
        --epochs 5 \
        --batch-size "${BATCH_SIZE}" \
        --lr 3e-4 \
        --box-weight 2.0 \
        --ltrb-weight 0.5 \
        --weak-negative-weight 0.15 \
        --device "${DEVICE}"
    echo ">>> Stage 2 Complete -> Checkpoint: artifacts/finetune_10k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
}

run_dashboard() {
    echo ""
    echo ">>> [STAGE 3/3] Generating Interactive Before & After Visual Comparison Dashboard..."
    uv run python scripts/visualize_finetune_comparison.py \
        --before-checkpoint artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt \
        --after-checkpoint artifacts/finetune_10k_hbb/checkpoints/mobilenetv4_p2_hbb_finetune_last.pt \
        --summary-json artifacts/finetune_10k_hbb/training_hbb_finetune_summary.json \
        --output-html artifacts/finetune_10k_hbb/dashboard_finetune_comparison.html \
        --samples-per-cohort 15 \
        --device "${DEVICE}"
    echo ">>> Dashboard Generated -> file://${ROOT_DIR}/artifacts/finetune_10k_hbb/dashboard_finetune_comparison.html"
}

case "${STAGE}" in
    all)
        run_stage1
        run_stage2
        run_dashboard
        ;;
    stage1)
        run_stage1
        ;;
    stage2)
        run_stage2
        ;;
    dashboard)
        run_dashboard
        ;;
    *)
        echo "Unknown stage: '${STAGE}'. Usage: $0 [all | stage1 | stage2 | dashboard] [device] [batch_size]"
        exit 1
        ;;
esac

echo ""
echo "================================================================================"
echo "Pipeline execution finished successfully!"
echo "================================================================================"
