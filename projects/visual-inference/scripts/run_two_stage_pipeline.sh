#!/usr/bin/env bash
# run_two_stage_pipeline.sh: End-to-End Orchestrator for MobileNetV4 Two-Stage Training & Master Comparison.
set -euo pipefail

STAGE="${1:-all}"
GEOMETRY="${2:-both}"
DEVICE="${3:-cuda}"
BATCH_SIZE="${4:-64}"
RECIPE="${5:-p2_baseline}"
RUN_TAG="${6:-}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

TAG_SUFFIX=""
if [ -n "${RUN_TAG}" ]; then
    TAG_SUFFIX="_${RUN_TAG}"
fi

TRAIN_HBB_DIR="artifacts/train_10k_hbb${TAG_SUFFIX}"
TRAIN_QUAD_DIR="artifacts/train_10k_quad${TAG_SUFFIX}"
FINETUNE_HBB_DIR="artifacts/finetune_10k_hbb${TAG_SUFFIX}"
FINETUNE_QUAD_DIR="artifacts/finetune_10k_quad${TAG_SUFFIX}"
DASHBOARD_HTML="artifacts/dashboard_master_comparison${TAG_SUFFIX}.html"

echo "================================================================================"
echo "MobileNetV4 Two-Stage Pipeline Orchestrator (Unified HBB & Quads)"
echo "Stage Selection : ${STAGE} (options: all | stage1 | stage2 | dashboard | backup)"
echo "Geometry Target : ${GEOMETRY} (options: both | hbb | quad)"
echo "Device          : ${DEVICE}"
echo "Batch Size      : ${BATCH_SIZE}"
echo "Recipe          : ${RECIPE} (options: p2_baseline | recipe2)"
echo "Run Tag         : ${RUN_TAG:-<default> (targets: ${TRAIN_HBB_DIR}, ${FINETUNE_HBB_DIR})}"
echo "Working Dir     : ${ROOT_DIR}"
echo "================================================================================"

backup_artifacts() {
    local BACKUP_DIR="artifacts/backup_10k_run1"
    echo ""
    echo ">>> [BACKUP] Snapshotting current 10k artifacts to ${BACKUP_DIR}..."
    mkdir -p "${BACKUP_DIR}"
    if [ -d "artifacts/train_10k_hbb" ]; then cp -r artifacts/train_10k_hbb "${BACKUP_DIR}/"; fi
    if [ -d "artifacts/train_10k_quad" ]; then cp -r artifacts/train_10k_quad "${BACKUP_DIR}/"; fi
    if [ -d "artifacts/finetune_10k_hbb" ]; then cp -r artifacts/finetune_10k_hbb "${BACKUP_DIR}/"; fi
    if [ -d "artifacts/finetune_10k_quad" ]; then cp -r artifacts/finetune_10k_quad "${BACKUP_DIR}/"; fi
    if [ -f "artifacts/dashboard_master_comparison.html" ]; then cp artifacts/dashboard_master_comparison.html "${BACKUP_DIR}/"; fi
    echo ">>> Backup complete: $(du -sh "${BACKUP_DIR}")"
}

run_stage1_hbb() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    echo ""
    echo ">>> [STAGE 1] Discovery Pre-Training (HBB) (15 Epochs on 10k SAM Dataset, Recipe: ${RECIPE})..."
    uv run python scripts/train_student_pilot.py \
        --geometry hbb \
        --recipe "${RECIPE}" \
        --epochs 15 \
        --batch-size "${BATCH_SIZE}" \
        --lr 0.0020 \
        --cohort-size 10000 \
        --pseudo-labels artifacts/pseudo_labels/sam_discovery.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir "${TRAIN_HBB_DIR}" \
        --device "${DEVICE}"
    echo ">>> Stage 1 HBB Complete -> ${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
}

run_stage1_quad() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    echo ""
    echo ">>> [STAGE 1] Discovery Pre-Training (QUAD) (15 Epochs on 10k SAM Dataset, Recipe: ${RECIPE})..."
    uv run python scripts/train_student_pilot.py \
        --geometry quad \
        --recipe "${RECIPE}" \
        --epochs 15 \
        --batch-size "${BATCH_SIZE}" \
        --lr 0.0020 \
        --cohort-size 10000 \
        --pseudo-labels artifacts/pseudo_labels/sam_discovery.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir "${TRAIN_QUAD_DIR}" \
        --device "${DEVICE}"
    echo ">>> Stage 1 Quad Complete -> ${TRAIN_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
}

run_stage2_hbb() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    local INIT_CKPT="${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
    echo ""
    echo ">>> [STAGE 2] Human GT Fine-Tuning (HBB) (5 Epochs with L1+CIoU Tightening from ${INIT_CKPT})..."
    uv run python scripts/finetune_student_gt.py \
        --geometry hbb \
        --init-checkpoint "${INIT_CKPT}" \
        --cohort-db artifacts/pseudo_labels/sam_discovery.sqlite \
        --gt-index /home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir "${FINETUNE_HBB_DIR}" \
        --epochs 5 \
        --cohort-size 10000 \
        --batch-size "${BATCH_SIZE}" \
        --lr 3e-4 \
        --box-weight 2.0 \
        --ltrb-weight 0.5 \
        --weak-negative-weight 0.15 \
        --device "${DEVICE}"
    echo ">>> Stage 2 HBB Complete -> ${FINETUNE_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt"
}

run_stage2_quad() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    local INIT_CKPT="${TRAIN_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
    echo ""
    echo ">>> [STAGE 2] Human GT Fine-Tuning (QUAD) (5 Epochs with Min-8 L1 + GWD from ${INIT_CKPT})..."
    uv run python scripts/finetune_student_gt.py \
        --geometry quad \
        --init-checkpoint "${INIT_CKPT}" \
        --cohort-db artifacts/pseudo_labels/sam_discovery.sqlite \
        --gt-index /home/sashamikoff/git/skillup/data/visual-inference-datasets/output/indexes/quad_train.sqlite \
        --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
        --output-dir "${FINETUNE_QUAD_DIR}" \
        --epochs 5 \
        --cohort-size 10000 \
        --batch-size "${BATCH_SIZE}" \
        --lr 3e-4 \
        --corner-weight 2.0 \
        --gwd-weight 2.0 \
        --weak-negative-weight 0.15 \
        --device "${DEVICE}"
    echo ">>> Stage 2 Quad Complete -> ${FINETUNE_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
}

run_dashboard() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    echo ""
    echo ">>> [DASHBOARD] Generating Unified Master Comparison Dashboard (HBB & Quads, Before/After & Side-by-Side)..."
    uv run python scripts/visualize_student_comparison.py \
        --hbb-before "${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt" \
        --hbb-after "${FINETUNE_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt" \
        --quad-before "${TRAIN_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt" \
        --quad-after "${FINETUNE_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad_finetune_best.pt" \
        --hbb-summary "${FINETUNE_HBB_DIR}/training_hbb_finetune_summary.json" \
        --quad-summary "${FINETUNE_QUAD_DIR}/training_quad_finetune_summary.json" \
        --output-html "${DASHBOARD_HTML}" \
        --samples-per-cohort 15 \
        --device "${DEVICE}"
    echo ">>> Master Dashboard Generated -> file://${ROOT_DIR}/${DASHBOARD_HTML}"
}

run_stage1() {
    if [ "${GEOMETRY}" = "both" ] || [ "${GEOMETRY}" = "hbb" ]; then
        run_stage1_hbb
    fi
    if [ "${GEOMETRY}" = "both" ] || [ "${GEOMETRY}" = "quad" ]; then
        run_stage1_quad
    fi
}

run_stage2() {
    if [ "${GEOMETRY}" = "both" ] || [ "${GEOMETRY}" = "hbb" ]; then
        run_stage2_hbb
    fi
    if [ "${GEOMETRY}" = "both" ] || [ "${GEOMETRY}" = "quad" ]; then
        run_stage2_quad
    fi
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
    backup)
        backup_artifacts
        ;;
    *)
        echo "Unknown stage: '${STAGE}'. Usage: $0 [all | stage1 | stage2 | dashboard | backup] [both | hbb | quad] [device] [batch_size] [recipe] [tag]"
        exit 1
        ;;
esac

echo ""
echo "================================================================================"
echo "Pipeline execution finished successfully!"
echo "================================================================================"
