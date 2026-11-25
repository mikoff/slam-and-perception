#!/usr/bin/env bash
# run_two_stage_pipeline.sh: End-to-End Orchestrator for MobileNetV4 Two-Stage Training & Master Comparison.
set -euo pipefail

STAGE="${1:-all}"
GEOMETRY="${2:-both}"
DEVICE="${3:-cuda}"
BATCH_SIZE="${4:-64}"
RECIPE="${5:-p2_baseline}"
RUN_TAG="${6:-}"

# If 5th argument is not a known recipe name, treat it as RUN_TAG with default recipe
if [ "${RECIPE}" != "p2_baseline" ] && [ "${RECIPE}" != "recipe2" ] && [ -z "${RUN_TAG}" ]; then
    RUN_TAG="${RECIPE}"
    RECIPE="p2_baseline"
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/../.." && pwd)"
DATA_ROOT="${VISUAL_INFERENCE_DATA_ROOT:-${REPO_ROOT}/data/visual-inference-datasets}"
cd "${ROOT_DIR}"

TAG_SUFFIX=""
if [ -n "${RUN_TAG}" ]; then
    TAG_SUFFIX="_${RUN_TAG}"
fi

COHORT_SIZE="${COHORT_SIZE:-10000}"
GT_INDEX_OVERRIDE="${GT_INDEX:-}"
GT_INDEX="${GT_INDEX_OVERRIDE:-${DATA_ROOT}/output/indexes/quad_train.sqlite}"
PSEUDO_LABELS_DB="${PSEUDO_LABELS_DB:-artifacts/pseudo_labels/sam_discovery.sqlite}"
VALIDATION_PSEUDO_LABELS="${VALIDATION_PSEUDO_LABELS:-artifacts/phase1/pseudo_labels/sam_pilot_800.json}"
PIPELINE_VAL_RATIO="${PIPELINE_VAL_RATIO:-0.10}"
PIPELINE_SPLIT_SEED="${PIPELINE_SPLIT_SEED:-42}"
COHORT_TAG="10k"
STAGE1_EPOCHS="${STAGE1_EPOCHS:-10}"
STAGE2_EPOCHS="${STAGE2_EPOCHS:-10}"
STAGE1_VAL_INTERVAL="${STAGE1_VAL_INTERVAL:-${VAL_INTERVAL:-${STAGE1_EPOCHS}}}"
STAGE2_VAL_INTERVAL="${STAGE2_VAL_INTERVAL:-${VAL_INTERVAL:-${STAGE2_EPOCHS}}}"

if [ "${STAGE}" = "stage1_enriched" ] || [ "${STAGE}" = "stage2_enriched" ] || [ "${STAGE}" = "all_enriched" ] || [ "${STAGE}" = "dashboard_enriched" ]; then
    COHORT_SIZE=14000
    GT_INDEX="${GT_INDEX_OVERRIDE:-${DATA_ROOT}/output/indexes/quad_train_enriched.sqlite}"
    PSEUDO_LABELS_DB="artifacts/pseudo_labels/sam_discovery_enriched.sqlite"
    COHORT_TAG="14k"
elif [ "${COHORT_SIZE}" -eq 8000 ]; then
    COHORT_TAG="8k"
    GT_INDEX="${GT_INDEX_OVERRIDE:-${DATA_ROOT}/output/indexes/quad_train_enriched.sqlite}"
    PSEUDO_LABELS_DB="artifacts/pseudo_labels/sam_discovery_enriched.sqlite"
elif [ "${COHORT_SIZE}" -gt 10000 ]; then
    COHORT_TAG="14k"
    PSEUDO_LABELS_DB="artifacts/pseudo_labels/sam_discovery_enriched.sqlite"
fi

TRAIN_HBB_DIR="artifacts/train_${COHORT_TAG}_hbb${TAG_SUFFIX}"
TRAIN_QUAD_DIR="artifacts/train_${COHORT_TAG}_quad${TAG_SUFFIX}"
FINETUNE_HBB_DIR="artifacts/finetune_${COHORT_TAG}_hbb${TAG_SUFFIX}"
FINETUNE_QUAD_DIR="artifacts/finetune_${COHORT_TAG}_quad${TAG_SUFFIX}"
DASHBOARD_HTML="artifacts/dashboard_master_comparison_${COHORT_TAG}${TAG_SUFFIX}.html"

echo "================================================================================"
echo "MobileNetV4 Two-Stage Pipeline Orchestrator (Unified HBB & Quads)"
echo "Stage Selection : ${STAGE} (options: all | all_enriched | stage1 | stage1_enriched | stage2 | stage2_enriched | dashboard | backup)"
echo "Cohort Size     : ${COHORT_SIZE} (tag: ${COHORT_TAG}, index: ${GT_INDEX})"
echo "Pseudo Labels   : ${PSEUDO_LABELS_DB}"
echo "Validation Data : ${VALIDATION_PSEUDO_LABELS}"
echo "Pipeline Split  : validation ratio ${PIPELINE_VAL_RATIO}, seed ${PIPELINE_SPLIT_SEED}"
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

run_isolation_audit() {
    local AUDIT_PATH="artifacts/training_isolation_${COHORT_TAG}${TAG_SUFFIX}.json"
    local DASHBOARD_MANIFEST="${EVALUATION_MANIFEST:-artifacts/dashboard_holdout_${COHORT_TAG}${TAG_SUFFIX}.json}"
    if [ ! -f "${DASHBOARD_MANIFEST}" ]; then
        echo ""
        echo ">>> [PREFLIGHT] Preparing validation holdout manifest for dashboard: ${DASHBOARD_MANIFEST}..."
        uv run python scripts/prepare_dashboard_holdout.py \
            --gt-index "${GT_INDEX}" \
            --cohort-size "${COHORT_SIZE}" \
            --samples-per-cohort "${SAMPLES_PER_COHORT:-35}" \
            --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
            --split-seed "${PIPELINE_SPLIT_SEED}" \
            --output "${DASHBOARD_MANIFEST}"
    fi
    local DASHBOARD_ARGS=()
    if [ -f "${DASHBOARD_MANIFEST}" ]; then
        DASHBOARD_ARGS+=(--dashboard-manifest "${DASHBOARD_MANIFEST}")
    fi
    echo ""
    echo ">>> [PREFLIGHT] Verifying pipeline-wide train/validation isolation..."
    uv run python scripts/audit_training_isolation.py \
        --cohort-db "${PSEUDO_LABELS_DB}" \
        --gt-index "${GT_INDEX}" \
        --cohort-size "${COHORT_SIZE}" \
        --val-pseudo-labels "${VALIDATION_PSEUDO_LABELS}" \
        --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
        --split-seed "${PIPELINE_SPLIT_SEED}" \
        --output "${AUDIT_PATH}" \
        "${DASHBOARD_ARGS[@]}"
    echo ">>> Isolation audit passed -> ${AUDIT_PATH}"
}

run_stage1_hbb() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    echo ""
    echo ">>> [STAGE 1] Discovery Pre-Training (HBB) (${STAGE1_EPOCHS} Epochs on ${COHORT_SIZE} SAM Dataset, Recipe: ${RECIPE}, Val Interval: ${STAGE1_VAL_INTERVAL})..."
    uv run python scripts/train_student_pilot.py \
        --geometry hbb \
        --recipe "${RECIPE}" \
        --epochs "${STAGE1_EPOCHS}" \
        --val-interval "${STAGE1_VAL_INTERVAL}" \
        --batch-size "${BATCH_SIZE}" \
        --lr 0.0020 \
        --cohort-size "${COHORT_SIZE}" \
        --gt-index "${GT_INDEX}" \
        --pseudo-labels "${PSEUDO_LABELS_DB}" \
        --val-pseudo-labels "${VALIDATION_PSEUDO_LABELS}" \
        --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
        --split-seed "${PIPELINE_SPLIT_SEED}" \
        --output-dir "${TRAIN_HBB_DIR}" \
        --device "${DEVICE}"
    echo ">>> Stage 1 HBB Complete -> ${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
}

run_stage1_quad() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    echo ""
    echo ">>> [STAGE 1] Discovery Pre-Training (QUAD) (${STAGE1_EPOCHS} Epochs on ${COHORT_SIZE} SAM Dataset, Recipe: ${RECIPE}, Val Interval: ${STAGE1_VAL_INTERVAL})..."
    uv run python scripts/train_student_pilot.py \
        --geometry quad \
        --recipe "${RECIPE}" \
        --epochs "${STAGE1_EPOCHS}" \
        --val-interval "${STAGE1_VAL_INTERVAL}" \
        --batch-size "${BATCH_SIZE}" \
        --lr 0.0020 \
        --cohort-size "${COHORT_SIZE}" \
        --gt-index "${GT_INDEX}" \
        --pseudo-labels "${PSEUDO_LABELS_DB}" \
        --val-pseudo-labels "${VALIDATION_PSEUDO_LABELS}" \
        --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
        --split-seed "${PIPELINE_SPLIT_SEED}" \
        --output-dir "${TRAIN_QUAD_DIR}" \
        --device "${DEVICE}"
    echo ">>> Stage 1 Quad Complete -> ${TRAIN_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
}

run_stage2_hbb() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    local INIT_CKPT="${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
    if [ ! -f "${INIT_CKPT}" ] && [ -f "artifacts/train_${COHORT_TAG}_hbb/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt" ]; then
        INIT_CKPT="artifacts/train_${COHORT_TAG}_hbb/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
    fi
    if [ ! -f "${INIT_CKPT}" ] && [ -f "artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt" ]; then
        INIT_CKPT="artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"
    fi
    local SCRATCH_ARGS=(--init-checkpoint "${INIT_CKPT}" --lr 3e-4)
    if [ "${FROM_SCRATCH:-0}" = "1" ]; then
        SCRATCH_ARGS=(--from-scratch --lr 1.5e-3)
    fi
    echo ""
    echo ">>> [STAGE 2] Human GT Training (HBB) (${STAGE2_EPOCHS} Epochs, Scratch=${FROM_SCRATCH:-0}, Val Interval: ${STAGE2_VAL_INTERVAL})..."
    uv run python scripts/finetune_student_gt.py \
        --geometry hbb \
        "${SCRATCH_ARGS[@]}" \
        --cohort-db "${PSEUDO_LABELS_DB}" \
        --gt-index "${GT_INDEX}" \
        --val-pseudo-labels "${VALIDATION_PSEUDO_LABELS}" \
        --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
        --split-seed "${PIPELINE_SPLIT_SEED}" \
        --output-dir "${FINETUNE_HBB_DIR}" \
        --epochs "${STAGE2_EPOCHS}" \
        --val-interval "${STAGE2_VAL_INTERVAL}" \
        --cohort-size "${COHORT_SIZE}" \
        --batch-size "${BATCH_SIZE}" \
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
    if [ ! -f "${INIT_CKPT}" ] && [ -f "artifacts/train_${COHORT_TAG}_quad/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt" ]; then
        INIT_CKPT="artifacts/train_${COHORT_TAG}_quad/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
    fi
    if [ ! -f "${INIT_CKPT}" ] && [ -f "artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt" ]; then
        INIT_CKPT="artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt"
    fi
    local SCRATCH_ARGS=(--init-checkpoint "${INIT_CKPT}" --lr 3e-4)
    if [ "${FROM_SCRATCH:-0}" = "1" ]; then
        SCRATCH_ARGS=(--from-scratch --lr 1.5e-3)
    fi
    echo ""
    echo ">>> [STAGE 2] Human GT Training (QUAD) (${STAGE2_EPOCHS} Epochs, Scratch=${FROM_SCRATCH:-0}, Val Interval: ${STAGE2_VAL_INTERVAL})..."
    uv run python scripts/finetune_student_gt.py \
        --geometry quad \
        "${SCRATCH_ARGS[@]}" \
        --cohort-db "${PSEUDO_LABELS_DB}" \
        --gt-index "${GT_INDEX}" \
        --val-pseudo-labels "${VALIDATION_PSEUDO_LABELS}" \
        --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
        --split-seed "${PIPELINE_SPLIT_SEED}" \
        --output-dir "${FINETUNE_QUAD_DIR}" \
        --epochs "${STAGE2_EPOCHS}" \
        --val-interval "${STAGE2_VAL_INTERVAL}" \
        --cohort-size "${COHORT_SIZE}" \
        --batch-size "${BATCH_SIZE}" \
        --corner-weight 2.0 \
        --gwd-weight 2.0 \
        --weak-negative-weight 0.15 \
        --device "${DEVICE}"
    echo ">>> Stage 2 Quad Complete -> ${FINETUNE_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad_finetune_best.pt"
}

run_dashboard() {
    local SUFFIX=""
    if [ "${RECIPE}" = "recipe2" ]; then SUFFIX="_recipe2"; fi
    local HBB_BEFORE="${TRAIN_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
    if [ ! -f "${HBB_BEFORE}" ] && [ -f "artifacts/train_${COHORT_TAG}_hbb/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt" ]; then
        HBB_BEFORE="artifacts/train_${COHORT_TAG}_hbb/checkpoints/mobilenetv4_p2_hbb${SUFFIX}_best.pt"
    fi
    if [ ! -f "${HBB_BEFORE}" ] && [ -f "artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt" ]; then
        HBB_BEFORE="artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt"
    fi
    local QUAD_BEFORE="${TRAIN_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
    if [ ! -f "${QUAD_BEFORE}" ] && [ -f "artifacts/train_${COHORT_TAG}_quad/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt" ]; then
        QUAD_BEFORE="artifacts/train_${COHORT_TAG}_quad/checkpoints/mobilenetv4_p2_quad${SUFFIX}_best.pt"
    fi
    if [ ! -f "${QUAD_BEFORE}" ] && [ -f "artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt" ]; then
        QUAD_BEFORE="artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt"
    fi
    echo ""
    local EVAL_MANIFEST="${EVALUATION_MANIFEST:-artifacts/dashboard_holdout_${COHORT_TAG}${TAG_SUFFIX}.json}"
    if [ ! -f "${EVAL_MANIFEST}" ]; then
        uv run python scripts/prepare_dashboard_holdout.py \
            --gt-index "${GT_INDEX}" --cohort-size "${COHORT_SIZE}" \
            --samples-per-cohort "${SAMPLES_PER_COHORT:-35}" \
            --pipeline-val-ratio "${PIPELINE_VAL_RATIO}" \
            --split-seed "${PIPELINE_SPLIT_SEED}" \
            --output "${EVAL_MANIFEST}"
    fi
    echo ">>> [DASHBOARD] Generating comparison on audited unseen images..."
    uv run python scripts/visualize_student_comparison.py \
        --hbb-before "${HBB_BEFORE}" \
        --hbb-after "${FINETUNE_HBB_DIR}/checkpoints/mobilenetv4_p2_hbb_finetune_best.pt" \
        --quad-before "${QUAD_BEFORE}" \
        --quad-after "${FINETUNE_QUAD_DIR}/checkpoints/mobilenetv4_p2_quad_finetune_best.pt" \
        --hbb-summary "${FINETUNE_HBB_DIR}/training_hbb_finetune_summary.json" \
        --quad-summary "${FINETUNE_QUAD_DIR}/training_quad_finetune_summary.json" \
        --evaluation-manifest "${EVAL_MANIFEST}" \
        --output-html "${DASHBOARD_HTML}" \
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
    all | all_enriched | stage1 | stage1_enriched | stage2 | stage2_enriched)
        run_isolation_audit
        ;;
esac

case "${STAGE}" in
    all)
        run_stage1
        run_stage2
        run_dashboard
        ;;
    all_enriched)
        run_stage1
        run_stage2
        run_dashboard
        ;;
    stage1)
        run_stage1
        ;;
    stage1_enriched)
        run_stage1
        ;;
    stage2)
        run_stage2
        ;;
    stage2_enriched)
        run_stage2
        ;;
    dashboard | dashboard_enriched)
        run_dashboard
        ;;
    backup)
        backup_artifacts
        ;;
    *)
        echo "Unknown stage: '${STAGE}'. Usage: $0 [all | all_enriched | stage1 | stage2 | stage2_enriched | dashboard | backup] [both | hbb | quad] [device] [batch_size] [recipe] [tag]"
        exit 1
        ;;
esac

echo ""
echo "================================================================================"
echo "Pipeline execution finished successfully!"
echo "================================================================================"
