#!/usr/bin/env bash
set -euo pipefail

echo "=========================================================="
echo "Starting Full 15-Epoch Training on 10k Dataset (HBB & QUAD)"
echo "Date: $(date)"
echo "=========================================================="

# 1. Train MobileNetV4 P2-HBB for 15 epochs
echo "[1/4] Training MobileNetV4 P2-HBB for 15 epochs..."
uv run python scripts/train_student_pilot.py \
    --geometry hbb \
    --recipe p2_baseline \
    --epochs 15 \
    --batch-size 64 \
    --lr 0.0020 \
    --pseudo-labels artifacts/pseudo_labels/sam_discovery.sqlite \
    --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
    --output-dir artifacts/train_10k_hbb

# 2. Train MobileNetV4 P2-QUAD for 15 epochs
echo "[2/4] Training MobileNetV4 P2-QUAD for 15 epochs..."
uv run python scripts/train_student_pilot.py \
    --geometry quad \
    --recipe p2_baseline \
    --epochs 15 \
    --batch-size 64 \
    --lr 0.0020 \
    --pseudo-labels artifacts/pseudo_labels/sam_discovery.sqlite \
    --val-pseudo-labels artifacts/phase1/pseudo_labels/sam_pilot_800.json \
    --output-dir artifacts/train_10k_quad

# 3. Generate HBB Dashboard
echo "[3/4] Generating HBB Interactive Dashboard..."
uv run python scripts/visualize_student_validation.py \
    --geometry hbb \
    --checkpoint artifacts/train_10k_hbb/checkpoints/mobilenetv4_p2_hbb_best.pt \
    --summary-json artifacts/train_10k_hbb/training_hbb_summary.json \
    --output-html artifacts/train_10k_hbb/dashboard_hbb.html \
    --samples-per-cohort 15 \
    --score-mode objectness \
    --centerness-alpha 0.0 \
    --nms-mode part_aware \
    --nms-iou-threshold 0.45 \
    --device cpu

# 4. Generate QUAD Dashboard
echo "[4/4] Generating QUAD Interactive Dashboard..."
uv run python scripts/visualize_student_validation.py \
    --geometry quad \
    --checkpoint artifacts/train_10k_quad/checkpoints/mobilenetv4_p2_quad_best.pt \
    --summary-json artifacts/train_10k_quad/training_quad_summary.json \
    --output-html artifacts/train_10k_quad/dashboard_quad.html \
    --samples-per-cohort 15 \
    --centerness-alpha 0.0 \
    --nms-mode part_aware \
    --nms-iou-threshold 0.45 \
    --device cpu

echo "=========================================================="
echo "15-Epoch Training and Visualization Dashboards Complete!"
echo "Date: $(date)"
echo "=========================================================="
