#!/usr/bin/env bash
set -Eeuo pipefail

echo "=== [Cloud Training Remote Entrypoint] ==="

# 0. Direct GitHub Checkout if requested
if [[ -n "${REPO_URL:-}" ]]; then
  echo "--> Cloning repository directly from GitHub: ${REPO_URL} (${COMMIT_SHA:-main})..."
  WORK_DIR="/tmp/workspace_$(date +%s)"
  git clone "${REPO_URL}" "${WORK_DIR}"
  cd "${WORK_DIR}"
  if [[ -n "${COMMIT_SHA:-}" ]]; then
    git checkout "${COMMIT_SHA}"
  fi
  if [[ -d "projects/visual-inference" ]]; then
    cd projects/visual-inference
  fi
fi

# Environment defaults
CONFIG_PATH="${CONFIG_PATH:-configs/phase3_attnres.yaml}"
NVME_DATASET_PATH="${NVME_DATASET_PATH:-/local/nvme/dataset}"
NFS_MOUNT_PATH="${NFS_MOUNT_PATH:-/mnt/nfs}"
RUN_NAME="${WANDB_RUN_NAME:-vi_quad_run_$(date +%Y%m%d_%H%M%S)}"

# AWS / S3 Credentials setup for private buckets (e.g. Backblaze B2)
export AWS_ACCESS_KEY_ID="${AWS_ACCESS_KEY_ID:-}"
export AWS_SECRET_ACCESS_KEY="${AWS_SECRET_ACCESS_KEY:-}"
export AWS_DEFAULT_REGION="${AWS_DEFAULT_REGION:-us-east-1}"

if [[ -d "${NFS_MOUNT_PATH}" ]]; then
  OUTPUT_DIR="${NFS_MOUNT_PATH}/runs/${RUN_NAME}"
else
  echo "--> Note: ${NFS_MOUNT_PATH} is not mounted. Saving checkpoints to local instance output directory."
  OUTPUT_DIR="./runs/${RUN_NAME}"
fi

mkdir -p "${NVME_DATASET_PATH}" "${OUTPUT_DIR}"

echo "--> Target NVMe dataset path: ${NVME_DATASET_PATH}"
echo "--> Output NFS checkpoint path: ${OUTPUT_DIR}"

# 1. Dataset Staging from S3 to NVMe (On-The-Fly Streaming Tar Extraction or Sync)
if [[ -n "${S3_BUCKET:-}" ]]; then
  echo "--> Optimizing AWS CLI for high-concurrency multi-threaded S3 transfer (64 threads)..."
  aws configure set default.s3.max_concurrent_requests 64
  aws configure set default.s3.multipart_chunksize 64MB
  aws configure set default.s3.max_queue_size 10000

  # Check if a single dataset.tar exists for zero-disk-overhead streaming extraction
  if aws s3 ls ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} "s3://${S3_BUCKET}/dataset/dataset.tar" >/dev/null 2>&1; then
    echo "--> Streaming s3://${S3_BUCKET}/dataset/dataset.tar on-the-fly directly to NVMe (${NVME_DATASET_PATH})..."
    aws s3 cp ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} "s3://${S3_BUCKET}/dataset/dataset.tar" - \
      | tar -x -C "${NVME_DATASET_PATH}"
  else
    echo "--> Staging dataset files from s3://${S3_BUCKET}/dataset to ${NVME_DATASET_PATH}..."
    aws s3 sync \
      ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} \
      "s3://${S3_BUCKET}/dataset" "${NVME_DATASET_PATH}" \
      --no-progress
  fi
fi



# 2. Checksum Verification
if [[ -f "${NVME_DATASET_PATH}/checksums.sha256" ]]; then
  echo "--> Verifying dataset SHA256 checksums..."
  (cd "${NVME_DATASET_PATH}" && sha256sum -c checksums.sha256)
else
  echo "--> Warning: No checksums.sha256 found in ${NVME_DATASET_PATH}, skipping verification."
fi

# 3. Setup Python Virtual Environment
if [[ ! -d ".venv" ]]; then
  echo "--> Installing dependencies via uv..."
  uv sync --frozen --group cloud
fi

# 4. W&B Login if credentials provided
if [[ -n "${WANDB_API_KEY:-}" ]]; then
  echo "--> Authenticating with Weights & Biases..."
  .venv/bin/python -m wandb login "${WANDB_API_KEY}"
fi

# 5. Launch Training
echo "--> Starting quad proposal training..."
EXTRA_ARGS=()
if [[ -n "${WANDB_PROJECT:-}" ]]; then
  EXTRA_ARGS+=("--wandb-project" "${WANDB_PROJECT}")
fi
if [[ -n "${WANDB_ENTITY:-}" ]]; then
  EXTRA_ARGS+=("--wandb-entity" "${WANDB_ENTITY}")
fi
if [[ -n "${WANDB_RUN_NAME:-}" ]]; then
  EXTRA_ARGS+=("--wandb-run-name" "${WANDB_RUN_NAME}")
fi
if [[ -n "${RESUME_PATH:-}" ]]; then
  echo "--> Resuming training from checkpoint: ${RESUME_PATH}"
  EXTRA_ARGS+=("--resume" "${RESUME_PATH}")
elif [[ -f "${OUTPUT_DIR}/last.pt" ]]; then
  echo "--> Found existing last.pt in ${OUTPUT_DIR}, resuming..."
  EXTRA_ARGS+=("--resume" "${OUTPUT_DIR}/last.pt")
fi

.venv/bin/python scripts/train_quad_proposals.py \
  --config "${CONFIG_PATH}" \
  --output-dir "${OUTPUT_DIR}" \
  "${EXTRA_ARGS[@]}"

# 6. Backup Checkpoints to S3 Object Storage
if [[ -n "${S3_BUCKET:-}" && -d "${OUTPUT_DIR}" ]]; then
  echo "--> Backing up run artifacts & checkpoints to s3://${S3_BUCKET}/runs/${RUN_NAME}..."
  aws s3 sync "${OUTPUT_DIR}" "s3://${S3_BUCKET}/runs/${RUN_NAME}" --no-progress
fi

echo "=== [Training Run Finished Successfully] ==="

