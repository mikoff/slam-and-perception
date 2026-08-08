#!/usr/bin/env bash
set -Eeuo pipefail

echo "=== [Cloud Training Remote Entrypoint] ==="

# 0. Ensure git is installed
if ! command -v git &> /dev/null; then
  echo "--> Installing git..."
  export DEBIAN_FRONTEND=noninteractive
  if command -v sudo &> /dev/null; then
    sudo -E apt-get update && sudo -E apt-get install -y git
  else
    apt-get update && apt-get install -y git
  fi
fi

# 0.1 Direct GitHub Checkout if requested
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
NVME_DATASET_PATH="${NVME_DATASET_PATH:-./dataset}"
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

# 0.5 Setup Python Virtual Environment
if ! command -v uv &> /dev/null; then
  echo "--> Installing uv..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
fi

if [[ ! -d ".venv" ]]; then
  echo "--> Installing dependencies via uv..."
  uv sync --frozen --group cloud
fi
export PATH="$PWD/.venv/bin:$PATH"

# 1. Dataset Staging from S3 to NVMe (On-The-Fly Streaming Tar Extraction or Sync)
if [[ -n "${S3_BUCKET:-}" ]]; then
  echo "--> Optimizing AWS CLI for high-concurrency multi-threaded S3 transfer (64 threads)..."
  aws configure set default.s3.max_concurrent_requests 64
  aws configure set default.s3.multipart_chunksize 64MB
  aws configure set default.s3.max_queue_size 10000

  echo "--> Attempting to stream dataset.tar.gz directly..."
  if aws s3 cp ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} "s3://${S3_BUCKET}/dataset/dataset.tar.gz" - | tar -xz -C "${NVME_DATASET_PATH}"; then
    echo "--> Successfully streamed dataset.tar.gz to NVMe."
  elif aws s3 cp ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} "s3://${S3_BUCKET}/dataset/dataset.tar" - | tar -x -C "${NVME_DATASET_PATH}"; then
    echo "--> Successfully streamed dataset.tar to NVMe."
  else
    echo "--> Staging dataset files from s3://${S3_BUCKET}/dataset to ${NVME_DATASET_PATH}..."
    aws s3 sync \
      ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} \
      "s3://${S3_BUCKET}/dataset" "${NVME_DATASET_PATH}" \
      --no-progress
  fi
fi




# 1.5 Symlink dataset to expected config path
# The config file phase3_attnres.yaml expects the dataset at ../../../data/visual-inference-datasets/output
# So we create a symlink to point it to the local NVMe dataset path.
echo "--> Symlinking dataset to expected config path..."
mkdir -p ../../data/visual-inference-datasets
ln -sfn "$(realpath "${NVME_DATASET_PATH}")" ../../data/visual-inference-datasets/output

# 2. Checksum Verification
if [[ -f "${NVME_DATASET_PATH}/checksums.sha256" ]]; then
  echo "--> Verifying dataset SHA256 checksums..."
  (cd "${NVME_DATASET_PATH}" && sha256sum -c checksums.sha256)
else
  echo "--> Warning: No checksums.sha256 found in ${NVME_DATASET_PATH}, skipping verification."
fi


# 4. W&B Login if credentials provided
if [[ -n "${WANDB_API_KEY:-}" ]]; then
  echo "--> Authenticating with Weights & Biases..."
  .venv/bin/python -m wandb login "${WANDB_API_KEY}"
fi

# 5. Launch Training
echo "--> Increasing file descriptor limit to prevent PyTorch OOM (Too many open files)..."
ulimit -n 65536 || true

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
if [[ -n "${BATCH_SIZE:-}" ]]; then
  echo "--> Overriding batch_size: ${BATCH_SIZE}"
  EXTRA_ARGS+=("--batch-size" "${BATCH_SIZE}")
else
  echo "--> Enabling dynamic VRAM & batch size autotuning for target GPU..."
  set +e
  AUTOTUNE_JSON=$(.venv/bin/python scripts/benchmark_quad_batch.py --config "${CONFIG_PATH}")
  set -e
  
  OPTIMAL_BATCH=$(echo "$AUTOTUNE_JSON" | .venv/bin/python -c "import sys, json; d=json.load(sys.stdin); print(d.get('recommended_candidate', {}).get('physical_batch_size', ''))" 2>/dev/null)
  
  if [ -z "$OPTIMAL_BATCH" ]; then
    echo "--> Autotune failed to find a valid candidate!"
    echo "$AUTOTUNE_JSON"
    exit 1
  fi
  
  echo "--> Best batch size determined by autotune: ${OPTIMAL_BATCH}"
  EXTRA_ARGS+=("--batch-size" "${OPTIMAL_BATCH}")
fi
if [[ -n "${WORKERS:-}" ]]; then
  echo "--> Overriding num_workers: ${WORKERS}"
  EXTRA_ARGS+=("--workers" "${WORKERS}")
fi
if [[ -n "${RESUME_PATH:-}" ]]; then
  if [[ "${RESUME_PATH}" == s3://* ]]; then
    echo "--> Downloading resume checkpoint from S3: ${RESUME_PATH}"
    mkdir -p /tmp/resume
    aws s3 cp "${RESUME_PATH}" /tmp/resume/checkpoint.pt ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"}
    EXTRA_ARGS+=("--resume" "/tmp/resume/checkpoint.pt")
  else
    echo "--> Resuming training from local checkpoint: ${RESUME_PATH}"
    EXTRA_ARGS+=("--resume" "${RESUME_PATH}")
  fi
elif [[ -f "${OUTPUT_DIR}/last.pt" ]]; then
  echo "--> Found existing last.pt in ${OUTPUT_DIR}, resuming..."
  EXTRA_ARGS+=("--resume" "${OUTPUT_DIR}/last.pt")
fi

if [[ -n "${EXTRA_TRAINING_ARGS:-}" ]]; then
  echo "--> [DEBUG] Injecting extra training arguments: ${EXTRA_TRAINING_ARGS}"
  read -ra EXTRA_PARSED <<< "$EXTRA_TRAINING_ARGS"
  EXTRA_ARGS+=("${EXTRA_PARSED[@]}")
fi

# 6. Periodic Background Checkpoint Sync & Watchdog Heartbeat
if [[ -n "${S3_BUCKET:-}" && -d "${OUTPUT_DIR}" ]]; then
  echo "--> Starting background periodic sync of checkpoints and watchdog heartbeat..."
  (
    LAST_SIZE=0
    UNCHANGED_COUNT=0
    
    while true; do
      sleep 300 # Run every 5 minutes
      
      # 1. Sync Checkpoints
      aws s3 sync \
        ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} \
        "${OUTPUT_DIR}" "s3://${S3_BUCKET}/runs/${RUN_NAME}" \
        --no-progress --quiet || true
        
      # 2. Watchdog Heartbeat
      if [[ -f /tmp/training.log ]]; then
        CURRENT_SIZE=$(stat -c%s "/tmp/training.log" 2>/dev/null || echo 0)
        
        if [[ "$CURRENT_SIZE" -gt "$LAST_SIZE" ]]; then
          LAST_SIZE=$CURRENT_SIZE
          UNCHANGED_COUNT=0
          
          # Ping Janitor to stay alive
          if [[ -n "${INSTANCE_ID:-}" ]]; then
            echo "{\"instance_id\": \"${INSTANCE_ID}\", \"provider\": \"${CLOUD_PROVIDER:-packet}\", \"timestamp\": $(date +%s)}" > /tmp/janitor_ping.json
            aws s3 cp /tmp/janitor_ping.json "s3://${S3_BUCKET}/janitor/${INSTANCE_ID}.json" ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} --quiet || true
          fi
        else
          UNCHANGED_COUNT=$((UNCHANGED_COUNT + 1))
          if [[ $UNCHANGED_COUNT -ge 6 ]]; then
            echo "--> 💀 WATCHDOG TIMEOUT: /tmp/training.log has not grown for 30 minutes! Aborting run..." >> /tmp/training.log
            pkill -f train_quad_proposals || true
            exit 1
          fi
        fi
      fi
    done
  ) &
  SYNC_PID=$!
fi

cleanup_and_terminate() {
  echo "--> Cleaning up background processes..."
  kill $SYNC_PID 2>/dev/null || true

  if [[ -n "${S3_BUCKET:-}" && -d "${OUTPUT_DIR}" ]]; then
    echo "--> Performing final backup of run artifacts & checkpoints before termination..."
    aws s3 sync \
      ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} \
      "${OUTPUT_DIR}" "s3://${S3_BUCKET}/runs/${RUN_NAME}" --no-progress || true
      
    if [[ -f /tmp/training.log ]]; then
      aws s3 cp /tmp/training.log "s3://${S3_BUCKET}/runs/${RUN_NAME}/training.log" ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} || true
    fi
  fi
  
  if [[ -n "${S3_BUCKET:-}" && -n "${INSTANCE_ID:-}" ]]; then
    echo "--> Deregistering instance from Janitor..."
    aws s3 rm "s3://${S3_BUCKET}/janitor/${INSTANCE_ID}.json" ${S3_ENDPOINT_URL:+--endpoint-url "${S3_ENDPOINT_URL}"} || true
  fi
  
  if [[ -n "${CLOUD_API_KEY:-}" && -n "${INSTANCE_ID:-}" ]]; then
    echo "--> Self-terminating instance ${INSTANCE_ID}..."
    export CLOUD_API_KEY="${CLOUD_API_KEY}"
    .venv/bin/python scripts/cloud/cli.py kill_instance --provider "${CLOUD_PROVIDER:-packet}" --instance-id "${INSTANCE_ID}" || true
  fi
}

# Ensure the background sync and instance are killed when this script exits (even on failure)
trap cleanup_and_terminate EXIT

echo "--> Starting quad proposal training via cloud daemon..."
.venv/bin/python scripts/cloud/run_cloud_daemon.py \
  --output-dir "${OUTPUT_DIR}" \
  --run-name "${RUN_NAME}" \
  .venv/bin/python scripts/train_quad_proposals.py \
  --config "${CONFIG_PATH}" \
  --output-dir "${OUTPUT_DIR}" \
  "${EXTRA_ARGS[@]}"



echo "=== [Training Run Finished Successfully] ==="

