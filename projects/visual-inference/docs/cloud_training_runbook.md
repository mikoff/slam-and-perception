# Provider-Agnostic Cloud GPU Training Runbook

This guide explains how to prepare, launch, monitor, and retrieve results from the single-GPU cloud training pipeline for `visual-inference` supporting **RunPod** (default) and **Packet.ai**.

---

## Overview

The lean cloud training pipeline automates full-scale training on cloud GPUs using GitHub Actions.

1. **Sanity Testing**: Runs the local unit test suite before spending cloud credits.
2. **GPU Provisioning**: Provisions a cloud GPU instance on **RunPod** or **Packet.ai** (e.g. RTX 4090, A100, or L40S) and attaches your persistent network/NFS volume.
3. **Zero-Local-Bandwidth Checkout**: The GPU instance directly `git clone`s your committed repository from GitHub at datacenter speeds (1 Gbps+), saving your local internet connection.
4. **Dataset Staging**: Downloads the production dataset from private S3 to local NVMe via high-speed cloud-to-cloud `aws s3 sync` and verifies SHA256 checksums.
5. **Training Execution**: Runs `train_quad_proposals.py` with automatic AMP, progress metrics, and W&B logging.
6. **Durable Checkpointing**: Saves checkpoints (`last.pt`, `best.pt`) directly to the mounted persistent network volume (`/mnt/nfs/runs/<run_name>/` or `/workspace/runs/<run_name>/`).
7. **Guaranteed Cleanup**: Automatically terminates the GPU instance on success, failure, or job cancellation.

---

## Prerequisite Setup

Before running training for the first time, complete these initial setup steps.

### 1. Cloud Accounts & Identifiers

Ensure you have created and obtained the following:
- **Cloud Account**: An active account with API access on **RunPod** (recommended) or Packet.ai.
- **API Key**: Generated from your cloud provider dashboard (`CLOUD_API_KEY`).
- **Persistent Volume ID**: A persistent volume created on your provider (e.g., 50 GB+) to hold your training run checkpoints and caches across instances (`CLOUD_PERSISTENT_VOLUME_ID`).
- **SSH Keypair**: An SSH key registered with your cloud account (`CLOUD_SSH_KEY_ID` and `CLOUD_SSH_PRIVATE_KEY`).
- **Private S3 Bucket**: S3-compatible storage containing your uploaded dataset.
- **Weights & Biases Account**: An API key and project name for monitoring runs.

### 2. GitHub Environment Secrets Setup

Go to your GitHub repository: **Settings** → **Environments** → **New environment** → Name it `cloud-production`.

Add the following **Environment Secrets**:

| Secret Name | Description | Default / Example |
|---|---|---|
| `CLOUD_PROVIDER` | Cloud Provider implementation (`runpod` or `packet`) | `runpod` |
| `CLOUD_API_KEY` | Your Cloud Provider (RunPod / Packet) API key | *(Required)* |
| `CLOUD_PERSISTENT_VOLUME_ID` | (Optional) The ID of your pre-created network volume | e.g. `vol_xyz123` |
| `CLOUD_SSH_KEY_ID` | The registered SSH key ID on your cloud provider | e.g. `key_abc456` |
| `CLOUD_SSH_PRIVATE_KEY` | The private key string corresponding to the registered SSH key | `-----BEGIN OPENSSH...` |
| `CLOUD_CONTAINER_IMAGE` | (Optional) Docker container image for pod boot | `runpod/pytorch:2.5.1-py3.11-cuda12.4.1-devel-ubuntu22.04` |
| `CLOUD_API_URL` | (Optional) Base API URL endpoint override | `https://api.runpod.io/graphql` |
| `S3_BUCKET` | The name of your private S3 / Backblaze B2 dataset bucket | e.g. `my-dataset-bucket` |
| `S3_ENDPOINT_URL` | Your S3 endpoint URL | e.g. `https://s3.us-west-004.backblazeb2.com` |
| `AWS_ACCESS_KEY_ID` | Access key / keyID for S3 authentication | e.g. Backblaze Key ID |
| `AWS_SECRET_ACCESS_KEY` | Secret access key for S3 authentication | e.g. Backblaze Application Key |
| `WANDB_API_KEY` | Your Weights & Biases API Key | *(Required for W&B)* |
| `WANDB_PROJECT` | W&B project name | e.g. `visual-inference` |
| `WANDB_ENTITY` | W&B username or organization entity name | e.g. `my-team` |


---

## Dataset Preparation & Upload (Workstation)

Upload your raw image archives, JSON annotations, pre-built SQLite indexes, and a `checksums.sha256` manifest to your S3 bucket under `s3://<S3_BUCKET>/dataset/`.

### Expected S3 Bucket Structure
```text
s3://<S3_BUCKET>/dataset/
├── images_train.tar (or images/ folder)
├── images_val.tar
├── proposals_train.json.zst (or proposals_train.json)
├── proposals_val.json.zst (or proposals_val.json)
├── proposals_train.sqlite
├── proposals_val.sqlite
└── checksums.sha256
```

### Option A: On-The-Fly Streaming Tar Upload (Recommended — Zero Temp Disk Overhead)
Stream-compress local dataset directly into your Cloudflare R2 / S3 bucket without creating a temporary `.tar` file on workstation disk:

```bash
tar -ch -C /path/to/local/data . | aws s3 cp - s3://YOUR_BUCKET_NAME/dataset/dataset.tar \
  --endpoint-url https://<ACCOUNT_ID>.r2.cloudflarestorage.com \
  --expected-size 107374182400
```
> **Note on Symlinks**: The `-h` (`--dereference`) flag forces `tar` to follow symlinks and archive the actual underlying target files. This ensures your remote GPU instance receives complete image/data files rather than broken symlink pointers.


### Option B: Directory Sync
```bash
aws s3 sync /path/to/local/data s3://YOUR_BUCKET_NAME/dataset \
  --endpoint-url https://YOUR_S3_ENDPOINT \
  --follow-symlinks
```


---

## (Optional) Hardware Batch Benchmarking

Before initiating long training runs on a new GPU model, you can run the benchmark tool to determine the fastest physical batch size and required gradient accumulation steps for effective batch 64:

```bash
python scripts/benchmark_quad_batch.py \
  --config configs/phase3_attnres.yaml \
  --device cuda \
  --batch-sizes "16,32,64" \
  --effective-batch-size 64
```

---

## How to Run Cloud Training

### Method A: Via GitHub Actions (Recommended)

1. Go to your GitHub Repository tab: **Actions**.
2. Select the workflow **"Cloud Production Training (Lean)"** in the left sidebar.
3. Click the **"Run workflow"** dropdown button on the right side.
4. Configure the run parameters:
   - **Cloud GPU Provider**: `runpod` (default) or `packet`.
   - **GPU Type**: Select `rtx4090` (default), `a100`, or `l40s`.
   - **Container Image**: Default Ubuntu 22.04 + PyTorch 2.4 image (`runpod/pytorch:2.4.0-py3.11-cuda12.4.1-devel-ubuntu22.04`).
   - **Training Configuration Path**: Default `configs/phase3_attnres.yaml`.
   - **W&B Run Name (optional)**: e.g. `phase3_attnres_run1`.
5. Click **"Run workflow"**.

---

### Method B: Via Workstation CLI (Manual Control)

If you prefer launching training directly from your terminal:

#### Step 1: Provision the GPU Instance
```bash
export CLOUD_PROVIDER="runpod"
export CLOUD_API_KEY="your_api_key"
export CLOUD_PERSISTENT_VOLUME_ID="your_volume_id"
export CLOUD_SSH_KEY_ID="your_ssh_key_id"

python scripts/cloud_orchestrator.py launch --provider runpod --gpu-type rtx4090 > instance.json
INSTANCE_ID=$(jq -r .instance_id instance.json)
IP_ADDRESS=$(jq -r .ip_address instance.json)
SSH_PORT=$(jq -r '.ssh_port // 22' instance.json)
echo "Launched GPU instance ${INSTANCE_ID} at ${IP_ADDRESS}:${SSH_PORT}"
```

#### Step 2: Execute Remote Entrypoint (Clones directly from GitHub)
```bash
# Run remote entrypoint via SSH (clones directly from GitHub at 1Gbps+ datacenter speed)
ssh -p ${SSH_PORT} devuser@${IP_ADDRESS} "\
  REPO_URL='https://github.com/YOUR_USER/YOUR_REPO.git' \
  COMMIT_SHA='main' \
  S3_BUCKET='your_bucket' \
  S3_ENDPOINT_URL='https://s3.us-west-004.backblazeb2.com' \
  AWS_ACCESS_KEY_ID='your_backblaze_key_id' \
  AWS_SECRET_ACCESS_KEY='your_backblaze_application_key' \
  WANDB_API_KEY='your_wandb_key' \
  WANDB_PROJECT='visual-inference' \
  WANDB_RUN_NAME='manual_run_1' \
  bash -s" < scripts/cloud/remote_entrypoint.sh
```

#### Step 3: Terminate Instance
```bash
python scripts/cloud_orchestrator.py terminate --provider runpod --instance-id "${INSTANCE_ID}"
```

---

## Monitoring & Checkpoints

### Live Monitoring
- **Weights & Biases**: Open your W&B project dashboard to monitor loss curves, learning rate schedule, recall metrics (`ar/100`), and system VRAM usage in real time.
- **GitHub Actions Logs**: View live terminal output of `aws s3 sync` staging progress and PyTorch epoch outputs directly in the GHA run console.

### Checkpoint Retrieval & Durability
Checkpoints are saved automatically and atomically to the attached persistent volume at `/mnt/nfs/runs/<run_name>/`:
- `last.pt`: Contains the full training state (model weights, EMA, optimizer state, LR scheduler, AMP GradScaler, epoch/step position, and metrics) for seamless resume.
- `best.pt`: Retains the highest-performing model checkpoint based on validation recall (`ar/100`).

Because volume storage is decoupled from the compute GPU instance, your checkpoints remain completely safe even after the GPU instance terminates.
