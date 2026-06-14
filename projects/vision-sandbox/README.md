# 👁️ Vision Sandbox

A sandbox project for training, evaluating, and playing around with neural networks, specifically targeting Vision Transformers (ViT), DINOv3, and related vision models.

## 🚀 Setup and Environment

This project uses [uv](https://github.com/astral-sh/uv) for fast, reliable Python dependency and environment management.

### Prerequisites

Ensure you have `uv` installed. If not, you can install it using:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Installation

1. Navigate to the project directory:
   ```bash
   cd projects/vision-sandbox
   ```

2. Sync the environment (this will create a `.venv` directory and install all dependencies):
   ```bash
   uv sync
   ```

3. Activate the environment:
   ```bash
   source .venv/bin/activate
   ```

## 🛠️ Project Structure

- `notebook/` - Jupyter notebooks for interactive development and plotting.
- `scripts/` - Python scripts for batch jobs, training, or long-running tasks.

## 🧪 Verification

You can verify that PyTorch is successfully installed and has access to your GPU (CUDA) by running:
```bash
uv run scripts/verify_env.py
```
Or by launching Jupyter/VS Code and opening the notebook:
- `notebook/verify_env.ipynb`
