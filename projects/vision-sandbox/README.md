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

- `notebook/` - Jupyter notebooks for interactive training and plotting.
  - [train_cifar_vit.ipynb](file:///home/sashamikoff/git/skillup/projects/vision-sandbox/notebook/train_cifar_vit.ipynb) - Interactive notebook for CIFAR-10 training and attention mapping.
- `scripts/` - Core training and download scripts.
  - [download_data.py](file:///home/sashamikoff/git/skillup/projects/vision-sandbox/scripts/download_data.py) - Downloads the CIFAR-10 dataset to the global `data` directory.
  - [train_cifar_vit.py](file:///home/sashamikoff/git/skillup/projects/vision-sandbox/scripts/train_cifar_vit.py) - Main batch-free training script using `torch.vmap` vectorization.

## 🚀 Usage

### 1. Download Dataset
Run the data downloader:
```bash
uv run scripts/download_data.py
```

### 2. Train ViT & Visualize Attention
Run the training script (which executes a fast subset training cycle and generates attention heatmaps):
```bash
uv run scripts/train_cifar_vit.py
```

The script will outputs its logs and save a validation plot comparing the original images, attention overlay, and positional embedding cosine similarity at:
* **Visualization Image**: [notebook/attention_visualization.png](file:///home/sashamikoff/git/skillup/projects/vision-sandbox/notebook/attention_visualization.png)



