import json
from pathlib import Path

notebook = {
 "cells": [
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "# 👁️ Optimized Vision Transformer Training on CIFAR-10\n",
    "\n",
    "This notebook compares two Vision Transformer (ViT) training implementations on CIFAR-10:\n",
    "\n",
    "1. **Option A (vmap-based)**: Your original single-image model design, batch-vectorized using `torch.func.vmap`, with DataLoader & AMP (bfloat16) optimizations.\n",
    "2. **Option B (Standard Batched + FlashAttention)**: A fully refactored, native batched model using PyTorch's native `scaled_dot_product_attention` (SDPA), Automatic Mixed Precision (AMP), TensorFloat-32 (TF32), and `torch.compile`."
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "import os\n",
    "import sys\n",
    "from pathlib import Path\n",
    "# Add project root to sys.path so we can import from src\n",
    "sys.path.append(str(Path.cwd().parent))\n",
    "\n",
    "import time\n",
    "import torch\n",
    "import torch.nn as nn\n",
    "import torch.optim as optim\n",
    "from torch.func import functional_call\n",
    "import matplotlib.pyplot as plt\n",
    "import numpy as np\n",
    "\n",
    "from src.utils import get_cifar_dataloaders\n",
    "from src.models_vmap import VisualTransformer as VmapVisualTransformer\n",
    "from src.models_batched import BatchedVisualTransformer\n",
    "\n",
    "device = torch.device(\"cuda\" if torch.cuda.is_available() else \"cpu\")\n",
    "print(f\"Using device: {device}\")"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "## 1. Load Data\n",
    "\n",
    "We load the CIFAR-10 dataset using the shared utility. We use a batch size of 512, 4 worker threads, and pin memory to maximize GPU data-transfer speed."
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "# Batch size of 512 to fully saturate the GPU\n",
    "train_loader, test_loader, testset = get_cifar_dataloaders(\n",
    "    batch_size=512, \n",
    "    num_workers=4, \n",
    "    pin_memory=True,\n",
    "    train_subset_size=10000, \n",
    "    test_subset_size=1000\n",
    ")"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "## 2. Train Option A: Optimized `vmap` (Single-Image Model)\n",
    "\n",
    "We train the original single-image architecture scaled with `torch.func.vmap`. Optimizations applied:\n",
    "* Batch size scaled to 512\n",
    "* `num_workers=4`, `pin_memory=True`\n",
    "* Fused AdamW optimizer\n",
    "* Automatic Mixed Precision (AMP) in `bfloat16`\n",
    "* `zero_grad(set_to_none=True)`"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "model_vmap = VmapVisualTransformer(\n",
    "    img_size=32, patch_size=4, in_channels=3, num_classes=10,\n",
    "    embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0\n",
    ").to(device)\n",
    "\n",
    "params = dict(model_vmap.named_parameters())\n",
    "buffers = dict(model_vmap.named_buffers())\n",
    "\n",
    "def forward_fn(params, buffers, x):\n",
    "    return functional_call(model_vmap, (params, buffers), x)\n",
    "    \n",
    "batched_forward = torch.func.vmap(forward_fn, in_dims=(None, None, 0))\n",
    "\n",
    "# Fused AdamW for faster updates on GPU\n",
    "optimizer = optim.AdamW(params.values(), lr=3.0e-3, weight_decay=5e-2, fused=True)\n",
    "epochs = 10\n",
    "scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)\n",
    "criterion = nn.CrossEntropyLoss()\n",
    "\n",
    "print(\"Training Option A (Optimized vmap)...\")\n",
    "start_vmap = time.time()\n",
    "\n",
    "for epoch in range(epochs):\n",
    "    model_vmap.train()\n",
    "    running_loss = 0.0\n",
    "    correct = 0\n",
    "    total = 0\n",
    "    \n",
    "    for inputs, targets in train_loader:\n",
    "        inputs, targets = inputs.to(device, non_blocking=True), targets.to(device, non_blocking=True)\n",
    "        \n",
    "        optimizer.zero_grad(set_to_none=True)\n",
    "        with torch.amp.autocast('cuda', dtype=torch.bfloat16):\n",
    "            outputs = batched_forward(params, buffers, inputs)\n",
    "            loss = criterion(outputs, targets)\n",
    "        \n",
    "        loss.backward()\n",
    "        optimizer.step()\n",
    "        \n",
    "        running_loss += loss.item()\n",
    "        _, predicted = outputs.max(1)\n",
    "        total += targets.size(0)\n",
    "        correct += predicted.eq(targets).sum().item()\n",
    "        \n",
    "    scheduler.step()\n",
    "    train_acc = 100. * correct / total\n",
    "    print(f\"Epoch {epoch+1:02d}/{epochs:02d} | Loss: {running_loss/len(train_loader):.4f} | Train Acc: {train_acc:.2f}%\")\n",
    "\n",
    "vmap_total_time = time.time() - start_vmap\n",
    "print(f\"Option A finished in {vmap_total_time:.2f}s\")"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "## 3. Train Option B: Standard Batched Model + FlashAttention (SDPA)\n",
    "\n",
    "We train the refactored, native batched model. Optimizations applied:\n",
    "* Batch size scaled to 512\n",
    "* `num_workers=4`, `pin_memory=True`\n",
    "* PyTorch's native `scaled_dot_product_attention` (SDPA) which dispatches to FlashAttention-2\n",
    "* TensorFloat-32 (TF32) enabled for PyTorch matrix multiplications\n",
    "* `torch.compile()` for kernel fusion\n",
    "* Fused AdamW optimizer\n",
    "* Automatic Mixed Precision (AMP) in `bfloat16`\n",
    "* `zero_grad(set_to_none=True)`"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "# Enable TF32 for extra matmul speedup\n",
    "torch.set_float32_matmul_precision('high')\n",
    "\n",
    "model_batched = BatchedVisualTransformer(\n",
    "    img_size=32, patch_size=4, in_channels=3, num_classes=10,\n",
    "    embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0\n",
    ").to(device)\n",
    "\n",
    "# Configure compiler settings to avoid autotuning freezes on some devices\n",
    "if os.environ.get(\"DISABLE_COMPILE\", \"0\") == \"1\":\n",
    "    print(\"Compilation disabled via DISABLE_COMPILE=1 environment variable.\")\n",
    "    model_batched_compiled = model_batched\n",
    "else:\n",
    "    try:\n",
    "        import torch._inductor.config as inductor_config\n",
    "        # Disable intensive autotuning to prevent compiler freezes on lower SM or VM GPUs\n",
    "        inductor_config.max_autotune = False\n",
    "        inductor_config.max_autotune_gemm = False\n",
    "    except Exception:\n",
    "        pass\n",
    "    print(\"Compiling model (this might take a minute on the first epoch)...\")\n",
    "    model_batched_compiled = torch.compile(model_batched)\n",
    "\n",
    "optimizer = optim.AdamW(model_batched_compiled.parameters(), lr=3.0e-3, weight_decay=5e-2, fused=True)\n",
    "scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)\n",
    "criterion = nn.CrossEntropyLoss()\n",
    "\n",
    "print(\"Training Option B (Standard Batched + FlashAttention + Compile)...\")\n",
    "start_batched = time.time()\n",
    "\n",
    "for epoch in range(epochs):\n",
    "    model_batched_compiled.train()\n",
    "    running_loss = 0.0\n",
    "    correct = 0\n",
    "    total = 0\n",
    "    \n",
    "    for inputs, targets in train_loader:\n",
    "        inputs, targets = inputs.to(device, non_blocking=True), targets.to(device, non_blocking=True)\n",
    "        \n",
    "        optimizer.zero_grad(set_to_none=True)\n",
    "        with torch.amp.autocast('cuda', dtype=torch.bfloat16):\n",
    "            outputs = model_batched_compiled(inputs)\n",
    "            loss = criterion(outputs, targets)\n",
    "            \n",
    "        loss.backward()\n",
    "        optimizer.step()\n",
    "        \n",
    "        running_loss += loss.item()\n",
    "        _, predicted = outputs.max(1)\n",
    "        total += targets.size(0)\n",
    "        correct += predicted.eq(targets).sum().item()\n",
    "        \n",
    "    scheduler.step()\n",
    "    train_acc = 100. * correct / total\n",
    "    print(f\"Epoch {epoch+1:02d}/{epochs:02d} | Loss: {running_loss/len(train_loader):.4f} | Train Acc: {train_acc:.2f}%\")\n",
    "    \n",
    "batched_total_time = time.time() - start_batched\n",
    "print(f\"Option B finished in {batched_total_time:.2f}s\")"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "## 4. Speed Comparison"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "speedup = vmap_total_time / batched_total_time\n",
    "print(f\"Option A (Optimized vmap): {vmap_total_time:.2f}s\")\n",
    "print(f\"Option B (Batched + SDPA + Compile): {batched_total_time:.2f}s\")\n",
    "print(f\"Speedup Factor: {speedup:.2fx} faster! (Note: Option B includes ~20-30s first-epoch compile overhead)\")"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "## 5. Visualizing Attention Maps & Positional Embeddings\n",
    "\n",
    "We visualize the attention maps and positional similarity. The batched model `model_batched` is fully backward-compatible. In `eval()` mode, it automatically computes manual attention and saves the `attn_weights` in the exact shape (`[Hn, Sl, Sl]`) required by this visualization code."
   ]
  },
  {
   "cell_type": "code",
   "execution_count": None,
   "metadata": {},
   "outputs": [],
   "source": [
    "# Set batched model to evaluation mode\n",
    "model_batched.eval()\n",
    "\n",
    "classes = ['airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']\n",
    "fig, axs = plt.subplots(3, 4, figsize=(12, 9))\n",
    "\n",
    "# Positional embeddings: shape is [1, num_patches + 1, embed_dim]\n",
    "pos_embed = model_batched.patch_embed.pos_embed.detach().cpu()\n",
    "spatial_pos = pos_embed[0, 1:]\n",
    "\n",
    "norm_pos = spatial_pos / spatial_pos.norm(dim=-1, keepdim=True)\n",
    "similarity = torch.mm(norm_pos, norm_pos.t()).numpy()\n",
    "\n",
    "for i in range(3):\n",
    "    img_tensor, label_idx = testset[i]\n",
    "    \n",
    "    with torch.no_grad():\n",
    "        # Pass single image directly (shape [3, 32, 32]). The model handles it automatically!\n",
    "        _ = model_batched(img_tensor.to(device))\n",
    "        \n",
    "    # Access attention weights from the batched multihead attention block\n",
    "    attn = model_batched.blocks[-1].msa.attn_weights.detach().cpu()\n",
    "    mean_attn = attn.mean(dim=0)\n",
    "    cls_attn = mean_attn[0, 1:]\n",
    "    \n",
    "    cls_attn_grid = cls_attn.reshape(8, 8).numpy()\n",
    "    cls_attn_resized = np.kron(cls_attn_grid, np.ones((4, 4)))\n",
    "    \n",
    "    img_np = img_tensor.permute(1, 2, 0).numpy()\n",
    "    img_np = img_np * np.array([0.2023, 0.1994, 0.2010]) + np.array([0.4914, 0.4822, 0.4465])\n",
    "    img_np = np.clip(img_np, 0, 1)\n",
    "    \n",
    "    axs[i, 0].imshow(img_np)\n",
    "    axs[i, 0].set_title(f\"Target: {classes[label_idx]}\")\n",
    "    axs[i, 0].axis('off')\n",
    "    \n",
    "    axs[i, 1].imshow(cls_attn_grid, cmap='viridis')\n",
    "    axs[i, 1].set_title(\"CLS Attention (8x8)\")\n",
    "    axs[i, 1].axis('off')\n",
    "    \n",
    "    axs[i, 2].imshow(img_np)\n",
    "    axs[i, 2].imshow(cls_attn_resized, cmap='jet', alpha=0.5)\n",
    "    axs[i, 2].set_title(\"Attention Overlay\")\n",
    "    axs[i, 2].axis('off')\n",
    "    \n",
    "    center_similarity = similarity[28].reshape(8, 8)\n",
    "    axs[i, 3].imshow(center_similarity, cmap='hot')\n",
    "    axs[i, 3].set_title(\"Pos Sim (Center vs All)\")\n",
    "    axs[i, 3].axis('off')\n",
    "\n",
    "plt.tight_layout()\n",
    "plt.show()"
   ]
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "name": "python"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}

output_path = Path("/home/sashamikoff/git/skillup/projects/vision-sandbox/notebook/train_cifar_vit.ipynb")
with open(output_path, "w", encoding="utf-8") as f:
    json.dump(notebook, f, indent=1)

print(f"Successfully wrote updated notebook to {output_path}")
