import os
import sys
import time
from pathlib import Path
import torch
import torch.nn as nn
import torch.optim as optim
from torch.func import functional_call
import matplotlib.pyplot as plt
import numpy as np

# Add project root to sys.path to enable src imports
project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))

from src.utils import get_cifar_dataloaders
from src.models_vmap import VisualTransformer as VmapVisualTransformer
from src.models_batched import BatchedVisualTransformer

def run_option_a(device, train_loader, epochs):
    print("\n" + "-"*50)
    print("Training Option A: Optimized vmap (Single-Image Model)")
    print("-"*50)
    
    model = VmapVisualTransformer(
        img_size=32, patch_size=4, in_channels=3, num_classes=10,
        embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0
    ).to(device)
    
    params = dict(model.named_parameters())
    buffers = dict(model.named_buffers())
    
    def forward_fn(params, buffers, x):
        return functional_call(model, (params, buffers), x)
        
    batched_forward = torch.func.vmap(forward_fn, in_dims=(None, None, 0))
    
    optimizer = optim.AdamW(params.values(), lr=3.0e-3, weight_decay=5e-2, fused=True)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.CrossEntropyLoss()
    
    start_time = time.time()
    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        correct = 0
        total = 0
        
        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device, non_blocking=True), targets.to(device, non_blocking=True)
            
            optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast('cuda', dtype=torch.bfloat16):
                outputs = batched_forward(params, buffers, inputs)
                loss = criterion(outputs, targets)
            
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            _, predicted = outputs.max(1)
            total += targets.size(0)
            correct += predicted.eq(targets).sum().item()
            
        scheduler.step()
        train_acc = 100. * correct / total
        print(f"Epoch {epoch+1:02d}/{epochs:02d} | Loss: {running_loss/len(train_loader):.4f} | Train Acc: {train_acc:.2f}%")
        
    total_time = time.time() - start_time
    print(f"Option A training finished in {total_time:.2f}s")
    return model, total_time

def run_option_b(device, train_loader, epochs):
    print("\n" + "-"*50)
    print("Training Option B: Standard Batched Model + FlashAttention (SDPA)")
    print("-"*50)
    
    # Enable TF32 for extra matmul speedup
    torch.set_float32_matmul_precision('high')
    
    model = BatchedVisualTransformer(
        img_size=32, patch_size=4, in_channels=3, num_classes=10,
        embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0
    ).to(device)
    
    # Configure compiler settings to avoid autotuning freezes on some devices
    if os.environ.get("DISABLE_COMPILE", "0") == "1":
        print("Compilation disabled via DISABLE_COMPILE=1 environment variable.")
        model_compiled = model
    else:
        try:
            import torch._inductor.config as inductor_config
            # Disable intensive autotuning to prevent compiler freezes on lower SM or VM GPUs
            inductor_config.max_autotune = False
            inductor_config.max_autotune_gemm = False
        except Exception:
            pass
        
        print("Compiling model (this might take a minute on the first epoch)...")
        model_compiled = torch.compile(model)
    
    optimizer = optim.AdamW(model_compiled.parameters(), lr=3.0e-3, weight_decay=5e-2, fused=True)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.CrossEntropyLoss()
    
    start_time = time.time()
    for epoch in range(epochs):
        model_compiled.train()
        running_loss = 0.0
        correct = 0
        total = 0
        
        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device, non_blocking=True), targets.to(device, non_blocking=True)
            
            optimizer.zero_grad(set_to_none=True)
            with torch.amp.autocast('cuda', dtype=torch.bfloat16):
                outputs = model_compiled(inputs)
                loss = criterion(outputs, targets)
                
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            _, predicted = outputs.max(1)
            total += targets.size(0)
            correct += predicted.eq(targets).sum().item()
            
        scheduler.step()
        train_acc = 100. * correct / total
        print(f"Epoch {epoch+1:02d}/{epochs:02d} | Loss: {running_loss/len(train_loader):.4f} | Train Acc: {train_acc:.2f}%")
        
    total_time = time.time() - start_time
    print(f"Option B training finished in {total_time:.2f}s")
    return model, total_time

def generate_visualizations(model, testset, device):
    print("\nGenerating attention maps and positional similarity plots...")
    model.eval()
    classes = ['airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']
    fig, axs = plt.subplots(3, 4, figsize=(12, 9))
    fig.suptitle("Vision Transformer Attention Maps Visualized", fontsize=16, fontweight='bold')
    
    pos_embed = model.patch_embed.pos_embed.detach().cpu()
    spatial_pos = pos_embed[0, 1:]
    
    norm_pos = spatial_pos / spatial_pos.norm(dim=-1, keepdim=True)
    similarity = torch.mm(norm_pos, norm_pos.t()).numpy()
    
    for i in range(3):
        img_tensor, label_idx = testset[i]
        
        with torch.no_grad():
            _ = model(img_tensor.to(device))
            
        attn = model.blocks[-1].msa.attn_weights.detach().cpu()
        mean_attn = attn.mean(dim=0)
        cls_attn = mean_attn[0, 1:]
        
        cls_attn_grid = cls_attn.reshape(8, 8).numpy()
        cls_attn_resized = np.kron(cls_attn_grid, np.ones((4, 4)))
        
        img_np = img_tensor.permute(1, 2, 0).numpy()
        img_np = img_np * np.array([0.2023, 0.1994, 0.2010]) + np.array([0.4914, 0.4822, 0.4465])
        img_np = np.clip(img_np, 0, 1)
        
        axs[i, 0].imshow(img_np)
        axs[i, 0].set_title(f"Target: {classes[label_idx]}")
        axs[i, 0].axis('off')
        
        axs[i, 1].imshow(cls_attn_grid, cmap='viridis')
        axs[i, 1].set_title("CLS Attention Grid (8x8)")
        axs[i, 1].axis('off')
        
        axs[i, 2].imshow(img_np)
        axs[i, 2].imshow(cls_attn_resized, cmap='jet', alpha=0.5)
        axs[i, 2].set_title("Attention Map Overlay")
        axs[i, 2].axis('off')
        
        center_similarity = similarity[28].reshape(8, 8)
        axs[i, 3].imshow(center_similarity, cmap='hot')
        axs[i, 3].set_title("Pos Sim (Center vs All)")
        axs[i, 3].axis('off')
        
    plt.tight_layout()
    save_path = project_root / "notebook" / "attention_visualization.png"
    plt.savefig(save_path, dpi=150)
    print(f"Visualizations saved to: {save_path}")
    plt.close()

def main():
    print("=" * 60)
    print("ViT Training Comparison (vmap vs Batched SDPA)")
    print("=" * 60)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")
    
    # 512 batch size to saturate the GPU
    train_loader, test_loader, testset = get_cifar_dataloaders(
        batch_size=512, 
        num_workers=4, 
        pin_memory=True,
        train_subset_size=10000, 
        test_subset_size=1000
    )
    
    epochs = 10
    
    # Run Option A
    _, time_a = run_option_a(device, train_loader, epochs)
    
    # Run Option B
    model_b, time_b = run_option_b(device, train_loader, epochs)
    
    print("\n" + "=" * 60)
    print("TRAINING SPEEDUP REPORT")
    print("=" * 60)
    print(f"Option A (Optimized vmap):           {time_a:.2f}s")
    print(f"Option B (Batched + SDPA + Compile): {time_b:.2f}s")
    print(f"Speedup Factor:                     {time_a / time_b:.2f}x faster!")
    print("=" * 60)
    
    # Generate visualizations using Option B (fully backwards compatible)
    generate_visualizations(model_b, testset, device)

if __name__ == "__main__":
    main()
