import torch
import torch.nn as nn

class BatchedPatchEmbedding(nn.Module):
    def __init__(self, img_size=32, patch_size=4, in_channels=3, embed_dim=192):
        super().__init__()
        self.patch_size = patch_size
        self.num_patches = (img_size // patch_size) ** 2
        self.proj = nn.Conv2d(in_channels, embed_dim, kernel_size=patch_size, stride=patch_size)
        self.cls_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, self.num_patches + 1, embed_dim))
        self._init_weights()

    def _init_weights(self):
        nn.init.xavier_uniform_(self.proj.weight)
        if self.proj.bias is not None:
            nn.init.zeros_(self.proj.bias)
        nn.init.normal_(self.cls_token, std=0.02)
        nn.init.normal_(self.pos_embed, std=0.02)

    def forward(self, x):
        # x is [B, C, H, W]
        x = self.proj(x)                  # [B, EmbedDim, H//P, W//P]
        x = x.flatten(2)                  # [B, EmbedDim, NumPatches]
        x = x.transpose(1, 2)             # [B, NumPatches, EmbedDim]
        cls_tokens = self.cls_token.expand(x.shape[0], -1, -1) # [B, 1, EmbedDim]
        x = torch.cat((cls_tokens, x), dim=1) # [B, NumPatches + 1, EmbedDim]
        x = x + self.pos_embed
        return x

class BatchedMultiheadSelfAttention(nn.Module):
    def __init__(self, embed_dim, num_heads, head_dim):
        super().__init__()
        self.num_heads = num_heads
        self.head_dim = head_dim
        self.qkv = nn.Linear(embed_dim, num_heads * head_dim * 3, bias=False)
        self.out_linear = nn.Linear(num_heads * head_dim, embed_dim)
        self.attn_weights = None

    def forward(self, x):
        B, Sl, D = x.shape
        qkv = self.qkv(x) # [B, Sl, Hn * Hd * 3]
        qkv = qkv.reshape(B, Sl, 3, self.num_heads, self.head_dim)
        q, k, v = qkv.unbind(2)
        
        # Transpose to [B, Hn, Sl, Hd]
        q = q.transpose(1, 2)
        k = k.transpose(1, 2)
        v = v.transpose(1, 2)
        
        if self.training:
            # Use PyTorch native SDPA for FlashAttention speedup during training
            out = torch.nn.functional.scaled_dot_product_attention(q, k, v)
            self.attn_weights = None
        else:
            # Manual attention during evaluation to preserve attention weights for visualization
            logits = torch.matmul(q, k.transpose(-2, -1)) / (self.head_dim ** 0.5)
            A = torch.softmax(logits, dim=-1)
            if B == 1:
                self.attn_weights = A.squeeze(0)  # [Hn, Sl, Sl] for backward compatibility
            else:
                self.attn_weights = A  # [B, Hn, Sl, Sl]
            out = torch.matmul(A, v)
            
        out = out.transpose(1, 2).reshape(B, Sl, self.num_heads * self.head_dim)
        return self.out_linear(out)

class MLP(nn.Module):
    def __init__(self, embed_dim, mlp_ratio=2.0):
        super().__init__()
        hidden_dim = int(embed_dim * mlp_ratio)
        self.fc1 = nn.Linear(embed_dim, hidden_dim)
        self.act = nn.GELU()
        self.fc2 = nn.Linear(hidden_dim, embed_dim)

    def forward(self, x):
        return self.fc2(self.act(self.fc1(x)))

class BatchedTransformerBlock(nn.Module):
    def __init__(self, embed_dim, num_heads, head_dim, mlp_ratio=2.0):
        super().__init__()
        self.ln1 = nn.LayerNorm(embed_dim)
        self.msa = BatchedMultiheadSelfAttention(embed_dim, num_heads, head_dim)
        self.ln2 = nn.LayerNorm(embed_dim)
        self.mlp = MLP(embed_dim, mlp_ratio)

    def forward(self, x):
        x = x + self.msa(self.ln1(x))
        x = x + self.mlp(self.ln2(x))
        return x

class BatchedVisualTransformer(nn.Module):
    def __init__(self, img_size=32, patch_size=4, in_channels=3, num_classes=10,
                 embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0):
        super().__init__()
        self.patch_embed = BatchedPatchEmbedding(img_size, patch_size, in_channels, embed_dim)
        self.blocks = nn.Sequential(*[
            BatchedTransformerBlock(embed_dim, num_heads, head_dim, mlp_ratio)
            for _ in range(depth)
        ])
        self.ln = nn.LayerNorm(embed_dim)
        self.head = nn.Linear(embed_dim, num_classes)
        nn.init.zeros_(self.head.weight)
        nn.init.zeros_(self.head.bias)

    def forward(self, x):
        # Support both batch [B, C, H, W] and single image [C, H, W] inputs
        is_single_image = (x.ndim == 3)
        if is_single_image:
            x = x.unsqueeze(0)
            
        x = self.patch_embed(x)
        x = self.blocks(x)
        x = self.ln(x)
        cls_token_out = x[:, 0]
        out = self.head(cls_token_out)
        
        if is_single_image:
            out = out.squeeze(0)
        return out
