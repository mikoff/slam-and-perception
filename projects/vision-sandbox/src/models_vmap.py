import torch
import torch.nn as nn

class PatchEmbedding(nn.Module):
    def __init__(self, img_size=32, patch_size=4, in_channels=3, embed_dim=192):
        super().__init__()
        self.patch_size = patch_size
        self.num_patches = (img_size // patch_size) ** 2
        self.proj = nn.Conv2d(in_channels, embed_dim, kernel_size=patch_size, stride=patch_size)
        self.cls_token = nn.Parameter(torch.zeros(1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(self.num_patches + 1, embed_dim))
        self._init_weights()

    def _init_weights(self):
        nn.init.xavier_uniform_(self.proj.weight)
        if self.proj.bias is not None:
            nn.init.zeros_(self.proj.bias)
        nn.init.normal_(self.cls_token, std=0.02)
        nn.init.normal_(self.pos_embed, std=0.02)

    def forward(self, x):
        x = self.proj(x)              # [EmbedDim, H//P, W//P]
        x = x.flatten(1)             # [EmbedDim, NumPatches]
        x = x.transpose(0, 1)        # [NumPatches, EmbedDim]
        
        x = torch.cat((self.cls_token, x), dim=0) # [NumPatches + 1, EmbedDim]
        x = x + self.pos_embed
        return x

class MultiheadSelfAttention(nn.Module):
    def __init__(self, embed_dim, num_heads, head_dim):
        super().__init__()
        self.num_heads = num_heads
        self.head_dim = head_dim
        self.qkv = nn.Linear(embed_dim, num_heads * head_dim * 3, bias=False)
        self.out_linear = nn.Linear(num_heads * head_dim, embed_dim)
        self.attn_weights = None

    def forward(self, x):
        Sl, D = x.shape
        qkv = self.qkv(x)
        q, k, v = qkv.chunk(3, dim=-1)
        q = q.reshape(Sl, self.num_heads, self.head_dim)
        k = k.reshape(Sl, self.num_heads, self.head_dim)
        v = v.reshape(Sl, self.num_heads, self.head_dim)
        
        logits = torch.einsum('s h d, k h d -> h s k', q, k)
        A = torch.softmax(logits / (self.head_dim ** 0.5), dim=-1)
        self.attn_weights = A
        
        sa = torch.einsum('h s k, k h d -> s h d', A, v)
        sa = sa.reshape(Sl, self.num_heads * self.head_dim)
        return self.out_linear(sa)

class MLP(nn.Module):
    def __init__(self, embed_dim, mlp_ratio=2.0):
        super().__init__()
        hidden_dim = int(embed_dim * mlp_ratio)
        self.fc1 = nn.Linear(embed_dim, hidden_dim)
        self.act = nn.GELU()
        self.fc2 = nn.Linear(hidden_dim, embed_dim)

    def forward(self, x):
        return self.fc2(self.act(self.fc1(x)))

class TransformerBlock(nn.Module):
    def __init__(self, embed_dim, num_heads, head_dim, mlp_ratio=2.0):
        super().__init__()
        self.ln1 = nn.LayerNorm(embed_dim)
        self.msa = MultiheadSelfAttention(embed_dim, num_heads, head_dim)
        self.ln2 = nn.LayerNorm(embed_dim)
        self.mlp = MLP(embed_dim, mlp_ratio)

    def forward(self, x):
        x_norm = self.ln1(x)
        attn_out = self.msa(x_norm)
        x = x + attn_out
        
        x_norm = self.ln2(x)
        mlp_out = self.mlp(x_norm)
        x = x + mlp_out
        return x

class VisualTransformer(nn.Module):
    def __init__(self, img_size=32, patch_size=4, in_channels=3, num_classes=10,
                 embed_dim=192, depth=6, num_heads=6, head_dim=32, mlp_ratio=2.0):
        super().__init__()
        self.patch_embed = PatchEmbedding(img_size, patch_size, in_channels, embed_dim)
        self.blocks = nn.Sequential(*[
            TransformerBlock(embed_dim, num_heads, head_dim, mlp_ratio)
            for _ in range(depth)
        ])
        self.ln = nn.LayerNorm(embed_dim)
        self.head = nn.Linear(embed_dim, num_classes)
        nn.init.zeros_(self.head.weight)
        nn.init.zeros_(self.head.bias)

    def forward(self, x):
        x = self.patch_embed(x)
        x = self.blocks(x)
        x = self.ln(x)
        cls_token_out = x[0]
        out = self.head(cls_token_out)
        return out
