# Correction Phase 4 closeout

Status: complete. The fully trained Phase 3 cloud quad EMA is the selected
proposal detector; the Phase 4 19,995-step EMA is retained as the control.

## Decision

Use `artifacts/phase3/checkpoints/best_ema.pt` (SHA-256
`deee0ce3919ffdc0f6c68d675896efe55831327a3c22cd605fc8b71264f3c71f`)
with EMA weights, NMS `0.78`, score threshold `0.20`, and K=100.

The current control is
`artifacts/phase4/local_quad_p3_seed42_step20000_v1/best_ema.pt` (SHA-256
`c9b00311f8758072299ec6437a043091e902b818d8481bf71b8e4de239183c2b`)
with EMA weights, NMS `0.75`, score threshold `0.20`, and K=100.

The machine-readable selection contract is
`configs/benchmarks/correction_phase4_quad_selection_v2.json`. Historical
training configs remain unchanged because their hashes describe the runs that
created the checkpoints; selected inference policies live in this contract.

## Bound identities

| Item | Identity |
|---|---|
| Dataset | `phase3-production-bg-policy-v2-2026-08-29` |
| Dataset contract | `0c41b8af31825feb6929067a4cb7128c36569af9c19cb6d14f5cf77dc3ec797a` |
| Validation manifest | `68f632d89c281923137d70e0914e706fee5d201dc7e83de51a992ec4163cecbf` |
| Frozen 800 subset | `e705233ada6e63cb58c259b610417a6226dd9cf158acc7737842ecc758e15806` |
| Full validation subset | `b47d0982991e6a51e0f46965983d0c134223a3b712a0c3e27a408fcf71d28ad3` |
| Full validation | 32,268 images; 175,977 ground-truth objects |

## Full-validation decision

| Metric | Cloud EMA, 0.78 | Current 20k EMA, 0.75 |
|---|---:|---:|
| AR@100 | **41.12%** | 35.38% |
| Recall@100/0.50 | **65.65%** | 63.11% |
| Recall@100/0.75 | **44.52%** | 36.11% |
| Object coverage | 80.03% | **81.29%** |
| Proposal object fraction | 27.41% | **31.05%** |
| Unmatched proposals | 73.03% | **68.35%** |
| Trusted-background false | 0.83% | **0.46%** |
| Duplicates | 7.67% | **7.16%** |
| Pairwise IoU >= 0.50 | 3.19% | **2.57%** |
| Small recall@100/0.50 | **74.93%** | 70.11% |
| Tiny recall@100/0.50 | **5.17%** | 2.75% |
| Thin recall@100/0.50 | **20.51%** | 17.14% |

The cloud checkpoint wins AR on every source and improves aggregate AR@100 by
5.74 percentage points and recall@0.75 by 8.41 points. NMS `0.78` reduces its
legacy duplication problem to 7.67%, only 0.50 points above the current
control. The current model remains cleaner and has slightly broader object
coverage, so it is retained as the downstream control rather than discarded.

## Evidence

- Cloud full validation: `full_validation_phase3_cloud_nms078.json`, SHA-256
  `386f040a4c51ffd3cb39d533bcb00be0b175dfb0a0da41786a0ad3e37ac7118b`.
- Current full validation: `full_validation_current20k_nms075.json`, SHA-256
  `01c646d6849616a9407f9876ccd50918c3e2fb0d3806a5ae6b3f279f93757cf4`.
- Cloud bounded sweep: `suppression_sweep_phase3_cloud_ema_narrow_frozen800.json`,
  SHA-256 `63eb72df3d187c7014d809d5306672027cc418644abc74bb9acd265983efe881`.
- Corrected cluster audit: `proposal_diversity_800_phase3_cloud_nms078_ema.json`,
  SHA-256 `e596094b33530b1bb69d1580eaa0cce60779bbf694887df5918dc4be992d7d05`.
- Dashboard: `quad_selection_frozen_800.html`, SHA-256
  `f0d86e4d028b708d02d57cfb4f04737ed75c3ebb5a7ec49c6311faea5f62ae33`.
- Visual audit: `visual_audit_frozen_800_cloud_quad_vs_current/index.html`.

All evidence paths above are under
`artifacts/phase4/local_quad_p3_seed42_step20000_v1/`.

## Phase boundary

Proposal-model selection is closed. SigLIP integration should use the selected
cloud checkpoint and keep the current 20k checkpoint as an A/B control. Further
proposal training requires a new contract; it is not a continuation of Phase 4.
