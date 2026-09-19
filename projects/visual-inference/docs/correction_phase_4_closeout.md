# Correction Phase 4 closeout

Status: complete with the existing quad EMA checkpoint retained. No full
20,000-step run, new production checkpoint, or deployment package is approved.

## Decision

The local matched pilots reject both Phase 4 challengers under the frozen
proposal-utility constraints. HBB improves recall and coverage but materially
reduces proposal object fraction and increases unmatched proposals. Continued
quad training improves AR and high-IoU recall, but the same two quality measures
move farther from the incumbent. Suppression calibration cannot remove the quad
regression without losing protected coverage and small-object recall.

The production incumbent remains the Phase 2 quad P3–P5 EMA state in
`artifacts/phase2/short_v1/quad_p3/last.pt`, SHA-256
`cf4d1a7228fab007317b1a9d6f2c037ccc139fa16e1c223a7d2d3a44fc8c095d`.

## Bound identities

| Item | Identity |
|---|---|
| Dataset | `phase3-production-bg-policy-v2-2026-08-29` |
| Dataset contract | `0c41b8af31825feb6929067a4cb7128c36569af9c19cb6d14f5cf77dc3ec797a` |
| Train manifest | `dd3111cda227ae45777e605bfec95daf58ef7c7e6234501d3a4507efa6d81457` |
| Validation manifest | `68f632d89c281923137d70e0914e706fee5d201dc7e83de51a992ec4163cecbf` |
| Frozen 800-image subset | `e705233ada6e63cb58c259b610417a6226dd9cf158acc7737842ecc758e15806` |
| HBB source commit | `565140437edcd472142d81bd958a3d0a30a637ea` |
| Quad source commit | `90dbceb0070c0a0bcc52b6a260f5d2b3954422b4` |

Both runs recorded clean project Git state, schema-8 indexes, identical source
mixtures, seed 42, effective batch 128, FP16 training, and EMA evaluation.

## Frozen utility result

All values use the approved geometry-specific suppression policy and identical
800-image cohort. Percentages are aggregate unless the row names a slice.

| Metric | Quad incumbent | Quad 2k | Quad 5k | HBB 2k |
|---|---:|---:|---:|---:|
| AR@100 | 16.56% | 21.52% | **24.30%** | 22.89% |
| Recall@100/0.50 | 41.02% | 47.20% | 50.09% | **51.27%** |
| Recall@100/0.75 | 10.96% | 17.24% | **20.83%** | 17.70% |
| Object coverage | 70.37% | 74.72% | 77.33% | **87.53%** |
| Proposal object fraction | **53.05%** | 52.97% | 50.66% | 43.71% |
| Unmatched proposals | **46.56%** | 53.74% | 57.11% | 58.98% |
| Trusted-background false | 1.95% | 1.47% | **1.28%** | 1.80% |
| Duplicates | 10.50% | 8.79% | 8.24% | **7.16%** |
| Small recall@100/0.50 | 33.41% | 35.45% | 37.09% | **43.51%** |
| Thin recall@100/0.50 | 7.94% | 13.70% | **15.39%** | 14.60% |

HBB fails proposal-object-fraction and unmatched gates against the quad
incumbent. The 2k quad diagnostic fails unmatched by +7.17 pp. At 5k, AR gains
another +2.78 pp over 2k, while object fraction falls 2.31 pp and unmatched
rises 3.38 pp. Against the incumbent, quad 5k fails object fraction by -2.39 pp
and unmatched by +10.55 pp. This is a clear failure, not a warning-band result;
seed 43 is not authorized.

## Suppression finding

The bounded quad sweep covered NMS 0.70–0.85 and scores 0.20–0.40, followed by
an evidence-guided NMS-0.80 score extension through 0.80. Scores through 0.40
are effectively identical because the retained top 100 usually exceed 0.40.
A 0.60 cutoff reduces unmatched to 47.68% but also lowers object coverage to
68.27% and small recall to 29.89%. A 0.65 cutoff passes unmatched at 43.02%
but retains only 61.97 proposals/image and further reduces protected recall.
No tested operating point passes all incumbent-relative gates.

## Artifact evidence

The machine-readable manifest is
`configs/benchmarks/correction_phase4_closeout_v1.json`. Important report hashes:

- HBB 2k utility: `d65337082682f7deccd30d88b16fcffb8388525f1ba428e0c935a068fda47360`;
- quad 2k utility: `62f087fc72fea48070418e30cc17096e41cbbcb63cb1c67bd45013eb38c0ee3c`;
- bounded suppression: `1eb985fff4f3d29d2dafe93875091bfa0fba656634d280c80edb44c91b4254da`;
- high-score extension: `2d63f83851736dd97a358d7ebe5cd9587821c7bac66bacf4f0bc53207ac5675f`;
- quad 5k utility: `b2444ab027192fbe638c7bae0fe986db79129d681c1b8a3ac6c9f913f0d816aa`.

The step-2,000 quad `best_ema.pt` had SHA-256
`381e88477b309aa7861b9b632d0158aae749776b30089117034c98a2cc1e4d90`
when evaluated, but the in-place continuation replaced it with the step-5,000
best EMA artifact. Its utility report remains valid and binds the original hash;
the intermediate weights are not locally recoverable. Future staged experiments
must use immutable step-named best checkpoints.

## Closed and deferred work

- Do not continue the unchanged recipe to 20,000 steps.
- Do not promote or package either Phase 4 challenger.
- Reuse the already evaluated/exported incumbent and its rollback evidence.
- Any new ranking/background-loss correction requires a new versioned contract
  and short matched pilot; it is not an automatic continuation of Phase 4.
- SigLIP, semantic crop utility, INT8, physical RPi acceptance, and deployment
  remain outside this proposal-only closeout.
