# Phase 1.2 Source Identity Verification

Verified on 2026-08-28 from the pinned official COCO 2017 instance annotations.
The prior published `output/` dataset was not replaced; verification artifacts
are isolated under `reports/step_1_2_coco_manifests`.

## Pinned inputs

| Input | SHA-256 |
|---|---|
| `instances_train2017.json` | `610fce4944abdeb15354cc765333805529359d12d88f2f711393ca586901d01d` |
| `instances_val2017.json` | `e8c7f7908f1d7278341fae127d0da654f102f11bd7b21d8aeefa635b8c810b6f` |

## Full-corpus reconciliation

| Measure | Count |
|---|---:|
| Official annotations read | 896,782 |
| Unique official identities retained | 896,782 |
| Multipart instances merged by official ID | 86,156 |
| Crowd regions mapped to ignore | 10,498 |
| Distinct-ID coincident-geometry groups retained | 3 |
| Geometry-based removals | 0 |
| Untraceable removals | 0 |
| Dataset Ninja filename-extension fallbacks | 1 |

The one filename fallback maps official image ID `320612` from
`000000320612.jpg` to the unique local file `000000320612.jpg.png`; the
substitution and reason are present in `source_provenance.json`.

Two degenerate source polygons were deterministically localized as ignore HBBs:
official annotation IDs `918` (`hot dog`) and `2206849` (`person`). They remain
in the manifest and are traceable in `invalid_geometries.csv`.

## Manifest audit

The isolated manifests contain 896,782 unique `(source split, official
annotation ID)` pairs, zero duplicate pairs, and zero missing required
provenance fields.

| Split/state | Count |
|---|---:|
| Train positive | 849,947 |
| Train ignore | 10,054 |
| Validation positive | 36,335 |
| Validation ignore | 446 |

## Determinism

Two complete normalization, geometry-conversion, and export runs produced
identical bytes. The proposal manifests were also built and hashed twice.

| Artifact | SHA-256 |
|---|---|
| Normalized annotation tree | `fb2d26d048e9cd57b3802b3898cb0c4df1f7ee5b56f589d1c20ed820a97ba2bb` |
| Detection annotation tree | `0ab5301750f97f9fd869c6218506ada4b539efe2e2d26879e6fac77182909258` |
| Train COCO export | `3ca86993e9e9650bfa69ae40893ddbf8003978f7f783576a33a1b6cbb629d147` |
| Validation COCO export | `2367b651948bed4c369aa94c4e2ea3390e7ac52c6ff38d71007fc7aedf7537ff` |
| Train proposal manifest | `62eae0aaa6789d5c8e58ebdf3d8216de42bfa8a2df565d3b17291c9bc09b5f72` |
| Validation proposal manifest | `521d4da6d597c1643e27151257ab9e535b487a665bfa85cb3389d81ae60c7a0a` |

## Owner review

The review bundle contains 100 deterministic random identities plus all three
distinct-ID coincident-geometry groups (103 cards). Reviewer `mikoff` approved
step 1.2 on 2026-08-28. The recovered page contains 98 PASS, four identity FAIL,
and one blank decision; all three coincident-geometry groups passed.

The four failed cards exactly match their pinned official annotation IDs, image
IDs, category IDs, boxes, and crowd flags. They are accepted as official-source
label anomalies rather than converter identity defects. The blank TV card is
accepted by the reviewer's overall approval. The machine-readable resolution is
stored in `reports/coco_identity_review/owner_resolution.json`.
