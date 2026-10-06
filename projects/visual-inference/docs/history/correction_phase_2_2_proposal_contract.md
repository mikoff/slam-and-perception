# Correction Phase 2.2 proposal contract

Reproduce the contract audit with `scripts/audit_phase2_proposal_contract.py`.

Status: implementation verified and owner-approved for Phase 2.3.

## Production boundary

The production proposal budget is fixed at `K=100`. The decoder may rank up to
300 pre-NMS candidates, but it emits at most 100 common records per image.
SigLIP crop rate, crop construction, and semantic crop review remain deferred.

Both HBB and quad decoders now retain the pyramid level and level-local dense
location index through final NMS. These values provide stable identity without
changing model tensors, scores, decoded geometry, or suppression behavior.

## Common record

`ProposalRecord` is the shared, immutable, JSON-safe interface, versioned as
`visual-inference-proposal.v1`:

| Field | Contract |
|---|---|
| `proposal_id` | `<source>:<image>:<geometry>:P<level>:<location>` |
| `image_id`, `source_dataset` | Source image identity, including dataset namespace |
| `rank`, `score` | Zero-based final rank and detector score |
| `geometry_type` | `hbb` or `quad` |
| `model_geometry` | Four ordered `(x, y)` points in letterboxed model pixels |
| `source_geometry` | The same geometry after inverse letterbox and source clipping |
| `pyramid_level` | Integer level such as 3 for P3 |
| `location_index` | Level-local row-major dense location |
| `source_transform` | Scale, offsets, source `[H,W]`, and model `[H,W]` |

HBB boxes use the common four-point representation in top-left, top-right,
bottom-right, bottom-left order. Quad decoding already canonicalizes its four
points. A rectangular quad and the equivalent HBB therefore have identical
model and source geometry in the common record.

Final records are ordered by descending score, then pyramid level and location
index. The last two keys make exact-score ties deterministic. Stable IDs derive
from source identity and the originating dense location rather than rank.

Source coordinates invert validation letterboxing as
`(model - offset) / scale`, then clip x to `[0,width]` and y to `[0,height]`.
The interface rejects non-positive/non-finite transforms, inconsistent tensor
lengths, missing decoder identity metadata, and non-positive proposal budgets.

## Verification and visual audit

Unit tests cover decoder metadata propagation, rectangular HBB/quad parity,
letterbox inversion, source-boundary clipping, deterministic tie ordering,
stable IDs, missing metadata, and `K` enforcement.

The local audit bundle is
`artifacts/phase2/short_v1/proposal_contract_audit/index.html`. It contains eight
source-balanced validation images: four COCO, two WoodScape, one BDD100K, and
one nuImages image. Each image has separate HBB and quad panels with identical
colors. Green is ground truth; amber is the top ten proposals. The full JSON
contains all records up to K=100 and has SHA-256
`b7ffae9b06365b4549fb7ae056be19e71b85f9cc00bb6553b706ebaee081fc43`.

Automated validation confirmed contiguous ranks, unique IDs, correct geometry
types, and source bounds for every emitted record. Perspective and fisheye
overlays visually confirm that predictions align with their original images.
Overlapping proposal clusters are expected current NMS behavior and move to
Phase 2.3 duplicate measurement and threshold calibration.

## Owner review

The owner accepted the coordinate audit and authorized Phase 2.3. `K=100`, the
identity format, four-point common geometry, and source-coordinate inversion are
therefore the active downstream contract.
