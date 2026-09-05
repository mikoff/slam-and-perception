# Phase 1.7 shared augmentation verification

Status: approved by the owner on 2026-08-29.

## Effective contract

HBB and quad transforms now sample one immutable parameter record per source
image, seed, and epoch. Both paths use the same pixel implementation for:

- horizontal flip;
- aspect-preserving scale and letterbox;
- horizontal and vertical translation;
- brightness, contrast, and saturation jitter;
- Gaussian blur with radius 0.1–1.0;
- JPEG recompression at quality 45–95;
- normalized FP32 additive noise with standard deviation 0.02;
- padding, ImageNet normalization, and valid-mask construction.

Geometry adapters consume the same flip, scale, and offsets. Rotation,
perspective, Mosaic, MixUp, CutMix, and synthetic insertion remain explicitly
excluded. Validation samples deterministic letterbox-only parameters regardless
of seed. Quad dataset epochs use shared-memory state so persistent workers see
epoch changes; replaying an epoch reproduces its image exactly.

Unknown augmentation fields, invalid probabilities/fractions, invalid
magnitudes, and invalid scale ranges fail while loading configuration. The run
contract and W&B configuration now include the effective policy, fixed bounds,
normalization, padding, validation behavior, and excluded operations.

## Automated evidence

Same-image/same-seed tests force every photometric operation on and verify:

- pixel-identical HBB and quad tensors;
- identical valid masks and affine metadata;
- equivalent HBB and rectangular-quad geometry for positive, ignore, and
  trusted-background states;
- seed-independent validation pixels;
- changed pixels across epochs and exact epoch replay through worker-shared state.

The bounded 10,000-seed operation audit selected:

| Operation | Count | Rate |
|---|---:|---:|
| Horizontal flip | 5,068 | 50.68% |
| Color jitter | 7,933 | 79.33% |
| Blur | 999 | 9.99% |
| JPEG | 970 | 9.70% |
| Noise | 1,050 | 10.50% |
| Scale / translation / letterbox | 10,000 each | 100% |

These observed rates agree with the requested 0.5/0.8/0.1/0.1/0.1
probabilities within normal deterministic sampling variation.

## Visual gate

The real-data audit contains 20 before/after cards across BDD100K, COCO,
nuImages, and WoodScape. It includes darkest/brightest luminance proxies,
tiny-object, boundary-object, trusted-background, and fisheye cases. COCO has no
trusted-background tile by approved contract, and that absence is explicit.

Fifteen reviewed targets remain in their original state. Five are not retained
after the sampled affine and current representability/visibility gates: three
tiny positives, one WoodScape positive, and one nuImages trusted-background
tile. The images remain physically plausible; these five cards require owner
review because the chosen 16px/P2-min-4 candidate is not yet the active P3/min-8
production architecture. The owner accepted the conservative shared policy as
physically plausible. Stronger geometric or photometric distortion is deferred
to a separately measured ablation rather than added to the production policy.
