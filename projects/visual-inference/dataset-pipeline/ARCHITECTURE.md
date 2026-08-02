# Module Purpose & Boundaries

This pipeline converts the four extracted Supervisely projects into two
views: a compatibility COCO view and the compact class-agnostic proposal
manifest consumed by `student_detector`.

It owns taxonomy mapping, geometry normalization, image linking, deterministic
splits, conversion reports, validation, G1 review recovery, and automatic
source-consistency adjudication. It does not train the detector, run SigLIP
classification, or equate source consistency with semantic visual correctness.

# Technical Contracts & Interfaces

- `quad-proposal-manifest.v1` stores image metadata plus `positive`, `ignore`,
  and `trusted_background` records.
- Every emitted positive has a finite clockwise convex four-point quad,
  geometry tier, fit coverage, tightness, and source annotation identity.
- COCO exports retain `quad` and geometry metadata for existing consumers;
  their `bbox` is always derived from min/max over all quad corners.
- `validate` checks both compatibility annotations and proposal manifests,
  including links, bounds, winding, convexity, and the 98% coverage floor.
- `audit-g1-determinism` rebuilds derived annotations twice, hashes their bytes,
  hashes raw JSON before/after, validates both runs, and verifies final links.
- `audit-g1` emits source-instance accounting, COCO VOC-20/COCO-60 benchmark
  views, a stratified 300-instance HTML review, and a hashed bundle manifest.
- `audit-g1-recover-review` converts a copied browser review into CSV evidence
  when the hosting surface blocks client-side file downloads.
- `audit-g1-adjudicate` independently recomputes source containment and metrics,
  validates state/tier contracts, and emits a reviewer-confirmable HTML bundle.

# Active Design Patterns & Decisions

- Bounded samples are deterministic and retained-positive-first, preventing a
  smoke run from silently selecting an unlabeled test slice.
- Polygon/mask geometry is fitted coverage-first; sub-pixel containment
  tolerance reflects raster equivalence. Tightness is source-area occupancy.
- Valid four-point sources are canonicalized clockwise. Invalid four-point
  sources use a rotated-rectangle tier before the final HBB fallback.
- Localizable zero-area source geometry becomes a one-pixel ignore HBB and is
  reported; it never becomes a positive or silent negative.
- Mixed polygon/rectangle duplicates prefer the non-rectangle geometry.
- Intermediate images are links into raw extraction; final COCO paths remain
  compatibility artifacts, not the proposal loader's source of truth.
- Open-world benchmark training keeps COCO VOC-20 positives and moves COCO-60
  instances to ignore; validation retains both and tags them seen/unseen.
- Eligible-positive accounting excludes pre-existing source-policy ignore
  regions, but requires geometry failures to remain localized ignore regions.
- Audit sampling is seeded and round-robin stratified by source, split, state,
  geometry, size, aspect, tightness, and radial-position bins.
- Automatic adjudication preserves prior decisions: any failed, unresolved, or
  machine-invalid audit record is recommended for quarantine as ignore.
- Visualization edges and vertex markers are alpha-composited so the source
  pixels remain visible; CSV review always has a visible copy fallback.

# Local Constraints & Gotchas

- A full archive scan can be expensive, especially nuImages; bounded runs are
  appropriate for acceptance gates but must report their limits explicitly.
- Regenerate conversion, export, merge, and validation after geometry changes;
  stale intermediate JSON can otherwise hide schema defects.
- `--force all` re-hashes roughly 98 GB of source archives; for geometry-only
  changes rerun `convert-detection`, `export-coco`, `merge`, then `validate`.
- Keep the compact manifest schema synchronized with the SQLite streaming index
  in `student_detector.data`.
- Do not treat a zero-annotation manifest as a successful data gate.
- `background_supervision: true` is only image metadata; it is not permission to
  train dense negatives without explicit spatial trusted-background regions.
- G1 remains pending until all visual decisions are resolved and systematic
  errors are absent, even when every automated bundle check passes.
- Automatic PASS proves agreement with stored source geometry, not that the
  source annotation covers the complete visually perceived physical object.
