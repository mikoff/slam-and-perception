# Module Purpose & Boundaries

This pipeline converts the four extracted Supervisely projects into two
views: a compatibility COCO view and the compact class-agnostic proposal
manifest consumed by `student_detector`.

It owns taxonomy mapping, geometry normalization, image linking, deterministic
splits, conversion reports, and validation. It does not train the detector or
run SigLIP classification.

# Technical Contracts & Interfaces

- `quad-proposal-manifest.v1` stores image metadata plus `positive`, `ignore`,
  and `trusted_background` records.
- Every emitted positive has a finite clockwise convex four-point quad,
  geometry tier, fit coverage, tightness, and source annotation identity.
- COCO exports retain `quad` and geometry metadata for existing consumers;
  their `bbox` is always derived from min/max over all quad corners.
- `validate` checks both compatibility annotations and proposal manifests,
  including links, bounds, winding, convexity, and the 98% coverage floor.

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
