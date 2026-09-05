# Module Purpose & Boundaries

This pipeline converts the four extracted Supervisely projects into two
views: a compatibility COCO view and the compact class-agnostic proposal
manifest consumed by `student_detector`.

It owns taxonomy mapping, generated supervision state, geometry normalization,
image linking, sequence-safe splits, conversion reports, validation, and visual
audit generation. It does not train the detector, run SigLIP classification,
or equate source consistency with semantic visual correctness.

# Technical Contracts & Interfaces

- `proposal-manifest.v2` is the loader-facing authority for `positive`, `ignore`,
  and `trusted_background`; compatibility COCO is not a state authority.
- Every positive has a valid clockwise quad, geometry metrics, source ID, and
  preserved source attributes; COCO uses the pinned official annotation ID.
- COCO exports retain `quad` and geometry metadata for existing consumers;
  their `bbox` is always derived from min/max over all quad corners.
- `validate` checks both compatibility annotations and proposal manifests,
  including links, bounds, winding, convexity, and the 98% coverage floor.
- Object-contract audit commands resolve every source class, emit exact source
  overlays, and recover category decisions from copied browser text.
- `audit-woodscape-supervision` renders generated ego, construction, and group
  states with neighboring supervision for owner review.

# Active Design Patterns & Decisions

- Bounded samples are deterministic, positive-first, train/validation
  interleaved, and place qualitative test frames last.
- Polygon/mask geometry is fitted coverage-first; sub-pixel containment
  tolerance reflects raster equivalence. Tightness is source-area occupancy.
- Valid four-point sources are canonicalized clockwise. Invalid four-point
  sources use a rotated-rectangle tier before the final HBB fallback.
- Trusted background comes only from owner-approved contract categories. A tile
  is retained at 100% occupancy after positive and ignore subtraction, using
  precedence `positive > ignore > trusted background > weak`.
- Source self-intersection, duplicate vertices, zero area, and out-of-frame
  repairs are logged; unrepairable local geometry is quarantined as ignore.
- Localizable zero-area source geometry becomes a one-pixel ignore HBB and is
  reported; it never becomes a positive or silent negative.
- Deduplication is identity-first: representations may collapse only when their
  source annotation identity agrees; coincident distinct identities survive.
- Intermediate images are links into raw extraction; final COCO remains compatibility-only.
- Full merge spills sorted per-export shards and streams atomic COCO/manifests to avoid OOM.
- Open-world benchmark training retains all auxiliary-source positives, keeps
  COCO VOC-20 positives, and moves COCO-60 instances to ignore. Validation tags
  COCO seen/unseen and auxiliary records for the common evaluator.
- Eligible-positive accounting excludes pre-existing source-policy ignore
  regions, but requires geometry failures to remain localized ignore regions.
- Audit sampling is seeded and round-robin stratified by source, split, state,
  geometry, size, aspect, tightness, and radial-position bins.
- Automatic adjudication is authoritative by owner decision. PASS retains the
  current state; a failed positive becomes ignore. The current full-corpus run
  has 9,083 PASS records and zero quarantines.
- Contained-component policy runs once during common manifest generation; HBB
  and quad readers consume identical state codes without reinterpretation.
- WoodScape validation holds out whole timestamp-derived labeled RGB sequences;
  raw test remains qualitative, and soiling train data never enters validation.

# Local Constraints & Gotchas

- Full archive scans are expensive; bounded gates must report their limits.
- Cached archive hashes may seed extraction only when canonical path, byte size,
  and nanosecond mtime still match; `--verify-archives` forces a fresh scan.
- Regenerate conversion, export, merge, and validation after geometry changes;
  stale intermediate JSON can otherwise hide schema defects.
- COCO official JSON hashes are verified before streaming normalization; full
  identity, geometry, export, and proposal-manifest hashes must reproduce.
- Keep the compact manifest schema synchronized with SQLite schema v8 in
  `student_detector.data`; `attributes` become queryable `attributes_json`.
- A WoodScape sequence with many synchronized frames can dominate a greedy
  holdout; deterministic bounded subset-sum selects the requested image fraction.
- Do not treat a zero-annotation manifest as a successful data gate.
- `background_supervision: true` is derived only when explicit spatial tiles
  exist. Source identity such as COCO alone never authorizes dense negatives.
- Owner acceptance replaces further manual annotation for this bounded review;
  new manifest hashes require reapplying the automatic policy.
- Automatic PASS proves agreement with stored source geometry, not that the
  source annotation covers the complete visually perceived physical object.
